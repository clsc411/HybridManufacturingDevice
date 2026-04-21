"""
DispenseUI.py — Tkinter GUI for Hybrid Casting System dispense setup.

Usage: py DispenseUI.py

Browse for a mold STL to auto-compute cavity volume, or enter volume manually.
Sends the target volume to the Arduino over serial.
Purge button flushes old resin from tubing between cycles.
"""

import time
import threading
import tkinter as tk
from tkinter import filedialog, simpledialog, font as tkfont

import trimesh
import serial as pyserial
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from CalculateMoldVolume import compute_cavity_volume_ml, compute_cavity_from_part_ml
from ThermalCalculations import simulate as thermal_simulate, RESIN_DENSITY, MOLD_DENSITY

# ── Defaults ────────────────────────────────────────────────────────────────
DEFAULT_PORT      = "COM5"
DEFAULT_BAUD      = 115200
DEFAULT_OVERFILL  = 0.0    # percent
DEFAULT_PURGE_ML  = 10.0   # mL to push through tubing during purge
NOZZLE_DEAD_ML    = 2.0  # mixing nozzle volume (3.2 mm radius, 70 mm length³)

# Conversion factors to millimeters (STL files may use different units)
UNIT_TO_MM = {"mm": 1.0, "cm": 10.0, "in": 25.4, "m": 1000.0}


class DispenseApp:
    def __init__(self, root):
        self.root = root
        root.title("Hybrid Casting System \u2014 Dispense Setup")
        root.resizable(True, True)

        # Persistent serial connection (shared across all operations)
        self._ser = None
        self._ser_port = None  # track which port we opened
        self._serial_lock = threading.Lock()  # prevent concurrent serial access
        self._cancel_event = threading.Event()  # signal running ops to abort

        # Printer monitor state (USB trigger — separate serial connection)
        self._printer_ser = None
        self._printer_monitoring = False
        self._monitor_thread = None

        # Computed values (set after STL load)
        self.cavity_ml = None
        self.outer_ml = None
        self.solid_ml = None
        self.target_ml = None
        self._stl_mesh = None           # trimesh object for 3D preview
        self._stl_file_path = None      # path to loaded STL for rescaling

        pad = {"padx": 10, "pady": 4}
        section_font = tkfont.Font(weight="bold", size=10)

        # ── Top-level two-column layout: controls | 3D preview ───────
        # Left column: scrollable canvas so the controls are accessible
        # even on smaller screens.
        root.grid_rowconfigure(0, weight=1)
        root.grid_columnconfigure(0, weight=0)
        root.grid_columnconfigure(1, weight=1)

        left_canvas = tk.Canvas(root, highlightthickness=0)
        left_scrollbar = tk.Scrollbar(root, orient="vertical",
                                      command=left_canvas.yview)
        left_canvas.configure(yscrollcommand=left_scrollbar.set)

        left_canvas.grid(row=0, column=0, sticky="ns")
        left_scrollbar.grid(row=0, column=1, sticky="ns")

        left_frame = tk.Frame(left_canvas)
        left_frame_id = left_canvas.create_window(
            (0, 0), window=left_frame, anchor="nw")

        def _on_left_configure(event):
            # Match canvas width to content so nothing is clipped
            left_canvas.configure(
                scrollregion=left_canvas.bbox("all"),
                width=event.width)
        left_frame.bind("<Configure>", _on_left_configure)

        # Mouse-wheel scrolling on the left panel
        def _on_mousewheel(event):
            left_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        left_canvas.bind_all("<MouseWheel>", _on_mousewheel)

        right_frame = tk.Frame(root)
        right_frame.grid(row=0, column=2, sticky="n", padx=10, pady=10)

        preview_frame = tk.LabelFrame(right_frame, text="STL Preview", font=section_font)
        preview_frame.pack(side="top")
        self._setup_preview(preview_frame)

        thermal_frame = tk.LabelFrame(right_frame, text="Thermal Analysis", font=section_font)
        thermal_frame.pack(side="top", pady=(8, 0))
        self._setup_thermal_plot(thermal_frame)

        # ── Process Control (Stop / Resume / Reset) ──────────────────
        ctrl_frame = tk.LabelFrame(right_frame, text="Process Control", font=section_font)
        ctrl_frame.pack(side="top", pady=(8, 0), fill="x")
        self._setup_process_controls(ctrl_frame)

        # ── Printer Monitor (USB Trigger) ────────────────────────────
        printer_frame = tk.LabelFrame(right_frame, text="Printer Monitor (USB Trigger)",
                                      font=section_font)
        printer_frame.pack(side="top", pady=(8, 0), fill="x")
        self._setup_printer_monitor(printer_frame)

        # ── STL Mode ───────────────────────────────────────────────────
        row = 0
        tk.Label(left_frame, text="STL Mode", font=section_font).grid(
            row=row, column=0, sticky="w", **pad)

        row += 1
        self.stl_mode_var = tk.StringVar(value="mold")
        tk.Radiobutton(left_frame, text="Mold STL (has cavity)",
                       variable=self.stl_mode_var, value="mold",
                       command=self._toggle_stl_mode).grid(
            row=row, column=0, columnspan=2, sticky="w", **pad)
        row += 1
        tk.Radiobutton(left_frame, text="Part STL (solid part → compute cavity)",
                       variable=self.stl_mode_var, value="part",
                       command=self._toggle_stl_mode).grid(
            row=row, column=0, columnspan=2, sticky="w", **pad)

        # ── Wall Thickness (Part mode only) ──────────────────────────────
        row += 1
        self._wall_label = tk.Label(left_frame, text="Wall thickness (mm):")
        self._wall_label.grid(row=row, column=0, sticky="w", **pad)
        self.wall_var = tk.StringVar(value="3.0")
        self._wall_entry = tk.Entry(left_frame, textvariable=self.wall_var, width=8)
        self._wall_entry.grid(row=row, column=1, sticky="w", **pad)
        self._wall_hint = tk.Label(left_frame,
                                   text="Mold wall thickness used in slicer", fg="gray")
        self._wall_hint.grid(row=row, column=2, sticky="w", **pad)
        # Hidden by default (mold mode)
        self._wall_label.grid_remove()
        self._wall_entry.grid_remove()
        self._wall_hint.grid_remove()

        # ── STL File Selection ──────────────────────────────────────────
        row += 1
        tk.Label(left_frame, text="STL File", font=section_font).grid(
            row=row, column=0, sticky="w", **pad)
        row += 1

        self.stl_path_var = tk.StringVar(value="No file selected")
        tk.Button(left_frame, text="Browse STL File...", command=self._browse_stl).grid(
            row=row, column=0, sticky="w", **pad)
        tk.Label(left_frame, textvariable=self.stl_path_var, fg="gray",
                 wraplength=350, justify="left").grid(
            row=row, column=1, columnspan=2, sticky="w", **pad)

        # ── STL Units ────────────────────────────────────────────────────
        row += 1
        tk.Label(left_frame, text="STL Units:").grid(
            row=row, column=0, sticky="w", **pad)
        self.unit_var = tk.StringVar(value="mm")
        unit_menu = tk.OptionMenu(left_frame, self.unit_var, *UNIT_TO_MM.keys())
        unit_menu.config(width=5)
        unit_menu.grid(row=row, column=1, sticky="w", **pad)
        tk.Label(left_frame, text="Unit used in CAD export", fg="gray").grid(
            row=row, column=2, sticky="w", **pad)

        # ── Print Scale ──────────────────────────────────────────────────
        row += 1
        tk.Label(left_frame, text="Print Scale %:").grid(
            row=row, column=0, sticky="w", **pad)
        self.scale_var = tk.StringVar(value="100")
        tk.Entry(left_frame, textvariable=self.scale_var, width=8).grid(
            row=row, column=1, sticky="w", **pad)
        self.rescale_btn = tk.Button(
            left_frame, text="Recalculate", command=self._recalculate_scale,
            state="disabled")
        self.rescale_btn.grid(row=row, column=2, sticky="w", **pad)

        # ── Results ─────────────────────────────────────────────────────
        row += 1
        tk.Label(left_frame, text="Results", font=section_font).grid(
            row=row, column=0, sticky="w", **pad)

        self.result_labels = {}
        self.result_name_labels = {}  # store label widgets for renaming
        for label_key, label_text in [
            ("outer", "Outer hull volume"),
            ("solid", "Mold solid volume"),
            ("cavity", "Cavity volume"),
            ("overfill", "Overfill"),
            ("nozzle", "Nozzle dead vol"),
            ("target", "Target to dispense"),
        ]:
            row += 1
            name_label = tk.Label(left_frame, text=f"{label_text}:")
            name_label.grid(row=row, column=0, sticky="w", **pad)
            self.result_name_labels[label_key] = name_label
            var = tk.StringVar(value="--")
            tk.Label(left_frame, textvariable=var, width=20, anchor="w").grid(
                row=row, column=1, sticky="w", **pad)
            self.result_labels[label_key] = var

            # "Include" checkbox next to Overfill
            if label_key == "overfill":
                self.overfill_include_var = tk.BooleanVar(value=True)
                tk.Checkbutton(left_frame, text="Include",
                               variable=self.overfill_include_var).grid(
                    row=row, column=2, sticky="w", **pad)

            # "Include" checkbox next to Nozzle dead vol
            if label_key == "nozzle":
                self.nozzle_var = tk.BooleanVar(value=True)
                tk.Checkbutton(left_frame, text="Include",
                               variable=self.nozzle_var).grid(
                    row=row, column=2, sticky="w", **pad)

        # ── Manual Entry ────────────────────────────────────────────────
        row += 1
        tk.Label(left_frame, text="OR Manual Entry", font=section_font).grid(
            row=row, column=0, sticky="w", **pad)
        row += 1
        tk.Label(left_frame, text="Volume (mL):").grid(
            row=row, column=0, sticky="w", **pad)
        self.manual_var = tk.StringVar()
        tk.Entry(left_frame, textvariable=self.manual_var, width=12).grid(
            row=row, column=1, sticky="w", **pad)
        tk.Button(left_frame, text="Use This Volume", command=self._use_manual).grid(
            row=row, column=2, sticky="w", **pad)

        # ── Settings ───────────────────────────────────────────────────
        row += 1
        tk.Label(left_frame, text="Settings", font=section_font).grid(
            row=row, column=0, sticky="w", **pad)

        row += 1
        tk.Label(left_frame, text="COM Port:").grid(row=row, column=0, sticky="w", **pad)
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        tk.Entry(left_frame, textvariable=self.port_var, width=8).grid(
            row=row, column=1, sticky="w", **pad)

        row += 1
        tk.Label(left_frame, text="Overfill %:").grid(row=row, column=0, sticky="w", **pad)
        self.overfill_var = tk.StringVar(value=str(DEFAULT_OVERFILL))
        tk.Entry(left_frame, textvariable=self.overfill_var, width=8).grid(
            row=row, column=1, sticky="w", **pad)

        # ── Staged Dispensing ─────────────────────────────────────────
        row += 1
        tk.Label(left_frame, text="Staged Dispensing", font=section_font).grid(
            row=row, column=0, sticky="w", **pad)

        row += 1
        self.staged_var = tk.BooleanVar(value=False)
        self.staged_check = tk.Checkbutton(
            left_frame, text="Pour in intervals", variable=self.staged_var,
            command=self._toggle_staged)
        self.staged_check.grid(row=row, column=0, sticky="w", **pad)

        row += 1
        tk.Label(left_frame, text="mL per pour:").grid(row=row, column=0, sticky="w", **pad)
        self.seg_ml_var = tk.StringVar(value="25.0")
        self.seg_ml_entry = tk.Entry(left_frame, textvariable=self.seg_ml_var, width=8,
                                     state="disabled")
        self.seg_ml_entry.grid(row=row, column=1, sticky="w", **pad)

        row += 1
        tk.Label(left_frame, text="Wait between (s):").grid(row=row, column=0, sticky="w", **pad)
        self.wait_s_var = tk.StringVar(value="30")
        self.wait_s_entry = tk.Entry(left_frame, textvariable=self.wait_s_var, width=8,
                                     state="disabled")
        self.wait_s_entry.grid(row=row, column=1, sticky="w", **pad)
        self.wait_hint = tk.Label(left_frame, text="5 s \u2013 600 s (10 min)", fg="gray")
        self.wait_hint.grid(row=row, column=2, sticky="w", **pad)

        # ── Buttons (two rows so they fit on smaller screens) ────────
        row += 1
        btn_frame_top = tk.Frame(left_frame)
        btn_frame_top.grid(row=row, column=0, columnspan=3, pady=(12, 2))

        self.send_btn = tk.Button(
            btn_frame_top, text="Send to Arduino", command=self._send_to_arduino,
            bg="#4CAF50", fg="white", height=2, width=18, state="disabled")
        self.send_btn.pack(side="left", padx=6)

        self.purge_btn = tk.Button(
            btn_frame_top, text="Purge Tubing", command=self._purge,
            bg="#FF9800", fg="white", height=2, width=18)
        self.purge_btn.pack(side="left", padx=6)

        self.override_btn = tk.Button(
            btn_frame_top, text="Override Signal", command=self._override_signal,
            bg="#D32F2F", fg="white", height=2, width=18)
        self.override_btn.pack(side="left", padx=6)

        row += 1
        btn_frame_bot = tk.Frame(left_frame)
        btn_frame_bot.grid(row=row, column=0, columnspan=3, pady=(2, 12))

        self.retract_btn = tk.Button(
            btn_frame_bot, text="Retract Syringes", command=self._retract,
            bg="#9C27B0", fg="white", height=2, width=18)
        self.retract_btn.pack(side="left", padx=6)

        self.test_btn = tk.Button(
            btn_frame_bot, text="Test Connection", command=self._test_connection,
            bg="#2196F3", fg="white", height=2, width=18)
        self.test_btn.pack(side="left", padx=6)

        # ── Purge Volume ──────────────────────────────────────────────
        row += 1
        tk.Label(left_frame, text="Purge volume (mL):").grid(row=row, column=0, sticky="w", **pad)
        self.purge_ml_var = tk.StringVar(value=str(DEFAULT_PURGE_ML))
        tk.Entry(left_frame, textvariable=self.purge_ml_var, width=8).grid(
            row=row, column=1, sticky="w", **pad)
        tk.Label(left_frame, text="Flush old resin before each cycle", fg="gray").grid(
            row=row, column=2, sticky="w", **pad)

        # ── Retract Volume ────────────────────────────────────────────
        row += 1
        tk.Label(left_frame, text="Retract volume (mL):").grid(row=row, column=0, sticky="w", **pad)
        self.retract_ml_var = tk.StringVar(value="5.0")
        tk.Entry(left_frame, textvariable=self.retract_ml_var, width=8).grid(
            row=row, column=1, sticky="w", **pad)
        tk.Label(left_frame, text="Pull plungers back to relieve pressure", fg="gray").grid(
            row=row, column=2, sticky="w", **pad)

        # ── Anti-Backflow Settings ───────────────────────────────────
        row += 1
        tk.Label(left_frame, text="Anti-Backflow", font=section_font).grid(
            row=row, column=0, sticky="w", **pad)

        row += 1
        tk.Label(left_frame, text="Post-dispense dwell (s):").grid(
            row=row, column=0, sticky="w", **pad)
        self.dwell_s_var = tk.StringVar(value="3")
        tk.Entry(left_frame, textvariable=self.dwell_s_var, width=8).grid(
            row=row, column=1, sticky="w", **pad)
        tk.Label(left_frame, text="Hold motor after pour (pressure equalize)",
                 fg="gray").grid(row=row, column=2, sticky="w", **pad)

        row += 1
        tk.Label(left_frame, text="Anti-drip retract (mm):").grid(
            row=row, column=0, sticky="w", **pad)
        self.retract_mm_var = tk.StringVar(value="0.5")
        tk.Entry(left_frame, textvariable=self.retract_mm_var, width=8).grid(
            row=row, column=1, sticky="w", **pad)
        tk.Label(left_frame, text="Plunger pullback after dwell (0 = off)",
                 fg="gray").grid(row=row, column=2, sticky="w", **pad)

        row += 1
        tk.Label(left_frame, text="Max retract (% of pour):").grid(
            row=row, column=0, sticky="w", **pad)
        self.retract_pct_var = tk.StringVar(value="10")
        tk.Entry(left_frame, textvariable=self.retract_pct_var, width=8).grid(
            row=row, column=1, sticky="w", **pad)
        tk.Label(left_frame, text="Cap retract for small pours (0 = no cap)",
                 fg="gray").grid(row=row, column=2, sticky="w", **pad)

        # ── Small-Volume Compensation ────────────────────────────────
        row += 1
        tk.Label(left_frame, text="Small-Volume Fix", font=section_font).grid(
            row=row, column=0, sticky="w", **pad)

        row += 1
        tk.Label(left_frame, text="Startup offset (mL):").grid(
            row=row, column=0, sticky="w", **pad)
        self.offset_ml_var = tk.StringVar(value="0.0")
        tk.Entry(left_frame, textvariable=self.offset_ml_var, width=8).grid(
            row=row, column=1, sticky="w", **pad)
        tk.Label(left_frame, text="Extra mL to compensate backlash/compliance",
                 fg="gray").grid(row=row, column=2, sticky="w", **pad)

        # ── Status Bar (spans full window width) ──────────────────────
        self.status_var = tk.StringVar(
            value="Ready. Purge tubing first, then load STL or enter volume.")
        self.status_label = tk.Label(
            root, textvariable=self.status_var, relief="sunken",
            anchor="w", padx=6, pady=4)
        self.status_label.grid(row=1, column=0, columnspan=3, sticky="ew", padx=10, pady=(0, 10))

    # ── 3D Preview ─────────────────────────────────────────────────────

    def _setup_preview(self, parent):
        """Create a matplotlib canvas for the isometric STL preview."""
        self._fig = Figure(figsize=(3.2, 3.2), dpi=90, facecolor="#f0f0f0")
        self._ax = self._fig.add_subplot(111, projection="3d")
        self._ax.set_axis_off()
        self._ax.set_title("No STL loaded", fontsize=9, color="gray")
        self._fig.subplots_adjust(left=0, right=1, bottom=0, top=0.92)

        self._canvas = FigureCanvasTkAgg(self._fig, master=parent)
        self._canvas.get_tk_widget().pack()
        self._canvas.draw()

    def _show_stl_preview(self, mesh):
        """Render the loaded trimesh mesh as an isometric 3D view."""
        ax = self._ax
        ax.clear()
        ax.set_axis_off()

        vertices = mesh.vertices
        faces = mesh.faces

        # Build polygon collection from mesh faces
        poly = Poly3DCollection(
            vertices[faces],
            alpha=0.85,
            facecolor="#5B9BD5",
            edgecolor="#2a2a2a",
            linewidth=0.15,
        )
        ax.add_collection3d(poly)

        # Auto-scale axes to mesh bounds
        mins = vertices.min(axis=0)
        maxs = vertices.max(axis=0)
        center = (mins + maxs) / 2
        span = (maxs - mins).max() / 2 * 1.15  # slight padding
        ax.set_xlim(center[0] - span, center[0] + span)
        ax.set_ylim(center[1] - span, center[1] + span)
        ax.set_zlim(center[2] - span, center[2] + span)

        # Isometric viewing angle
        ax.view_init(elev=30, azim=-45)
        ax.set_title("Mold Preview", fontsize=9)

        self._canvas.draw()

    # ── Thermal Plot ─────────────────────────────────────────────────────

    def _setup_thermal_plot(self, parent):
        """Create a matplotlib canvas for the thermal simulation graph."""
        self._therm_fig = Figure(figsize=(3.2, 2.4), dpi=90, facecolor="#f0f0f0")
        self._therm_ax = self._therm_fig.add_subplot(111)
        self._therm_ax.set_xlabel("Time (s)", fontsize=8)
        self._therm_ax.set_ylabel("Temperature (C)", fontsize=8)
        self._therm_ax.set_title("No data yet", fontsize=9, color="gray")
        self._therm_fig.subplots_adjust(left=0.18, right=0.95, bottom=0.18, top=0.88)

        self._therm_canvas = FigureCanvasTkAgg(self._therm_fig, master=parent)
        self._therm_canvas.get_tk_widget().pack()

        self._therm_info_var = tk.StringVar(value="")
        tk.Label(parent, textvariable=self._therm_info_var, fg="#333",
                 font=("TkDefaultFont", 8), justify="left").pack(anchor="w", padx=4)
        self._therm_canvas.draw()

    def _run_thermal(self, cavity_ml, solid_ml, mesh):
        """Run thermal simulation using STL-derived values and update graph."""
        # Derive parameters from the loaded STL and volume calculations
        m_resin_g = cavity_ml * RESIN_DENSITY       # grams of resin
        m_mold_g = solid_ml * MOLD_DENSITY           # grams of mold (solid volume in mL)
        A_surface_cm2 = mesh.area / 100.0            # mesh.area is mm^2 -> cm^2

        t, Tr, Tm, peak_r, peak_m = thermal_simulate(
            m_resin_g=m_resin_g,
            A_surface_cm2=A_surface_cm2,
            m_mold_g=m_mold_g,
        )

        # Update the graph
        ax = self._therm_ax
        ax.clear()
        ax.plot(t, Tr - 273.15, label="Resin", color="#D94F4F", linewidth=1.2)
        ax.plot(t, Tm - 273.15, label="Mold", color="#5B9BD5", linewidth=1.2)
        ax.set_xlabel("Time (s)", fontsize=8)
        ax.set_ylabel("Temperature (C)", fontsize=8)
        ax.set_title("Cure Temperature vs Time", fontsize=9)
        ax.legend(fontsize=7, loc="upper right")
        ax.tick_params(labelsize=7)
        self._therm_canvas.draw()

        self._therm_info_var.set(
            f"Peak resin: {peak_r:.1f} C  |  Peak mold: {peak_m:.1f} C  |  "
            f"Resin mass: {m_resin_g:.1f} g")

    # ── Process Control ────────────────────────────────────────────────

    def _setup_process_controls(self, parent):
        """Create Stop / Resume / Reset buttons for mid-dispense control."""
        btn_row = tk.Frame(parent)
        btn_row.pack(pady=6, padx=6)

        self.stop_btn = tk.Button(
            btn_row, text="Stop", command=self._process_stop,
            bg="#F44336", fg="white", height=2, width=12)
        self.stop_btn.pack(side="left", padx=4)

        self.resume_btn = tk.Button(
            btn_row, text="Resume", command=self._process_resume,
            bg="#4CAF50", fg="white", height=2, width=12)
        self.resume_btn.pack(side="left", padx=4)

        self.reset_btn = tk.Button(
            btn_row, text="Reset", command=self._process_reset,
            bg="#FF9800", fg="white", height=2, width=12)
        self.reset_btn.pack(side="left", padx=4)

        self._proc_status_var = tk.StringVar(value="No active process")
        tk.Label(parent, textvariable=self._proc_status_var, fg="#555",
                 font=("TkDefaultFont", 8)).pack(anchor="w", padx=6, pady=(0, 4))

    def _send_simple_command(self, cmd, on_success, on_error, status_msg,
                             timeout=None):
        """Send a single command to Arduino in a background thread."""
        port = self.port_var.get().strip()
        self._set_status(status_msg)

        def do_cmd():
            if not self._serial_lock.acquire(timeout=5):
                self.root.after(0, lambda: on_error(
                    "Serial port busy — another operation is running."))
                return
            try:
                ser = self._ensure_serial(port=port, timeout=timeout)
                ser.write(f"{cmd}\n".encode())
                # Read lines until OK
                response_lines = []
                while True:
                    if self._cancel_event.is_set():
                        raise RuntimeError("Cancelled by user")
                    line = ser.readline().decode().strip()
                    if line:
                        response_lines.append(line)
                    if line == "OK":
                        break
                    if line == "":
                        raise RuntimeError("Arduino did not respond (timeout)")
                self.root.after(0, lambda: on_success(response_lines))
            except Exception as e:
                self._on_serial_error()
                self.root.after(0, lambda e=e: on_error(str(e)))
            finally:
                self._serial_lock.release()

        threading.Thread(target=do_cmd, daemon=True).start()

    # ── Process Control: Stop / Resume / Reset ─────────────────────────
    #
    # These commands must work even when another thread holds the serial
    # lock (e.g. during an Override-triggered dispense).  Strategy:
    #   STOP  — write-only; the running operation detects the pause.
    #   RESET — write + signal cancel; close serial to force recovery.
    #   RESUME — needs its own long timeout since it triggers a dispense.

    def _process_stop(self):
        """Send STOP to pause the current dispense.

        Writes directly to the serial port so it works even while an
        Override thread is reading.  The Arduino's checkForStopCommand()
        picks it up during moveSteps().
        """
        ser = self._ser
        if ser is None or not ser.is_open:
            self._set_status("No serial connection — cannot send STOP.", color="red")
            self._proc_status_var.set("Stop failed — no connection")
            return
        try:
            ser.write(b"STOP\n")
            self._proc_status_var.set("STOP sent — waiting for pause...")
            self._set_status("STOP sent to Arduino.", color="#D84315")
        except Exception as e:
            self._proc_status_var.set("Stop failed")
            self._set_status(f"Stop failed: {e}", color="red")

    def _process_resume(self):
        """Send RESUME to continue a paused dispense."""
        self._send_simple_command(
            "RESUME",
            on_success=self._on_resume_success,
            on_error=self._on_resume_error,
            status_msg="Sending RESUME to Arduino...")

    def _on_resume_success(self, lines):
        done = any("DONE" in l for l in lines)
        paused_again = any("PAUSED" in l for l in lines)
        if done:
            self._proc_status_var.set("Dispense complete")
            self._set_status("Dispense complete after resume.", color="green")
        elif paused_again:
            self._proc_status_var.set("Process PAUSED again")
            self._set_status("Process paused again. Use Resume to continue.", color="#D84315")
        else:
            self._proc_status_var.set("Resuming...")
            self._set_status("Resume sent — dispensing.", color="green")

    def _on_resume_error(self, msg):
        self._proc_status_var.set("Resume failed")
        self._set_status(f"Resume failed: {msg}", color="red")

    def _process_reset(self):
        """Send RESET and force-recover the serial connection.

        Writes RESET directly (works even if another thread holds the
        lock), then signals the running operation to cancel and closes
        the serial port so the next command gets a clean connection.
        """
        ser = self._ser
        if ser is not None and ser.is_open:
            try:
                ser.write(b"RESET\n")
            except Exception:
                pass
        # Signal any running operation to stop reading
        self._cancel_event.set()
        # Close the connection so running threads error out cleanly and
        # the next operation reconnects with a fresh handshake + RESET.
        self._close_serial()
        self._cancel_event.clear()
        self._proc_status_var.set("No active process")
        self._set_status(
            "Reset sent. Connection will refresh on next command.",
            color="green")
        self._enable_buttons()

    # ── Printer Monitor (USB Trigger) ─────────────────────────────────

    def _setup_printer_monitor(self, parent):
        """UI for monitoring the printer's serial port for a trigger keyword.

        When the printer's end G-code sends ``M118 DISPENSE_NOW``, this
        monitor catches it and automatically sends GO to the Arduino.
        Requires printing from SD card (so the printer's USB serial is free).
        """
        row1 = tk.Frame(parent)
        row1.pack(fill="x", padx=6, pady=4)

        tk.Label(row1, text="Printer Port:").pack(side="left")
        self.printer_port_var = tk.StringVar(value="COM3")
        tk.Entry(row1, textvariable=self.printer_port_var, width=8).pack(
            side="left", padx=4)

        tk.Label(row1, text="Trigger:").pack(side="left", padx=(8, 0))
        self.trigger_keyword_var = tk.StringVar(value="DISPENSE_NOW")
        tk.Entry(row1, textvariable=self.trigger_keyword_var, width=16).pack(
            side="left", padx=4)

        row2 = tk.Frame(parent)
        row2.pack(fill="x", padx=6, pady=4)

        self.monitor_btn = tk.Button(
            row2, text="Start Monitoring", command=self._toggle_monitor,
            bg="#607D8B", fg="white", height=1, width=18)
        self.monitor_btn.pack(side="left", padx=4)

        self._monitor_status_var = tk.StringVar(value="Not monitoring")
        tk.Label(parent, textvariable=self._monitor_status_var, fg="#555",
                 font=("TkDefaultFont", 8)).pack(anchor="w", padx=6, pady=(0, 4))

        tk.Label(parent, text="Print from SD card so printer USB is free",
                 fg="gray", font=("TkDefaultFont", 7)).pack(
            anchor="w", padx=6, pady=(0, 4))

    def _toggle_monitor(self):
        if self._printer_monitoring:
            self._stop_monitor()
        else:
            self._start_monitor()

    def _start_monitor(self):
        printer_port = self.printer_port_var.get().strip()
        if not printer_port:
            self._monitor_status_var.set("No printer port specified")
            return

        if self.target_ml is None:
            self._set_status(
                "Set a target volume first (Send to Arduino), then start monitoring.",
                color="red")
            return

        try:
            self._printer_ser = pyserial.Serial(printer_port, 115200, timeout=1)
        except Exception as e:
            self._monitor_status_var.set(f"Failed to open port")
            self._set_status(
                f"Cannot open printer port {printer_port}: {e}", color="red")
            return

        self._printer_monitoring = True
        self.monitor_btn.config(text="Stop Monitoring", bg="#F44336")
        self._monitor_status_var.set(
            f"Monitoring {printer_port} for trigger...")
        self._set_status(
            f"Monitoring printer on {printer_port}. Start your print from SD card.",
            color="#1565C0")

        self._monitor_thread = threading.Thread(
            target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()

    def _stop_monitor(self):
        self._printer_monitoring = False
        if self._printer_ser is not None:
            try:
                self._printer_ser.close()
            except Exception:
                pass
            self._printer_ser = None
        self.monitor_btn.config(text="Start Monitoring", bg="#607D8B")
        self._monitor_status_var.set("Not monitoring")

    def _monitor_loop(self):
        """Background thread: read printer serial, watch for trigger keyword."""
        keyword = self.trigger_keyword_var.get().strip()
        while self._printer_monitoring:
            try:
                if self._printer_ser is None or not self._printer_ser.is_open:
                    break
                line = self._printer_ser.readline().decode(
                    errors="replace").strip()
                if not line:
                    continue
                if keyword in line:
                    self.root.after(0, self._on_printer_trigger)
                    return  # exit cleanly — trigger handled
            except Exception:
                break

        # Exited unexpectedly (not via _stop_monitor or trigger)
        if self._printer_monitoring:
            self._printer_monitoring = False
            self.root.after(0, lambda: (
                self.monitor_btn.config(
                    text="Start Monitoring", bg="#607D8B"),
                self._monitor_status_var.set("Monitor disconnected"),
            ))

    def _on_printer_trigger(self):
        """Called on the main thread when the printer sends the trigger."""
        self._stop_monitor()
        self._monitor_status_var.set("TRIGGER RECEIVED — dispensing...")
        self._set_status(
            "Printer trigger received! Sending GO to Arduino...",
            color="#D84315")

        if self.target_ml is None:
            self._monitor_status_var.set("Dispense failed")
            self._set_status(
                "Trigger received but no target volume set!", color="red")
            return

        port = self.port_var.get().strip()
        target = self.target_ml
        staged = None
        try:
            staged = self._get_staged_params()
        except ValueError:
            pass
        dwell_s = self._get_dwell_s()
        retract_mm = self._get_retract_mm()
        retract_pct = self._get_retract_pct()
        offset_ml = self._get_offset_ml()

        # Send full command sequence (like Override) to ensure clean state
        def do_trigger():
            if not self._serial_lock.acquire(timeout=5):
                self.root.after(0, lambda: self._on_auto_dispense_error(
                    "Serial port busy — another operation is running."))
                return
            try:
                # Force a fresh connection so the Arduino resets cleanly
                self._close_serial()
                ser = self._ensure_serial(port=port)

                # Build command sequence: configure then GO
                if staged:
                    seg_ml, wait_s = staged
                    cmds = [
                        f"SEG {seg_ml:.2f}\n",
                        f"WAIT {wait_s}\n",
                    ]
                else:
                    cmds = [
                        "SEG 0\n",
                        "WAIT 0\n",
                    ]
                cmds += [
                    f"DWELL {dwell_s}\n",
                    f"RETMM {retract_mm:.2f}\n",
                    f"RETPCT {retract_pct:.1f}\n",
                    f"OFFSET {offset_ml:.2f}\n",
                    f"V {target:.2f}\n",
                    "GO\n",
                ]

                response_lines = []
                for cmd in cmds:
                    ser.write(cmd.encode())
                    while True:
                        if self._cancel_event.is_set():
                            raise RuntimeError("Cancelled by user")
                        line = ser.readline().decode().strip()
                        if line:
                            response_lines.append(line)
                        if line == "OK":
                            break
                        if line == "":
                            raise RuntimeError("Arduino did not respond (timeout)")

                print("[Auto-dispense serial log]")
                for l in response_lines:
                    print(f"  << {l}")

                self.root.after(0, lambda: self._on_auto_dispense_success(
                    response_lines))
            except Exception as e:
                self._on_serial_error()
                self.root.after(0, lambda e=e: self._on_auto_dispense_error(
                    str(e)))
            finally:
                self._serial_lock.release()

        threading.Thread(target=do_trigger, daemon=True).start()

    def _on_auto_dispense_success(self, lines):
        done = any("DONE" in l for l in lines)
        paused = any("PAUSED" in l for l in lines)
        aborted = any("ABORTED" in l or "Aborted" in l for l in lines)
        limit = any("LIMIT" in l for l in lines)
        # Log full Arduino response for debugging
        print("[Auto-dispense response]")
        for l in lines:
            print(f"  << {l}")
        if done:
            self._monitor_status_var.set("Dispense complete")
            self._set_status(
                "Auto-dispense complete! Print + pour cycle finished.",
                color="green")
        elif aborted or limit:
            detail = next((l for l in lines if "LIMIT" in l or "Aborted" in l), "unknown")
            self._monitor_status_var.set("Dispense ABORTED")
            self._proc_status_var.set("Process ABORTED — send Reset")
            self._set_status(
                f"Auto-dispense aborted: {detail}", color="red")
        elif paused:
            self._proc_status_var.set("Process PAUSED")
            self._monitor_status_var.set("Dispense paused")
            self._set_status(
                "Dispense paused. Use Resume to continue.", color="#D84315")
        else:
            # Fallthrough — log what we got so we can debug
            self._monitor_status_var.set("Dispense sent (no DONE received)")
            self._set_status(
                f"GO sent but no DONE received. Arduino said: {'; '.join(lines)}",
                color="#D84315")

    def _on_auto_dispense_error(self, msg):
        self._monitor_status_var.set("Dispense failed")
        self._set_status(f"Auto-dispense failed: {msg}", color="red")

    # ── Helpers ─────────────────────────────────────────────────────────

    def _set_status(self, msg, color="black"):
        self.status_var.set(msg)
        self.status_label.config(fg=color)

    def _get_overfill(self):
        try:
            return float(self.overfill_var.get()) / 100.0
        except ValueError:
            return DEFAULT_OVERFILL / 100.0

    def _get_purge_ml(self):
        try:
            val = float(self.purge_ml_var.get())
            return val if val > 0 else DEFAULT_PURGE_ML
        except ValueError:
            return DEFAULT_PURGE_ML

    def _get_dwell_s(self):
        try:
            val = int(float(self.dwell_s_var.get()))
            return max(val, 0)
        except ValueError:
            return 3

    def _get_retract_mm(self):
        try:
            val = float(self.retract_mm_var.get())
            return max(val, 0.0)
        except ValueError:
            return 0.5

    def _get_retract_pct(self):
        try:
            val = float(self.retract_pct_var.get())
            return max(val, 0.0)
        except ValueError:
            return 10.0

    def _get_offset_ml(self):
        try:
            val = float(self.offset_ml_var.get())
            return max(val, 0.0)
        except ValueError:
            return 0.0

    def _toggle_stl_mode(self):
        """Show/hide wall thickness field based on STL mode selection."""
        if self.stl_mode_var.get() == "part":
            self._wall_label.grid()
            self._wall_entry.grid()
            self._wall_hint.grid()
        else:
            self._wall_label.grid_remove()
            self._wall_entry.grid_remove()
            self._wall_hint.grid_remove()
        # Re-run calculation if an STL is already loaded
        if self._stl_file_path is not None:
            self._load_stl_with_scale(self._stl_file_path)

    def _get_wall_thickness(self):
        """Return wall thickness in mm (after unit conversion)."""
        try:
            val = float(self.wall_var.get())
            return val if val > 0 else 3.0
        except ValueError:
            return 3.0

    def _toggle_staged(self):
        state = "normal" if self.staged_var.get() else "disabled"
        self.seg_ml_entry.config(state=state)
        self.wait_s_entry.config(state=state)

    def _get_staged_params(self):
        """Return (segment_ml, wait_s) or None if staged dispensing is off.
        Raises ValueError on bad input."""
        if not self.staged_var.get():
            return None
        seg = float(self.seg_ml_var.get())
        wait = int(float(self.wait_s_var.get()))
        if seg <= 0:
            raise ValueError("mL per pour must be greater than 0.")
        if wait < 5 or wait > 600:
            raise ValueError("Wait interval must be between 5 and 600 seconds.")
        return seg, wait

    # ── Persistent serial connection ────────────────────────────────────

    def _ensure_serial(self, port=None, timeout=None):
        """Return an open serial connection, reusing the existing one if
        possible.  Only the first call pays the 2-second Arduino boot cost.
        `port` must be read on the main thread and passed in.
        timeout=None means block until data arrives (no timeout)."""
        if port is None:
            port = self._ser_port or "COM5"

        # Already open on the right port?
        if (self._ser is not None
                and self._ser.is_open
                and self._ser_port == port):
            self._ser.timeout = timeout
            self._ser.reset_input_buffer()   # flush stale data
            return self._ser

        # Close stale connection (different port or closed)
        self._close_serial()

        # Open with retry (Windows sometimes holds the port briefly)
        for attempt in range(3):
            try:
                self._ser = pyserial.Serial(port, DEFAULT_BAUD, timeout=timeout)
                self._ser_port = port
                break
            except (OSError, pyserial.SerialException):
                if attempt == 2:
                    raise
                time.sleep(1)

        # Wait for Arduino boot (only happens on fresh connection)
        time.sleep(2)
        self._ser.reset_input_buffer()

        # Clear any prior abort state
        self._ser.write(b"RESET\n")
        while True:
            line = self._ser.readline().decode().strip()
            if line == "OK":
                break
            if line == "":
                self._close_serial()
                raise RuntimeError("Arduino did not respond after boot (timeout)")

        return self._ser

    def _close_serial(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
            self._ser_port = None

    def _on_serial_error(self):
        """Call when a serial operation fails — drop the connection so the
        next operation reconnects cleanly."""
        self._close_serial()

    # ─────────────────────────────────────────────────────────────────────

    def _disable_buttons(self):
        self.send_btn.config(state="disabled")
        self.purge_btn.config(state="disabled")
        self.retract_btn.config(state="disabled")
        self.override_btn.config(state="disabled")
        self.test_btn.config(state="disabled")

    def _enable_buttons(self):
        self.purge_btn.config(state="normal")
        self.retract_btn.config(state="normal")
        self.override_btn.config(state="normal")
        self.test_btn.config(state="normal")
        if self.target_ml is not None:
            self.send_btn.config(state="normal")

    # ── STL Loading ─────────────────────────────────────────────────────

    def _update_results(self, cavity_ml, outer_ml, solid_ml):
        overfill = self._get_overfill() if self.overfill_include_var.get() else 0.0
        nozzle = NOZZLE_DEAD_ML if self.nozzle_var.get() else 0.0
        target = cavity_ml * (1 + overfill) + nozzle

        self.cavity_ml = cavity_ml
        self.outer_ml = outer_ml
        self.solid_ml = solid_ml
        self.target_ml = target

        # Update label names based on STL mode
        is_part = self.stl_mode_var.get() == "part"
        self.result_name_labels["outer"].config(
            text="Part volume (hull):" if is_part else "Outer hull volume:")
        self.result_name_labels["solid"].config(
            text="Mold plastic (est.):" if is_part else "Mold solid volume:")
        self.result_name_labels["cavity"].config(
            text="Resin to fill cavity:" if is_part else "Cavity volume:")

        self.result_labels["outer"].set(f"{outer_ml:.2f} mL")
        self.result_labels["solid"].set(f"{solid_ml:.2f} mL")
        self.result_labels["cavity"].set(f"{cavity_ml:.2f} mL")
        if self.overfill_include_var.get():
            self.result_labels["overfill"].set(
                f"{cavity_ml * overfill:.2f} mL  ({overfill*100:.0f}%)")
        else:
            self.result_labels["overfill"].set("not included")
        if self.nozzle_var.get():
            self.result_labels["nozzle"].set(f"{NOZZLE_DEAD_ML:.2f} mL  (mixer nozzle)")
        else:
            self.result_labels["nozzle"].set("not included")
        self.result_labels["target"].set(f"{target:.2f} mL")

        self.send_btn.config(state="normal")

    def _get_scale(self):
        """Return the print scale as a fraction (e.g. 0.5 for 50%)."""
        try:
            val = float(self.scale_var.get()) / 100.0
            return val if val > 0 else 1.0
        except ValueError:
            return 1.0

    def _browse_stl(self):
        path = filedialog.askopenfilename(
            title="Select Mold STL",
            filetypes=[("STL files", "*.stl"), ("All files", "*.*")])
        if not path:
            return

        self._stl_file_path = path
        self.stl_path_var.set(path)
        self._load_stl_with_scale(path)

    def _get_unit_factor(self):
        """Return the factor to convert STL units to mm."""
        return UNIT_TO_MM.get(self.unit_var.get(), 1.0)

    def _load_stl_with_scale(self, path):
        scale = self._get_scale()
        unit_factor = self._get_unit_factor()
        combined = scale * unit_factor  # convert to mm, then apply print scale
        unit = self.unit_var.get()
        mode = self.stl_mode_var.get()
        wall_mm = self._get_wall_thickness() * unit_factor  # wall in STL units → mm
        mode_label = "Part STL" if mode == "part" else "Mold STL"
        self._set_status(f"Loading {mode_label} ({unit}, {scale*100:.0f}% scale)...")
        self.send_btn.config(state="disabled")

        def compute():
            try:
                mesh = trimesh.load_mesh(path)
                if combined != 1.0:
                    mesh.apply_scale(combined)

                if mode == "part":
                    cavity_ml, outer_ml, solid_ml = compute_cavity_from_part_ml(
                        path, wall_thickness_mm=wall_mm, scale=combined)
                else:
                    cavity_ml, outer_ml, solid_ml = compute_cavity_volume_ml(
                        path, scale=combined)

                self.root.after(0, lambda: self._on_stl_success(
                    cavity_ml, outer_ml, solid_ml, mesh))
            except Exception as e:
                self.root.after(0, lambda: self._on_stl_error(str(e)))

        threading.Thread(target=compute, daemon=True).start()

    def _recalculate_scale(self):
        """Re-run volume calculation and preview with a new scale."""
        if self._stl_file_path is None:
            self._set_status("Load an STL file first.", color="red")
            return
        self._load_stl_with_scale(self._stl_file_path)

    def _on_stl_success(self, cavity_ml, outer_ml, solid_ml, mesh=None):
        self._update_results(cavity_ml, outer_ml, solid_ml)
        self.rescale_btn.config(state="normal")
        if mesh is not None:
            self._stl_mesh = mesh
            self._show_stl_preview(mesh)
            self._run_thermal(cavity_ml, solid_ml, mesh)
        scale = self._get_scale()
        unit = self.unit_var.get()
        mode = self.stl_mode_var.get()
        notes = []
        if mode == "part":
            notes.append("part mode")
        if unit != "mm":
            notes.append(f"units: {unit}")
        if scale != 1.0:
            notes.append(f"scale: {scale*100:.0f}%")
        note_str = f" ({', '.join(notes)})" if notes else ""
        nozzle_note = f" (incl. {NOZZLE_DEAD_ML:.2f} mL nozzle)" if self.nozzle_var.get() else ""
        self._set_status(
            f"STL loaded{note_str}. Cavity = {cavity_ml:.2f} mL. "
            f"Target = {self.target_ml:.2f} mL{nozzle_note}. Ready to send.",
            color="green")

    def _on_stl_error(self, msg):
        self._set_status(f"STL error: {msg}", color="red")

    # ── Manual Entry ────────────────────────────────────────────────────

    def _use_manual(self):
        raw = self.manual_var.get().strip()
        try:
            vol = float(raw)
            if vol <= 0:
                raise ValueError("must be > 0")
        except ValueError:
            self._set_status("Invalid volume. Enter a positive number (e.g. 85.5).", color="red")
            return

        self.target_ml = vol

        for key in self.result_labels:
            self.result_labels[key].set("--")
        self.result_labels["target"].set(f"{vol:.2f} mL  (manual)")

        self.stl_path_var.set("Manual entry")
        self.send_btn.config(state="normal")
        self._set_status(f"Manual volume set: {vol:.2f} mL. Ready to send.", color="green")

    # ── Send Volume to Arduino ──────────────────────────────────────────

    def _send_to_arduino(self):
        if self.target_ml is None:
            self._set_status("No target volume set. Load an STL or enter manually.", color="red")
            return

        try:
            staged = self._get_staged_params()
        except ValueError as e:
            self._set_status(str(e), color="red")
            return

        port = self.port_var.get().strip()
        target = self.target_ml
        dwell_s = self._get_dwell_s()
        retract_mm = self._get_retract_mm()
        retract_pct = self._get_retract_pct()
        offset_ml = self._get_offset_ml()

        self._disable_buttons()
        self._set_status(f"Connecting to Arduino on {port}...")

        def send():
            if not self._serial_lock.acquire(timeout=5):
                self.root.after(0, lambda: self._on_send_error(
                    "Serial port busy — another operation is running."))
                return
            try:
                ser = self._ensure_serial(port=port)

                # Clear any prior state
                ser.write(b"RESET\n")
                while True:
                    line = ser.readline().decode().strip()
                    if line == "OK":
                        break
                    if line == "":
                        raise RuntimeError("Arduino did not respond (timeout)")

                # Build command list: configure staged params then set volume
                if staged:
                    seg_ml, wait_s = staged
                    cmds = [
                        "RPM 4\n",
                        f"SEG {seg_ml:.2f}\n",
                        f"WAIT {wait_s}\n",
                        f"DWELL {dwell_s}\n",
                        f"RETMM {retract_mm:.2f}\n",
                        f"RETPCT {retract_pct:.1f}\n",
                        f"OFFSET {offset_ml:.2f}\n",
                        f"V {target:.2f}\n",
                    ]
                else:
                    cmds = [
                        "RPM 4\n",
                        "SEG 0\n",
                        "WAIT 0\n",
                        f"DWELL {dwell_s}\n",
                        f"RETMM {retract_mm:.2f}\n",
                        f"RETPCT {retract_pct:.1f}\n",
                        f"OFFSET {offset_ml:.2f}\n",
                        f"V {target:.2f}\n",
                    ]

                for cmd in cmds:
                    ser.write(cmd.encode())
                    while True:
                        line = ser.readline().decode().strip()
                        if line == "OK":
                            break
                        if line == "":
                            raise RuntimeError("Arduino did not respond (timeout)")

                self.root.after(0, lambda: self._on_send_success(target, staged))
            except Exception as e:
                self._on_serial_error()
                self.root.after(0, lambda e=e: self._on_send_error(str(e)))
            finally:
                self._serial_lock.release()

        threading.Thread(target=send, daemon=True).start()

    def _on_send_success(self, target_ml, staged):
        self._enable_buttons()
        if staged:
            seg_ml, wait_s = staged
            self._set_status(
                f"Arduino set: {target_ml:.2f} mL in {seg_ml:.1f} mL pours, "
                f"{wait_s} s apart. Start your print job.",
                color="green")
        else:
            self._set_status(
                f"Arduino confirmed: TARGET_TOTAL_ML = {target_ml:.2f} mL. "
                f"Start your print job.",
                color="green")

    def _on_send_error(self, msg):
        self._enable_buttons()
        self._set_status(f"Send failed: {msg}", color="red")

    # Test Connection

    def _test_connection(self):
        port = self.port_var.get().strip()
        self._set_status(f"Testing connection on {port}...")
        self.test_btn.config(state="disabled")

        def do_test():
            if not self._serial_lock.acquire(timeout=5):
                self.root.after(0, lambda: self._on_test_result(
                    False, "Serial port busy — another operation is running."))
                return
            try:
                ser = self._ensure_serial(port=port, timeout=5)
                ser.write(b"PING\n")
                # Read lines until we get OK (skip any status messages)
                got_ok = False
                while True:
                    line = ser.readline().decode().strip()
                    if line == "OK":
                        got_ok = True
                        break
                    if line == "":
                        break  # timeout
                if got_ok:
                    self.root.after(0, lambda: self._on_test_result(
                        True, f"Connected to Arduino on {port}."))
                else:
                    self.root.after(0, lambda: self._on_test_result(
                        False, f"No response from Arduino on {port}."))
            except Exception as e:
                self._on_serial_error()
                self.root.after(0, lambda e=e: self._on_test_result(
                    False, str(e)))
            finally:
                self._serial_lock.release()

        threading.Thread(target=do_test, daemon=True).start()

    def _on_test_result(self, success, msg):
        self.test_btn.config(state="normal")
        if success:
            self._set_status(msg, color="green")
        else:
            self._set_status(f"Connection failed: {msg}", color="red")

    # Purge

    def _purge(self):
        port = self.port_var.get().strip()
        purge_ml = self._get_purge_ml()

        self._disable_buttons()
        self._set_status(f"Purging {purge_ml:.1f} mL through tubing... (motor is running)")

        def do_purge():
            if not self._serial_lock.acquire(timeout=5):
                self.root.after(0, lambda: self._on_purge_error(
                    "Serial port busy — another operation is running."))
                return
            try:
                ser = self._ensure_serial(port=port)

                # Clear any prior abort state so purge can execute
                ser.write(b"RESET\n")
                while True:
                    line = ser.readline().decode().strip()
                    if line == "OK":
                        break
                    if line == "":
                        raise RuntimeError("Arduino did not respond (timeout)")

                ser.write(f"PURGE {purge_ml:.2f}\n".encode())

                # Read lines until "OK" — Arduino prints progress then OK
                error_msg = None
                while True:
                    if self._cancel_event.is_set():
                        raise RuntimeError("Cancelled by user")
                    line = ser.readline().decode().strip()
                    if line == "OK":
                        break
                    if line == "":
                        raise RuntimeError("Arduino did not respond (timeout)")
                    if "LIMIT" in line or "Aborted" in line:
                        error_msg = line

                if error_msg:
                    self.root.after(0, lambda m=error_msg: self._on_purge_error(m))
                else:
                    self.root.after(0, lambda: self._on_purge_success(purge_ml))
            except Exception as e:
                self._on_serial_error()
                self.root.after(0, lambda e=e: self._on_purge_error(str(e)))
            finally:
                self._serial_lock.release()

        threading.Thread(target=do_purge, daemon=True).start()

    def _on_purge_success(self, purge_ml):
        self._enable_buttons()
        self._set_status(
            f"Purge complete ({purge_ml:.1f} mL). "
            f"Tubing is clear. Ready for next cycle.",
            color="green")

    def _on_purge_error(self, msg):
        self._enable_buttons()
        self._set_status(f"Purge failed: {msg}", color="red")

    # Retract Syringes

    def _get_retract_ml(self):
        try:
            val = float(self.retract_ml_var.get())
            return val if val > 0 else 5.0
        except ValueError:
            return 5.0

    def _retract(self):
        port = self.port_var.get().strip()
        retract_ml = self._get_retract_ml()

        self._disable_buttons()
        self._set_status(f"Retracting {retract_ml:.1f} mL... (motor is running)")

        def do_retract():
            if not self._serial_lock.acquire(timeout=5):
                self.root.after(0, lambda: self._on_retract_error(
                    "Serial port busy — another operation is running."))
                return
            try:
                ser = self._ensure_serial(port=port)

                # Clear any prior abort state so retract can execute
                ser.write(b"RESET\n")
                while True:
                    line = ser.readline().decode().strip()
                    if line == "OK":
                        break
                    if line == "":
                        raise RuntimeError("Arduino did not respond (timeout)")

                ser.write(f"RETRACT {retract_ml:.2f}\n".encode())

                error_msg = None
                while True:
                    if self._cancel_event.is_set():
                        raise RuntimeError("Cancelled by user")
                    line = ser.readline().decode().strip()
                    if line == "OK":
                        break
                    if line == "":
                        raise RuntimeError("Arduino did not respond (timeout)")
                    if "LIMIT" in line or "Aborted" in line:
                        error_msg = line

                if error_msg:
                    self.root.after(0, lambda m=error_msg: self._on_retract_error(m))
                else:
                    self.root.after(0, lambda: self._on_retract_success(retract_ml))
            except Exception as e:
                self._on_serial_error()
                self.root.after(0, lambda e=e: self._on_retract_error(str(e)))
            finally:
                self._serial_lock.release()

        threading.Thread(target=do_retract, daemon=True).start()

    def _on_retract_success(self, retract_ml):
        self._enable_buttons()
        self._set_status(
            f"Retract complete ({retract_ml:.1f} mL). Syringes pulled back.",
            color="green")

    def _on_retract_error(self, msg):
        self._enable_buttons()
        self._set_status(f"Retract failed: {msg}", color="red")

    # Override Printer Signal (for testing)

    def _override_signal(self):
        """Manually trigger the dispense by sending OVERRIDE to the Arduino.
        Requires the user to type 'duck' as a safety confirmation."""
        answer = simpledialog.askstring(
            "Confirm Override",
            "This will manually trigger the dispense signal.\n\n"
            "Type 'duck' to confirm:",
            parent=self.root)

        if answer is None:
            return  # user cancelled
        if answer.strip().lower() != "duck":
            self._set_status("Override cancelled — incorrect confirmation word.", color="red")
            return

        if self.target_ml is None:
            self._set_status("No target volume set. Load an STL or enter volume first.", color="red")
            return

        port = self.port_var.get().strip()
        staged = None
        try:
            staged = self._get_staged_params()
        except ValueError:
            pass
        dwell_s = self._get_dwell_s()
        retract_mm = self._get_retract_mm()
        retract_pct = self._get_retract_pct()
        offset_ml = self._get_offset_ml()

        self._disable_buttons()
        self._set_status("Sending override signal to Arduino...")

        def do_override():
            if not self._serial_lock.acquire(timeout=5):
                self.root.after(0, lambda: self._on_override_error(
                    "Serial port busy — another operation is running."))
                return
            try:
                ser = self._ensure_serial(port=port)

                # Clear any prior abort state
                ser.write(b"RESET\n")
                while True:
                    line = ser.readline().decode().strip()
                    if line == "OK":
                        break
                    if line == "":
                        raise RuntimeError("Arduino did not respond (timeout)")

                # Send settings then trigger
                if staged:
                    seg_ml, wait_s = staged
                    cmds = [
                        f"SEG {seg_ml:.2f}\n",
                        f"WAIT {wait_s}\n",
                        f"DWELL {dwell_s}\n",
                        f"RETMM {retract_mm:.2f}\n",
                        f"RETPCT {retract_pct:.1f}\n",
                        f"OFFSET {offset_ml:.2f}\n",
                        f"V {self.target_ml:.2f}\n",
                        "OVERRIDE\n",
                    ]
                else:
                    cmds = [
                        "SEG 0\n",
                        "WAIT 0\n",
                        f"DWELL {dwell_s}\n",
                        f"RETMM {retract_mm:.2f}\n",
                        f"RETPCT {retract_pct:.1f}\n",
                        f"OFFSET {offset_ml:.2f}\n",
                        f"V {self.target_ml:.2f}\n",
                        "OVERRIDE\n",
                    ]

                error_msg = None
                paused = False
                log_lines = []
                for cmd in cmds:
                    log_lines.append(f">> {cmd.strip()}")
                    ser.write(cmd.encode())
                    while True:
                        if self._cancel_event.is_set():
                            raise RuntimeError("Cancelled by user")
                        line = ser.readline().decode().strip()
                        if line:
                            log_lines.append(f"<< {line}")
                        if line == "OK":
                            break
                        if line == "":
                            log_dump = "\n".join(log_lines)
                            raise RuntimeError(
                                f"Arduino did not respond (timeout)\n"
                                f"Serial log:\n{log_dump}")
                        if "LIMIT" in line or "Aborted" in line:
                            error_msg = line
                        if "PAUSED" in line or "STATE: PAUSED" in line:
                            paused = True

                log_dump = "\n".join(log_lines)
                print(f"[Override serial log]\n{log_dump}")

                if error_msg:
                    self.root.after(0, lambda m=error_msg: self._on_override_error(m))
                elif paused:
                    self.root.after(0, self._on_override_paused)
                else:
                    self.root.after(0, self._on_override_success)
            except Exception as e:
                self._on_serial_error()
                self.root.after(0, lambda e=e: self._on_override_error(str(e)))
            finally:
                self._serial_lock.release()

        threading.Thread(target=do_override, daemon=True).start()

    def _on_override_success(self):
        self._enable_buttons()
        self._set_status(
            "Override complete — dispense finished. Monitor the pour.",
            color="green")

    def _on_override_paused(self):
        self._enable_buttons()
        self._proc_status_var.set("Process PAUSED — Resume or Reset")
        self._set_status(
            "Dispense paused by STOP. Use Resume to continue or Reset to cancel.",
            color="#D84315")

    def _on_override_error(self, msg):
        self._enable_buttons()
        self._set_status(f"Override failed: {msg}", color="red")


def main():
    root = tk.Tk()
    app = DispenseApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (_cleanup(app), root.destroy()))
    root.mainloop()

def _cleanup(app):
    app._stop_monitor()
    app._close_serial()


if __name__ == "__main__":
    main()
