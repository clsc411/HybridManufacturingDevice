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
DEFAULT_OVERFILL  = 5.0    # percent
DEFAULT_PURGE_ML  = 10.0   # mL to push through tubing during purge
NOZZLE_DEAD_ML    = 0.374  # mixing nozzle dead volume (374.17 mm³)

# Conversion factors to millimeters (STL files may use different units)
UNIT_TO_MM = {"mm": 1.0, "cm": 10.0, "in": 25.4, "m": 1000.0}


class DispenseApp:
    def __init__(self, root):
        self.root = root
        root.title("Hybrid Casting System \u2014 Dispense Setup")
        root.resizable(False, False)

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
        left_frame = tk.Frame(root)
        left_frame.grid(row=0, column=0, sticky="n")

        right_frame = tk.Frame(root)
        right_frame.grid(row=0, column=1, sticky="n", padx=10, pady=10)

        preview_frame = tk.LabelFrame(right_frame, text="STL Preview", font=section_font)
        preview_frame.pack(side="top")
        self._setup_preview(preview_frame)

        thermal_frame = tk.LabelFrame(right_frame, text="Thermal Analysis", font=section_font)
        thermal_frame.pack(side="top", pady=(8, 0))
        self._setup_thermal_plot(thermal_frame)

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

        # ── Buttons ───────────────────────────────────────────────────
        row += 1
        btn_frame = tk.Frame(left_frame)
        btn_frame.grid(row=row, column=0, columnspan=3, pady=12)

        self.send_btn = tk.Button(
            btn_frame, text="Send to Arduino", command=self._send_to_arduino,
            bg="#4CAF50", fg="white", height=2, width=18, state="disabled")
        self.send_btn.pack(side="left", padx=6)

        self.purge_btn = tk.Button(
            btn_frame, text="Purge Tubing", command=self._purge,
            bg="#FF9800", fg="white", height=2, width=18)
        self.purge_btn.pack(side="left", padx=6)

        self.override_btn = tk.Button(
            btn_frame, text="Override Signal", command=self._override_signal,
            bg="#D32F2F", fg="white", height=2, width=18)
        self.override_btn.pack(side="left", padx=6)

        self.test_btn = tk.Button(
            btn_frame, text="Test Connection", command=self._test_connection,
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

        # ── Status Bar (spans full window width) ──────────────────────
        self.status_var = tk.StringVar(
            value="Ready. Purge tubing first, then load STL or enter volume.")
        self.status_label = tk.Label(
            root, textvariable=self.status_var, relief="sunken",
            anchor="w", padx=6, pady=4)
        self.status_label.grid(row=1, column=0, columnspan=2, sticky="ew", padx=10, pady=(0, 10))

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

    def _disable_buttons(self):
        self.send_btn.config(state="disabled")
        self.purge_btn.config(state="disabled")
        self.test_btn.config(state="disabled")

    def _enable_buttons(self):
        self.purge_btn.config(state="normal")
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

        self._disable_buttons()
        self._set_status(f"Connecting to Arduino on {port}...")

        def send():
            try:
                with pyserial.Serial(port, DEFAULT_BAUD, timeout=120) as ser:
                    time.sleep(2)
                    ser.reset_input_buffer()

                    # Build command list: configure staged params then set volume
                    if staged:
                        seg_ml, wait_s = staged
                        cmds = [
                            "RPM 8\n",
                            f"SEG {seg_ml:.2f}\n",
                            f"WAIT {wait_s}\n",
                            f"V {target:.2f}\n",
                        ]
                    else:
                        cmds = [
                            "RPM 8\n",
                            "SEG 0\n",
                            "WAIT 0\n",
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
                self.root.after(0, lambda e=e: self._on_send_error(str(e)))

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
            try:
                with pyserial.Serial(port, DEFAULT_BAUD, timeout=5) as ser:
                    time.sleep(2)
                    ser.reset_input_buffer()
                    ser.write(b"PING\n")
                    line = ser.readline().decode().strip()
                    if line == "OK":
                        self.root.after(0, lambda: self._on_test_result(
                            True, f"Connected to Arduino on {port}."))
                    else:
                        self.root.after(0, lambda: self._on_test_result(
                            False, f"Unexpected response: '{line}'"))
            except Exception as e:
                self.root.after(0, lambda e=e: self._on_test_result(
                    False, str(e)))

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
            try:
                # Purge takes time (motor moves), so use a long timeout.
                # Arduino prints status lines before final "OK".
                with pyserial.Serial(port, DEFAULT_BAUD, timeout=120) as ser:
                    time.sleep(2)
                    ser.reset_input_buffer()

                    ser.write(f"PURGE {purge_ml:.2f}\n".encode())

                    # Read lines until "OK" — Arduino prints progress then OK
                    while True:
                        line = ser.readline().decode().strip()
                        if line == "OK":
                            break
                        if line == "":
                            raise RuntimeError("Arduino did not respond (timeout)")

                self.root.after(0, lambda: self._on_purge_success(purge_ml))
            except Exception as e:
                self.root.after(0, lambda e=e: self._on_purge_error(str(e)))

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
        self._disable_buttons()
        self._set_status("Sending override signal to Arduino...")

        def do_override():
            try:
                staged = self._get_staged_params()
            except ValueError:
                staged = None

            try:
                with pyserial.Serial(port, DEFAULT_BAUD, timeout=120) as ser:
                    time.sleep(2)
                    ser.reset_input_buffer()

                    # Re-send settings (Arduino resets when serial opens)
                    if staged:
                        seg_ml, wait_s = staged
                        cmds = [
                            f"SEG {seg_ml:.2f}\n",
                            f"WAIT {wait_s}\n",
                            f"V {self.target_ml:.2f}\n",
                            "OVERRIDE\n",
                        ]
                    else:
                        cmds = [
                            "SEG 0\n",
                            "WAIT 0\n",
                            f"V {self.target_ml:.2f}\n",
                            "OVERRIDE\n",
                        ]

                    for cmd in cmds:
                        ser.write(cmd.encode())
                        while True:
                            line = ser.readline().decode().strip()
                            if line == "OK":
                                break
                            if line == "":
                                raise RuntimeError("Arduino did not respond (timeout)")
                self.root.after(0, self._on_override_success)
            except Exception as e:
                self.root.after(0, lambda e=e: self._on_override_error(str(e)))

        threading.Thread(target=do_override, daemon=True).start()

    def _on_override_success(self):
        self._enable_buttons()
        self._set_status(
            "Override sent — dispense triggered manually. Monitor the pour.",
            color="green")

    def _on_override_error(self, msg):
        self._enable_buttons()
        self._set_status(f"Override failed: {msg}", color="red")


def main():
    root = tk.Tk()
    DispenseApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
