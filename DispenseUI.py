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
from tkinter import filedialog, font as tkfont

import serial as pyserial
from CalculateMoldVolume import compute_cavity_volume_ml, send_volume_to_arduino

# ── Defaults ────────────────────────────────────────────────────────────────
DEFAULT_PORT      = "COM3"
DEFAULT_BAUD      = 115200
DEFAULT_OVERFILL  = 5.0    # percent
DEFAULT_PURGE_ML  = 10.0   # mL to push through tubing during purge


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

        pad = {"padx": 10, "pady": 4}
        section_font = tkfont.Font(weight="bold", size=10)

        # ── STL File Selection ──────────────────────────────────────────
        row = 0
        tk.Label(root, text="Mold STL File", font=section_font).grid(
            row=row, column=0, sticky="w", **pad)
        row += 1

        self.stl_path_var = tk.StringVar(value="No file selected")
        tk.Button(root, text="Browse STL File...", command=self._browse_stl).grid(
            row=row, column=0, sticky="w", **pad)
        tk.Label(root, textvariable=self.stl_path_var, fg="gray",
                 wraplength=350, justify="left").grid(
            row=row, column=1, columnspan=2, sticky="w", **pad)

        # ── Results ─────────────────────────────────────────────────────
        row += 1
        tk.Label(root, text="Results", font=section_font).grid(
            row=row, column=0, sticky="w", **pad)

        self.result_labels = {}
        for label_text in [
            "Outer hull volume",
            "Mold solid volume",
            "Cavity volume",
            "Overfill",
            "Target to dispense",
        ]:
            row += 1
            tk.Label(root, text=f"{label_text}:").grid(
                row=row, column=0, sticky="w", **pad)
            var = tk.StringVar(value="--")
            tk.Label(root, textvariable=var, width=20, anchor="w").grid(
                row=row, column=1, sticky="w", **pad)
            self.result_labels[label_text] = var

        # ── Manual Entry ────────────────────────────────────────────────
        row += 1
        tk.Label(root, text="OR Manual Entry", font=section_font).grid(
            row=row, column=0, sticky="w", **pad)
        row += 1
        tk.Label(root, text="Volume (mL):").grid(
            row=row, column=0, sticky="w", **pad)
        self.manual_var = tk.StringVar()
        tk.Entry(root, textvariable=self.manual_var, width=12).grid(
            row=row, column=1, sticky="w", **pad)
        tk.Button(root, text="Use This Volume", command=self._use_manual).grid(
            row=row, column=2, sticky="w", **pad)

        # ── Settings ───────────────────────────────────────────────────
        row += 1
        tk.Label(root, text="Settings", font=section_font).grid(
            row=row, column=0, sticky="w", **pad)

        row += 1
        tk.Label(root, text="COM Port:").grid(row=row, column=0, sticky="w", **pad)
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        tk.Entry(root, textvariable=self.port_var, width=8).grid(
            row=row, column=1, sticky="w", **pad)

        row += 1
        tk.Label(root, text="Overfill %:").grid(row=row, column=0, sticky="w", **pad)
        self.overfill_var = tk.StringVar(value=str(DEFAULT_OVERFILL))
        tk.Entry(root, textvariable=self.overfill_var, width=8).grid(
            row=row, column=1, sticky="w", **pad)

        # ── Buttons ───────────────────────────────────────────────────
        row += 1
        btn_frame = tk.Frame(root)
        btn_frame.grid(row=row, column=0, columnspan=3, pady=12)

        self.send_btn = tk.Button(
            btn_frame, text="Send to Arduino", command=self._send_to_arduino,
            bg="#4CAF50", fg="white", height=2, width=18, state="disabled")
        self.send_btn.pack(side="left", padx=6)

        self.purge_btn = tk.Button(
            btn_frame, text="Purge Tubing", command=self._purge,
            bg="#FF9800", fg="white", height=2, width=18)
        self.purge_btn.pack(side="left", padx=6)

        # ── Purge Volume ──────────────────────────────────────────────
        row += 1
        tk.Label(root, text="Purge volume (mL):").grid(row=row, column=0, sticky="w", **pad)
        self.purge_ml_var = tk.StringVar(value=str(DEFAULT_PURGE_ML))
        tk.Entry(root, textvariable=self.purge_ml_var, width=8).grid(
            row=row, column=1, sticky="w", **pad)
        tk.Label(root, text="Flush old resin before each cycle", fg="gray").grid(
            row=row, column=2, sticky="w", **pad)

        # ── Status Bar ─────────────────────────────────────────────────
        row += 1
        self.status_var = tk.StringVar(
            value="Ready. Purge tubing first, then load STL or enter volume.")
        self.status_label = tk.Label(
            root, textvariable=self.status_var, relief="sunken",
            anchor="w", padx=6, pady=4)
        self.status_label.grid(row=row, column=0, columnspan=3, sticky="ew", padx=10, pady=(0, 10))

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

    def _disable_buttons(self):
        self.send_btn.config(state="disabled")
        self.purge_btn.config(state="disabled")

    def _enable_buttons(self):
        self.purge_btn.config(state="normal")
        if self.target_ml is not None:
            self.send_btn.config(state="normal")

    # ── STL Loading ─────────────────────────────────────────────────────

    def _update_results(self, cavity_ml, outer_ml, solid_ml):
        overfill = self._get_overfill()
        target = cavity_ml * (1 + overfill)

        self.cavity_ml = cavity_ml
        self.outer_ml = outer_ml
        self.solid_ml = solid_ml
        self.target_ml = target

        self.result_labels["Outer hull volume"].set(f"{outer_ml:.2f} mL")
        self.result_labels["Mold solid volume"].set(f"{solid_ml:.2f} mL")
        self.result_labels["Cavity volume"].set(f"{cavity_ml:.2f} mL")
        self.result_labels["Overfill"].set(f"{cavity_ml * overfill:.2f} mL  ({overfill*100:.0f}%)")
        self.result_labels["Target to dispense"].set(f"{target:.2f} mL")

        self.send_btn.config(state="normal")

    def _browse_stl(self):
        path = filedialog.askopenfilename(
            title="Select Mold STL",
            filetypes=[("STL files", "*.stl"), ("All files", "*.*")])
        if not path:
            return

        self.stl_path_var.set(path)
        self._set_status("Loading STL and computing cavity volume...")
        self.send_btn.config(state="disabled")

        def compute():
            try:
                cavity_ml, outer_ml, solid_ml = compute_cavity_volume_ml(path)
                self.root.after(0, lambda: self._on_stl_success(cavity_ml, outer_ml, solid_ml))
            except Exception as e:
                self.root.after(0, lambda: self._on_stl_error(str(e)))

        threading.Thread(target=compute, daemon=True).start()

    def _on_stl_success(self, cavity_ml, outer_ml, solid_ml):
        self._update_results(cavity_ml, outer_ml, solid_ml)
        self._set_status(
            f"STL loaded. Cavity = {cavity_ml:.2f} mL. "
            f"Target = {self.target_ml:.2f} mL. Ready to send.",
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
        self.result_labels["Target to dispense"].set(f"{vol:.2f} mL  (manual)")

        self.stl_path_var.set("Manual entry")
        self.send_btn.config(state="normal")
        self._set_status(f"Manual volume set: {vol:.2f} mL. Ready to send.", color="green")

    # ── Send Volume to Arduino ──────────────────────────────────────────

    def _send_to_arduino(self):
        if self.target_ml is None:
            self._set_status("No target volume set. Load an STL or enter manually.", color="red")
            return

        port = self.port_var.get().strip()
        target = self.target_ml

        self._disable_buttons()
        self._set_status(f"Connecting to Arduino on {port}...")

        def send():
            try:
                send_volume_to_arduino(port, DEFAULT_BAUD, target)
                self.root.after(0, lambda: self._on_send_success(target))
            except Exception as e:
                self.root.after(0, lambda: self._on_send_error(str(e)))

        threading.Thread(target=send, daemon=True).start()

    def _on_send_success(self, target_ml):
        self._enable_buttons()
        self._set_status(
            f"Arduino confirmed: TARGET_TOTAL_ML = {target_ml:.2f} mL. "
            f"Start your print job.",
            color="green")

    def _on_send_error(self, msg):
        self._enable_buttons()
        self._set_status(f"Send failed: {msg}", color="red")

    # ── Purge ───────────────────────────────────────────────────────────

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
                self.root.after(0, lambda: self._on_purge_error(str(e)))

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


def main():
    root = tk.Tk()
    DispenseApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
