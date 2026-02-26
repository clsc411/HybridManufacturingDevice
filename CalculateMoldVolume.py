"""
CalculateMoldVolume.py
Usage: python CalculateMoldVolume.py <path_to_part.stl>

Computes resin volume from a watertight part STL (mesh volume = cavity volume),
then sends the target to the Arduino dispenser over serial.

The printer trigger on D8 fires the actual dispense — this script only sets
the volume. Run it before starting the print job.
"""

import sys
import time
import trimesh
import serial

# ── CONFIG ──────────────────────────────────────────────────────────────────
PORT      = "COM3"    # must match Arduino port (check Device Manager)
BAUD      = 115200
OVERFILL  = 0.05      # 5% extra to ensure full fill
DEAD_ML   = 6.0       # mixer + tubing dead volume (measure once and set this)
# ────────────────────────────────────────────────────────────────────────────

def compute_volume_ml(stl_path):
    mesh = trimesh.load_mesh(stl_path)
    if not mesh.is_watertight:
        raise ValueError(f"STL '{stl_path}' is not watertight. Repair it before use.")
    vol_ml = mesh.volume / 1000.0  # mm^3 -> mL
    return vol_ml

def send_volume_to_arduino(port, baud, target_ml):
    with serial.Serial(port, baud, timeout=3) as ser:
        # Arduino resets when serial opens — wait for it to print ready message
        time.sleep(2)
        ser.reset_input_buffer()  # discard startup text

        cmd = f"V {target_ml:.2f}\n"
        ser.write(cmd.encode())

        # Read response — Arduino always replies "OK" after a command
        response = ser.readline().decode().strip()
        if response != "OK":
            raise RuntimeError(f"Unexpected Arduino response: '{response}'")

    print(f"Arduino confirmed: TARGET_TOTAL_ML set to {target_ml:.2f} mL")

def main():
    if len(sys.argv) < 2:
        print("Usage: python CalculateMoldVolume.py <part.stl>")
        sys.exit(1)

    stl_path = sys.argv[1]

    vol_ml = compute_volume_ml(stl_path)
    target_ml = vol_ml * (1 + OVERFILL) + DEAD_ML

    print(f"Part mesh volume:    {vol_ml:.2f} mL")
    print(f"Overfill ({OVERFILL*100:.0f}%):      {vol_ml * OVERFILL:.2f} mL")
    print(f"Dead volume:         {DEAD_ML:.2f} mL")
    print(f"Target to dispense:  {target_ml:.2f} mL")

    send_volume_to_arduino(PORT, BAUD, target_ml)

    print()
    print("Volume set. Start your print job — the printer trigger on D8 will")
    print("fire the dispense automatically when the print finishes.")

if __name__ == "__main__":
    main()
