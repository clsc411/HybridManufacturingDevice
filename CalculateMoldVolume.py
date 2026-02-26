"""
CalculateMoldVolume.py
Usage: python CalculateMoldVolume.py <path_to_mold.stl>

Computes the cavity volume from a mold STL using:
    cavity volume = convex hull volume - mold solid volume

Works for any mold geometry — rectangular outer walls OR conformal walls that
follow the part shape. Interior cavity complexity does not matter.
Sprues, gates, and vents are automatically included in the cavity volume.

Sends the computed target volume to the Arduino over serial.
The printer trigger on D8 fires the actual dispense — run this before starting the print.
"""

import sys
import time
import trimesh
import serial

# ── CONFIG ──────────────────────────────────────────────────────────────────
PORT     = "COM3"    # must match Arduino port (check Device Manager)
BAUD     = 115200
OVERFILL = 0.05      # 5% extra to ensure the cavity fills completely
DEAD_ML  = 6.0       # mixer + tubing dead volume (measure once on your hardware)
# ────────────────────────────────────────────────────────────────────────────

def compute_cavity_volume_ml(mold_stl_path):
    mesh = trimesh.load_mesh(mold_stl_path)

    # Attempt automatic repair if the mesh isn't watertight.
    # Common causes: open-top molds, minor CAD export issues.
    if not mesh.is_watertight:
        trimesh.repair.fill_holes(mesh)
        trimesh.repair.fix_winding(mesh)
        trimesh.repair.fix_inversion(mesh, multibody=True)

    if not mesh.is_watertight:
        raise ValueError(
            f"Mold STL '{mold_stl_path}' could not be made watertight.\n"
            "Open it in your CAD tool, check for open edges or non-manifold geometry,\n"
            "repair and re-export as a closed solid."
        )

    # Convex hull volume = tight outer envelope around the mold's actual surface.
    # Works for both rectangular molds (exact) and conformal-wall molds (close
    # approximation that slightly overestimates, erring toward more resin = safe).
    # Unlike a bounding box, the convex hull hugs the mold shape rather than
    # enclosing large volumes of empty air around non-rectangular outer walls.
    outer_vol_mm3 = mesh.convex_hull.volume

    # Mold solid volume = the printed plastic the mesh encloses.
    # abs() handles the rare case of an inside-out mesh (flipped normals).
    mold_solid_mm3 = abs(mesh.volume)

    # Cavity = everything inside the outer envelope that isn't solid mold material.
    # Captures any interior shape: complex cavities, undercuts, sprues, gates, vents.
    cavity_mm3 = outer_vol_mm3 - mold_solid_mm3

    if cavity_mm3 <= 0:
        raise ValueError(
            f"Computed cavity volume is {cavity_mm3:.1f} mm³ (zero or negative).\n"
            "Likely causes:\n"
            "  - STL is a solid part, not a mold\n"
            "  - Mesh normals are inverted (try repairing in your CAD tool)"
        )

    cavity_ml = cavity_mm3 / 1000.0  # mm³ → mL

    # Sanity checks: warn if the ratio looks wrong
    fill_ratio = cavity_mm3 / outer_vol_mm3
    if fill_ratio > 0.90:
        print(f"WARNING: cavity is {fill_ratio*100:.1f}% of the outer hull volume.\n"
               "         Verify the STL is a mold with walls, not an empty shell.")
    if fill_ratio < 0.01:
        print(f"WARNING: cavity is only {fill_ratio*100:.2f}% of the outer hull volume.\n"
               "         Verify the STL is a mold and not a solid part.")

    return cavity_ml, outer_vol_mm3 / 1000.0, mold_solid_mm3 / 1000.0


def send_volume_to_arduino(port, baud, target_ml):
    with serial.Serial(port, baud, timeout=3) as ser:
        # Arduino resets when serial opens — wait for it to finish booting
        time.sleep(2)
        ser.reset_input_buffer()  # discard the startup "Resin Dispenser Ready." message

        cmd = f"V {target_ml:.2f}\n"
        ser.write(cmd.encode())

        # Arduino always replies "OK" after every command
        response = ser.readline().decode().strip()
        if response != "OK":
            raise RuntimeError(
                f"Unexpected Arduino response: '{response}'\n"
                "Check that the correct COM port is set and no other program\n"
                "(e.g. Serial Monitor) is using it."
            )

    print(f"Arduino confirmed: TARGET_TOTAL_ML set to {target_ml:.2f} mL")


def main():
    if len(sys.argv) < 2:
        print("Usage: python CalculateMoldVolume.py <mold.stl>")
        sys.exit(1)

    mold_stl_path = sys.argv[1]

    print(f"Loading mold STL: {mold_stl_path}")
    cavity_ml, bbox_ml, solid_ml = compute_cavity_volume_ml(mold_stl_path)
    target_ml = cavity_ml * (1 + OVERFILL) + DEAD_ML

    print(f"\nOuter hull volume:   {bbox_ml:.2f} mL  (convex envelope of mold surface)")
    print(f"Mold solid volume:   {solid_ml:.2f} mL  (printed plastic walls)")
    print(f"Cavity volume:       {cavity_ml:.2f} mL  (space to fill with resin)")
    print(f"Overfill ({OVERFILL*100:.0f}%):      {cavity_ml * OVERFILL:.2f} mL")
    print(f"Dead volume:         {DEAD_ML:.2f} mL  (mixer + tubing)")
    print(f"Target to dispense:  {target_ml:.2f} mL")

    send_volume_to_arduino(PORT, BAUD, target_ml)

    print()
    print("Volume set. Start your print job.")
    print("The printer trigger on D8 will fire the dispense when the print finishes.")


if __name__ == "__main__":
    main()
