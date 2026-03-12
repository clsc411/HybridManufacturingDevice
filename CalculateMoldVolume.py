"""
CalculateMoldVolume.py

Usage (automatic — STL provided):
    python CalculateMoldVolume.py <path_to_mold.stl>
    Computes cavity volume from the mold geometry and sends it to the Arduino.

Usage (manual — no STL):
    python CalculateMoldVolume.py
    Prompts the operator to enter the dispense volume directly.

Cavity volume method (automatic mode):
    cavity volume = convex hull volume - mold solid volume
    Works for any mold geometry — rectangular outer walls OR conformal walls.
    Sprues, gates, and vents are automatically included.

Sends the computed or entered volume to the Arduino over serial.
The printer trigger on D8 fires the actual dispense — run this before starting the print.
"""

import sys
import time
import trimesh
import serial

# ── CONFIG ──────────────────────────────────────────────────────────────────
PORT     = "COM5"    # must match Arduino port (check Device Manager)
BAUD     = 115200
OVERFILL = 0.05      # 5% extra to ensure the cavity fills completely
DEAD_ML  = 6.0       # mixer + tubing dead volume (measure once on your hardware)
# ────────────────────────────────────────────────────────────────────────────

def _load_and_repair(stl_path, scale=1.0):
    """Load an STL, apply scale, and attempt watertight repair. Returns mesh."""
    mesh = trimesh.load_mesh(stl_path)

    if scale != 1.0:
        mesh.apply_scale(scale)

    if not mesh.is_watertight:
        trimesh.repair.fill_holes(mesh)
        trimesh.repair.fix_winding(mesh)
        trimesh.repair.fix_inversion(mesh, multibody=True)

    if not mesh.is_watertight:
        raise ValueError(
            f"STL '{stl_path}' could not be made watertight.\n"
            "Open it in your CAD tool, check for open edges or non-manifold geometry,\n"
            "repair and re-export as a closed solid."
        )

    return mesh


def compute_cavity_volume_ml(mold_stl_path, scale=1.0):
    mesh = _load_and_repair(mold_stl_path, scale)

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
            "  - STL is a solid part, not a mold (use 'Part STL' mode instead)\n"
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


def compute_cavity_from_part_ml(part_stl_path, wall_thickness_mm, scale=1.0):
    """Compute mold cavity volume from a solid part STL and wall thickness.

    The cavity volume equals the part volume (the mold is a negative of the part).
    The outer mold volume is the bounding box of the part expanded by wall_thickness
    on each side — this matches what slicers like Cura generate for their mold feature.

    Returns (cavity_ml, outer_ml, solid_ml) — same tuple shape as
    compute_cavity_volume_ml so callers can use either interchangeably.
    """
    mesh = _load_and_repair(part_stl_path, scale)

    part_vol_mm3 = abs(mesh.volume)
    if part_vol_mm3 <= 0:
        raise ValueError("Part volume is zero — the STL may be empty or degenerate.")

    cavity_ml = part_vol_mm3 / 1000.0

    # Outer hull = convex hull of the part (the tight envelope around the part).
    # This is what the user sees as the part's outer volume.
    outer_vol_mm3 = mesh.convex_hull.volume
    outer_ml = outer_vol_mm3 / 1000.0

    # Mold solid volume = the printed plastic walls around the part.
    # Estimated from the bounding box + wall offset minus the part volume.
    bounds = mesh.bounds  # [[xmin,ymin,zmin], [xmax,ymax,zmax]]
    dims = bounds[1] - bounds[0]
    outer_dims = dims + 2 * wall_thickness_mm
    mold_box_mm3 = outer_dims[0] * outer_dims[1] * outer_dims[2]
    mold_solid_mm3 = mold_box_mm3 - part_vol_mm3
    solid_ml = mold_solid_mm3 / 1000.0

    return cavity_ml, outer_ml, solid_ml


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


def get_target_ml_from_stl(mold_stl_path):
    """Automatic mode: compute target volume from mold STL geometry."""
    print(f"Loading mold STL: {mold_stl_path}")
    cavity_ml, outer_ml, solid_ml = compute_cavity_volume_ml(mold_stl_path)
    target_ml = cavity_ml * (1 + OVERFILL) + DEAD_ML

    print(f"\nOuter hull volume:   {outer_ml:.2f} mL  (convex envelope of mold surface)")
    print(f"Mold solid volume:   {solid_ml:.2f} mL  (printed plastic walls)")
    print(f"Cavity volume:       {cavity_ml:.2f} mL  (space to fill with resin)")
    print(f"Overfill ({OVERFILL*100:.0f}%):      {cavity_ml * OVERFILL:.2f} mL")
    print(f"Dead volume:         {DEAD_ML:.2f} mL  (mixer + tubing)")
    print(f"Target to dispense:  {target_ml:.2f} mL")
    return target_ml


def get_target_ml_manual():
    """Manual mode: prompt operator to enter volume directly."""
    print("No STL file provided — manual volume entry mode.")
    print("(To use automatic mode, run: python CalculateMoldVolume.py <mold.stl>)\n")
    while True:
        try:
            raw = input("Enter target dispense volume in mL: ").strip()
            target_ml = float(raw)
            if target_ml <= 0:
                print("Volume must be greater than 0. Try again.")
                continue
            print(f"Target to dispense:  {target_ml:.2f} mL")
            return target_ml
        except ValueError:
            print("Invalid input — enter a number (e.g. 85.5). Try again.")


def main():
    if len(sys.argv) >= 2:
        target_ml = get_target_ml_from_stl(sys.argv[1])
    else:
        target_ml = get_target_ml_manual()

    send_volume_to_arduino(PORT, BAUD, target_ml)

    print()
    print("Volume set. Start your print job.")
    print("The printer trigger on D8 will fire the dispense when the print finishes.")


if __name__ == "__main__":
    main()
