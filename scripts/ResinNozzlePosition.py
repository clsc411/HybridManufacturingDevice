# ResinNozzlePosition.py — Cura Post-Processing Script
#
# Auto-positions the mixing nozzle over the center of the printed mold
# for resin dispensing. Reads the print bounding box from Cura's G-code
# header (;MINX, ;MAXX, etc.), applies the Y offset for the mixing nozzle,
# and raises Z for clearance.
#
# Installation:
#   Copy this file to your Cura scripts folder:
#     Windows: %APPDATA%/cura/<version>/scripts/
#   Then restart Cura. Enable via:
#     Extensions > Post Processing > Modify G-Code > Add a script
#
# Usage:
#   Put the marker comment ;RESIN_NOZZLE_POSITION in your Cura end G-code
#   where you want the positioning commands inserted. If no marker is found,
#   the commands are appended to the end of the G-code.

from ..Script import Script
import re


class ResinNozzlePosition(Script):
    """Positions the mixing nozzle over the mold center after printing."""

    def getSettingDataString(self):
        return """{
            "name": "Resin Nozzle Positioning",
            "key": "ResinNozzlePosition",
            "metadata": {},
            "version": 2,
            "settings": {
                "z_clearance": {
                    "label": "Z Clearance (mm)",
                    "description": "Height above the tallest printed layer to raise the nozzle before moving over the mold.",
                    "type": "float",
                    "default_value": 20.0
                },
                "nozzle_y_offset": {
                    "label": "Mixing Nozzle Y Offset (mm)",
                    "description": "Distance the mixing nozzle sits behind the extruder (+Y direction). The extruder moves forward by this amount so the mixing nozzle ends up centered over the print.",
                    "type": "float",
                    "default_value": 78.0
                },
                "dwell_time": {
                    "label": "Dwell Time (seconds)",
                    "description": "Pause after positioning so the operator can verify the nozzle is centered over the mold before dispensing.",
                    "type": "int",
                    "default_value": 5
                },
                "trigger_dispense": {
                    "label": "Send DISPENSE_NOW",
                    "description": "Send M118 DISPENSE_NOW after positioning to trigger the Arduino via DispenseUI. Disable for dry-run positioning tests.",
                    "type": "bool",
                    "default_value": true
                },
                "travel_speed": {
                    "label": "XY Travel Speed (mm/min)",
                    "description": "Feedrate for XY positioning moves.",
                    "type": "float",
                    "default_value": 3000.0
                },
                "z_speed": {
                    "label": "Z Travel Speed (mm/min)",
                    "description": "Feedrate for the Z lift move.",
                    "type": "float",
                    "default_value": 1000.0
                }
            }
        }"""

    def execute(self, data):
        z_clearance = self.getSettingValueByKey("z_clearance")
        y_offset = self.getSettingValueByKey("nozzle_y_offset")
        dwell_time = self.getSettingValueByKey("dwell_time")
        trigger = self.getSettingValueByKey("trigger_dispense")
        travel_speed = self.getSettingValueByKey("travel_speed")
        z_speed = self.getSettingValueByKey("z_speed")

        # --- Read print bounding box from Cura's G-code header ---
        # Cura writes ;MINX, ;MAXX, ;MINY, ;MAXY, ;MAXZ comments in the
        # header. These reflect the actual print bounds and exclude purge
        # lines, skirt moves outside the model, and start/end G-code moves.
        min_x = None
        max_x = None
        min_y = None
        max_y = None
        max_z = None

        for line in data[0].split("\n"):
            stripped = line.strip()
            if stripped.startswith(";MINX:"):
                min_x = float(stripped.split(":")[1])
            elif stripped.startswith(";MAXX:"):
                max_x = float(stripped.split(":")[1])
            elif stripped.startswith(";MINY:"):
                min_y = float(stripped.split(":")[1])
            elif stripped.startswith(";MAXY:"):
                max_y = float(stripped.split(":")[1])
            elif stripped.startswith(";MAXZ:"):
                max_z = float(stripped.split(":")[1])

        # If header bounds not found, fall back to scanning layer moves
        if any(v is None for v in [min_x, max_x, min_y, max_y, max_z]):
            min_x = float('inf')
            max_x = float('-inf')
            min_y = float('inf')
            max_y = float('-inf')
            max_z = 0.0
            in_layer = False

            for layer_index in range(len(data)):
                for line in data[layer_index].split("\n"):
                    stripped = line.strip()

                    # Only start counting after we see a ;LAYER: marker
                    # to skip purge lines in the start G-code
                    if stripped.startswith(";LAYER:"):
                        in_layer = True
                        continue

                    # Stop counting if we hit end G-code markers
                    if stripped.startswith(";END") or stripped == "G91":
                        in_layer = False

                    if not in_layer:
                        continue

                    if not re.match(r'G[01]\s', stripped):
                        continue

                    x_match = re.search(r'X([-\d.]+)', stripped)
                    y_match = re.search(r'Y([-\d.]+)', stripped)
                    z_match = re.search(r'Z([-\d.]+)', stripped)

                    if x_match:
                        min_x = min(min_x, float(x_match.group(1)))
                        max_x = max(max_x, float(x_match.group(1)))
                    if y_match:
                        min_y = min(min_y, float(y_match.group(1)))
                        max_y = max(max_y, float(y_match.group(1)))
                    if z_match:
                        max_z = max(max_z, float(z_match.group(1)))

            if min_x == float('inf') or min_y == float('inf') or max_z == 0:
                return data

        # --- Compute target positions ---
        center_x = (min_x + max_x) / 2.0
        center_y = (min_y + max_y) / 2.0
        extruder_y = center_y - y_offset
        safe_z = max_z + z_clearance

        # Build warning if Y is out of bounds
        warning = ""
        if extruder_y < 0:
            warning = (
                f";!! WARNING: Extruder Y={extruder_y:.1f}mm is off the bed!\n"
                f";!! Print center Y={center_y:.1f}mm is too close to the front edge\n"
                f";!! for a {y_offset:.0f}mm nozzle offset. Move your model back on the bed.\n"
                f";!! Clamping extruder Y to 0.\n"
            )
            extruder_y = 0.0

        # --- Build the positioning G-code block ---
        lines = [
            "",
            ";=== Resin Nozzle Auto-Positioning (computed by post-processing script) ===",
            f";Print bounds: X[{min_x:.1f} .. {max_x:.1f}]  Y[{min_y:.1f} .. {max_y:.1f}]",
            f";Print height: {max_z:.1f}mm",
            f";Print center: X={center_x:.1f}  Y={center_y:.1f}",
            f";Nozzle offset: {y_offset:.0f}mm behind extruder (+Y)",
            f";Target: extruder X={center_x:.1f} Y={extruder_y:.1f}, Z={safe_z:.1f}",
        ]

        if warning:
            lines.append(warning.rstrip())

        lines += [
            f"G1 Z{safe_z:.1f} F{z_speed:.0f} ;Safe Z: print top ({max_z:.1f}) + {z_clearance:.0f}mm clearance",
            f"G1 X{center_x:.1f} Y{extruder_y:.1f} F{travel_speed:.0f} ;Center mixing nozzle over mold",
            f"G4 S{dwell_time} ;Dwell {dwell_time}s — verify nozzle position",
        ]

        if trigger:
            lines.append("M118 DISPENSE_NOW ;Trigger Arduino via USB serial")

        lines += [
            "M84 X Y E ;Disable XYE steppers (keep Z held)",
            ";=== End Resin Nozzle Positioning ===",
            "",
        ]

        position_block = "\n".join(lines)

        # --- Insert into G-code at marker, or append to end ---
        marker = ";RESIN_NOZZLE_POSITION"
        for i in range(len(data)):
            if marker in data[i]:
                data[i] = data[i].replace(marker, position_block)
                return data

        # No marker found — append to the very end
        data[-1] += position_block
        return data
