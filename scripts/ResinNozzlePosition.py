# ResinNozzlePosition.py — Cura Post-Processing Script
#
# Auto-positions the mixing nozzle over the center of the printed mold
# for resin dispensing. Scans G-code moves filtered by Cura's ;TYPE:
# annotations to find the model geometry bounds (excluding skirt/brim),
# applies the Y offset for the mixing nozzle, and raises Z for clearance.
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

        # --- Find model bounding box by scanning G-code moves ---
        # Cura annotates each section with ;TYPE: comments. We only count
        # model geometry (walls, skin, fill) and skip skirt/brim/support,
        # which can extend far beyond the actual mold and skew the center.
        MODEL_TYPES = {"WALL-OUTER", "WALL-INNER", "SKIN", "FILL"}

        min_x = float('inf')
        max_x = float('-inf')
        min_y = float('inf')
        max_y = float('-inf')
        max_z = 0.0
        current_type = None
        current_z = 0.0

        for layer in data:
            for line in layer.split("\n"):
                stripped = line.strip()

                # Track which type of geometry we're in
                if stripped.startswith(";TYPE:"):
                    current_type = stripped[6:]
                    continue

                # Track Z position across all moves
                if re.match(r'G[01]\s', stripped):
                    z_match = re.search(r'Z([-\d.]+)', stripped)
                    if z_match:
                        current_z = float(z_match.group(1))

                # Only measure XY bounds for model geometry moves
                if current_type not in MODEL_TYPES:
                    continue

                if not re.match(r'G[01]\s', stripped):
                    continue

                x_match = re.search(r'X([-\d.]+)', stripped)
                y_match = re.search(r'Y([-\d.]+)', stripped)

                if x_match:
                    x = float(x_match.group(1))
                    min_x = min(min_x, x)
                    max_x = max(max_x, x)
                if y_match:
                    y = float(y_match.group(1))
                    min_y = min(min_y, y)
                    max_y = max(max_y, y)
                if x_match or y_match:
                    max_z = max(max_z, current_z)

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
