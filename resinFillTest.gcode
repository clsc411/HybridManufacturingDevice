; Resin fill test — positions nozzle over cup and triggers Arduino
; Cup height: 25mm + 5mm clearance = 30mm Z height
; No homing or printing — just position and trigger

; Raise Z first to clear the cup
G91            ; relative positioning
G1 Z40 F1000   ; raise Z 40mm (safe height above cup)
G90            ; absolute positioning

; Move extruder so mixing nozzle is over mold center (X160, Y150 - 78mm offset)
G1 X160 Y72 F3000

; Lower to 30mm above bed (25mm cup + 5mm clearance)
G1 Z30 F1000

G4 S5          ; wait 5 seconds to verify position

; Trigger Arduino via USB serial (DispenseUI monitors for this keyword)
M118 DISPENSE_NOW

; Disable steppers except Z
M84 X Y E
