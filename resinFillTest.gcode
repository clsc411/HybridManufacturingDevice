; Resin fill test — positions nozzle over cup and triggers Arduino
; Cup height: 25mm + 5mm clearance = 30mm Z height
; No homing or printing — just position and trigger

; Raise Z first to clear the cup
G91            ; relative positioning
G1 Z40 F1000   ; raise Z 40mm (safe height above cup)
G90            ; absolute positioning

; Move extruder so mixing nozzle is over mold center (X160, Y150 - 84.93mm offset)
G1 X160 Y65.07 F3000

; Lower to 30mm above bed (25mm cup + 5mm clearance)
G1 Z30 F1000

G4 S5          ; wait 5 seconds to verify position

; Double-pulse trigger pattern for Arduino
M106 P0 S255   ; pulse 1 ON
G4 S1          ; hold 1 second
M106 P0 S0     ; pulse 1 OFF
G4 S1          ; wait 1 second
M106 P0 S255   ; pulse 2 ON → Arduino triggers on this rising edge
G4 S1          ; hold 1 second
M106 P0 S0     ; OFF

; Disable steppers except Z
M84 X Y E
