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