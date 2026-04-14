G28 ;Home

M420 S1;
G92 E0 ;Reset Extruder
G1 Z2.0 F3000 ;Move Z Axis up
G1 X10.1 Y20 Z0.28 F5000.0 ;Move to start position
G1 X10.1 Y200.0 Z0.28 F1500.0 E15 ;Draw the first line
G1 X10.4 Y200.0 Z0.28 F5000.0 ;Move to side a little
G1 X10.4 Y20 Z0.28 F1500.0 E30 ;Draw the second line
G92 E0 ;Reset Extruder
G1 Z2.0 F3000 ;Move Z Axis up

; end Gcode

G91   ;Relative positioning
G1 E-2 F2700  ;Retract a bit
G1 E-2 Z0.2 F2400  ;Retract and raise Z 
G1 Z3.0 ;Raise Z more
G90  ;Absolute positioning

;Shut down heaters and fan
M106 S0  ;Turn off fan
M104 S0  ;Turn off hotend
M140 S0 ;Turn off bed

;Wait 5 minutes for mold to settle
;REMOVE THIS COMMENT WHEN IN PRODUCTION
;G4 S300 ;Dwell 5 minutes

;Position mixing nozzle over mold center
G1 X145 Y85.07 F3000 ;Extruder Y - 84.93mm to align mixing nozzle

G4 S5          ; wait 5 seconds

; 7-pulse trigger pattern for Arduino
M106 P0 S255
G4 S1
M106 P0 S0
G4 P500
M106 P0 S255
G4 P500
M106 P0 S0
G4 P500
M106 P0 S255
G4 P500
M106 P0 S0
G4 P500
M106 P0 S255
G4 P500
M106 P0 S0
G4 P500
M106 P0 S255
G4 P500
M106 P0 S0
G4 P500
M106 P0 S255
G4 P500
M106 P0 S0
G4 P500
M106 P0 S255
G4 P500
M106 P0 S0


;Disable steppers except Z
M84 X Y E
