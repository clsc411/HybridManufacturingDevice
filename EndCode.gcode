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

;The ResinNozzlePosition post-processing script replaces this marker
;with auto-computed positioning: Z clearance, XY center over mold,
;nozzle Y offset, dwell, and M118 DISPENSE_NOW trigger.
;RESIN_NOZZLE_POSITION
