G28            ; home first (safety)
G4 S5          ; wait 5 seconds so you can watch D8
M106 P0 S255   ; FAN0/KFAN2 ON
G4 S2          ; hold 2 seconds
M106 P0 S0     ; FAN0/KFAN2 OFF → falling edge triggers Arduino dispense
