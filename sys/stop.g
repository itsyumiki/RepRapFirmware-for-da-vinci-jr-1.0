; Leave all LPC-controlled thermal outputs safe.
M568 P0 A0
M106 P0 S0
M106 P1 S0
; Move Z to its maximum, then park outside the nominal 150x150mm build area.
G90
G1 Z155 F300
G1 X-1 Y-5 F1500
