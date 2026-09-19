; Da Vinci Jr 1.0 configuration

M550 P"Da Vinci Jr 1.0"

; On-board AC-1203D-RP buzzer on SAM4E PA2/PWMH2.
M300 C"buzzer"

; Driver 0=X, 1=Y, 2=Z, 3=E1.
M584 X0 Y1 Z2 E3

; The stock TB62269 direction inputs are high for clockwise rotation.
; Y is intentionally reversed so positive Y uses the opposite motor direction.
M569 P0 S1
M569 P1 S0
M569 P2 S1
M569 P3 S1

; Stock mechanics. X/Y/E use 3200 microsteps/revolution. Z uses 6400.
; The TB62269 microstep mode is hardware-set, so there is intentionally no M350.
M92 X80 Y80 Z5120 E96

; Recovered stock planner limits. M203 S1 uses mm/s; M566 uses mm/min.
M203 S1 X1500 Y1500 Z5 E150
M201 X9000 Y9000 Z5 E10000
M204 P3000 T3000
M566 X1200 Y1200 Z24 E300

; Recovered stock Cartesian travel limits.
M208 X-1:169 Y-5:175 Z0:155

; Require homing before ordinary axis motion and enforce the configured limits.
M564 H1 S1

; X/Z home low; Y also homes low (!!!).
M574 X1 S1 P"xstop"
M574 Y1 S1 P"ystop"
M574 Z1 S1 P"zstop"

; Verified LPC1115 peripherals only; NFC is intentionally not exposed.
; PIO1_0 = hotend NTC, PIO0_9 = heater, PIO2_5 = hotend fan, PIO1_10 = reflow fan.
; The hotend NTC is 100K, connected from the ADC node to ground with the MCU side pulled up.
; B4267/R820 is a recovered-calibration fit; R820 remains an estimate until measured.
M308 S0 P"lpc.ntc" Y"thermistor" T100000 B4267 C0 R820
M950 H0 C"lpc.heater" T0 Q250
M143 H0 S265

M950 F0 C"lpc.fan" Q250
M106 P0 C"Hotend fan" H0 T45:45
M950 F1 C"lpc.reflowfan" Q250
M106 P1 C"Reflow fan" H0 T45:45

M591 D0 P1 C"lpc.filament" S2

; Declaring heater 0 as the tool heater enables RRF's default hotend model.
; Neither verified fan is assumed to be the slicer's part-cooling fan.
M563 P0 D0 H0 F-1
T0
