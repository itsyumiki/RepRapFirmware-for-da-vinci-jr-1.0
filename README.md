# RepRapFirmware for Da Vinci Jr 1.0

This repository contains the RepRapFirmware fork for the XYZPrinting Da Vinci Jr 1.0 3D printer.

For more information, [visit the main repository](https://github.com/itsyumiki/da-vinci-jr-1.0-hacking).

This repository will contain only the source code, releases and compiling instructions. The rest of the documentation will be in the main repository.

## Updating the LPC1115 firmware

After building with `just build`, place `build/LpcFirmware/Lpc1115Firmware.bin`
in `0:/firmware/Lpc1115Firmware.bin` on the printer's SD card and run
`M997 S3`. To use a different file in the firmware directory, pass its name
with `P`, for example `M997 S3 P"Lpc1115Firmware-test.bin"`.

The SAM4E enters the LPC1115 read-only memory (ROM) in-system programming (ISP)
boot loader through the onboard reset/ISP signals and programs it over their
existing UART connection. Serial Wire Debug (SWD) flashing via `just flash-lpc`
remains available as a recovery path.
