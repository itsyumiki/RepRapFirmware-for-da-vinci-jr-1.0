# Installation

OctoPrint does not send end-of-job G-code by default for a streamed print.
Copy this directory's `scripts/gcode/afterPrintDone` file to the same path
inside the OctoPrint configuration directory. On a default Linux installation,
that destination is `~/.octoprint/scripts/gcode/afterPrintDone`.

The script calls the printer's `sys/stop.g`, so streamed OctoPrint jobs use
the same shutdown and parking sequence as prints completed by RepRapFirmware.
