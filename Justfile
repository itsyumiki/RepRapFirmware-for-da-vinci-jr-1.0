set shell := ["bash", "-eu", "-o", "pipefail", "-c"]
device := env("DEVICE", "/dev/tty.usbmodem1201")

default:
    @just --list

# Build both Da Vinci Jr 1.0 MCU firmware images with the command-line toolchain.
build:
    python3 tools/native_build.py build
    python3 tools/lpc_build.py build

# Build a standalone LPC1115 image that blinks the D18 status LED.
build-lpc-demo:
    python3 tools/lpc_build.py build-demo

# Remove all native build outputs.
clean:
    python3 tools/native_build.py clean
    python3 tools/lpc_build.py clean

flash-sam:
    bossac --port={{ device }} -e -w -v -b build/RepRapFirmware/DaVinciJrFirmware.bin

flash-lpc:
    openocd -f interface/cmsis-dap.cfg -f target/lpc11xx.cfg \
        -c "adapter speed 100" -c "program build/LpcFirmware/Lpc1115Firmware.elf verify reset exit"
