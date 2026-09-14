#!/usr/bin/env python3
from __future__ import annotations

import argparse
import concurrent.futures
import os
import platform
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILD_ROOT = ROOT / "build"
SOURCE_SUFFIXES = {".c", ".cc", ".cpp"}

CPU_FLAGS = (
    "-mcpu=cortex-m4",
    "-mthumb",
    "-fno-math-errno",
    "-mfpu=fpv4-sp-d16",
    "-mfloat-abi=hard",
)
SECTION_FLAGS = ("-ffunction-sections", "-fdata-sections")


def include_flags(*paths: str) -> tuple[str, ...]:
    return tuple(flag for path in paths for flag in ("-I", str(ROOT / path)))


def c_flags(
    *,
    defines: tuple[str, ...],
    includes: tuple[str, ...],
    warnings: tuple[str, ...] = (),
    optimize: bool = True,
    fp16: bool = True,
) -> tuple[str, ...]:
    return (
        *(("-Os",) if optimize else ()),
        "-std=gnu99",
        *CPU_FLAGS,
        *(("-mfp16-format=ieee",) if fp16 else ()),
        *SECTION_FLAGS,
        "-nostdlib",
        "-Wundef",
        "-Wdouble-promotion",
        *warnings,
        "-fsingle-precision-constant",
        *(f"-D{define}" for define in defines),
        *includes,
    )


def cxx_flags(
    *,
    defines: tuple[str, ...],
    includes: tuple[str, ...],
    warnings: tuple[str, ...] = (),
    exceptions: bool = False,
    stack_usage: bool = False,
) -> tuple[str, ...]:
    return (
        "-Os",
        "-std=gnu++17",
        *CPU_FLAGS,
        "-mfp16-format=ieee",
        *SECTION_FLAGS,
        "-fno-threadsafe-statics",
        "-fno-rtti",
        "-fexceptions" if exceptions else "-fno-exceptions",
        "-nostdlib",
        "-Wundef",
        "-Wdouble-promotion",
        *warnings,
        "-fsingle-precision-constant",
        *(("-fstack-usage",) if stack_usage else ()),
        *(f"-D{define}" for define in defines),
        *includes,
    )


FREERTOS_INCLUDES = include_flags(
    "FreeRTOS/src/include",
    "FreeRTOS/src/portable/GCC/ARM_CM4F",
)
CORE_C_INCLUDES = include_flags(
    "CoreN2G/src",
    "CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include",
    "CoreN2G/src/SAM4S_4E_E70",
    "CoreN2G/src/SAM4S_4E_E70/SAM4E",
    "CoreN2G/src/SAM4S_4E_E70/asf",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers/pmc",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/utils",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor",
    "CoreN2G/src/SAM4S_4E_E70/asf/common/utils",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e/include",
    "CoreN2G/src/SAM4S_4E_E70/asf/common/services/clock",
    "CoreN2G/src/SAM4S_4E_E70/asf/common/services/sleepmgr",
    "CoreN2G/src/SAM4S_4E_E70/asf/common/services/usb",
    "CoreN2G/src/SAM4S_4E_E70/asf/common/services/usb/class/cdc",
    "CoreN2G/src/SAM4S_4E_E70/asf/common/services/usb/class/cdc/device",
    "CoreN2G/src/SAM4S_4E_E70/asf/common/services/usb/udc",
    "RRFLibraries/src",
)
SHARED_CXX_INCLUDES = include_flags("Shared/src")

CORE_CXX_INCLUDES = include_flags(
    "CoreN2G/src",
    "CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include",
    "CoreN2G/src/SAM4S_4E_E70",
    "CoreN2G/src/SAM4S_4E_E70/SAM4E",
    "CoreN2G/src/SAM4S_4E_E70/asf",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/utils",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e/include",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor",
    "CoreN2G/src/SAM4S_4E_E70/asf/common/utils",
    "CoreN2G/src/SAM4S_4E_E70/asf/common/services/usb",
    "CoreN2G/src/SAM4S_4E_E70/asf/common/services/usb/class/cdc",
    "CoreN2G/src/SAM4S_4E_E70/asf/common/services/usb/class/cdc/device",
    "CoreN2G/src/SAM4S_4E_E70/asf/common/services/usb/udc",
    "RRFLibraries/src",
    "FreeRTOS/src/include",
    "FreeRTOS/src/portable/GCC/ARM_CM4F",
)
RRF_C_INCLUDES = include_flags(
    "CANlib",
    "CoreN2G",
    "FreeRTOS",
    "CoreN2G/src",
    "CoreN2G/src/SAM4S_4E_E70",
    "CoreN2G/src/SAM4S_4E_E70/SAM4E",
    "CoreN2G/src/SAM4S_4E_E70/asf",
    "CoreN2G/src/SAM4S_4E_E70/asf/common/utils",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/utils",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e/include",
    "CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include",
    "RepRapFirmware/src",
    "RepRapFirmware/src/Networking/MQTT/MQTT_C/include",
    "RRFLibraries/src",
)
RRF_CXX_INCLUDES = include_flags(
    "CANlib",
    "CoreN2G",
    "FreeRTOS",
    "CoreN2G/src",
    "CoreN2G/src/SAM4S_4E_E70",
    "CoreN2G/src/SAM4S_4E_E70/SAM4E",
    "CoreN2G/src/SAM4S_4E_E70/asf/common/utils",
    "CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/sam4e/include",
    "CoreN2G/src/SAM4S_4E_E70/asf",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/utils",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/preprocessor",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/header_files",
    "CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers",
    "RepRapFirmware/src",
    "RepRapFirmware/src/Hardware/SAM4E",
    "RepRapFirmware/src/Networking",
    "RepRapFirmware/src/Networking/MQTT/MQTT_C/include",
    "WiFiSocketServerRTOS/src/include",
    "FreeRTOS/src/include",
    "FreeRTOS/src/portable/GCC/ARM_CM4F",
    "RRFLibraries/src",
    "Shared/src",
    "CANlib/src",
)


@dataclass(frozen=True)
class Project:
    name: str
    c_flags: tuple[str, ...] = ()
    cxx_flags: tuple[str, ...] = ()


PROJECTS = (
    Project(
        "FreeRTOS",
        c_flags=c_flags(
            defines=("__SAM4E8E__", "noexcept="),
            includes=FREERTOS_INCLUDES,
            optimize=False,
            fp16=False,
        ),
    ),
    Project(
        "RRFLibraries",
        cxx_flags=cxx_flags(
            defines=("__SAM4E8E__", "RTOS"),
            includes=FREERTOS_INCLUDES,
        ),
    ),
    Project(
        "CoreN2G",
        c_flags=c_flags(
            defines=(
                "__SAM4E8E__",
                "noexcept=",
                "SUPPORT_SDHC=1",
                "SUPPORT_USB=1",
                "RTOS",
            ),
            includes=CORE_C_INCLUDES,
            warnings=("-Werror=return-type", "-Werror=implicit"),
        ),
        cxx_flags=cxx_flags(
            defines=("__SAM4E8E__", "SUPPORT_SDHC=1", "SUPPORT_USB=1", "RTOS"),
            includes=CORE_CXX_INCLUDES,
            warnings=("-Werror=return-type", "-Wsuggest-override"),
            stack_usage=True,
        ),
    ),
    Project(
        "Shared",
        cxx_flags=cxx_flags(
            defines=(),
            includes=SHARED_CXX_INCLUDES,
        ),
    ),
    Project(
        "CANlib",
        cxx_flags=cxx_flags(
            defines=("__SAM4E8E__", "RTOS"),
            includes=include_flags(
                "RRFLibraries/src",
                "CoreN2G/src",
                "FreeRTOS/src/include",
                "FreeRTOS/src/portable/GCC/ARM_CM4F",
            ),
            warnings=("-Werror=return-type",),
        ),
    ),
)
FIRMWARE = Project(
    "RepRapFirmware",
    c_flags=c_flags(
        defines=(
            "__SAM4E8E__",
            "RTOS",
            "DA_VINCI_JR",
            "MQTTC_PAL_FILE=Networking/MQTT/mqtt_pal.h",
            "noexcept=",
        ),
        includes=RRF_C_INCLUDES,
        warnings=("-Werror=return-type", "-Werror=implicit"),
    ),
    cxx_flags=cxx_flags(
        defines=(
            "__SAM4E8E__",
            "RTOS",
            "DA_VINCI_JR",
            "MQTTC_PAL_FILE=Networking/MQTT/mqtt_pal.h",
            "_XOPEN_SOURCE",
        ),
        includes=RRF_CXX_INCLUDES,
        warnings=("-Werror=return-type", "-Wsuggest-override"),
        exceptions=True,
        stack_usage=True,
    ),
)


def run(command: list[str]) -> None:
    subprocess.run(command, check=True)


def tool(name: str) -> str:
    executable = os.environ.get("CROSS_COMPILE", "arm-none-eabi-") + name
    if shutil.which(executable) is None:
        raise RuntimeError(f"{executable} was not found in PATH")
    return executable


def sources(project: Project) -> list[Path]:
    return sorted(
        path
        for path in (ROOT / project.name / "src").rglob("*")
        if path.is_file() and path.suffix in SOURCE_SUFFIXES
    )


def compile_source(project: Project, source: Path) -> Path:
    relative = source.relative_to(ROOT / project.name)
    obj = (
        BUILD_ROOT / project.name / "obj" / relative.with_suffix(relative.suffix + ".o")
    )
    obj.parent.mkdir(parents=True, exist_ok=True)
    if source.suffix == ".c":
        compiler, flags = tool("gcc"), project.c_flags
    else:
        compiler, flags = tool("g++"), project.cxx_flags
    if not flags:
        raise RuntimeError(f"no compiler flags for {source.relative_to(ROOT)}")
    run([compiler, *flags, "-c", str(source), "-o", str(obj)])
    return obj


def compile_project(project: Project) -> list[Path]:
    project_sources = sources(project)
    print(f"Compiling {project.name}: {len(project_sources)} sources", flush=True)
    with concurrent.futures.ThreadPoolExecutor(
        max_workers=os.cpu_count() or 1
    ) as executor:
        return list(
            executor.map(
                lambda source: compile_source(project, source), project_sources
            )
        )


def archive_project(project: Project) -> Path:
    objects = compile_project(project)
    archive = BUILD_ROOT / project.name / f"lib{project.name}.a"
    run([tool("ar"), "rcs", str(archive), *map(str, objects)])
    return archive


def crc_appender() -> Path:
    variants = {
        (
            "Linux",
            "x86_64",
        ): "RepRapFirmware/Tools/CrcAppender/linux-x86_64/CrcAppender",
        (
            "Linux",
            "aarch64",
        ): "RepRapFirmware/Tools/CrcAppender/linux-aarch64/CrcAppender",
        (
            "Darwin",
            "x86_64",
        ): "RepRapFirmware/Tools/CrcAppender/macos-x86_64/CrcAppender",
        (
            "Darwin",
            "arm64",
        ): "RepRapFirmware/Tools/CrcAppender/macos-arm64/CrcAppender",
    }
    key = (platform.system(), platform.machine().lower())
    try:
        return ROOT / variants[key]
    except KeyError as error:
        raise RuntimeError(
            f"bundled CrcAppender is not supported on {key[0]} {key[1]}"
        ) from error


def build_firmware(archives: dict[str, Path]) -> Path:
    objects = compile_project(FIRMWARE)
    output_dir = BUILD_ROOT / FIRMWARE.name
    elf = output_dir / "DaVinciJrFirmware.elf"
    binary = output_dir / "DaVinciJrFirmware.bin"
    map_file = output_dir / "DaVinciJrFirmware.map"
    linker_script = ROOT / "RepRapFirmware/src/Hardware/SAM4E/sam4e8e_flash.ld"
    link_flags = [
        "--specs=nosys.specs",
        "-Os",
        "-Wl,--gc-sections",
        "-Wl,--fatal-warnings",
        "-Wl,--no-warn-rwx-segment",
        "-mcpu=cortex-m4",
        "-mfpu=fpv4-sp-d16",
        "-mfloat-abi=hard",
        f"-T{linker_script}",
        f"-Wl,-Map,{map_file}",
        "-mthumb",
        "-Wl,--cref",
        "-Wl,--check-sections",
        "-Wl,--entry=Reset_Handler",
        "-Wl,--unresolved-symbols=report-all",
        "-Wl,--warn-common",
        "-Wl,--warn-section-align",
        "-Wl,--warn-unresolved-symbols",
        "-Wl,--start-group",
    ]
    run(
        [
            tool("gcc"),
            *link_flags,
            *map(str, objects),
            *(
                str(archives[name])
                for name in ("CANlib", "Shared", "CoreN2G", "RRFLibraries", "FreeRTOS")
            ),
            "-lsupc++",
            "-Wl,--end-group",
            "-lm",
            "-o",
            str(elf),
        ]
    )
    run([tool("objcopy"), "-O", "binary", str(elf), str(binary)])
    run([str(crc_appender()), str(binary)])
    run([tool("size"), str(elf)])
    print(f"Firmware: {binary.relative_to(ROOT)}", flush=True)
    return binary


def clean() -> None:
    shutil.rmtree(BUILD_ROOT, ignore_errors=True)


def build() -> None:
    clean()
    archives = {project.name: archive_project(project) for project in PROJECTS}
    build_firmware(archives)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build the Da Vinci Jr 1.0 RepRapFirmware image"
    )
    parser.add_argument("command", choices=("build", "clean"))
    args = parser.parse_args()
    try:
        clean() if args.command == "clean" else build()
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
