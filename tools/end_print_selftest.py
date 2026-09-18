from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def commands(path: Path) -> list[str]:
    return [
        command
        for line in path.read_text().splitlines()
        if (command := line.partition(";")[0].strip())
    ]


assert commands(ROOT / "sys" / "stop.g") == [
    "M568 P0 A0",
    "M106 P0 S0",
    "M106 P1 S0",
    "G90",
    "G1 Z155 F300",
    "G1 X-1 Y-5 F1500",
]
assert commands(ROOT / "octoprint" / "scripts" / "gcode" / "afterPrintDone") == [
    'M98 P"0:/sys/stop.g"',
]
print("End-of-print self-test passed")
