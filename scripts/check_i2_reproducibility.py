"""CI check for the I2 evidence scripts (RES-498).

    python -m scripts.check_i2_reproducibility [--quick]

Each evidence script is run twice into fresh temporary directories. The two
outputs must be byte-identical (determinism); any difference fails. Each SVG
must parse as XML and contain a title and drawn polylines. Agreement with the
committed bytes under runs/i2 is reported but does not fail the check, since
platform floating-point libraries may differ in the last digits. ``--quick``
runs only the jump and benchmark evidence.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

SCRIPTS = {
    "res492": "scripts.run_i2_jumps",
    "res493": "scripts.run_i2_back_squat",
    "res494": "scripts.run_i2_deadlift",
    "res495": "scripts.run_i2_bench_press",
    "res496": "scripts.run_i2_olympic_pulls",
    "res498": "scripts.run_i2_benchmark",
}
QUICK = ("res492", "res498")


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _svg_ok(path: Path) -> bool:
    root = ET.parse(path).getroot()
    tags = [el.tag.split("}")[-1] for el in root.iter()]
    return "title" in tags and tags.count("polyline") + tags.count("path") > 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args(argv)
    report = {}
    failed = False
    for key, module_name in SCRIPTS.items():
        if args.quick and key not in QUICK:
            continue
        module = importlib.import_module(module_name)
        with tempfile.TemporaryDirectory() as a, tempfile.TemporaryDirectory() as b:
            module.main(["--out", a])
            module.main(["--out", b])
            files = sorted(p.name for p in Path(a).iterdir())
            entry = {}
            for name in files:
                da, db = _digest(Path(a) / name), _digest(Path(b) / name)
                committed = Path("runs/i2") / key / name
                entry[name] = {
                    "deterministic": da == db,
                    "matches_committed": committed.exists() and _digest(committed) == da,
                    "svg_structure_ok": _svg_ok(Path(a) / name) if name.endswith(".svg") else None,
                }
                failed |= da != db or entry[name]["svg_structure_ok"] is False
            report[key] = entry
    print(json.dumps(report, indent=1))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
