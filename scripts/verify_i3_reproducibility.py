"""RES-504 fresh-process reproduction, with independent benchmark exit gates.

All generated outputs and worker logs stay under the project. Historical I2
outputs and the old same-process checker remain unchanged.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

MODULES = {
    "res492": "scripts.run_i2_jumps", "res493": "scripts.run_i2_back_squat",
    "res494": "scripts.run_i2_deadlift", "res495": "scripts.run_i2_bench_press",
    "res496": "scripts.run_i2_olympic_pulls", "res498": "scripts.run_i2_benchmark",
}
EXPECTED_FAULTS = {"SCALED_FORCE_1.02": 10, "NAN_FORCE": 10, "TENSILE_FOOT_FORCE": 10,
                   "CANDIDATE_TOLERANCE_FIELD": 10, "ANATOMICAL_LABEL_FIELD": 10, "OMIT": 26}


def benchmark_gate(report: dict) -> bool:
    if report.get("positive_controls", {}).get("passed") != 10 or report["positive_controls"].get("n") != 10:
        return False
    faults = report.get("fault_detection", {})
    return (set(faults) == set(EXPECTED_FAULTS)
            and all(faults[key].get("detected") == count and faults[key].get("n") == count
                    for key, count in EXPECTED_FAULTS.items()))


def inventory(directory: Path) -> dict[str, str]:
    return {p.relative_to(directory).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(directory.rglob("*")) if p.is_file()}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="runs/i3/res504/repro")
    parser.add_argument("--quick", action="store_true", help="jumps and benchmark only")
    args = parser.parse_args(argv)
    project = Path(__file__).resolve().parents[1]
    output = (project / args.out).resolve()
    if not output.is_relative_to(project) or output == project:
        parser.error("output must be a subdirectory inside the project")
    output.mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix="fresh-", dir=output))
    report = {"issue": "RES-504", "python": sys.version, "workers": [], "comparisons": {},
              "source_sha256": {p.relative_to(project).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                                for folder in ("biosyngen", "biosyngen_benchmark", "scripts", "tests")
                                for p in sorted((project / folder).glob("**/*.py"))},
              "claim_ceilings": ["NUMERICAL_VERIFICATION_ONLY", "SIMULATED_MEASUREMENT_ONLY",
                                 "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED", "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED"]}
    failed = False
    for case, module in MODULES.items():
        if args.quick and case not in {"res492", "res498"}:
            continue
        inventories = []
        for replica in ("a", "b"):
            destination = run / replica / case
            command = [sys.executable, "-m", module, "--out", str(destination)]
            result = subprocess.run(command, cwd=project, capture_output=True, text=True)
            (run / f"{case}-{replica}.txt").write_text(result.stdout + result.stderr, encoding="utf-8", newline="\n")
            report["workers"].append({"case": case, "replica": replica, "command": command, "exit": result.returncode})
            failed |= result.returncode != 0
            files = inventory(destination)
            failed |= not files
            inventories.append(files)
            if case == "res498":
                path = destination / "benchmark.json"
                gate = path.exists() and benchmark_gate(json.loads(path.read_text(encoding="utf-8")))
                report["workers"][-1]["independent_benchmark_gate"] = gate
                failed |= not gate
        equal = inventories[0] == inventories[1]
        report["comparisons"][case] = {"identical": equal, "a": inventories[0], "b": inventories[1]}
        failed |= not equal
    report["passed"] = not failed
    (run / "receipt.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({"receipt": str(run / "receipt.json"), "passed": not failed,
                      "fresh_worker_processes": len(report["workers"])}))
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
