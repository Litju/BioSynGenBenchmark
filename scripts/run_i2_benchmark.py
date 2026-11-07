"""RES-498 multi-exercise benchmark: coverage, independent evaluation and fault injection.

    python -m scripts.run_i2_benchmark --out runs/i2/res498

For every implemented family, the generator trajectory is exported and
evaluated by the independent I2 evaluator (positive control). Fault-injected
copies (scaled force, NaN, tensile force, candidate tolerance, anatomical
labels, each omitted external support group) must be refused. Detection
rates are reported with Wilson 95 % intervals. Synthetic only.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from biosyngen.i2.bench import build_bench_press
from biosyngen.i2.deadlift import build_deadlift
from biosyngen.i2.export import export_submission
from biosyngen.i2.jumps import build_jump
from biosyngen.i2.olympic import PullProtocol, build_pull
from biosyngen.i2.power_clean import build_power_clean
from biosyngen.i2.squat import SquatProtocol, build_back_squat
from biosyngen_benchmark.i2_coverage import coverage_matrix
from biosyngen_benchmark.i2_evaluator import FAULTS, drop_group, evaluate_i2, fault_outcome
from biosyngen_benchmark.suite import wilson


def families():
    yield "CMJ", build_jump("CMJ")
    yield "SJ", build_jump("SJ")
    yield "LOADED_CMJ", build_jump("LOADED_CMJ")
    yield "LOADED_JUMP_SQUAT", build_jump("LOADED_JUMP_SQUAT")
    yield "BACK_SQUAT", build_back_squat(SquatProtocol(repetitions=1))
    yield "DEADLIFT", build_deadlift()
    yield "BENCH_PRESS", build_bench_press()
    yield "CLEAN_PULL", build_pull(PullProtocol(variant="CLEAN_PULL"))
    yield "SNATCH_PULL", build_pull(PullProtocol(variant="SNATCH_PULL"))
    yield "POWER_CLEAN_PULL_ONLY", build_power_clean()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="runs/i2/res498")
    out = Path(parser.parse_args(argv).out)
    out.mkdir(parents=True, exist_ok=True)
    results = {}
    tallies: dict[str, list[int]] = {}
    for name, scenario in families():
        submission = export_submission(scenario.run())
        positive = evaluate_i2(submission)
        case = {"positive": {k: v["outcome"] for k, v in positive["findings"].items()},
                "momentum_residual_n_s": positive["findings"]["LINEAR_MOMENTUM_CLOSURE"].get("residual_n_s"),
                "faults": {}}
        controls = [(f, fn, expected, claim) for f, (fn, expected, claim) in FAULTS.items()]
        controls += [(f"OMIT_{g}", (lambda s, g=g: drop_group(s, g)), ("REJECTED",), "LINEAR_MOMENTUM_CLOSURE")
                     for g in submission["external_support_groups"]]
        for fault, fn, expected, claim in controls:
            outcome = fault_outcome(evaluate_i2(fn(submission)), claim)
            detected = outcome in expected
            case["faults"][fault] = {"outcome": outcome, "detected": detected}
            key = fault.split("_")[0] if fault.startswith("OMIT") else fault
            tally = tallies.setdefault(key, [0, 0])
            tally[0] += int(detected)
            tally[1] += 1
        results[name] = case
    positives = sum(1 for c in results.values()
                    if c["positive"]["LINEAR_MOMENTUM_CLOSURE"] == "PASS" and c["positive"]["CONTACT_ADMISSIBILITY"] == "PASS")
    summary = {
        "issue": "RES-498",
        "claim_ceilings": ["NUMERICAL_VERIFICATION_ONLY", "SIMULATED_MEASUREMENT_ONLY",
                           "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED", "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED"],
        "positive_controls": {"passed": positives, "n": len(results), "wilson95": wilson(positives, len(results))},
        "fault_detection": {k: {"detected": d, "n": n, "wilson95": wilson(d, n)} for k, (d, n) in sorted(tallies.items())},
        "families": results,
        "coverage": coverage_matrix(),
    }
    (out / "benchmark.json").write_text(json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n",
                                        encoding="utf-8", newline="\n")
    print(json.dumps({"positive": summary["positive_controls"], "faults": summary["fault_detection"]}, indent=1, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
