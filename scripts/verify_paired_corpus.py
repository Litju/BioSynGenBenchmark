"""Post-generation independent SOURCE and protected-reference qualification.

This process never imports the generator. Geometry tests compare observations
to protected mechanical coordinates only after generation. Physical claims
use the existing independently authored evaluator and fixed tolerances.
"""

import argparse
from bisect import bisect_right
import csv
import hashlib
import json
from pathlib import Path

from biosyngen_benchmark.i2_evaluator import evaluate_i2
from biosyngen_benchmark.paired_source import verify_source
from biosyngen_benchmark.paired_events import verify_events


def verify_reference(source, truth):
    ref = json.loads((truth / "reference.json").read_text())
    times, z = ref["physics_times_s"], ref["attachment_vertical_m"]
    resolution = max(b - a for a, b in zip(times, times[1:]))
    if (truth / "mechanics_submission.json").exists():
        mechanics = evaluate_i2(json.loads((truth / "mechanics_submission.json").read_text()))
        if mechanics["status"] != "EVALUATED":
            raise ValueError(f"MECHANICS_SCHEMA:{mechanics}")
    else:
        with (truth / "support_intervals.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        impulse = sum(float(r["fz_N"]) * (float(r["end_s"]) - float(r["start_s"])) for r in rows)
        residual = impulse - ref["mass_kg"] * ref["gravity_m_s2"] * times[-1] - (
            ref["vertical_momentum_kg_m_s"][-1] - ref["vertical_momentum_kg_m_s"][0])
        mechanics = {"status": "EVALUATED", "findings": {"LINEAR_MOMENTUM_CLOSURE": {
            "outcome": "PASS" if abs(residual) < 1e-8 else "REJECTED", "residual_N_s": residual}}}
    geometry = {"status": "NOT_EVALUABLE_NO_DERIVED_SOURCE"}
    if (source / "derived/vertical.csv").exists():
        with (source / "derived/vertical.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        def height(t):
            k = max(0, min(len(times) - 2, bisect_right(times, t) - 1))
            w = (t - times[k]) / (times[k + 1] - times[k])
            return z[k] + w * (z[k + 1] - z[k])
        zero = height(ref["lpt_true_sample_times_s"][0])
        violations, max_error, max_width = 0, 0.0, 0.0
        for row in rows:
            t = ref["lpt_true_sample_times_s"][int(row["sample_index"])]
            expected = height(t) - zero
            low, high = float(row["lower_m"]), float(row["upper_m"])
            violations += not low <= expected <= high
            max_error = max(max_error, abs(float(row["displacement_m"]) - expected))
            max_width = max(max_width, high - low)
        geometry = {"status": "PASS" if not violations else "REJECTED", "samples": len(rows),
            "violations": violations, "max_error_m": max_error, "max_interval_width_m": max_width}
    transitions = []
    for i, (before, after) in enumerate(zip(ref.get("contacts", []), ref.get("contacts", [])[1:])):
        for name in after:
            if before[name] != after[name]:
                transitions.append({"contact": name, "before": before[name], "after": after[name],
                    "bracket_s": [times[i], times[i + 1]], "classification": "SAMPLED_MODEL_REFERENCE"})
    baseline = ref.get("parameters", {}).get("baseline_s", 1.2)
    # Independent kinematic baseline requirement, fixed before inspecting cases.
    quiet = [v / ref["mass_kg"] for t, v in zip(times, ref["vertical_momentum_kg_m_s"]) if 0.2 <= t <= baseline - 0.1]
    quiet_max = max(map(abs, quiet), default=float("inf"))
    outcomes = [v["outcome"] for k, v in mechanics["findings"].items() if k in
                ("LINEAR_MOMENTUM_CLOSURE", "CONTACT_ADMISSIBILITY")]
    event_checks = {"passed": True, "scope": "ANALYTICAL_GUIDE_EVENTS"}
    model_refusals = []
    if (truth / "model_events.json").exists():
        analysis = json.loads((truth / "model_events.json").read_text())["analysis"]
        event_checks = verify_events(ref, analysis)
        model_refusals = list(analysis.get("refusals", []))
        model_refusals.extend(reason for rep in analysis.get("reps", []) for reason in rep.get("refusals", []))
    return {"mechanics": mechanics, "geometry": geometry, "support_transitions": transitions,
        "termination": ref["termination"],
        "independent_reference_events": event_checks,
        "model_refusals": model_refusals,
        "reference_resolution_s": resolution, "quiet_baseline_max_com_speed_m_s": quiet_max,
        "quiet_baseline": "PASS" if quiet_max < 0.001 else "REJECTED",
        "passed": all(v == "PASS" for v in outcomes) and geometry["status"] != "REJECTED" and quiet_max < 0.001 and event_checks["passed"] and not model_refusals and ref["termination"] == "DURATION_REACHED",
        "physical_reference_scope": "SAMPLED_MODEL_SUPPORT_AND_EVENTS_ONLY_NOT_SENSOR_ANATOMY",
        "anatomical_phase_and_empirical_validity": "NOT_ESTABLISHED"}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    run = json.loads((args.root / "run.json").read_text())
    report = {"requested_count": run["requested_count"], "generation_refusals": run["refused_count"], "cases": []}
    for case in run["cases"]:
        result = {"case_id": case["case_id"], "protocol": case["protocol"]}
        if case["status"] == "REFUSED":
            result.update(passed=False, generation_refusal=case["reason"])
        else:
            source, truth = args.root / "SOURCE" / case["case_id"], args.root / "TRUTH" / case["case_id"]
            capture = json.loads((args.root / "DIAGNOSTICS" / case["case_id"] / "capture_authority.json").read_text())
            result["source"] = verify_source(source, capture)
            for line in (truth / "checksums.sha256").read_text().splitlines():
                digest, name = line.split("  ", 1)
                if hashlib.sha256((truth / name).read_bytes()).hexdigest() != digest:
                    raise ValueError("TRUTH_CHECKSUM_MISMATCH")
            result.update(verify_reference(source, truth))
        report["cases"].append(result)
    report["qualified_count"] = sum(c["passed"] for c in report["cases"])
    report["failed_or_refused_count"] = report["requested_count"] - report["qualified_count"]
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({k: v for k, v in report.items() if k != "cases"}))
    return 1 if report["failed_or_refused_count"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
