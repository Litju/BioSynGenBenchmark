"""Post-execution comparison. This evaluator alone may read protected TRUTH.

No generator or product algorithms are imported. Physical references are
sampled mechanics/contact modes or the explicitly analytical guide world.
Threshold-confirmed candidate motion is not relabeled as physical onset.
"""

import argparse
from bisect import bisect_right
import csv
import json
from pathlib import Path


def compare(source, truth, product):
    reference = json.loads((truth / "reference.json").read_text())
    output = json.loads(product.read_text())
    times = reference["physics_times_s"]
    z = reference["attachment_vertical_m"] if "attachment_vertical_m" in reference else reference["bar_vertical_m"]
    report = {"world_kind": reference["world_kind"], "termination": reference["termination"],
        "reference_resolution_s": max(b - a for a, b in zip(times, times[1:])),
        "comparisons_are_post_execution": True, "generator_imports": 0,
        "geometry": {}, "candidate_events": {}, "support_reference_transitions": []}
    if (source / "derived/vertical.csv").exists():
        rows = list(csv.DictReader((source / "derived/vertical.csv").open()))
        physical = []
        for t in reference["lpt_true_sample_times_s"]:
            k = max(0, min(len(times) - 2, bisect_right(times, t) - 1))
            w = (t - times[k]) / (times[k + 1] - times[k])
            physical.append(z[k] + w * (z[k + 1] - z[k]))
        violations, error, width = 0, 0.0, 0.0
        for row, value in zip(rows, physical):
            expected = value - physical[0]
            low, high = float(row["lower_m"]), float(row["upper_m"])
            violations += not low <= expected <= high
            error = max(error, abs(float(row["displacement_m"]) - expected))
            width = max(width, high - low)
        assert len(rows) == len(physical)
        assert violations == 0, "geometry interval failed physical reference"
        report["geometry"] = {"compared_samples": len(rows), "interval_violations": violations,
            "max_absolute_vertical_reconstruction_error_m": error, "maximum_interval_width_m": width,
            "interpretation": "ASSUMED_SENSOR_BOUND_NUMERICAL_CONTAINMENT_ONLY"}
    for channel in ("force", "lpt"):
        clock = reference["clocks"][channel]
        native_to_physics = lambda t: (t - clock["offset_s"]) / (1 + clock["drift_ppm"] * 1e-6)
        candidates = output["segmentation"][channel]
        report["candidate_events"][channel] = {"authority": candidates["event_authority"],
            "numerical_state": candidates["result"]["numerical_result"]["state"],
            "events": [{"kind": e["kind"], "time_s": None if e.get("time_s") is None else native_to_physics(e["time_s"]),
                "physical_clock_bracket_s": [native_to_physics(e["bracket_start_time_s"]),
                                              native_to_physics(e["bracket_stop_time_s"])]}
                for e in candidates["result"]["numerical_result"].get("events", [])]}
    if reference["world_kind"] == "IDEAL_VERTICAL_GUIDE_TWO_MASS_APPARATUS":
        events = report["candidate_events"]["lpt"]["events"]
        report["analytical_reference_events_s"] = reference["events"]
        onsets = [e for e in events if e["kind"] == "MOTION_ONSET" and e["time_s"] is not None]
        report["confirmed_motion_minus_mathematical_start_s"] = (
            onsets[0]["time_s"] - reference["events"]["mathematical_motion_start_s"] if onsets else None)
        report["reversal_bracket_contains_exact_bottom"] = any(
            e["kind"] == "COORDINATE_REVERSAL" and e["physical_clock_bracket_s"][0] <= reference["events"]["bottom_s"] <= e["physical_clock_bracket_s"][1]
            for e in events)
    else:
        contacts = reference["contacts"]
        for k, (before, after) in enumerate(zip(contacts, contacts[1:])):
            for name in after:
                if before[name] != after[name]:
                    report["support_reference_transitions"].append({"contact": name,
                        "before": before[name], "after": after[name], "time_s": times[k + 1]})
        report["instantaneous_impulse_count"] = len(reference["impulses_N_s"])
        report["support_conformance"] = "NOT_ESTABLISHED; two observed Fz channels do not directly measure bar-floor reaction"
    report["event_phase_conformance"] = "NOT_QUALIFIED; production candidate authority retained"
    report["source_manifest_sha256_matches_reference"] = (
        output["source_manifest_sha256"] == reference["source_manifest_sha256"])
    assert report["source_manifest_sha256_matches_reference"]
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source", "truth", "product", "out"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args(argv)
    result = compare(args.source, args.truth, args.product)
    with args.out.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({"geometry": result["geometry"], "event_phase_conformance": result["event_phase_conformance"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
