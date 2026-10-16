"""Compare independent platform qualification receipts, not floating-point CSV bytes."""
import argparse
import json
from pathlib import Path


def compare(left, right):
    if left["requested_count"] != 16 or right["requested_count"] != 16:
        raise ValueError("PLATFORM_CASE_DENOMINATOR_MISMATCH")
    if left["qualified_count"] != 16 or right["qualified_count"] != 16:
        raise ValueError("PLATFORM_PHYSICAL_CHECK_FAILED")
    if any(len(report["cases"]) != 16 or len({c["case_id"] for c in report["cases"]}) != 16
           for report in (left, right)):
        raise ValueError("PLATFORM_CASE_INVENTORY_MISMATCH")
    differences = []
    for a, b in zip(left["cases"], right["cases"]):
        if a["case_id"] != b["case_id"] or a["protocol"] != b["protocol"]:
            raise ValueError("PLATFORM_CASE_IDENTITY_MISMATCH")
        for field in ("source", "synchronization", "nominal_rates_hz", "dropped_sample_counts", "force_saturated_sample_count"):
            if a["source"][field] != b["source"][field]:
                raise ValueError(f"PLATFORM_SOURCE_DECISION_MISMATCH:{field}")
        for name in ("LINEAR_MOMENTUM_CLOSURE", "CONTACT_ADMISSIBILITY"):
            if a["mechanics"]["findings"].get(name, {}).get("outcome") != b["mechanics"]["findings"].get(name, {}).get("outcome"):
                raise ValueError("PLATFORM_MECHANICAL_DECISION_MISMATCH")
        ga, gb = a["geometry"], b["geometry"]
        if ga["status"] != gb["status"]:
            raise ValueError("PLATFORM_GEOMETRY_DECISION_MISMATCH")
        # 50 micrometres is half the default encoder count. Each platform must
        # separately pass the actual physical interval-containment check.
        if ga["status"] == "PASS" and abs(ga["max_error_m"] - gb["max_error_m"]) > 5e-5:
            raise ValueError("PLATFORM_GEOMETRY_ERROR_DIFFERENCE")
        ea = a["independent_reference_events"].get("findings", [])
        eb = b["independent_reference_events"].get("findings", [])
        if len(ea) != len(eb):
            raise ValueError("PLATFORM_EVENT_INVENTORY_MISMATCH")
        for x, y in zip(ea, eb):
            if x["event"] != y["event"] or x["outcome"] != y["outcome"]:
                raise ValueError("PLATFORM_EVENT_DECISION_MISMATCH")
            tx, ty = x["declared_time_s"], y["declared_time_s"]
            if (tx is None) != (ty is None):
                raise ValueError("PLATFORM_EVENT_ABSENCE_MISMATCH")
            if tx is not None:
                delta = abs(tx - ty)
                # Two maximum recorded integration intervals; no physiological
                # accuracy interpretation. Missing events never become passes.
                bound = 2 * max(a["reference_resolution_s"], b["reference_resolution_s"])
                if delta > bound:
                    raise ValueError("PLATFORM_EVENT_TIME_DIFFERENCE")
                differences.append(delta)
    return {"status": "PASS", "compared_cases": 16, "event_times_compared": len(differences),
        "max_event_time_difference_s": max(differences, default=0),
        "geometry_error_difference_limit_m": 5e-5, "scope": "NUMERICAL_VERIFICATION_ONLY"}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    paths = sorted(args.root.rglob("verification-a.json"))
    if len(paths) != 2:
        raise ValueError("EXACTLY_TWO_PLATFORM_RECEIPTS_REQUIRED")
    result = compare(*(json.loads(p.read_text()) for p in paths))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
