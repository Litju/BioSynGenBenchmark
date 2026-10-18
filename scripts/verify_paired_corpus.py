"""Post-generation independent SOURCE and protected-reference qualification.

This process never imports the generator. Geometry tests compare observations
to protected mechanical coordinates only after generation. Physical claims
use the existing independently authored evaluator and fixed tolerances.

Evidence admission is closed and versioned (BSMB-PAIRED-TRUTH-INVENTORY-1.0):
the SOURCE profile selects the exact expected TRUTH file set, the reference
must name the exact SOURCE manifest digest it was generated with, and absent
or incomplete evidence refuses with a typed code instead of defaulting to a
pass. Only the positively identified constrained vertical guide keeps its
deliberate analytical support and event exception. Unkeyed SHA-256 detects
accidental or partial substitution; it cannot authenticate a package whose
every authority was forged together.
"""

import argparse
from bisect import bisect_right
import csv
import json
import math
import os
from pathlib import Path
import re

from biosyngen_benchmark.i2_evaluator import evaluate_i2
from biosyngen_benchmark.paired_source import (
    DERIVED_PATH, SLACK_REFUSAL, SourceRefused, contained_file, digest, finite_number, is_link,
    strict_json, verify_source)
from biosyngen_benchmark.paired_events import verify_events

REPORT_SCHEMA = "BSMB-PAIRED-QUALIFICATION-REPORT-2.0"
TRUTH_SCHEMA = "BSMB-PAIRED-TRUTH-1.0"
TRUTH_INVENTORY_CONTRACT = "BSMB-PAIRED-TRUTH-INVENTORY-1.0"
GUIDE_PROTOCOL = "guided_vertical"
GUIDE_PROFILE = "P2B-OPEN-DUAL-VFORCE-LPT"
CABLE_PROFILE = "BSMB-PAIRED-CABLE-ANGLE-1.1"
GUIDE_WORLD = "IDEAL_VERTICAL_GUIDE_TWO_MASS_APPARATUS"
GUIDE_TRUTH = frozenset({"reference.json", "support_intervals.csv"})
MODELED_TRUTH = GUIDE_TRUTH | {"model_events.json", "context_qualification.json", "mechanics_submission.json"}
GUIDE_EVENTS = ("mathematical_motion_start_s", "bottom_s", "motion_end_s")
JUMP_EVENTS = ("takeoff", "touchdown", "com_velocity_reversal")
FLOOR_EVENTS = ("bar_floor_lift_off", "bar_floor_return")
REPS = "PER_REPETITION"
REP_EVENTS = ("descent", "bottom", "model_threshold_completion")
# Closed expected sampled model-reference event claims for each modeled protocol.
EXPECTED_EVENTS = {"cmj": JUMP_EVENTS, "loaded_cmj": JUMP_EVENTS, "loaded_jump_squat": JUMP_EVENTS,
    "sj": JUMP_EVENTS[:2], "back_squat": REPS, "bench_press": REPS, "deadlift": FLOOR_EVENTS,
    "clean_pull": FLOOR_EVENTS, "snatch_pull": FLOOR_EVENTS, "power_clean_pull": FLOOR_EVENTS}
MODELED_MECHANICS = ("LINEAR_MOMENTUM_CLOSURE", "CONTACT_ADMISSIBILITY")
GUIDE_MECHANICS = ("LINEAR_MOMENTUM_CLOSURE", "UNILATERAL_SUPPORT_SIGN")
GEOMETRY_QUALIFIED = ("PASS", "NOT_APPLICABLE_CONSTRAINED_VERTICAL_GUIDE")
SLACK_GEOMETRY = "NOT_EVALUABLE_SLACK_CABLE_REFUSED"
MISSING_EVIDENCE = {"TRUTH_INVENTORY_INCOMPLETE", "TAUT_DERIVED_OBSERVATION_MISSING", "SOURCE_FILE_MISSING",
    "REQUIRED_MECHANICS_FINDING_MISSING", "MODEL_EVENTS_INCOMPLETE", "GUIDE_EVENTS_INCOMPLETE",
    "CAPTURE_AUTHORITY_MISSING"}
CASE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*\Z")
TRUTH_NAME = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.-]*\Z")


class ReferenceRefused(ValueError):
    pass


def require(condition, code):
    if not condition:
        raise ReferenceRefused(code)


def verify_truth_inventory(truth, expected):
    """Closed, checksummed, link-free flat TRUTH inventory for one protocol class."""
    checksums = truth / "checksums.sha256"
    require(truth.is_dir() and not is_link(truth) and os.path.lexists(checksums), "TRUTH_INVENTORY_INCOMPLETE")
    actual = set()
    for entry in os.scandir(truth):
        require(not is_link(entry.path) and entry.is_file(follow_symlinks=False), "TRUTH_LINK_OR_DIRECTORY_REFUSED")
        actual.add(entry.name)
    sealed = {}
    for line in checksums.read_text(encoding="utf-8").splitlines():
        value, separator, name = line.partition("  ")
        require(separator and len(value) == 64 and set(value) <= set("0123456789abcdef"), "TRUTH_CHECKSUM_LINE_MALFORMED")
        require(TRUTH_NAME.match(name) and name not in sealed and name != "checksums.sha256", "UNSAFE_TRUTH_PATH")
        sealed[name] = value
    require(expected <= set(sealed) and expected <= actual, "TRUTH_INVENTORY_INCOMPLETE")
    require(set(sealed) == expected and actual == expected | {"checksums.sha256"}, "TRUTH_INVENTORY_UNEXPECTED")
    for name, value in sealed.items():
        require(digest(truth / name) == value, "TRUTH_CHECKSUM_MISMATCH")


def truth_json(path):
    try:
        return strict_json(path.read_text(encoding="utf-8"))
    except SourceRefused as exc:
        raise ReferenceRefused(f"TRUTH_{exc}") from None


def read_csv(path):
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def numbers(rows, fields, code):
    try:
        values = [[float(row[f]) for f in fields] for row in rows]
    except (KeyError, TypeError, ValueError):
        raise ReferenceRefused(code) from None
    require(all(math.isfinite(v) for row in values for v in row), code)
    return values


def guide_mechanics(truth, ref, times):
    """Deliberate guide-only analytical support check: endpoint impulse and sign."""
    rows = numbers(read_csv(truth / "support_intervals.csv"), ("start_s", "end_s", "fz_N"), "SUPPORT_LEDGER_MALFORMED")
    require(rows, "SUPPORT_LEDGER_MALFORMED")
    impulse = sum(fz * (end - start) for start, end, fz in rows)
    residual = impulse - ref["mass_kg"] * ref["gravity_m_s2"] * times[-1] - (
        ref["vertical_momentum_kg_m_s"][-1] - ref["vertical_momentum_kg_m_s"][0])
    minimum = min(fz for _, _, fz in rows)
    # Same unilateral tolerance as the independent I2 evaluator (1e-6 W).
    sign_ok = minimum >= -1e-6 * ref["mass_kg"] * ref["gravity_m_s2"]
    return {"status": "EVALUATED", "authority": "GUIDE_ANALYTICAL_SUPPORT_LEDGER_ONLY", "findings": {
        "LINEAR_MOMENTUM_CLOSURE": {"outcome": "PASS" if abs(residual) < 1e-8 else "REJECTED",
                                    "residual_N_s": residual, "scope": "GUIDE_ENDPOINT_SCALAR_IMPULSE_ONLY"},
        "UNILATERAL_SUPPORT_SIGN": {"outcome": "PASS" if sign_ok else "REJECTED", "minimum_fz_N": minimum}}}


def guide_events(ref, times):
    events = ref.get("events")
    require(type(events) is dict and set(events) == set(GUIDE_EVENTS) and
            all(finite_number(events[k]) for k in GUIDE_EVENTS), "GUIDE_EVENTS_INCOMPLETE")
    values = [events[k] for k in GUIDE_EVENTS]
    ordered = times[0] < values[0] < values[1] < values[2] < times[-1]
    return {"passed": ordered, "scope": "ANALYTICAL_GUIDE_EVENTS", "declared_s": dict(events),
            "findings": [], "contract": "CONSTRAINED_VERTICAL_GUIDE_ANALYTICAL_MOTION_ONLY"}


def modeled_events(truth, ref, protocol):
    document = truth_json(truth / "model_events.json")
    require(type(document) is dict and type(document.get("analysis")) is dict, "MODEL_EVENTS_MALFORMED")
    analysis = document["analysis"]
    expected = EXPECTED_EVENTS[protocol]
    if expected == REPS:
        reps = analysis.get("reps")
        require(type(reps) is list and reps and all(type(r) is dict for r in reps), "MODEL_EVENTS_INCOMPLETE")
        expected = tuple(f"rep{r.get('index')}_{name}" for r in reps for name in REP_EVENTS)
    try:
        checks = verify_events(ref, analysis)
    except (KeyError, TypeError, IndexError, ValueError):
        raise ReferenceRefused("MODEL_EVENTS_MALFORMED") from None
    names = [f["event"] for f in checks["findings"]]
    require(sorted(names) == sorted(expected), "MODEL_EVENTS_INCOMPLETE")
    refusals = analysis.get("refusals", [])
    require(type(refusals) is list, "MODEL_EVENTS_MALFORMED")
    refusals = list(refusals) + [reason for rep in analysis.get("reps", []) for reason in rep.get("refusals", [])]
    return checks, refusals


def evaluate_geometry(source, manifest, ref, times, z):
    derived = manifest["derived_observation"]
    path = derived.get("path")
    # Untrusted manifest path: compared as a string before any filesystem use.
    require(path is None or path == DERIVED_PATH, "DERIVED_PATH_UNSUPPORTED")
    angle = read_csv(contained_file(source, "streams/angle.csv", "UNSAFE_SOURCE_PATH"))
    slack = any(row.get("slack") == "1" for row in angle)
    if path is None:
        require(slack and derived.get("refusal") == SLACK_REFUSAL, "TAUT_DERIVED_OBSERVATION_MISSING")
        return {"status": SLACK_GEOMETRY, "refusal": SLACK_REFUSAL}
    require(not slack and derived.get("refusal") is None, "SLACK_DERIVED_FALSE_ACCEPT")
    target = contained_file(source, DERIVED_PATH, "DERIVED_PATH_UNSUPPORTED")
    records = [r for r in manifest["source_files"] if r.get("path") == DERIVED_PATH]
    require(len(records) == 1 and records[0].get("role") == "DERIVED_OBSERVATION" and
            records[0].get("sha256") == digest(target), "DERIVED_PROVENANCE_INVALID")
    rows = read_csv(target)
    require(len(rows) >= 3 and len(rows) == len(angle), "DERIVED_SAMPLES_INSUFFICIENT")
    values = numbers(rows, ("sample_index", "displacement_m", "lower_m", "upper_m"), "DERIVED_SAMPLE_MALFORMED")
    lpt_times = ref["lpt_true_sample_times_s"]

    def height(t):
        k = max(0, min(len(times) - 2, bisect_right(times, t) - 1))
        w = (t - times[k]) / (times[k + 1] - times[k])
        return z[k] + w * (z[k + 1] - z[k])
    zero = height(lpt_times[0])
    violations, max_error, max_width = 0, 0.0, 0.0
    for index, displacement, low, high in values:
        require(index == int(index) and 0 <= int(index) < len(lpt_times), "DERIVED_SAMPLE_INDEX_INVALID")
        expected = height(lpt_times[int(index)]) - zero
        violations += not low <= expected <= high
        max_error = max(max_error, abs(displacement - expected))
        max_width = max(max_width, high - low)
    return {"status": "PASS" if not violations else "REJECTED", "samples": len(rows),
        "violations": violations, "max_error_m": max_error, "max_interval_width_m": max_width}


def verify_reference(source, truth, protocol=None):
    source, truth = Path(source), Path(truth)
    manifest = strict_json(contained_file(source, "manifest.json", "UNSAFE_SOURCE_PATH").read_text(encoding="utf-8"))
    guide = manifest.get("profile_id") == GUIDE_PROFILE
    require(guide or manifest.get("profile_id") == CABLE_PROFILE, "SOURCE_PROFILE_UNSUPPORTED")
    verify_truth_inventory(truth, GUIDE_TRUTH if guide else MODELED_TRUTH)
    ref = truth_json(truth / "reference.json")
    require(type(ref) is dict and ref.get("reference_schema_id") == TRUTH_SCHEMA, "TRUTH_SCHEMA_UNSUPPORTED")
    # Same-trial binding: TRUTH must name the exact current SOURCE manifest.
    require(ref.get("source_manifest_sha256") == digest(source / "manifest.json"), "TRUTH_SOURCE_BINDING_MISMATCH")
    require(manifest.get("trial_id") == manifest.get("acquisition_id") == manifest.get("bundle_id") == truth.name,
            "TRUTH_SOURCE_IDENTITY_MISMATCH")
    declared = ref.get("protocol")
    require(protocol is None or declared == protocol, "TRUTH_PROTOCOL_MISMATCH")
    if guide:
        require(declared == GUIDE_PROTOCOL and ref.get("world_kind") == GUIDE_WORLD, "TRUTH_PROTOCOL_MISMATCH")
    else:
        require(declared in EXPECTED_EVENTS, "TRUTH_PROTOCOL_MISMATCH")
    times, z = ref["physics_times_s"], ref["attachment_vertical_m"]
    resolution = max(b - a for a, b in zip(times, times[1:]))
    if guide:
        mechanics, required = guide_mechanics(truth, ref, times), GUIDE_MECHANICS
        geometry = {"status": "NOT_APPLICABLE_CONSTRAINED_VERTICAL_GUIDE"}
        event_checks, model_refusals = guide_events(ref, times), []
    else:
        mechanics = evaluate_i2(truth_json(truth / "mechanics_submission.json"))
        require(mechanics["status"] == "EVALUATED", "MECHANICS_SCHEMA_INVALID")
        required = MODELED_MECHANICS
        geometry = evaluate_geometry(source, manifest, ref, times, z)
        event_checks, model_refusals = modeled_events(truth, ref, declared)
    require(all(name in mechanics["findings"] for name in required), "REQUIRED_MECHANICS_FINDING_MISSING")
    outcomes = {name: mechanics["findings"][name]["outcome"] for name in required}
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
    passed = (all(v == "PASS" for v in outcomes.values()) and geometry["status"] != "REJECTED" and
              quiet_max < 0.001 and event_checks["passed"] and not model_refusals and
              ref["termination"] == "DURATION_REACHED")
    return {"mechanics": mechanics, "required_mechanics_outcomes": outcomes, "geometry": geometry,
        "support_transitions": transitions,
        "termination": ref["termination"],
        "independent_reference_events": event_checks,
        "model_refusals": model_refusals,
        "truth_inventory_contract": TRUTH_INVENTORY_CONTRACT,
        "truth_source_binding": "EXACT_SOURCE_MANIFEST_SHA256_UNKEYED",
        "reference_resolution_s": resolution, "quiet_baseline_max_com_speed_m_s": quiet_max,
        "quiet_baseline": "PASS" if quiet_max < 0.001 else "REJECTED",
        "passed": passed,
        "model_reference_status": "MODEL_REFERENCE_QUALIFIED" if passed else "MODEL_REFERENCE_REJECTED",
        "physical_reference_scope": "SAMPLED_MODEL_SUPPORT_AND_EVENTS_ONLY_NOT_SENSOR_ANATOMY",
        "anatomical_phase_and_empirical_validity": "NOT_ESTABLISHED"}


def verify_case(root, case, result):
    case_id = case["case_id"]
    source, truth = root / "SOURCE" / case_id, root / "TRUTH" / case_id
    authority = root / "DIAGNOSTICS" / case_id
    if not os.path.lexists(authority / "capture_authority.json"):
        raise ReferenceRefused("CAPTURE_AUTHORITY_MISSING")
    capture = strict_json(contained_file(authority, "capture_authority.json", "UNSAFE_DIAGNOSTIC_PATH").read_text(
        encoding="utf-8"))
    result["source"] = verify_source(source, capture)
    reference = verify_reference(source, truth, protocol=case["protocol"])
    require(case.get("source_manifest_sha256") == digest(source / "manifest.json") and
            case.get("truth_inventory_sha256") == digest(truth / "checksums.sha256"), "RUN_RECEIPT_BINDING_MISMATCH")
    result.update(reference)


def classify(result):
    """Separate denominators: no single count implies full-stream readiness."""
    source = result.get("source")
    sync = source.get("synchronization") if isinstance(source, dict) else None
    geometry = result.get("geometry", {}).get("status")
    expected = []
    # A declared scope/parameter refusal is expected; an OS write failure is not.
    if "generation_refusal" in result and not result["generation_refusal"].startswith("OS_WRITE_FAILURE:"):
        expected.append("GENERATION_REFUSED")
    if sync == "REFUSED_UNBOUNDED_SAMPLE_JITTER":
        expected.append(sync)
    if geometry == SLACK_GEOMETRY:
        expected.append(geometry)
    full = (result["passed"] and sync == "BOUNDED_CAPTURE_MAPPING" and geometry in GEOMETRY_QUALIFIED)
    result.update(synchronization=sync, expected_refusals=expected,
        source_integrity="PASS" if isinstance(source, dict) else result.get("source_integrity", "NOT_ATTEMPTED"),
        full_case_status="FULL_CASE_QUALIFIED" if full else "NOT_FULL_CASE_QUALIFIED")
    result.setdefault("model_reference_status", "MODEL_REFERENCE_REJECTED")


def summarize(run, cases):
    count = lambda predicate: sum(1 for c in cases if predicate(c))
    return {"report_schema_id": REPORT_SCHEMA, "requested_count": run["requested_count"],
        "generation_refusal_count": count(lambda c: "generation_refusal" in c),
        "source_integrity_pass_count": count(lambda c: c["source_integrity"] == "PASS"),
        "model_reference_qualified_count": count(lambda c: c["model_reference_status"] == "MODEL_REFERENCE_QUALIFIED"),
        "bounded_synchronization_count": count(lambda c: c["synchronization"] == "BOUNDED_CAPTURE_MAPPING"),
        "synchronization_refused_count": count(lambda c: c["synchronization"] == "REFUSED_UNBOUNDED_SAMPLE_JITTER"),
        "geometry_pass_count": count(lambda c: c.get("geometry", {}).get("status") == "PASS"),
        "geometry_not_applicable_guide_count": count(
            lambda c: c.get("geometry", {}).get("status") == "NOT_APPLICABLE_CONSTRAINED_VERTICAL_GUIDE"),
        "not_evaluable_count": count(lambda c: c.get("geometry", {}).get("status") == SLACK_GEOMETRY),
        "full_case_qualified_count": count(lambda c: c["full_case_status"] == "FULL_CASE_QUALIFIED"),
        "expected_refusal_count": count(lambda c: c["expected_refusals"]),
        "evidence_refused_count": count(lambda c: "evidence_refusal" in c),
        "missing_evidence_count": count(lambda c: c.get("evidence_refusal") in MISSING_EVIDENCE),
        "denominator_scope": "model-reference qualification is not synchronization, geometry or full-case "
            "readiness; each count is reported separately over requested_count"}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    run = strict_json((args.root / "run.json").read_text(encoding="utf-8"))
    ids = [c.get("case_id") for c in run["cases"]]
    if run["requested_count"] != len(ids) or len(set(ids)) != len(ids) or not all(
            type(i) is str and CASE_ID.match(i) for i in ids):
        raise ReferenceRefused("RUN_RECEIPT_CASE_INVENTORY_INVALID")
    cases = []
    for case in run["cases"]:
        result = {"case_id": case["case_id"], "protocol": case["protocol"], "passed": False}
        if case["status"] == "REFUSED":
            result.update(generation_refusal=case["reason"], source_integrity="NOT_ATTEMPTED_GENERATION_REFUSED")
        else:
            try:
                verify_case(args.root, case, result)
            except (SourceRefused, ReferenceRefused) as exc:
                stage = "SOURCE" if "source" not in result else "REFERENCE"
                result.update(passed=False, evidence_refusal=str(exc), evidence_refusal_stage=stage)
                if stage == "SOURCE":
                    result["source_integrity"] = "REFUSED"
        classify(result)
        cases.append(result)
    report = {**summarize(run, cases), "cases": cases}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({k: v for k, v in report.items() if k != "cases"}, sort_keys=True))
    return 0 if report["model_reference_qualified_count"] == report["requested_count"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
