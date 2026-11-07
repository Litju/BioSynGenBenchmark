"""Read SOURCE only through installed Plate2bar APIs; never open generator truth.

Run with Plate2bar's qualified interpreter. Production source is unmodified.
Output is a candidate execution receipt, not scientific qualification.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys


def deny_reference_reads(event, arguments):
    if event == "open" and isinstance(arguments[0], (str, bytes, os.PathLike)):
        parts = Path(os.fsdecode(arguments[0])).parts
        if any(part.upper() == "TRUTH" for part in parts):
            raise PermissionError("PRODUCTION_REFERENCE_TRUTH_ACCESS_REFUSED")


sys.addaudithook(deny_reference_reads)

# Product API imports initialize a default catalog. Keep that disposable
# bootstrap catalog with the other local integration state, outside Git.
os.environ.setdefault("PLATE2BAR_PROJECT_ROOT", str(
    Path(__file__).resolve().parents[1] / "runs/i3/res524/working/product-bootstrap"))

from plate2bar_instrument.headless_application import Plate2barApplication
from plate2bar_instrument.measurement import (ChannelMapping, DelimitedTextAdapter,
    ImportProfile, SourceArtifact, TimebasePolicy, admit_force_component, admit_lpt_component,
    import_open_bundle, admit_coupled_measurement_trial, ComponentPairingEvidence)
from plate2bar_instrument.computation import (CanonicalTrialTimeline, ClockMapping,
    ComputationRefusal, TimingUncertainty)
from plate2bar_instrument.measurement.segmentation import MOTION_METHOD, FORCE_DEPARTURE_METHOD
from plate2bar_instrument.measurement.segmentation_operating_profiles import FROZEN_V1

STAMP = "2026-10-15T00:00:00Z"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_source(root):
    document = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    records = {}
    for line in (root / "checksums.sha256").read_text().splitlines():
        expected, relative = line.split("  ", 1)
        path = (root / relative).resolve()
        if not path.is_relative_to(root.resolve()) or path.is_symlink():
            raise ValueError("SOURCE_PATH_REFUSED")
        if digest(path) != expected:
            raise ValueError("SOURCE_CHECKSUM_MISMATCH")
        records[relative] = expected
    actual = {p.relative_to(root).as_posix() for p in root.rglob("*") if p.is_file()}
    if actual != set(records) | {"checksums.sha256"} or "manifest.json" not in records:
        raise ValueError("SOURCE_INVENTORY_MISMATCH")
    for record in document["source_files"]:
        if (records.get(record["path"]) != record["sha256"] or
                (root / record["path"]).stat().st_size != record["byte_length"]):
            raise ValueError("SOURCE_RECORD_MISMATCH")
    return document


def import_stream(app, root, document, name, *, derived=False):
    force = name == "force"
    relative = "derived/vertical.csv" if derived else f"streams/{name}.csv"
    source = DelimitedTextAdapter(delimiter=",", numeric_fields=(
        ("time_s", "force_left_N", "force_right_N") if force else ("time_s", "displacement_m")),
        field_units={"time_s": "s", **({"force_left_N": "N", "force_right_N": "N"} if force else {"displacement_m": "m"})},
        time_field_id="time_s").parse(SourceArtifact.from_bytes((root / relative).read_bytes(),
            original_filename=Path(relative).name, declared_format="csv", media_type="text/csv", ingested_at=STAMP))
    details = document[f"{name}_stream"]
    frame = document["derived_observation"]["frame_id"] if derived else details["frame_id"]
    state = document["derived_observation"]["source_processing_state"] if derived else details["processing"]["state"]
    columns = (("force_left_N", "FZ_LEFT"), ("force_right_N", "FZ_RIGHT")) if force else (("displacement_m", "LPT_DISPLACEMENT"),)
    maps = tuple(ChannelMapping(source_field_id=field, canonical_quantity_id=quantity,
        source_unit="N" if force else "m", canonical_unit="N" if force else "m",
        physical_dimension="FORCE" if force else "LENGTH", directional=True,
        reference_frame=frame, axis="Z" if force or derived else "DECLARED_LPT_AXIS",
        sign_convention="UP" if force or derived else details["positive_direction"],
        object_id="PLATE_" + field if force else "BAR_COLLAR_LPT",
        measurement_point=None if force else "BAR_COLLAR", source_calibration_state="CALIBRATION_DECLARED",
        source_processing_state=state)
        for field, quantity in columns)
    provenance = {"component_pipeline": "FORCE_PIPE_01" if force else "LPT_PIPE_01",
        "clock_domain_id": name, "clock_domain_kind": "native_device_acquisition_clock",
        "timebase_id": name + "-native-timebase", "canonical_timeline_id": name + "-local",
        "acquisition_id": document["acquisition_id"], "source_manifest_sha256": digest(root / "manifest.json")}
    if not force:
        provenance.update({"coordinate_kind": "VERTICAL_ATTACHMENT_DISPLACEMENT" if derived else "LPT_SENSOR_COORDINATE",
            "source_coordinate_id": state + "-coordinate",
            "attachment_point_id": "BAR_COLLAR", "displacement_semantics": "REFERENCE_RELATIVE_DISPLACEMENT",
            "reference_semantics": "EXPLICIT_REFERENCE_REQUIRED_AT_P1"})
    profile = ImportProfile(profile_id="BSMB-" + name + ("-derived" if derived else "-raw"), profile_version="1.0.0",
        schema_fingerprint=source.schema.fingerprint, adapter_input_schema_id=source.schema.adapter_input_schema_id,
        mappings=maps, timebase=TimebasePolicy.observed(timestamp_field_id="time_s", timestamp_unit="s"),
        profile_provenance=provenance)
    result = app.import_normalized_source("bsmb-pilot", source, profile)
    component = (admit_force_component if force else admit_lpt_component)(result.measurement)
    return result, component, profile


def mapping_from_captures(component, document, root, name, timeline):
    captures = json.loads(document["operator_notes"])["sync_capture"]
    if (captures["contract_id"] != "BSMB-TWO-EDGE-CAPTURE-1.0" or
            captures["recording_hashes"][name] != digest(root / f"streams/{name}.csv")):
        raise ValueError("SYNC_CAPTURE_RECORDING_MISMATCH")
    x0, x1 = captures["devices"][name]["edge_native_times_s"]
    y0, y1 = captures["reference_edge_times_s"]
    if not all(math.isfinite(x) for x in (x0, x1, y0, y1)) or x1 <= x0 or y1 <= y0:
        raise ValueError("SYNC_EDGES_UNQUALIFIED")
    scale = (y1 - y0) / (x1 - x0)
    return ClockMapping.explicit_affine(source_clock_domain_id=component.time_authority.clock_domain.identity,
        canonical_timeline=timeline, scale=scale, offset_seconds=y0 - scale * x0,
        evidence_identity=digest(root / "manifest.json") + ":two-edge:" + name,
        evidence_contract_id="BSMB-TWO-EDGE-CAPTURE-1.0", created_by="BSMB-SOURCE-READER",
        source_timebase_id=component.time_authority.timebase.timebase_id,
        timing_uncertainty=TimingUncertainty.quantified(bound_seconds=32 * math.ulp(max(1.0, y1)),
            method_id="IDEAL_CAPTURE_AFFINE_FIT_BINARY64_GUARD", evidence_identity="SOURCE_VISIBLE_TWO_EDGE_CAPTURE",
            meaning="Assumed ideal simulated external trigger capture; float guard only; not empirical uncertainty"))


def execute(source, state_root, *, protocol):
    document = validate_source(source)
    state_root.mkdir(parents=True, exist_ok=False)
    result = {"source_manifest_sha256": digest(source / "manifest.json"), "production_truth_access_count": 0,
              "truth_injection_count": 0, "protocol": protocol,
              "truth_read_protection": "PYTHON_OPEN_AUDIT_DENIES_TRUTH_DIRECTORIES; native OS file access not audited"}
    if document["profile_id"] == "P2B-OPEN-DUAL-VFORCE-LPT":
        opened = import_open_bundle(source, state_root / "preserved-open", STAMP)
        result["open_bundle_import"] = {"readiness": opened.readiness_state.value,
            "package_seal_sha256": opened.package_seal_sha256}
    app = Plate2barApplication(state_root / "projects")
    app.create_project("bsmb-pilot")
    f_import, force, f_profile = import_stream(app, source, document, "force")
    l_import, lpt, l_profile = import_stream(app, source, document, "lpt")
    result["import_profile_hashes"] = {"force": f_profile.semantic_hash, "lpt": l_profile.semantic_hash}
    result["raw_component_admission"] = "PASS"
    snapshots = app.persist_component_snapshots("bsmb-pilot", force_component=force, lpt_component=lpt,
        force_source_artifact_id=f_import.persisted.source_artifact_id,
        lpt_source_artifact_id=l_import.persisted.source_artifact_id)
    timeline = CanonicalTrialTimeline(timeline_id=document["trial_id"] + "-trigger-clock", created_by="BSMB-SOURCE-READER")
    mappings = [mapping_from_captures(c, document, source, name, timeline)
                for name, c in (("force", force), ("lpt", lpt))]
    authorities = [app.persist_clock_mapping("bsmb-pilot", m, source_timebase=c.time_authority.timebase)
                   for m, c in zip(mappings, (force, lpt))]
    coupled = app.admit_coupled_trial("bsmb-pilot", trial_id=document["trial_id"], trial_version="1.0.0",
        acquisition_id=document["acquisition_id"], pairing_evidence_id="two-edge-source-receipt",
        canonical_timeline_id=timeline.timeline_id,
        force_component_artifact_id=snapshots[0].component_artifact_id,
        lpt_component_artifact_id=snapshots[1].component_artifact_id,
        force_mapping_artifact_id=authorities[0].output_artifact_ids[0],
        lpt_mapping_artifact_id=authorities[1].output_artifact_ids[0])
    result["coupling"] = coupled.trial.to_dict()
    # Negative control uses the same input components but their unqualified
    # local-clock mappings: matching rates do not authorize synchronization.
    try:
        admit_coupled_measurement_trial(project_id="bsmb-pilot", trial_id="negative-sync", trial_version="1.0.0",
            canonical_trial_timeline=timeline, force_component=force, lpt_component=lpt,
            force_mapping=force.time_authority.mapping, lpt_mapping=lpt.time_authority.mapping,
            pairing_evidence=ComponentPairingEvidence.from_components(evidence_identity="negative-sync",
                project_id="bsmb-pilot", trial_id="negative-sync", acquisition_id=document["acquisition_id"],
                force_component=force, lpt_component=lpt))
    except ComputationRefusal as exc:
        assert exc.code.value == "CLOCK_MAPPING_UNQUALIFIED", str(exc)
        result["unqualified_sync_refusal"] = exc.code.value
    else:
        raise AssertionError("unqualified clock unexpectedly admitted")
    # Real production segmentation, frozen operating profiles, no truth labels.
    result["segmentation"] = {}
    for name, method, snapshot in (("force", FORCE_DEPARTURE_METHOD, snapshots[0]), ("lpt", MOTION_METHOD, snapshots[1])):
        result["segmentation"][name] = app.execute_automatic_segmentation("bsmb-pilot", snapshot.component_artifact_id,
            profile=None, operating_profile={"operating_profile_id": FROZEN_V1,
                "method_id": method, "protocol_id": protocol})
    if "derived_observation" in document:
        try:
            import_stream(app, source, document, "lpt", derived=True)
        except ComputationRefusal as exc:
            assert exc.code.value == "PROCESSING_STATE_UNSUPPORTED", str(exc)
            result["derived_admission"] = {"refusal_code": exc.code.value, "message": str(exc)}
        else:
            raise AssertionError("derived source unexpectedly admitted as raw")
    cold = Plate2barApplication(state_root / "projects")
    reopened = cold.reopen_coupled_trial("bsmb-pilot", coupled.coupled_trial_artifact_id)
    assert reopened.trial.to_dict() == coupled.trial.to_dict()
    for imported in (f_import, l_import):
        reopened_import = cold.reopen_normalized_import("bsmb-pilot", imported.canonical_artifact_id)
        assert reopened_import.scientific_content_hash == imported.scientific_content_hash
    result["cold_reopen"] = "EXACT_PRODUCTION_IDENTITY_PASS"
    result["status"] = "CANDIDATE_EXECUTED_NOT_SCIENTIFICALLY_QUALIFIED"
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--state", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--protocol", choices=("BACK_SQUAT_DUAL_V1", "DEADLIFT_DUAL_V1"), required=True)
    args = parser.parse_args(argv)
    if args.out.exists():
        parser.error("output must be fresh")
    result = execute(args.source.resolve(), args.state, protocol=args.protocol)
    with args.out.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({"status": result["status"], "raw_component_admission": result["raw_component_admission"],
                      "derived_admission": result.get("derived_admission"), "cold_reopen": result["cold_reopen"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
