"""Actual open-adapter adversaries against fresh generated source, not holdouts."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

from qualify_paired_source import import_open_bundle, execute, ComputationRefusal, STAMP
from plate2bar_instrument.measurement import MeasurementRefusal


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("positive", "oblique", "different_trial", "out"):
        parser.add_argument("--" + name.replace("_", "-"), type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    results = []
    for case in ("corruption", "different_physical_trial", "unqualified_cable_frame"):
        source = args.out / case
        if case == "unqualified_cable_frame":
            source.mkdir()
            (source / "streams").mkdir()
            for name in ("force", "lpt"):
                shutil.copyfile(args.oblique / f"streams/{name}.csv", source / f"streams/{name}.csv")
            doc = json.loads((args.oblique / "manifest.json").read_text())
            doc["profile_id"] = "P2B-OPEN-DUAL-VFORCE-LPT"
            doc.pop("derived_observation")
            doc["source_files"] = [r for r in doc["source_files"] if r["path"] in ("streams/force.csv", "streams/lpt.csv")]
            (source / "manifest.json").write_bytes((json.dumps(doc, sort_keys=True) + "\n").encode())
            checks = [f"{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.relative_to(source).as_posix()}\n"
                      for p in sorted(source.rglob("*")) if p.is_file()]
            (source / "checksums.sha256").write_bytes("".join(checks).encode())
            expected = "UNKNOWN_FRAME"
        else:
            shutil.copytree(args.positive, source)
            if case == "corruption":
                with (source / "streams/force.csv").open("ab") as stream:
                    stream.write(b"\n")
            else:
                shutil.copyfile(args.different_trial / "streams/force.csv", source / "streams/force.csv")
            expected = "CHECKSUM_MISMATCH"
        try:
            import_open_bundle(source, args.out / (case + "-preserved"), STAMP)
        except MeasurementRefusal as exc:
            assert exc.code == expected, str(exc)
            results.append({"case": case, "expected": expected, "actual": exc.code})
        else:
            raise AssertionError("adversarial source unexpectedly admitted")
    source = args.out / "declared_processed_source"
    shutil.copytree(args.positive, source)
    manifest = source / "manifest.json"
    doc = json.loads(manifest.read_text())
    doc["lpt_stream"]["processing"] = {"state": "VENDOR_PROCESSED_DECLARED",
        "method": "ADVERSARIAL_DECLARED_SOURCE_PROCESSING", "implementation_version": "1.0",
        "parameters": {}, "evidence_reference": "MANUFACTURED_STAGE_REFUSAL_CONTROL"}
    manifest.write_bytes((json.dumps(doc, sort_keys=True) + "\n").encode())
    (source / "checksums.sha256").write_bytes("".join(
        f"{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.relative_to(source).as_posix()}\n"
        for p in sorted(source.rglob("*")) if p.is_file() and p.name != "checksums.sha256").encode())
    try:
        execute(source.resolve(), args.out / "processed-state-project", protocol="BACK_SQUAT_DUAL_V1")
    except ComputationRefusal as exc:
        assert exc.code.value == "PROCESSING_STATE_UNSUPPORTED", str(exc)
        results.append({"case": "declared_processed_source", "expected": "PROCESSING_STATE_UNSUPPORTED",
                        "actual": exc.code.value})
    else:
        raise AssertionError("declared processing stage was lost before component admission")
    (args.out / "result.json").write_bytes((json.dumps(results, indent=2) + "\n").encode())
    print(json.dumps(results))


if __name__ == "__main__":
    main()
