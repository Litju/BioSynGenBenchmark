"""Independent source-only checks. No generator imports and no TRUTH access.

An out-of-band simulated capture receipt binds the complete manifest. This is
an operator-controlled manufactured authority, not cryptographic hardware
authentication or an empirical synchronization calibration.
"""

import csv
import hashlib
import json
import math


class SourceRefused(ValueError):
    pass


def require(condition, code):
    if not condition:
        raise SourceRefused(code)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_rows(path):
    require(path.is_file(), "SOURCE_FILE_MISSING")
    with path.open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    require(len(rows) >= 3, "INSUFFICIENT_SAMPLES")
    try:
        require(all(math.isfinite(float(v)) for row in rows for v in row.values()), "NONFINITE_SAMPLE")
    except (TypeError, ValueError):
        raise SourceRefused("MALFORMED_SAMPLE") from None
    return rows


def verify_source(source, capture_authority):
    manifest = json.loads((source / "manifest.json").read_text(encoding="utf-8"))
    require(manifest["profile_id"] in ("P2B-OPEN-DUAL-VFORCE-LPT", "BSMB-PAIRED-CABLE-ANGLE-1.0") and
            manifest["profile_version"] == "1.0", "SOURCE_PROFILE_UNSUPPORTED")
    require(capture_authority is not None, "INDEPENDENT_CAPTURE_AUTHORITY_REQUIRED")
    require(capture_authority.get("source_manifest_sha256") == digest(source / "manifest.json"),
            "CAPTURE_AUTHORITY_MANIFEST_MISMATCH")
    actual = {p.relative_to(source).as_posix() for p in source.rglob("*") if p.is_file()}
    sealed = {}
    for line in (source / "checksums.sha256").read_text().splitlines():
        value, name = line.split("  ", 1)
        require(name not in sealed and not name.startswith(("/", "\\")) and
                ".." not in name.replace("\\", "/").split("/"), "UNSAFE_CHECKSUM_PATH")
        sealed[name] = value
    require(set(sealed) == actual - {"checksums.sha256"}, "SOURCE_INVENTORY_MISMATCH")
    for name, value in sealed.items():
        require(digest(source / name) == value, "CHECKSUM_MISMATCH")
    for record in manifest["source_files"]:
        require(record["path"] in sealed and sealed[record["path"]] == record["sha256"], "MANIFEST_HASH_MISMATCH")
    notes = json.loads(manifest["operator_notes"])
    capture = notes["sync_capture"]
    require(capture == {k: v for k, v in capture_authority.items() if k != "source_manifest_sha256"},
            "CAPTURE_RECEIPT_MISMATCH")
    require(capture["capture_error_bound_s"] > 0, "CAPTURE_UNCERTAINTY_MISSING")
    ref = capture["reference_edge_times_s"]
    require(len(ref) >= 4 and all(b > a for a, b in zip(ref, ref[1:])), "CAPTURE_ANCHORS_INSUFFICIENT")
    streams, coverage, gaps = {}, {}, {}
    for name in ("force", "lpt"):
        spec = manifest[f"{name}_stream"]
        require(spec["path"] == f"streams/{name}.csv", "STREAM_PATH_UNSUPPORTED")
        require(spec["processing"]["state"] == "RAW_UNFILTERED_DECLARED", "PROCESSING_STATE_UNSUPPORTED")
        require(spec["frame_id"] == ("LAB_VERTICAL_Z_UP" if name == "force" or manifest["profile_id"] ==
                "P2B-OPEN-DUAL-VFORCE-LPT" else "CABLE_LENGTH_COORDINATE"), "FRAME_UNSUPPORTED")
        require(spec["timebase"]["clock_id"] == name, "CLOCK_ID_MISMATCH")
        if name == "lpt":
            require(spec["attachment_point"] in ("BAR_COLLAR", "HIP_BELT"), "ATTACHMENT_UNSUPPORTED")
        rows = read_rows(source / spec["path"])
        rate = spec["timebase"]["nominal_sampling_rate_hz"]
        require(rate in (500, 1000), "RATE_UNSUPPORTED")
        indices, times = [int(row["sample_index"]) for row in rows], [float(row["time_s"]) for row in rows]
        require(all(b > a for a, b in zip(indices, indices[1:])) and all(b > a for a, b in zip(times, times[1:])),
                "CLOCK_RESET_OR_NONMONOTONIC")
        require(all(abs((b - a) - (j - i) / rate) < 1e-10 for a, b, i, j in
                    zip(times, times[1:], indices, indices[1:])), "NATIVE_GRID_INCONSISTENT")
        gaps[name] = sum(b - a - 1 for a, b in zip(indices, indices[1:]))
        require(digest(source / spec["path"]) == capture["recording_hashes"][name], "CAPTURE_STREAM_HASH_MISMATCH")
        edges = capture["devices"][name]["edge_native_times_s"]
        require(len(edges) == len(ref) and edges[-1] > edges[0], "CLOCK_CAPTURE_INVALID")
        scale = (ref[-1] - ref[0]) / (edges[-1] - edges[0])
        offset = ref[0] - scale * edges[0]
        bound = 2 * capture["capture_error_bound_s"] * scale + 1e-12
        require(all(abs(offset + scale * x - y) <= bound for x, y in zip(edges[1:-1], ref[1:-1])),
                "HELD_OUT_CAPTURE_RESIDUAL")
        declared = manifest["synchronization"]["stream_mappings"][name]
        require(abs(declared["offset_s"] - offset) < 1e-12 and abs(declared["drift_scale"] - scale) < 1e-12,
                "DECLARED_MAPPING_DISAGREES_WITH_CAPTURE")
        coverage[name] = [offset + scale * times[0], offset + scale * times[-1]]
        streams[name] = rows
    require(min(v[1] for v in coverage.values()) > max(v[0] for v in coverage.values()), "NO_CLOCK_OVERLAP")
    jitter = notes["acquisition_jitter_policy"]
    # Receipt uncertainty bounds edge capture, not unobserved sample-time jitter.
    sync_outcome = "REFUSED_UNBOUNDED_SAMPLE_JITTER" if jitter["finite_error_bound_s"] is None else "BOUNDED_CAPTURE_MAPPING"
    if manifest["profile_id"] != "P2B-OPEN-DUAL-VFORCE-LPT":
        angle_path = source / "streams/angle.csv"
        angle = read_rows(angle_path)
        require(len(angle) == len(streams["lpt"]), "ANGLE_COVERAGE_MISMATCH")
        zero = notes["observed_encoder_zero_m"]
        for a, lpt in zip(angle, streams["lpt"]):
            require(a["sample_index"] == lpt["sample_index"] and a["time_s"] == lpt["time_s"], "ANGLE_GRID_MISMATCH")
            require(abs(float(a["encoder_length_m"]) - zero - float(lpt["displacement_m"])) < 1e-12,
                    "ENCODER_ABSOLUTE_REFERENCE_MISMATCH")
            require(0 <= abs(float(a["inclination_rad"])) <= math.pi + 0.02, "ANGLE_DOMAIN_UNSUPPORTED")
        derived = manifest["derived_observation"]
        require(derived["source_processing_state"] == "DERIVED_GEOMETRY_RECONSTRUCTION_V1", "DERIVED_STAGE_INVALID")
        if any(int(a["slack"]) for a in angle):
            require(derived["path"] is None and derived["refusal"] == "SLACK_CABLE_VERTICAL_RECONSTRUCTION_REFUSED",
                    "SLACK_DERIVED_FALSE_ACCEPT")
        elif derived["path"] is not None:
            reconstructed = read_rows(source / derived["path"])
            require(len(reconstructed) == len(angle), "DERIVED_COVERAGE_MISMATCH")
            zero_height = zero * math.cos(notes["observed_inclination_zero_rad"])
            for a, d in zip(angle, reconstructed):
                require(a["sample_index"] == d["sample_index"] and a["time_s"] == d["time_s"], "DERIVED_GRID_MISMATCH")
                expected = float(a["encoder_length_m"]) * math.cos(float(a["inclination_rad"])) - zero_height
                require(abs(float(d["displacement_m"]) - expected) < 1e-12, "DERIVED_NOT_FROM_OBSERVED_GEOMETRY")
                require(float(d["lower_m"]) <= expected <= float(d["upper_m"]), "DERIVED_UNCERTAINTY_INVALID")
    return {"source": "PASS", "synchronization": sync_outcome, "coverage_s": coverage,
            "force_saturated_sample_count": (len(notes["force_saturated_sample_indices"])
                if "force_saturated_sample_indices" in notes else None),
            "dropped_sample_counts": gaps, "nominal_rates_hz": {name: manifest[f"{name}_stream"]["timebase"][
                "nominal_sampling_rate_hz"] for name in streams},
            "authority_scope": "OUT_OF_BAND_MANUFACTURED_RECEIPT_NOT_EMPIRICAL_AUTHENTICATION"}
