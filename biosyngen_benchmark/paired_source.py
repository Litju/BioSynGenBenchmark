"""Independent source-only checks. No generator imports and no TRUTH access.

An out-of-band simulated capture receipt binds the complete manifest. This is
an operator-controlled manufactured authority, not cryptographic hardware
authentication or an empirical synchronization calibration.

Manifest-controlled paths are untrusted. They are validated as strings before
any filesystem use and every opened file must be a sealed, link-free member of
SOURCE. This is a software input boundary, not an operating-system sandbox.
"""

import csv
import hashlib
import json
import math
import os
from pathlib import Path
import stat

DERIVED_PATH = "derived/vertical.csv"
CAPTURE_CONTRACT = "BSMB-FOUR-EDGE-CAPTURE-1.0"
# Versioned modeled edge-capture assumption of BSMB-FOUR-EDGE-CAPTURE-1.0. It
# is the admissible upper envelope for this contract, not device metrology.
CAPTURE_ERROR_BOUND_MAX_S = 2e-6
SLACK_REFUSAL = "SLACK_CABLE_VERTICAL_RECONSTRUCTION_REFUSED"


class SourceRefused(ValueError):
    pass


def require(condition, code):
    if not condition:
        raise SourceRefused(code)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def strict_json(text):
    """Python accepts NaN/Infinity literals; source evidence must not."""
    def nonfinite(constant):
        raise SourceRefused("NONFINITE_JSON_NUMBER")
    try:
        return json.loads(text, parse_constant=nonfinite)
    except json.JSONDecodeError:
        raise SourceRefused("MALFORMED_JSON") from None


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def is_link(path):
    info = os.lstat(path)
    return stat.S_ISLNK(info.st_mode) or bool(
        getattr(info, "st_file_attributes", 0) & stat.FILE_ATTRIBUTE_REPARSE_POINT)


def safe_relative(name, code):
    """Accept only plain POSIX relative names; checked before filesystem use."""
    require(type(name) is str and name and not any(c in name for c in "\\:\0"), code)
    require(all(part not in ("", ".", "..") for part in name.split("/")), code)
    return name


def contained_file(source, name, code):
    safe_relative(name, code)
    current = Path(source)
    for part in name.split("/"):
        current = current / part
        require(os.path.lexists(current), "SOURCE_FILE_MISSING")
        require(not is_link(current), "SOURCE_LINK_REFUSED")
    require(current.resolve().is_relative_to(Path(source).resolve()) and current.is_file(), code)
    return current


def source_inventory(source):
    files = set()
    for directory, dirs, names in os.walk(source, followlinks=False):
        for name in dirs + names:
            require(not is_link(Path(directory) / name), "SOURCE_LINK_REFUSED")
        files.update((Path(directory) / name).relative_to(source).as_posix() for name in names)
    return files


def validate_capture(capture):
    require(capture.get("contract_id") == CAPTURE_CONTRACT, "CAPTURE_CONTRACT_UNSUPPORTED")
    bound = capture.get("capture_error_bound_s")
    require(finite_number(bound) and 0 < bound <= CAPTURE_ERROR_BOUND_MAX_S, "CAPTURE_UNCERTAINTY_INVALID")

    def ordered(values):
        return (type(values) is list and len(values) == 4 and all(finite_number(v) for v in values)
                and all(b > a for a, b in zip(values, values[1:])))
    require(ordered(capture.get("reference_edge_times_s")), "CAPTURE_ANCHORS_INSUFFICIENT")
    require(capture.get("fit_edge_indices") == [0, 3] and capture.get("held_out_edge_indices") == [1, 2],
            "CAPTURE_EDGE_ROLES_UNSUPPORTED")
    devices = capture.get("devices")
    require(type(devices) is dict and set(devices) == {"force", "lpt"} and
            all(type(d) is dict and ordered(d.get("edge_native_times_s")) for d in devices.values()),
            "CLOCK_CAPTURE_INVALID")


def read_rows(path, columns):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        require(reader.fieldnames == list(columns), "SOURCE_COLUMNS_INVALID")
        rows = list(reader)
    require(len(rows) >= 3, "INSUFFICIENT_SAMPLES")
    try:
        require(all(math.isfinite(float(v)) for row in rows for v in row.values()), "NONFINITE_SAMPLE")
    except (TypeError, ValueError):
        raise SourceRefused("MALFORMED_SAMPLE") from None
    return rows


def verify_source(source, capture_authority):
    source = Path(source)
    actual = source_inventory(source)
    manifest = strict_json(contained_file(source, "manifest.json", "UNSAFE_SOURCE_PATH").read_text(encoding="utf-8"))
    require(
        (manifest["profile_id"] == "P2B-OPEN-DUAL-VFORCE-LPT" and manifest["profile_version"] == "1.0")
        or (manifest["profile_id"] == "BSMB-PAIRED-CABLE-ANGLE-1.1" and manifest["profile_version"] == "1.1"),
        "SOURCE_PROFILE_UNSUPPORTED",
    )
    cable_profile = manifest["profile_id"] == "BSMB-PAIRED-CABLE-ANGLE-1.1"
    derived = manifest.get("derived_observation")
    if cable_profile:
        # Untrusted manifest path: exactly one sealed location or a declared null.
        require(type(derived) is dict and (derived.get("path") is None or derived.get("path") == DERIVED_PATH),
                "DERIVED_PATH_UNSUPPORTED")
    else:
        require(derived is None, "DERIVED_STAGE_UNSUPPORTED_FOR_PROFILE")
    require(capture_authority is not None, "INDEPENDENT_CAPTURE_AUTHORITY_REQUIRED")
    require(capture_authority.get("source_manifest_sha256") == digest(source / "manifest.json"),
            "CAPTURE_AUTHORITY_MANIFEST_MISMATCH")
    sealed = {}
    for line in contained_file(source, "checksums.sha256", "UNSAFE_SOURCE_PATH").read_text().splitlines():
        value, separator, name = line.partition("  ")
        require(separator and len(value) == 64 and set(value) <= set("0123456789abcdef"), "CHECKSUM_LINE_MALFORMED")
        safe_relative(name, "UNSAFE_CHECKSUM_PATH")
        require(name not in sealed, "UNSAFE_CHECKSUM_PATH")
        sealed[name] = value
    require(set(sealed) == actual - {"checksums.sha256"}, "SOURCE_INVENTORY_MISMATCH")
    for name, value in sealed.items():
        require(digest(contained_file(source, name, "UNSAFE_CHECKSUM_PATH")) == value, "CHECKSUM_MISMATCH")
    declared_roles = {}
    for record in manifest["source_files"]:
        require(record["path"] in sealed and sealed[record["path"]] == record["sha256"], "MANIFEST_HASH_MISMATCH")
        require(record["path"] not in declared_roles, "SOURCE_ROLE_DUPLICATE")
        declared_roles[record["path"]] = record["role"]
    expected_roles = {
        "streams/force.csv": "FORCE_STREAM",
        "streams/lpt.csv": "LPT_STREAM",
    }
    if cable_profile:
        expected_roles["streams/angle.csv"] = "OBSERVED_AUXILIARY_STREAM"
        # The declared stage, not file presence, selects the expected inventory.
        if derived["path"] == DERIVED_PATH:
            expected_roles[DERIVED_PATH] = "DERIVED_OBSERVATION"
    require(set(declared_roles) == set(expected_roles), "SOURCE_ROLE_INVENTORY_MISMATCH")
    require(all(declared_roles[p] == role for p, role in expected_roles.items()), "SOURCE_ROLE_INVALID")
    notes = strict_json(manifest["operator_notes"])
    require(manifest["lpt_stream"]["zero_reference"] == "FIRST_ACQUIRED_ENCODER_SAMPLE",
            "ENCODER_ZERO_REFERENCE_UNSUPPORTED")
    require(type(notes.get("observed_encoder_zero_sample_index")) is int and
            notes["observed_encoder_zero_sample_index"] == 0, "ENCODER_ZERO_INDEX_UNSUPPORTED")
    capture = notes["sync_capture"]
    require(capture == {k: v for k, v in capture_authority.items() if k != "source_manifest_sha256"},
            "CAPTURE_RECEIPT_MISMATCH")
    validate_capture(capture)
    ref = capture["reference_edge_times_s"]
    streams, coverage, gaps = {}, {}, {}
    for name in ("force", "lpt"):
        spec = manifest[f"{name}_stream"]
        require(spec["path"] == f"streams/{name}.csv", "STREAM_PATH_UNSUPPORTED")
        require(spec["unit"] == ("N" if name == "force" else "m"), "SOURCE_UNIT_UNSUPPORTED")
        columns = (("sample_index", "time_s", "force_left_N", "force_right_N") if name == "force" else
                   ("sample_index", "time_s", "displacement_m"))
        require(spec["columns"] == list(columns), "MANIFEST_COLUMNS_INVALID")
        require(spec["processing"]["state"] == "RAW_UNFILTERED_DECLARED", "PROCESSING_STATE_UNSUPPORTED")
        require(spec["frame_id"] == ("LAB_VERTICAL_Z_UP" if name == "force" or manifest["profile_id"] ==
                "P2B-OPEN-DUAL-VFORCE-LPT" else "CABLE_LENGTH_COORDINATE"), "FRAME_UNSUPPORTED")
        require(spec["timebase"]["clock_id"] == name, "CLOCK_ID_MISMATCH")
        if name == "lpt":
            require(spec["attachment_point"] in ("BAR_COLLAR", "HIP_BELT"), "ATTACHMENT_UNSUPPORTED")
        rows = read_rows(contained_file(source, spec["path"], "STREAM_PATH_UNSUPPORTED"), columns)
        rate = spec["timebase"]["nominal_sampling_rate_hz"]
        expected_rate = 500 if name == "lpt" and notes["acquisition_profile"] == "BSMB-FORCE1000-LPT500-1.0" else 1000
        require(rate == expected_rate, "RATE_UNSUPPORTED")
        indices, times = [int(row["sample_index"]) for row in rows], [float(row["time_s"]) for row in rows]
        require(all(b > a for a, b in zip(indices, indices[1:])) and all(b > a for a, b in zip(times, times[1:])),
                "CLOCK_RESET_OR_NONMONOTONIC")
        require(all(abs((b - a) - (j - i) / rate) < 1e-10 for a, b, i, j in
                    zip(times, times[1:], indices, indices[1:])), "NATIVE_GRID_INCONSISTENT")
        gaps[name] = sum(b - a - 1 for a, b in zip(indices, indices[1:]))
        require(digest(contained_file(source, spec["path"], "STREAM_PATH_UNSUPPORTED")) ==
                capture["recording_hashes"][name], "CAPTURE_STREAM_HASH_MISMATCH")
        edges = capture["devices"][name]["edge_native_times_s"]
        scale = (ref[-1] - ref[0]) / (edges[-1] - edges[0])
        offset = ref[0] - scale * edges[0]
        require(math.isfinite(scale) and scale > 0 and math.isfinite(offset), "CLOCK_CAPTURE_INVALID")
        bound = 2 * capture["capture_error_bound_s"] * scale + 1e-12
        require(all(abs(offset + scale * x - y) <= bound for x, y in zip(edges[1:-1], ref[1:-1])),
                "HELD_OUT_CAPTURE_RESIDUAL")
        declared = manifest["synchronization"]["stream_mappings"][name]
        require(finite_number(declared["offset_s"]) and finite_number(declared["drift_scale"]),
                "DECLARED_MAPPING_DISAGREES_WITH_CAPTURE")
        require(abs(declared["offset_s"] - offset) < 1e-12 and abs(declared["drift_scale"] - scale) < 1e-12,
                "DECLARED_MAPPING_DISAGREES_WITH_CAPTURE")
        coverage[name] = [offset + scale * times[0], offset + scale * times[-1]]
        streams[name] = rows
    require(min(v[1] for v in coverage.values()) > max(v[0] for v in coverage.values()), "NO_CLOCK_OVERLAP")
    zero_native = notes.get("observed_encoder_zero_native_time_s")
    require(type(zero_native) in (int, float) and math.isfinite(zero_native),
            "ENCODER_ZERO_NATIVE_TIME_INVALID")
    first_lpt = streams["lpt"][0]
    expected_native_time = zero_native + (
        int(first_lpt["sample_index"]) - notes["observed_encoder_zero_sample_index"]
    ) / manifest["lpt_stream"]["timebase"]["nominal_sampling_rate_hz"]
    require(abs(float(first_lpt["time_s"]) - expected_native_time) < 1e-10,
            "ENCODER_ZERO_TIMEBASE_MISMATCH")
    jitter = notes["acquisition_jitter_policy"]
    require(jitter["finite_error_bound_s"] is None or (finite_number(jitter["finite_error_bound_s"]) and
            jitter["finite_error_bound_s"] >= 0), "JITTER_POLICY_INVALID")
    # Receipt uncertainty bounds edge capture, not unobserved sample-time jitter.
    sync_outcome = "REFUSED_UNBOUNDED_SAMPLE_JITTER" if jitter["finite_error_bound_s"] is None else "BOUNDED_CAPTURE_MAPPING"
    if cable_profile:
        angle = read_rows(contained_file(source, "streams/angle.csv", "UNSAFE_SOURCE_PATH"), ("sample_index", "time_s", "encoder_length_m", "inclination_rad", "slack"))
        require(len(angle) == len(streams["lpt"]), "ANGLE_COVERAGE_MISMATCH")
        zero = notes["observed_encoder_zero_m"]
        for a, lpt in zip(angle, streams["lpt"]):
            require(a["sample_index"] == lpt["sample_index"] and a["time_s"] == lpt["time_s"], "ANGLE_GRID_MISMATCH")
            require(abs(float(a["encoder_length_m"]) - zero - float(lpt["displacement_m"])) < 1e-12,
                    "ENCODER_ABSOLUTE_REFERENCE_MISMATCH")
            require(0 <= abs(float(a["inclination_rad"])) <= math.pi + 0.02, "ANGLE_DOMAIN_UNSUPPORTED")
        require(derived["source_processing_state"] == "DERIVED_GEOMETRY_RECONSTRUCTION_V1", "DERIVED_STAGE_INVALID")
        if any(int(a["slack"]) for a in angle):
            require(derived["path"] is None and derived["refusal"] == SLACK_REFUSAL,
                    "SLACK_DERIVED_FALSE_ACCEPT")
        else:
            # A taut angle-aware recording declares a derivation; omission is not a pass.
            require(derived["path"] == DERIVED_PATH and derived["refusal"] is None,
                    "TAUT_DERIVED_OBSERVATION_MISSING")
            reconstructed = read_rows(contained_file(source, DERIVED_PATH, "DERIVED_PATH_UNSUPPORTED"), ("sample_index", "time_s", "displacement_m", "lower_m", "upper_m"))
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
