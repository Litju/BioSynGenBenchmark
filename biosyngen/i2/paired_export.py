"""Manufactured paired source packages. No Plate2bar implementation is imported.

Open 1.0 is used only for the constrained vertical apparatus. Free-weight
records retain cable coordinates in a local versioned source package, consumed
through Plate2bar's existing CSV/ImportProfile path, not its vertical-only open
adapter. No reference events or latent clock arrays enter SOURCE.
"""

import csv
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

from . import CLAIM_CEILINGS
from .acquisition import DUAL_1KHZ_V1
from .cable_geometry import InclinationModel, PROFILE_ID, RECONSTRUCTION_ID, observe_cable_angle, reconstruct_vertical
from .deadlift import DeadliftProtocol, build_deadlift, deadlift_lpt
from .instruments import CableLptModel, LptAttachment, observe_dual_plates
from .squat import SquatProtocol, build_back_squat, squat_lpt

SOURCE_PROFILE = "BSMB-PAIRED-CABLE-ANGLE-1.0"
STAMP = "2026-10-15T00:00:00Z"  # Versioned manufactured acquisition epoch, not wall time.


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def write_csv(path, header, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(header)
        writer.writerows(rows)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def seal(root, document):
    document["source_files"] = [
        {"path": p.relative_to(root).as_posix(), "byte_length": p.stat().st_size,
         "sha256": sha(p), "media_type": "text/csv" if p.suffix == ".csv" else "application/json",
         "role": "FORCE_STREAM" if p.name == "force.csv" else "LPT_STREAM" if p.name == "lpt.csv" else "ORIGINAL_SOURCE"}
        for p in sorted(root.rglob("*")) if p.is_file()]
    write_json(root / "manifest.json", document)
    with (root / "checksums.sha256").open("x", encoding="utf-8", newline="\n") as stream:
        for p in sorted(root.rglob("*")):
            if p.is_file() and p.name != "checksums.sha256":
                stream.write(f"{sha(p)}  {p.relative_to(root).as_posix()}\n")


def guided_vertical_world():
    """Two moving masses on an ideal vertical guide, driven through stationary feet.

    Body=75kg and bar=20kg share z(t), with a fixed COM height offset. The guide
    has no vertical reaction. Fz=M(g+z'') is the ONLY vertical external support.
    This is a constrained apparatus analytical world, never a free-weight squat.
    """
    def motion(t):
        if not 1.2 < t < 3.2:
            return 1.5, 0.0, 0.0
        u = (t - 1.2) / 2
        # Smooth down/up sin^4 trajectory, zero acceleration at either endpoint.
        s, c = math.sin(math.pi * u), math.cos(math.pi * u)
        z = 1.5 - 0.4 * s**4
        v = -0.8 * math.pi * s**3 * c
        a = -0.4 * math.pi**2 * (3 * s*s*c*c - s**4)
        return z, v, a
    times = [i / 1000 for i in range(4401)]
    model = SimpleNamespace(
        point_world=lambda t, body, local: (0.0, motion(t)[0]),
        system=SimpleNamespace(contacts=[SimpleNamespace(contact_id=s,
            plate_frame=SimpleNamespace(side=s.upper(), origin_world_m=(0, 0)))
            for s in ("left", "right")]))
    frames = [SimpleNamespace(t_start_s=t, t_end_s=times[i + 1], contact_forces={
        side: SimpleNamespace(force_world_n=(0, 0.5 * 95 * (9.81 +
            (motion(times[i + 1])[1] - motion(t)[1]) / (times[i + 1] - t))),
                              point_world_m=(0, 0)) for side in ("left", "right")})
              for i, t in enumerate(times[:-1])]
    world = SimpleNamespace(model=model, times=lambda: times, states=lambda: times,
                            frames=frames, impacts=[], termination="DURATION_REACHED")
    cable = CableLptModel(LptAttachment("bar", (0, 0), 0, "BAR_COLLAR"), anchor_m=(0, 0, 0))
    return world, cable, {"world_kind": "IDEAL_VERTICAL_GUIDE_TWO_MASS_APPARATUS",
        "body_mass_kg": 75, "bar_mass_kg": 20, "guide_vertical_reaction_n": 0,
        "events": {"mathematical_motion_start_s": 1.2, "bottom_s": 2.2, "motion_end_s": 3.2},
        "bar_vertical_m": [motion(t)[0] for t in times]}


def build_case(protocol):
    if protocol == "guided_vertical":
        return guided_vertical_world()
    if protocol == "back_squat":
        world = build_back_squat(SquatProtocol(repetitions=1, start_hold_s=1.2)).run()
        cable = squat_lpt(world)
    elif protocol == "deadlift":
        world = build_deadlift(DeadliftProtocol(setup_hold_s=1.2)).run()
        cable = deadlift_lpt(world)
    else:
        raise ValueError("unsupported protocol")
    # References are physical coordinates/contact modes, not controller stages.
    return world, cable, {"world_kind": "FREE_WEIGHT_MULTIBODY",
        "bar_vertical_m": [s.bodies[world.model.load_body_id].position_m[1] for s in world.states()],
        "contacts": [{key: contact.mode.value for key, contact in s.contacts.items()} for s in world.states()],
        "impulses_N_s": [asdict(impact) for impact in world.impacts]}


def timebase(clock, name):
    return {"clock_id": name, "authority": "TIMESTAMP", "nominal_sampling_rate_hz": 1000,
            "time_origin": "DEVICE_NATIVE_ORIGIN", "regularity": "DECLARED_REGULAR",
            "missing_sample_policy": "GAPS_PRESERVED_REPORTED", "duplicate_sample_policy": "REFUSE",
            "non_monotonic_sample_policy": "REFUSE"}


def trigger_evidence(clocks, end, hashes):
    # A simulated external pulse generator fans two edges into both devices'
    # hardware timestamp capture inputs. Clock maps will be FIT from these
    # source-visible observations by the integration reader, not read from TRUTH.
    return {"contract_id": "BSMB-TWO-EDGE-CAPTURE-1.0",
        "mechanism": "SIMULATED_EXTERNAL_TWO_PULSE_HARDWARE_CAPTURE",
        "reference_clock_id": "external-pulse-clock", "reference_edge_times_s": [0.0, end],
        "edge_ids": ["pulse-start", "pulse-end"], "capture_error_bound_s": 0.0,
        "capture_accuracy_status": "ASSUMED_IDEAL_MANUFACTURED_NOT_EMPIRICAL",
        "recording_hashes": hashes,
        "devices": {name: {"clock_id": name, "edge_native_times_s": [
            clock.offset_s, clock.offset_s + clock.rate_scale * end]} for name, clock in clocks.items()}}


def export_case(root: Path, *, protocol, seed, case_id):
    root = Path(root)
    source = root / "SOURCE" / case_id
    truth = root / "TRUTH" / case_id
    source.mkdir(parents=True, exist_ok=False)
    world, cable, reference = build_case(protocol)
    force_model, cable = DUAL_1KHZ_V1.instruments(cable)
    # Explicit development acquisition: constant drift, ideal hardware capture,
    # no unbounded sample jitter; independent clock identities remain distinct.
    force_model = replace(force_model, clock=replace(force_model.clock, jitter_sd_s=0), seed=seed + 11)
    cable = replace(cable, clock=replace(cable.clock, jitter_sd_s=0), seed=seed + 12)
    angle_model = InclinationModel(seed=seed + 93)
    force = observe_dual_plates(world, force_model)
    observed = observe_cable_angle(world, cable, angle_model)
    vertical = reconstruct_vertical(observed.samples, length_resolution_m=cable.encoder_resolution_m,
        angle_error_bound_rad=angle_model.error_bound_rad, angle_resolution_rad=angle_model.resolution_rad)
    write_csv(source / "streams/force.csv", ("sample_index", "time_s", "force_left_N", "force_right_N"),
        ((s.index, s.native_time_s, s.fz_left_n, s.fz_right_n) for s in force.samples))
    write_csv(source / "streams/lpt.csv", ("sample_index", "time_s", "displacement_m"),
        ((s.index, s.native_time_s, s.displacement_m) for s in observed.samples))
    hashes = {name: sha(source / f"streams/{name}.csv") for name in ("force", "lpt")}
    clocks = {"force": force_model.clock, "lpt": cable.clock}
    sync = trigger_evidence(clocks, world.times()[-1], hashes)
    identity = {"identity_status": "DECLARED_BY_OPERATOR", "manufacturer": "BioSynGen synthetic hypothesis",
                "model": "BSMB-DUAL1K-1.0", "serial_number": case_id, "firmware_version": None,
                "acquisition_software": "BSMB-DUAL1K-1.0", "exporter_version": "1.0"}
    calibration = {"status": "CALIBRATION_DECLARED", "zeroing_status": "ZERO_DECLARED",
                   "calibration_id": "BSMB-ASSUMED-SI-1.0", "calibration_date": None,
                   "evidence_reference": "ASSUMED_MANUFACTURED_PARAMETERS_NOT_EMPIRICAL_CALIBRATION"}
    force_stream = {"path": "streams/force.csv", "columns": ["sample_index", "time_s", "force_left_N", "force_right_N"],
        "unit": "N", "frame_id": "LAB_VERTICAL_Z_UP", "positive_direction": "UP",
        "timebase": timebase(force_model.clock, "force"), "left_plate": identity,
        "right_plate": {**identity, "serial_number": case_id + "-right"}, "calibration": calibration,
        "processing": {"state": "RAW_UNFILTERED_DECLARED"}}
    lpt_stream = {"path": "streams/lpt.csv", "columns": ["sample_index", "time_s", "displacement_m"],
        "unit": "m", "frame_id": "LAB_VERTICAL_Z_UP" if protocol == "guided_vertical" else "CABLE_LENGTH_COORDINATE",
        "positive_direction": "UP" if protocol == "guided_vertical" else "AWAY_FROM_REEL",
        "timebase": timebase(cable.clock, "lpt"), "device": identity, "attachment_point": "BAR_COLLAR",
        "cable_direction": "CONSTRAINED_VERTICAL_GUIDE" if protocol == "guided_vertical" else "VARIABLE_INCLINATION_OBSERVED",
        "angle_correction": "NONE", "zero_reference": "FIRST_RECORDED_ENCODER_SAMPLE",
        "calibration": calibration, "processing": {"state": "RAW_UNFILTERED_DECLARED"}}
    document = {"profile_id": "P2B-OPEN-DUAL-VFORCE-LPT" if protocol == "guided_vertical" else SOURCE_PROFILE,
        "profile_version": "1.0", "bundle_id": case_id, "acquisition_id": case_id,
        "session_id": "BSMB-DEVELOPMENT", "trial_id": case_id, "created_at_utc": STAMP,
        "force_stream": force_stream, "lpt_stream": lpt_stream,
        "synchronization": {"authority_class": "MANUFACTURED_ANALYTICAL_AFFINE_MAPPING",
            "evidence_reference": "operator_notes:two-edge-capture",
            "stream_mappings": {name: {"offset_s": -c.offset_s / c.rate_scale,
                "drift_scale": 1 / c.rate_scale, "evidence_reference": "operator_notes:two-edge-capture"}
                for name, c in clocks.items()}},
        "operator_notes": json.dumps({"claim_ceilings": list(CLAIM_CEILINGS), "sync_capture": sync,
            "apparatus": reference["world_kind"], "acquisition_profile": DUAL_1KHZ_V1.profile_id,
            "force_front_end": {k: v for k, v in asdict(force_model).items() if k != "clock"},
            "lpt_encoder_resolution_m": cable.encoder_resolution_m,
            "raw_definition": "ADC/encoder output; acquisition analog antialias stage retained; no post-acquisition software filtering"}, sort_keys=True)}
    if protocol != "guided_vertical":
        write_csv(source / "streams/angle.csv", ("sample_index", "time_s", "encoder_length_m", "inclination_rad", "slack"),
            ((s.index, s.native_time_s, s.encoder_length_m, s.inclination_rad, int(s.slack)) for s in observed.samples))
        write_csv(source / "derived/vertical.csv", ("sample_index", "time_s", "displacement_m", "lower_m", "upper_m"),
            ((s.index, s.native_time_s, *v) for s, v in zip(observed.samples, vertical)))
        document["derived_observation"] = {"profile_id": RECONSTRUCTION_ID,
            "source_processing_state": "DERIVED_GEOMETRY_RECONSTRUCTION_V1", "quantity": "VERTICAL_ATTACHMENT_DISPLACEMENT",
            "frame_id": "LAB_VERTICAL_Z_UP", "source_geometry_profile": PROFILE_ID,
            "angle_model": asdict(angle_model), "length_resolution_m": cable.encoder_resolution_m,
            "attachment": asdict(cable.attachment), "anchor_m": cable.anchor_m,
            "path": "derived/vertical.csv", "uncertainty": "BOUNDED_INTERVAL_NOT_STANDARD_DEVIATION"}
    seal(source, document)
    write_json(truth / "reference.json", {"claim_ceilings": list(CLAIM_CEILINGS), **reference,
        "physics_times_s": world.times(), "lpt_true_sample_times_s": observed.true_times_s,
        "clocks": {name: asdict(c) for name, c in clocks.items()},
        "termination": world.termination, "source_manifest_sha256": sha(source / "manifest.json")})
    return {"case_id": case_id, "protocol": protocol, "force_count": len(force.samples),
        "lpt_count": len(observed.samples), "source_bytes": sum(p.stat().st_size for p in source.rglob("*") if p.is_file()),
        "source_manifest_sha256": sha(source / "manifest.json")}
