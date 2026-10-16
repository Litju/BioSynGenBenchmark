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
import random
from pathlib import Path
from types import SimpleNamespace

from . import CLAIM_CEILINGS
from .acquisition import DUAL_1KHZ_V1, LEGACY_MIXED_RATE_V1
from .corpus_config import validate_parameters
from .cable_geometry import InclinationModel, PROFILE_ID, RECONSTRUCTION_ID, observe_cable_angle, reconstruct_vertical
from .deadlift import DeadliftProtocol, build_deadlift, deadlift_lpt
from .instruments import CableLptModel, LptAttachment, observe_dual_plates, default_lpt, attachment_world
from .human import Anthropometry
from .jumps import build_jump
from .bench import BenchProtocol, build_bench_press, bench_lpt
from .olympic import PullProtocol, build_pull, pull_lpt
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


def guided_vertical_world(parameters=None):
    """Two moving masses on an ideal vertical guide, driven through stationary feet.

    Body=75kg and bar=20kg share z(t), with a fixed COM height offset. The guide
    has no vertical reaction. Fz=M(g+z'') is the ONLY vertical external support.
    This is a constrained apparatus analytical world, never a free-weight squat.
    """
    p = parameters or {}
    mass = p.get("body_mass_kg", 75) + p.get("load_mass_kg", 20)
    baseline, tempo = p.get("baseline_s", 1.2), p.get("tempo_scale", 1.0)
    def motion(t):
        if not baseline < t < baseline + 2 * tempo:
            return 1.5, 0.0, 0.0
        u = (t - baseline) / (2 * tempo)
        # Smooth down/up sin^4 trajectory, zero acceleration at either endpoint.
        s, c = math.sin(math.pi * u), math.cos(math.pi * u)
        z = 1.5 - 0.4 * s**4
        v = -0.8 * math.pi * s**3 * c / tempo
        a = -0.4 * math.pi**2 * (3 * s*s*c*c - s**4) / tempo**2
        return z, v, a
    times = [i / 1000 for i in range(round((baseline + 2 * tempo + 1.2) * 1000) + 1)]
    model = SimpleNamespace(
        point_world=lambda t, body, local: (0.0, motion(t)[0]),
        system=SimpleNamespace(contacts=[SimpleNamespace(contact_id=s,
            plate_frame=SimpleNamespace(side=s.upper(), origin_world_m=(0, 0)))
            for s in ("left", "right")]))
    frames = [SimpleNamespace(t_start_s=t, t_end_s=times[i + 1], contact_forces={
        side: SimpleNamespace(force_world_n=(0, 0.5 * mass * (9.81 +
            (motion(times[i + 1])[1] - motion(t)[1]) / (times[i + 1] - t))),
                              point_world_m=(0, 0)) for side in ("left", "right")})
              for i, t in enumerate(times[:-1])]
    world = SimpleNamespace(model=model, times=lambda: times, states=lambda: times,
                            frames=frames, impacts=[], termination="DURATION_REACHED")
    cable = CableLptModel(LptAttachment("bar", (0, 0), 0, "BAR_COLLAR"), anchor_m=(0, 0, 0))
    return world, cable, {"world_kind": "IDEAL_VERTICAL_GUIDE_TWO_MASS_APPARATUS",
        "body_mass_kg": p.get("body_mass_kg", 75), "bar_mass_kg": p.get("load_mass_kg", 20), "guide_vertical_reaction_n": 0,
        "events": {"mathematical_motion_start_s": baseline, "bottom_s": baseline + tempo, "motion_end_s": baseline + 2 * tempo},
        "attachment_vertical_m": [motion(t)[0] for t in times],
        "vertical_momentum_kg_m_s": [mass * motion(t)[1] for t in times],
        "mass_kg": mass, "gravity_m_s2": 9.81}


def build_case(protocol, parameters=None):
    p = validate_parameters(parameters or {}, protocol)
    if protocol == "guided_vertical":
        return guided_vertical_world(p)
    anthropometry = Anthropometry(body_mass_kg=p.get("body_mass_kg", 75), stature_m=p.get("stature_m", 1.78))
    baseline, tempo = p.get("baseline_s", 1.2), p.get("tempo_scale", 1.0)
    if protocol == "back_squat":
        world = build_back_squat(SquatProtocol(repetitions=1, start_hold_s=baseline,
            load_mass_kg=p.get("load_mass_kg", 60), descent_time_s=1.2 * tempo,
            ascent_time_s=tempo, ascent_effort=p.get("effort", 1), bar_friction=p.get("friction", 0.8)),
            anthropometry=anthropometry).run()
        cable = squat_lpt(world)
    elif protocol == "deadlift":
        world = build_deadlift(DeadliftProtocol(setup_hold_s=baseline,
            load_mass_kg=p.get("load_mass_kg", 100), lift_s=1.4 * tempo, lower_s=1.6 * tempo,
            bar_floor_friction=p.get("friction", 0.5)), anthropometry=anthropometry).run()
        cable = deadlift_lpt(world)
    elif protocol in ("cmj", "sj", "loaded_cmj", "loaded_jump_squat"):
        # Retain the builder's subtype-specific posture hypotheses.
        scenario = build_jump(protocol.upper(), anthropometry=anthropometry,
            load_mass_kg=p.get("load_mass_kg"), duration_s=2.6 + baseline - 0.3 + 0.4 * (tempo - 1))
        strategy = replace(scenario.strategy, command_time_s=baseline,
            descent_time_s=0.4 * tempo, ankle_effort=p.get("effort", scenario.strategy.ankle_effort))
        world = build_jump(protocol.upper(), anthropometry=anthropometry,
            load_mass_kg=p.get("load_mass_kg"), strategy=strategy, duration_s=scenario.duration_s).run()
        cable = default_lpt(world)
    elif protocol == "bench_press":
        world = build_bench_press(BenchProtocol(repetitions=1, setup_hold_s=baseline,
            load_mass_kg=p.get("load_mass_kg", 60), lower_s=1.2 * tempo, press_s=1.5 * tempo),
            anthropometry=anthropometry).run()
        cable = bench_lpt(world)
    elif protocol in ("clean_pull", "snatch_pull", "power_clean_pull"):
        world = build_pull(PullProtocol(variant="SNATCH_PULL" if protocol == "snatch_pull" else "CLEAN_PULL",
            load_mass_kg=p.get("load_mass_kg", 60 if protocol == "power_clean_pull" else 70),
            setup_hold_s=baseline, pull_s=tempo, ankle_effort=p.get("effort", 1), leg_effort=p.get("effort", 1)),
            anthropometry=anthropometry).run()
        cable = pull_lpt(world)
    else:
        raise ValueError("unsupported protocol")
    # References are physical coordinates/contact modes, not controller stages.
    return world, cable, {"world_kind": "PLANAR_MULTIBODY_RESEARCH_FIXTURE",
        "attachment_vertical_m": [attachment_world(world, s, cable.attachment)[2] for s in world.states()],
        "mass_kg": world.model.total_mass_kg(), "gravity_m_s2": world.model.gravity_m_s2,
        "vertical_momentum_kg_m_s": [world.model.system.linear_momentum(s)[1] for s in world.states()],
        "contacts": [{key: contact.mode.value for key, contact in s.contacts.items()} for s in world.states()],
        "contact_specs": [asdict(c) for c in world.model.system.contacts],
        "states": [asdict(s) for s in world.states()],
        "physical_events": [asdict(e) for e in world.events],
        "impulses_N_s": [asdict(impact) for impact in world.impacts]}


def timebase(clock, name):
    return {"clock_id": name, "authority": "TIMESTAMP", "nominal_sampling_rate_hz": 1 / clock.sample_period_s,
            "time_origin": "DEVICE_NATIVE_ORIGIN", "regularity": "DECLARED_REGULAR",
            "missing_sample_policy": "GAPS_PRESERVED_REPORTED", "duplicate_sample_policy": "REFUSE",
            "non_monotonic_sample_policy": "REFUSE"}


def trigger_evidence(clocks, end, hashes, seed=0):
    # A simulated external pulse generator fans four edges into both devices'
    # hardware timestamp capture inputs. Clock maps will be FIT from these
    # source-visible observations by the integration reader, not read from TRUTH.
    edges = [0.0, end / 3, 2 * end / 3, end]
    rng = random.Random(seed + 419)
    return {"contract_id": "BSMB-FOUR-EDGE-CAPTURE-1.0",
        "mechanism": "SIMULATED_EXTERNAL_HARDWARE_CAPTURE",
        "reference_clock_id": "external-pulse-clock", "reference_edge_times_s": edges,
        "fit_edge_indices": [0, 3], "held_out_edge_indices": [1, 2],
        "edge_ids": ["pulse-start", "pulse-check-1", "pulse-check-2", "pulse-end"], "capture_error_bound_s": 2e-6,
        "capture_accuracy_status": "ASSUMED_BOUNDED_MANUFACTURED_NOT_EMPIRICAL",
        "recording_hashes": hashes,
        "devices": {name: {"clock_id": name, "edge_native_times_s": [
            clock.offset_s + clock.rate_scale * t + rng.uniform(-2e-6, 2e-6) for t in edges]}
            for name, clock in clocks.items()}}


def export_case(root: Path, *, protocol, seed, case_id, parameters=None):
    root = Path(root)
    p = validate_parameters(parameters or {}, protocol)
    source = root / "SOURCE" / case_id
    truth = root / "TRUTH" / case_id
    source.mkdir(parents=True, exist_ok=False)
    world, cable, reference = build_case(protocol, p)
    profile = LEGACY_MIXED_RATE_V1 if p.get("acquisition_profile") == "legacy_mixed" else DUAL_1KHZ_V1
    force_model, cable = profile.instruments(cable)
    # Constant drift and separately configured sample jitter; capture uncertainty
    # describes the external edge input, not sample jitter or empirical accuracy.
    force_model = replace(force_model, clock=replace(force_model.clock,
        offset_s=p.get("force_offset_s", force_model.clock.offset_s),
        drift_ppm=p.get("force_drift_ppm", force_model.clock.drift_ppm),
        jitter_sd_s=p.get("jitter_sd_s", 0)), seed=seed + 11,
        filter_cutoff_hz=p.get("force_cutoff_hz", force_model.filter_cutoff_hz),
        adc_range=(2500, p.get("force_adc_range_n", 5000), 1500))
    def channel(c):
        return replace(c, gain=tuple(tuple(v * p.get("force_gain_scale", 1) for v in row) for row in c.gain),
            noise_sd=tuple(v * p.get("force_noise_scale", 1) for v in c.noise_sd))
    force_model = replace(force_model, left=channel(force_model.left), right=channel(force_model.right))
    cable = replace(cable, clock=replace(cable.clock,
        offset_s=p.get("lpt_offset_s", cable.clock.offset_s),
        drift_ppm=p.get("lpt_drift_ppm", cable.clock.drift_ppm),
        jitter_sd_s=p.get("jitter_sd_s", 0)), seed=seed + 12,
        encoder_resolution_m=p.get("encoder_resolution_m", cable.encoder_resolution_m),
        max_retraction_speed_m_s=p.get("retraction_speed_m_s", cable.max_retraction_speed_m_s))
    angle_model = InclinationModel(seed=seed + 93, error_bound_rad=p.get("angle_error_bound_rad", 0.0005),
                                   resolution_rad=p.get("angle_resolution_rad", 0.00001))
    force = observe_dual_plates(world, force_model)
    observed = observe_cable_angle(world, cable, angle_model)
    def recorded(samples, name):
        drop = set(p.get(f"{name}_drop_indices", []))
        if drop and max(drop) >= len(samples):
            raise ValueError(f"DROP_INDEX_OUT_OF_RECORD:{name}")
        return tuple(s for s in samples if s.index not in drop and
            p.get(f"{name}_start_s", 0) <= s.index * (force_model.clock.sample_period_s if name == "force" else cable.clock.sample_period_s)
            <= p.get(f"{name}_end_s", float("inf")))
    force_samples, lpt_samples = recorded(force.samples, "force"), recorded(observed.samples, "lpt")
    if len(force_samples) < 3 or len(lpt_samples) < 3:
        raise ValueError("INSUFFICIENT_ACQUISITION_COVERAGE")
    # Keep the encoder zero at acquisition start even when the exported recording
    # is cropped. Its observed absolute reference is source-visible below.
    geometry_refusal = None
    try:
        all_vertical = reconstruct_vertical(observed.samples, length_resolution_m=cable.encoder_resolution_m,
            angle_error_bound_rad=angle_model.error_bound_rad, angle_resolution_rad=angle_model.resolution_rad)
        vertical = [all_vertical[s.index] for s in lpt_samples]
    except ValueError as exc:
        if str(exc) != "SLACK_CABLE_VERTICAL_RECONSTRUCTION_REFUSED":
            raise
        vertical, geometry_refusal = None, str(exc)
    write_csv(source / "streams/force.csv", ("sample_index", "time_s", "force_left_N", "force_right_N"),
        ((s.index, s.native_time_s, s.fz_left_n, s.fz_right_n) for s in force_samples))
    write_csv(source / "streams/lpt.csv", ("sample_index", "time_s", "displacement_m"),
        ((s.index, s.native_time_s, s.displacement_m) for s in lpt_samples))
    hashes = {name: sha(source / f"streams/{name}.csv") for name in ("force", "lpt")}
    clocks = {"force": force_model.clock, "lpt": cable.clock}
    sync = trigger_evidence(clocks, world.times()[-1], hashes, seed)
    mappings = {}
    for name, device in sync["devices"].items():
        first, last = device["edge_native_times_s"][0], device["edge_native_times_s"][-1]
        scale = world.times()[-1] / (last - first)
        mappings[name] = {"offset_s": -first * scale, "drift_scale": scale,
            "evidence_reference": "operator_notes:four-edge-capture"}
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
        "timebase": timebase(cable.clock, "lpt"), "device": identity, "attachment_point": cable.attachment.label,
        "cable_direction": "CONSTRAINED_VERTICAL_GUIDE" if protocol == "guided_vertical" else "VARIABLE_INCLINATION_OBSERVED",
        "angle_correction": "NONE", "zero_reference": "FIRST_RECORDED_ENCODER_SAMPLE",
        "calibration": calibration, "processing": {"state": "RAW_UNFILTERED_DECLARED"}}
    document = {"profile_id": "P2B-OPEN-DUAL-VFORCE-LPT" if protocol == "guided_vertical" else SOURCE_PROFILE,
        "profile_version": "1.0", "bundle_id": case_id, "acquisition_id": case_id,
        "session_id": "BSMB-DEVELOPMENT", "trial_id": case_id, "created_at_utc": STAMP,
        "force_stream": force_stream, "lpt_stream": lpt_stream,
        "synchronization": {"authority_class": "MANUFACTURED_ANALYTICAL_AFFINE_MAPPING",
            "evidence_reference": "operator_notes:four-edge-capture", "stream_mappings": mappings},
        "operator_notes": json.dumps({"claim_ceilings": list(CLAIM_CEILINGS), "sync_capture": sync,
            "generator_contract_id": "BSMB-GENERATOR-1.0",
            "rights": {"license": "Apache-2.0", "basis": "PROJECT_AUTHORED_MANUFACTURED_ONLY",
                "external_participant_data": False},
            "apparatus": reference["world_kind"], "acquisition_profile": profile.profile_id,
            "force_front_end": {k: v for k, v in asdict(force_model).items() if k != "clock"},
            "lpt_encoder_resolution_m": cable.encoder_resolution_m,
            "force_saturated_sample_indices": [s.index for s in force_samples if s.saturated],
            "observed_encoder_zero_m": observed.samples[0].encoder_length_m,
            "observed_inclination_zero_rad": observed.samples[0].inclination_rad,
            "acquisition_jitter_policy": {"distribution": "GAUSSIAN_UNBOUNDED" if p.get("jitter_sd_s", 0) else "ZERO",
                "standard_deviation_s": p.get("jitter_sd_s", 0), "finite_error_bound_s": None if p.get("jitter_sd_s", 0) else 0},
            "raw_definition": "ADC/encoder output; acquisition analog antialias stage retained; no post-acquisition software filtering"}, sort_keys=True)}
    if protocol != "guided_vertical":
        write_csv(source / "streams/angle.csv", ("sample_index", "time_s", "encoder_length_m", "inclination_rad", "slack"),
            ((s.index, s.native_time_s, s.encoder_length_m, s.inclination_rad, int(s.slack)) for s in lpt_samples))
        if vertical is not None:
            write_csv(source / "derived/vertical.csv", ("sample_index", "time_s", "displacement_m", "lower_m", "upper_m"),
                ((s.index, s.native_time_s, *v) for s, v in zip(lpt_samples, vertical)))
        document["derived_observation"] = {"profile_id": RECONSTRUCTION_ID,
            "source_processing_state": "DERIVED_GEOMETRY_RECONSTRUCTION_V1", "quantity": "VERTICAL_ATTACHMENT_DISPLACEMENT",
            "frame_id": "LAB_VERTICAL_Z_UP", "source_geometry_profile": PROFILE_ID,
            "angle_model": asdict(angle_model), "length_resolution_m": cable.encoder_resolution_m,
            "attachment": asdict(cable.attachment), "anchor_m": cable.anchor_m,
            "path": "derived/vertical.csv" if vertical is not None else None,
            "refusal": geometry_refusal, "domain": "FIXED_ANCHOR_PLANAR_ONLY_SPATIAL_MOTION_REFUSED",
            "uncertainty": "BOUNDED_INTERVAL_NOT_STANDARD_DEVIATION"}
    seal(source, document)
    force_rng = random.Random(force_model.seed)
    force_times = [max(0, force_model.clock.true_simulation_time(i, force_rng)) for i in range(len(force.samples))]
    write_json(truth / "reference.json", {"claim_ceilings": list(CLAIM_CEILINGS), **reference,
        "reference_schema_id": "BSMB-PAIRED-TRUTH-1.0", "protocol": protocol,
        "reference_attachment": asdict(cable.attachment),
        "coordinate_semantics": "declared attachment height; HIP_BELT is not bar or COM",
        "parameters": p, "physics_times_s": world.times(), "lpt_true_sample_times_s": observed.true_times_s,
        "force_true_sample_times_s": force_times,
        "clocks": {name: asdict(c) for name, c in clocks.items()},
        "termination": world.termination, "source_manifest_sha256": sha(source / "manifest.json")})
    write_csv(truth / "support_intervals.csv", ("start_s", "end_s", "contact_id", "fx_N", "fz_N"),
        ((f.t_start_s, f.t_end_s, key, *c.force_world_n) for f in world.frames for key, c in f.contact_forces.items()))
    if protocol != "guided_vertical":
        from .phases import analyze_jump
        from .squat import analyze_squat
        from .deadlift import analyze_deadlift
        from .bench import analyze_bench
        from .olympic import analyze_pull
        analysis = (analyze_jump(world) if protocol in ("cmj", "sj", "loaded_cmj", "loaded_jump_squat") else
            analyze_squat(world) if protocol == "back_squat" else analyze_deadlift(world) if protocol == "deadlift" else
            analyze_bench(world) if protocol == "bench_press" else analyze_pull(world))
        write_json(truth / "model_events.json", {"analysis": asdict(analysis),
            "event_authority": "GENERATOR_MODEL_REFERENCE_NOT_INDEPENDENT_EVALUATOR",
            "time_resolution_s": max(f.t_end_s - f.t_start_s for f in world.frames),
            "anatomical_events_from_sensors": "NOT_ESTABLISHED"})
        from .qualification import qualify_context
        write_json(truth / "context_qualification.json", qualify_context(world))
        from .export import export_submission
        write_json(truth / "mechanics_submission.json", export_submission(world))
    with (truth / "checksums.sha256").open("x", encoding="utf-8", newline="\n") as stream:
        for path in sorted(truth.iterdir()):
            if path.name != "checksums.sha256":
                stream.write(f"{sha(path)}  {path.name}\n")
    # Independent capture authority is supplied out of band to the verifier.
    # A completely forged package cannot authenticate itself with its own hash.
    write_json(root / "DIAGNOSTICS" / case_id / "capture_authority.json",
               {**sync, "source_manifest_sha256": sha(source / "manifest.json")})
    return {"case_id": case_id, "protocol": protocol, "force_count": len(force_samples),
        "lpt_count": len(lpt_samples), "source_bytes": sum(p.stat().st_size for p in source.rglob("*") if p.is_file()),
        "truth_inventory_sha256": sha(truth / "checksums.sha256"),
        "source_manifest_sha256": sha(source / "manifest.json")}
