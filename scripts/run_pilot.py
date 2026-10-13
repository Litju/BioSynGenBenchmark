"""Run the deterministic I0 mechanics, observation, and benchmark vertical slice."""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import hashlib
import html
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

from biosyngen.mechanics import MassProperties, Phase, PhaseMode, Scenario, simulate
from biosyngen.observations import (
    AttachmentTarget,
    CableLptConfig,
    ClockModel,
    ForcePlateConfig,
    observe_cable_lpt,
    observe_force_plate,
)
from biosyngen_benchmark.core import (
    CableGeometryPoint,
    CableGeometryReference,
    ClaimKind,
    ClockContract,
    ForceImpulseReference,
    ForceMeasurements,
    FlightReference,
    LptMeasurements,
    MeasurementSubmission,
    SupportCoverage,
    evaluate,
)


ROOT = Path(__file__).resolve().parents[1]
RUN_ROOT = ROOT / "runs" / "overnight" / "i0-queue-20261012"
FORMAT_VERSION = "I0-PILOT-2026-10-12-v1"
SEED = 20261012
CLAIM_CEILING = [
    "NUMERICAL_VERIFICATION_ONLY",
    "SIMULATED_MEASUREMENT_ONLY",
    "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED",
    "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED",
]

COLORS = {
    "blue": "#0072B2",
    "orange": "#D55E00",
    "green": "#009E73",
    "purple": "#CC79A7",
    "gold": "#E69F00",
    "black": "#222222",
    "grid": "#DDE2E7",
    "text": "#20262C",
    "stance": "#D7ECF7",
    "flight": "#FFF1C4",
}


def scenario_config() -> Scenario:
    return Scenario(
        name="manufactured_vertical_controlled_stance_then_flight",
        masses=MassProperties(body_mass_kg=78.0, bar_mass_kg=22.0, bar_offset_m=0.32),
        phases=(
            Phase(0.37, PhaseMode.STANCE, 1250.0),
            Phase(0.41, PhaseMode.FLIGHT, 0.0),
        ),
        initial_body_com_height_m=0.92,
        initial_vertical_velocity_m_s=0.05,
        gravity_m_s2=9.81,
        sample_interval_s=0.02,
        # I1/RES-481: I0 world kept as an arithmetic fixture; its flight
        # outlasts ballistic return, so it is explicitly labeled legacy.
        legacy_i0_unchecked_flight=True,
    )


def plot_observation_configs() -> tuple[ForcePlateConfig, CableLptConfig]:
    return (
        ForcePlateConfig(
            clock=ClockModel(0.04, offset_s=0.003, drift_ppm=50.0),
            offset_n=-1.0,
            noise_std_n=15.0,
            quantization_n=5.0,
            saturation_n=2000.0,
            seed=SEED,
            missing_indices=frozenset({3}),
        ),
        CableLptConfig(
            clock=ClockModel(0.025, offset_s=-0.007, drift_ppm=-35.0),
            anchor_x_m=-0.2,
            anchor_z_m=0.1,
            attachment_x_m=0.4,
            attachment_target=AttachmentTarget.BAR_ATTACHMENT,
            length_offset_m=0.001,
            length_noise_std_m=0.003,
            length_quantization_m=0.001,
            length_saturation_m=2.5,
            rate_offset_m_s=-0.001,
            rate_noise_std_m_s=0.015,
            rate_quantization_m_s=0.005,
            rate_saturation_m_s=5.0,
            angle_noise_std_rad=0.004,
            angle_quantization_rad=0.002,
            seed=SEED + 1,
            missing_indices=frozenset({4}),
        ),
    )


def clean_evaluation_configs() -> tuple[ForcePlateConfig, CableLptConfig]:
    return (
        ForcePlateConfig(clock=ClockModel(0.2)),
        CableLptConfig(
            clock=ClockModel(0.1),
            anchor_x_m=-0.2,
            anchor_z_m=0.1,
            attachment_x_m=0.4,
            attachment_target=AttachmentTarget.BAR_ATTACHMENT,
        ),
    )


def independent_state_at(scenario: Scenario, time_s: float) -> tuple[float, float, float, float]:
    """Closed-form reference equations independent of the mechanics module."""
    if not 0.0 <= time_s <= sum(phase.duration_s for phase in scenario.phases):
        raise ValueError("reference time is outside the scenario")
    body_height_m = scenario.initial_body_com_height_m
    body_velocity_m_s = scenario.initial_vertical_velocity_m_s
    elapsed_s = 0.0
    total_mass_kg = scenario.masses.body_mass_kg + (
        scenario.masses.bar_mass_kg or 0.0
    )
    for phase in scenario.phases:
        duration_in_phase_s = min(max(time_s - elapsed_s, 0.0), phase.duration_s)
        force_n = phase.normal_force_n if phase.mode is PhaseMode.STANCE else 0.0
        acceleration_m_s2 = force_n / total_mass_kg - scenario.gravity_m_s2
        body_height_m += (
            body_velocity_m_s * duration_in_phase_s
            + 0.5 * acceleration_m_s2 * duration_in_phase_s**2
        )
        body_velocity_m_s += acceleration_m_s2 * duration_in_phase_s
        elapsed_s += phase.duration_s
        if time_s <= elapsed_s:
            break
    bar_offset_m = scenario.masses.bar_offset_m if scenario.masses.bar_mass_kg else 0.0
    bar_height_m = body_height_m + bar_offset_m
    bar_velocity_m_s = body_velocity_m_s
    return body_height_m, body_velocity_m_s, bar_height_m, bar_velocity_m_s


def _float_bound(scale: float, operations: int) -> float:
    return 64.0 * sys.float_info.epsilon * max(1.0, abs(scale)) * max(1, operations)


def _clock_contract(config_clock: ClockModel) -> ClockContract:
    return ClockContract(
        sample_period_native_s=config_clock.sample_period_s,
        offset_native_s=config_clock.offset_s,
        rate_scale=config_clock.rate_scale,
        qualified=True,
    )


def _public_force(series, config: ForcePlateConfig) -> ForceMeasurements:
    return ForceMeasurements(
        sample_indices=tuple(sample.sample_index for sample in series.samples),
        native_times_s=tuple(sample.native_time_s for sample in series.samples),
        total_force_n=tuple(sample.total_vertical_force_n for sample in series.samples),
        clock=_clock_contract(config.clock),
        support_coverage=SupportCoverage.TOTAL_SYSTEM,
        source_stage=series.metadata.source_stage,
        units=series.metadata.units,
        frame=series.metadata.frame,
        positive_direction=series.metadata.positive_direction,
    )


def _public_lpt(series, config: CableLptConfig) -> LptMeasurements:
    return LptMeasurements(
        sample_indices=tuple(sample.sample_index for sample in series.samples),
        native_times_s=tuple(sample.native_time_s for sample in series.samples),
        cable_length_m=tuple(sample.cable_length_m for sample in series.samples),
        cable_rate_m_s=tuple(sample.cable_rate_m_s for sample in series.samples),
        cable_angle_rad=tuple(sample.cable_angle_rad for sample in series.samples),
        clock=_clock_contract(config.clock),
        source_stage=series.metadata.source_stage,
        attachment_target=config.attachment_target.value,
        length_units="m",
        rate_units="m/s",
        angle_units="rad",
    )


def _make_evaluator_inputs(
    scenario: Scenario,
    force_series,
    force_config: ForcePlateConfig,
    lpt_series,
    lpt_config: CableLptConfig,
) -> tuple[MeasurementSubmission, dict[str, object]]:
    public_force = _public_force(force_series, force_config)
    public_lpt = _public_lpt(lpt_series, lpt_config)
    start_time_s = 0.0
    end_time_s = 0.2
    start_state = independent_state_at(scenario, start_time_s)
    end_state = independent_state_at(scenario, end_time_s)
    mass_kg = scenario.masses.body_mass_kg + (
        scenario.masses.bar_mass_kg or 0.0
    )
    other_impulse_n_s = -mass_kg * scenario.gravity_m_s2 * (
        end_time_s - start_time_s
    )
    impulse_reference = ForceImpulseReference(
        mass_kg=mass_kg,
        start_simulation_time_s=start_time_s,
        end_simulation_time_s=end_time_s,
        initial_velocity_m_s=start_state[1],
        final_velocity_m_s=end_state[1],
        other_external_impulse_n_s=other_impulse_n_s,
        external_impulses_complete=True,
        tolerance_n_s=_float_bound(250.0, 8),
        time_tolerance_s=_float_bound(end_time_s, 4),
        expected_clock=public_force.clock,
    )
    flight_times_s = (0.4, 0.6)
    flight_reference = FlightReference(
        simulation_times_s=flight_times_s,
        expected_force_n=0.0,
        tolerance_n=_float_bound(1250.0, 8),
        time_tolerance_s=_float_bound(max(flight_times_s), 4),
        expected_clock=public_force.clock,
    )
    lpt_points: list[CableGeometryPoint] = []
    anchor = (lpt_config.anchor_x_m, lpt_config.anchor_z_m)
    for sample in lpt_series.samples:
        simulation_time_s = lpt_config.clock.simulation_time(sample.native_time_s)
        _, _, bar_height_m, bar_velocity_m_s = independent_state_at(
            scenario, simulation_time_s
        )
        lpt_points.append(
            CableGeometryPoint(
                simulation_time_s=simulation_time_s,
                anchor_xz_m=anchor,
                attachment_xz_m=(lpt_config.attachment_x_m, bar_height_m),
                attachment_velocity_xz_m_s=(0.0, bar_velocity_m_s),
            )
        )
    geometry_scale = max(
        1.0,
        max(math.hypot(point.attachment_xz_m[0] - point.anchor_xz_m[0],
                       point.attachment_xz_m[1] - point.anchor_xz_m[1])
            for point in lpt_points),
    )
    rate_scale = max(
        1.0,
        max(abs(point.attachment_velocity_xz_m_s[1]) for point in lpt_points),
    )
    cable_reference = CableGeometryReference(
        points=tuple(lpt_points),
        length_tolerance_m=_float_bound(geometry_scale, len(lpt_points)),
        rate_tolerance_m_s=_float_bound(rate_scale, len(lpt_points)),
        angle_tolerance_rad=_float_bound(math.pi, len(lpt_points)),
        time_tolerance_s=_float_bound(scenario.phases[-1].duration_s, 4),
        expected_clock=public_lpt.clock,
    )
    submission = MeasurementSubmission(
        candidate_id="i0-manufactured-public-observations",
        claims=(
            ClaimKind.FORCE_IMPULSE_BALANCE,
            ClaimKind.BALLISTIC_FLIGHT,
            ClaimKind.CABLE_GEOMETRY,
            ClaimKind.BIOLOGICAL_VALIDITY,
            ClaimKind.REAL_PAIRED_FORCE_LPT,
        ),
        force=public_force,
        lpt=public_lpt,
        impulse_reference=impulse_reference,
        flight_reference=flight_reference,
        cable_reference=cable_reference,
    )
    reference_data = {
        "force_impulse": asdict(impulse_reference),
        "flight": asdict(flight_reference),
        "cable_geometry": asdict(cable_reference),
    }
    public_data = {
        "candidate_id": submission.candidate_id,
        "claims": [claim.value for claim in submission.claims],
        "force": asdict(public_force),
        "lpt": asdict(public_lpt),
        "independent_known_answer_references": reference_data,
        "empirical_reference": None,
    }
    return submission, public_data


def _build_evaluations(submission: MeasurementSubmission):
    positive = evaluate(submission)

    assert submission.force is not None
    assert submission.lpt is not None
    assert submission.flight_reference is not None
    assert submission.impulse_reference is not None
    assert submission.cable_reference is not None

    force_values = list(submission.force.total_force_n)
    force_values[1] = float(force_values[1]) + 20.0
    inconsistent_impulse_force = replace(
        submission.force,
        total_force_n=tuple(force_values),
    )
    inconsistent_impulse = evaluate(
        replace(
            submission,
            claims=(ClaimKind.FORCE_IMPULSE_BALANCE,),
            force=inconsistent_impulse_force,
            lpt=None,
        )
    )

    force_values = list(submission.force.total_force_n)
    flight_index = submission.force.sample_indices.index(2)
    force_values[flight_index] = 30.0
    impossible_flight_force = replace(
        submission.force,
        total_force_n=tuple(force_values),
    )
    impossible_flight = evaluate(
        replace(
            submission,
            claims=(ClaimKind.BALLISTIC_FLIGHT,),
            force=impossible_flight_force,
            lpt=None,
        )
    )

    cable_rates = list(submission.lpt.cable_rate_m_s)
    cable_rates[0] = -float(cable_rates[0])
    wrong_projection_lpt = replace(
        submission.lpt,
        cable_rate_m_s=tuple(cable_rates),
    )
    wrong_projection = evaluate(
        replace(
            submission,
            claims=(ClaimKind.CABLE_GEOMETRY,),
            force=None,
            lpt=wrong_projection_lpt,
        )
    )

    partial_force = replace(
        submission.force,
        support_coverage=SupportCoverage.PARTIAL,
        left_force_n=None,
        right_force_n=None,
        bilateral_split_fraction=None,
    )
    missing_evidence = evaluate(
        replace(
            submission,
            candidate_id="i0-incomplete-evidence",
            claims=(
                ClaimKind.FORCE_IMPULSE_BALANCE,
                ClaimKind.CABLE_GEOMETRY,
                ClaimKind.BILATERAL_FORCE,
                ClaimKind.BIOLOGICAL_VALIDITY,
                ClaimKind.REAL_PAIRED_FORCE_LPT,
            ),
            force=partial_force,
            cable_reference=None,
        )
    )
    return {
        "positive_known_answer": positive,
        "inconsistent_impulse_control": inconsistent_impulse,
        "impossible_flight_force_control": impossible_flight,
        "incorrect_cable_projection_control": wrong_projection,
        "incomplete_evidence": missing_evidence,
    }


def _scenario_manifest(scenario: Scenario) -> dict[str, object]:
    return {
        "name": scenario.name,
        "body_mass_kg": scenario.masses.body_mass_kg,
        "bar_mass_kg": scenario.masses.bar_mass_kg,
        "bar_offset_m": scenario.masses.bar_offset_m,
        "total_mass_kg": scenario.masses.total_mass_kg,
        "gravity_m_s2": scenario.gravity_m_s2,
        "initial_body_com_height_m": scenario.initial_body_com_height_m,
        "initial_vertical_velocity_m_s": scenario.initial_vertical_velocity_m_s,
        "sample_interval_s": scenario.sample_interval_s,
        "phases": [
            {
                "duration_s": phase.duration_s,
                "mode": phase.mode.value,
                "prescribed_normal_force_n": phase.normal_force_n,
            }
            for phase in scenario.phases
        ],
    }


def build_run_artifacts() -> dict[str, bytes]:
    scenario = scenario_config()
    trajectory = simulate(scenario)
    plot_force_config, plot_lpt_config = plot_observation_configs()
    force_plot_series = observe_force_plate(trajectory, plot_force_config)
    lpt_plot_series = observe_cable_lpt(trajectory, plot_lpt_config)
    clean_force_config, clean_lpt_config = clean_evaluation_configs()
    clean_force_series = observe_force_plate(trajectory, clean_force_config)
    clean_lpt_series = observe_cable_lpt(trajectory, clean_lpt_config)
    submission, public_evaluator_input = _make_evaluator_inputs(
        scenario,
        clean_force_series,
        clean_force_config,
        clean_lpt_series,
        clean_lpt_config,
    )
    evaluation_reports = _build_evaluations(submission)
    figure_svg = render_pilot_figure(
        scenario,
        trajectory,
        force_plot_series,
        plot_force_config,
        lpt_plot_series,
        plot_lpt_config,
    )

    pilot_data = {
        "format_version": FORMAT_VERSION,
        "evidence_class": "MANUFACTURED_AND_SIMULATED_ONLY",
        "claim_ceiling": CLAIM_CEILING,
        "scenario": _scenario_manifest(scenario),
        "latent_mechanics": {
            "state_samples": [
                {
                    **asdict(sample),
                    "mode": sample.mode.value,
                }
                for sample in trajectory.samples
            ],
            "transitions": [
                {
                    "time_s": event.time_s,
                    "from_mode": event.from_mode.value,
                    "to_mode": event.to_mode.value,
                }
                for event in trajectory.transitions
            ],
        },
        "simulated_observations_for_figure": {
            "force_clock": asdict(force_plot_series.clock),
            "force_metadata": asdict(force_plot_series.metadata),
            "force_samples": [asdict(sample) for sample in force_plot_series.samples],
            "lpt_clock": asdict(lpt_plot_series.clock),
            "lpt_metadata": asdict(lpt_plot_series.metadata),
            "lpt_samples": [asdict(sample) for sample in lpt_plot_series.samples],
            "error_assumptions": {
                "seed": SEED,
                "force_offset_n": plot_force_config.offset_n,
                "force_noise_std_n": plot_force_config.noise_std_n,
                "force_quantization_n": plot_force_config.quantization_n,
                "force_missing_indices": sorted(plot_force_config.missing_indices),
                "lpt_length_offset_m": plot_lpt_config.length_offset_m,
                "lpt_length_noise_std_m": plot_lpt_config.length_noise_std_m,
                "lpt_length_quantization_m": plot_lpt_config.length_quantization_m,
                "lpt_rate_noise_std_m_s": plot_lpt_config.rate_noise_std_m_s,
                "lpt_angle_noise_std_rad": plot_lpt_config.angle_noise_std_rad,
                "lpt_missing_indices": sorted(plot_lpt_config.missing_indices),
                "calibration_status": "assumed synthetic parameters, not empirically calibrated",
            },
        },
        "evaluator_input_separate_from_latent_state": public_evaluator_input,
    }
    reports = {
        name: asdict(report)
        for name, report in evaluation_reports.items()
    }
    report_markdown = _pilot_report_markdown(
        scenario,
        evaluation_reports,
        plot_force_config,
        plot_lpt_config,
    )
    base_artifacts = {
        "pilot-data.json": _json_bytes(pilot_data),
        "evaluator-input.json": _json_bytes(public_evaluator_input),
        "evaluation-report.json": _json_bytes(
            {
                "format_version": FORMAT_VERSION,
                "claim_ceiling": CLAIM_CEILING,
                "reports": reports,
            }
        ),
        "pilot-figure.svg": figure_svg.encode("utf-8"),
        "pilot-report.md": report_markdown.encode("utf-8"),
    }
    source_paths = {
        "biosyngen/mechanics.py": ROOT / "biosyngen" / "mechanics.py",
        "biosyngen/observations.py": ROOT / "biosyngen" / "observations.py",
        "biosyngen_benchmark/core.py": ROOT / "biosyngen_benchmark" / "core.py",
        "scripts/run_pilot.py": Path(__file__).resolve(),
    }
    manifest = {
        "format_version": FORMAT_VERSION,
        "software": {"python": sys.version.split()[0]},
        "source_sha256": {
            name: hashlib.sha256(path.read_bytes()).hexdigest()
            for name, path in sorted(source_paths.items())
        },
        "scenario": _scenario_manifest(scenario),
        "random_seed": SEED,
        "external_dependencies": [],
        "scientific_claim_ceiling": CLAIM_CEILING,
        "artifact_sha256": {
            name: hashlib.sha256(content).hexdigest()
            for name, content in sorted(base_artifacts.items())
        },
        "determinism": "No timestamps, machine paths, or run identifiers are included in scientific artifacts.",
    }
    return {**base_artifacts, "manifest.json": _json_bytes(manifest)}


def _pilot_report_markdown(
    scenario: Scenario,
    reports,
    force_config: ForcePlateConfig,
    lpt_config: CableLptConfig,
) -> str:
    positive = reports["positive_known_answer"]
    adversarial = {
        name: report
        for name, report in reports.items()
        if name.endswith("_control")
    }
    missing = reports["incomplete_evidence"]
    adversarial_findings = [
        f"{finding.code}: {finding.outcome.value} — {finding.reason}"
        for report in adversarial.values()
        for finding in report.findings
    ]
    positive_codes = [
        f"{finding.claim.value}: {finding.outcome.value} ({finding.code})"
        for finding in positive.findings
    ]
    missing_codes = [
        f"{finding.claim.value}: {finding.outcome.value} ({finding.code})"
        for finding in missing.findings
    ]
    return "\n".join(
        [
            "# I0 integrated mechanics and measurement pilot",
            "",
            "Evidence class: manufactured latent mechanics plus simulated FORCE/LPT observations.",
            "",
            "## Context and assumptions",
            "",
            f"- Scenario: {scenario.name}; body mass {scenario.masses.body_mass_kg:g} kg; "
            f"optional rigidly coupled bar mass {scenario.masses.bar_mass_kg:g} kg.",
            f"- Prescribed stance force: {scenario.phases[0].normal_force_n:g} N for "
            f"{scenario.phases[0].duration_s:g} s; ideal flight follows for "
            f"{scenario.phases[1].duration_s:g} s with zero latent ground reaction.",
            f"- FORCE clock: period {force_config.clock.sample_period_s:g} s, "
            f"offset {force_config.clock.offset_s:g} s, drift {force_config.clock.drift_ppm:g} ppm.",
            f"- LPT clock: period {lpt_config.clock.sample_period_s:g} s, "
            f"offset {lpt_config.clock.offset_s:g} s, drift {lpt_config.clock.drift_ppm:g} ppm.",
            "- Force and LPT observations retain independent native time vectors. The figure maps each vector using its declared affine clock only for display.",
            f"- FORCE perturbations: offset {force_config.offset_n:g} N, Gaussian noise SD {force_config.noise_std_n:g} N, "
            f"quantization {force_config.quantization_n:g} N, saturation ±{force_config.saturation_n:g} N, "
            f"missing sample indices {sorted(force_config.missing_indices)}.",
            f"- LPT perturbations: length offset {lpt_config.length_offset_m:g} m and noise SD {lpt_config.length_noise_std_m:g} m; "
            f"rate offset {lpt_config.rate_offset_m_s:g} m/s and noise SD {lpt_config.rate_noise_std_m_s:g} m/s; "
            f"angle noise SD {lpt_config.angle_noise_std_rad:g} rad; missing sample indices {sorted(lpt_config.missing_indices)}.",
            "- Quantization and saturation settings are in pilot-data.json. All noise, offsets, quantization, saturation, and clock drift are illustrative assumptions; none are calibrated to a device.",
            "",
            "## Independent numerical report",
            "",
            "The evaluator received public clean simulated observation arrays and explicit independent known-answer references. It did not receive the latent trajectory object.",
            "",
            f"Known-answer counts: evaluated={positive.counts.evaluated}, eligible={positive.counts.eligible}, "
            f"admitted={positive.counts.admitted}, rejected={positive.counts.rejected}, "
            f"not-evaluable={positive.counts.not_evaluable}.",
            *[f"- {code}" for code in positive_codes],
            "",
            "Adversarial controls:",
            *[f"- {finding}" for finding in adversarial_findings],
            "",
            f"Missing-evidence counts: evaluated={missing.counts.evaluated}, eligible={missing.counts.eligible}, "
            f"admitted={missing.counts.admitted}, rejected={missing.counts.rejected}, "
            f"not-evaluable={missing.counts.not_evaluable}.",
            *[f"- {code}" for code in missing_codes],
            "",
            "## Claim limits",
            "",
            "NUMERICAL_VERIFICATION_ONLY; SIMULATED_MEASUREMENT_ONLY; "
            "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED; "
            "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.",
            "",
            "This run does not validate human biomechanics, instrument fidelity, vendor exports, real paired FORCE/LPT, or a universal credibility score. The vertical model has no segmented body, muscle control, horizontal dynamics, foot geometry, or landing impact.",
            "",
        ]
    )


def _json_bytes(value: object) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")


def _digest_bundle(bundle: dict[str, bytes]) -> tuple[str, dict[str, str]]:
    hashes = {name: hashlib.sha256(content).hexdigest() for name, content in sorted(bundle.items())}
    normalized = "\n".join(f"{name}:{hashes[name]}" for name in sorted(hashes)).encode("utf-8")
    return hashlib.sha256(normalized).hexdigest(), hashes


def _write_or_verify(path: Path, content: bytes, replace_existing: bool) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        existing = path.read_bytes()
        if existing == content:
            return "identical"
        if not replace_existing:
            raise FileExistsError(
                f"Existing artifact differs; inspect it and rerun with --replace-existing: {path}"
            )
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as temporary:
        temporary.write(content)
        temporary_path = Path(temporary.name)
    os.replace(temporary_path, path)
    return "written"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--replace-existing",
        action="store_true",
        help="replace differing run files after inspecting them",
    )
    args = parser.parse_args()

    first = build_run_artifacts()
    second = build_run_artifacts()
    if first != second:
        raise RuntimeError("same-seed run produced different scientific artifacts")
    digest, file_hashes = _digest_bundle(first)
    run_directories = (
        RUN_ROOT / "end-to-end-run-a",
        RUN_ROOT / "end-to-end-run-b",
    )
    actions: list[str] = []
    for directory in run_directories:
        directory.mkdir(parents=True, exist_ok=True)
        for name, content in sorted(first.items()):
            actions.append(_write_or_verify(directory / name, content, args.replace_existing))

    reproducibility = {
        "format_version": FORMAT_VERSION,
        "same_seed_same_config_same_version": first == second,
        "normalized_scientific_artifact_digest": digest,
        "files_in_digest": sorted(first),
        "artifact_sha256": file_hashes,
        "comparison_rule": "SHA-256 of sorted relative filenames and file contents; run directory name and wall-clock time excluded.",
    }
    reproducibility_path = RUN_ROOT / "reproducibility.json"
    reproducibility_action = _write_or_verify(
        reproducibility_path,
        _json_bytes(reproducibility),
        args.replace_existing,
    )
    svg = ET.fromstring(first["pilot-figure.svg"])
    if svg.tag != "{http://www.w3.org/2000/svg}svg":
        raise RuntimeError("figure artifact is not a valid SVG document")
    print("PASS: I01 mechanics, I02 observations, and I03 independent evaluation completed.")
    print(f"Same-seed normalized digest: {digest}")
    print(f"SVG title: {svg.findtext('{http://www.w3.org/2000/svg}title')}")
    print(f"Reproducibility record ({reproducibility_action}): {reproducibility_path}")
    for directory in run_directories:
        print(f"Run directory: {directory}")
    print(f"Artifacts {sorted(set(actions))}: {len(first)} files per run.")


def _svg_escape(value: str) -> str:
    return html.escape(value, quote=True)


def _svg_text(
    x: float,
    y: float,
    value: str,
    *,
    size: int = 12,
    color: str = COLORS["text"],
    anchor: str = "start",
    weight: str = "normal",
    transform: str | None = None,
) -> str:
    transform_attribute = "" if transform is None else f' transform="{transform}"'
    return (
        f'<text x="{x:.2f}" y="{y:.2f}" font-family="Arial,Helvetica,sans-serif" '
        f'font-size="{size}" fill="{color}" text-anchor="{anchor}" '
        f'font-weight="{weight}"{transform_attribute}>'
        f"{_svg_escape(value)}</text>"
    )


def _format_tick(value: float) -> str:
    if abs(value) < 5e-7:
        return "0"
    return f"{value:.2f}".rstrip("0").rstrip(".")


def _padded_range(values: list[float], include_zero: bool = False) -> tuple[float, float]:
    low = min(values)
    high = max(values)
    if include_zero:
        low = min(low, 0.0)
        high = max(high, 0.0)
    span = high - low
    if span == 0.0:
        span = max(1.0, abs(high) * 0.1)
    pad = span * 0.08
    return low - pad, high + pad


def _panel_frame(
    svg: list[str],
    letter: str,
    title: str,
    x: float,
    y: float,
    width: float,
    height: float,
    x_limits: tuple[float, float],
    y_limits: tuple[float, float],
    x_label: str,
    y_label: str,
) -> dict[str, float]:
    svg.append(
        f'<rect x="{x}" y="{y}" width="{width}" height="{height}" '
        f'rx="10" fill="#FFFFFF" stroke="#CDD4DB" stroke-width="1.2"/>'
    )
    svg.append(_svg_text(x + 16, y + 24, f"{letter}. {title}", size=16, weight="bold"))
    plot = {
        "x": x + 72,
        "y": y + 57,
        "width": width - 92,
        "height": height - 112,
        "xmin": x_limits[0],
        "xmax": x_limits[1],
        "ymin": y_limits[0],
        "ymax": y_limits[1],
    }
    for tick_index in range(5):
        fraction = tick_index / 4.0
        tick_y = plot["y"] + fraction * plot["height"]
        tick_value = plot["ymax"] - fraction * (plot["ymax"] - plot["ymin"])
        svg.append(
            f'<line x1="{plot["x"]:.2f}" y1="{tick_y:.2f}" '
            f'x2="{plot["x"] + plot["width"]:.2f}" y2="{tick_y:.2f}" '
            f'stroke="{COLORS["grid"]}" stroke-width="1"/>'
        )
        svg.append(_svg_text(plot["x"] - 8, tick_y + 4, _format_tick(tick_value), size=10, anchor="end"))
    for tick_index in range(5):
        fraction = tick_index / 4.0
        tick_x = plot["x"] + fraction * plot["width"]
        tick_value = plot["xmin"] + fraction * (plot["xmax"] - plot["xmin"])
        svg.append(
            f'<line x1="{tick_x:.2f}" y1="{plot["y"]:.2f}" '
            f'x2="{tick_x:.2f}" y2="{plot["y"] + plot["height"]:.2f}" '
            f'stroke="{COLORS["grid"]}" stroke-width="1"/>'
        )
        svg.append(
            _svg_text(
                tick_x,
                plot["y"] + plot["height"] + 18,
                _format_tick(tick_value),
                size=10,
                anchor="middle",
            )
        )
    svg.append(
        f'<line x1="{plot["x"]:.2f}" y1="{plot["y"] + plot["height"]:.2f}" '
        f'x2="{plot["x"] + plot["width"]:.2f}" y2="{plot["y"] + plot["height"]:.2f}" '
        'stroke="#35404A" stroke-width="1.2"/>'
    )
    svg.append(
        f'<line x1="{plot["x"]:.2f}" y1="{plot["y"]:.2f}" '
        f'x2="{plot["x"]:.2f}" y2="{plot["y"] + plot["height"]:.2f}" '
        'stroke="#35404A" stroke-width="1.2"/>'
    )
    svg.append(
        _svg_text(
            plot["x"] + plot["width"] / 2.0,
            y + height - 12,
            x_label,
            size=11,
            anchor="middle",
        )
    )
    svg.append(
        _svg_text(
            x + 17,
            plot["y"] + plot["height"] / 2.0,
            y_label,
            size=11,
            anchor="middle",
            transform=f"rotate(-90 {x + 17} {plot['y'] + plot['height'] / 2.0})",
        )
    )
    return plot


def _map_point(plot: dict[str, float], x: float, y: float) -> tuple[float, float]:
    x_pixel = plot["x"] + (x - plot["xmin"]) / (plot["xmax"] - plot["xmin"]) * plot["width"]
    y_pixel = plot["y"] + (plot["ymax"] - y) / (plot["ymax"] - plot["ymin"]) * plot["height"]
    return x_pixel, y_pixel


def _draw_marker(
    svg: list[str],
    x: float,
    y: float,
    color: str,
    marker: str,
    size: float = 4.0,
) -> None:
    if marker == "circle":
        svg.append(
            f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{size:.2f}" '
            f'fill="#FFFFFF" stroke="{color}" stroke-width="2"/>'
        )
    elif marker == "square":
        svg.append(
            f'<rect x="{x - size:.2f}" y="{y - size:.2f}" width="{2 * size:.2f}" '
            f'height="{2 * size:.2f}" fill="#FFFFFF" stroke="{color}" stroke-width="2"/>'
        )
    elif marker == "triangle":
        svg.append(
            f'<path d="M{x:.2f},{y - size - 1:.2f} L{x + size + 1:.2f},{y + size:.2f} '
            f'L{x - size - 1:.2f},{y + size:.2f} Z" fill="#FFFFFF" '
            f'stroke="{color}" stroke-width="2"/>'
        )
    elif marker == "x":
        svg.append(
            f'<path d="M{x - size:.2f},{y - size:.2f} L{x + size:.2f},{y + size:.2f} '
            f'M{x - size:.2f},{y + size:.2f} L{x + size:.2f},{y - size:.2f}" '
            f'fill="none" stroke="{color}" stroke-width="2.2"/>'
        )


def _draw_series(
    svg: list[str],
    plot: dict[str, float],
    points: list[tuple[float, float | None]],
    *,
    color: str,
    marker: str,
    dash: str = "",
    width: float = 2.0,
) -> None:
    segments: list[list[tuple[float, float]]] = []
    active: list[tuple[float, float]] = []
    valid_points: list[tuple[float, float]] = []
    for x_value, y_value in points:
        if y_value is None:
            if active:
                segments.append(active)
                active = []
            continue
        valid_points.append((x_value, y_value))
        active.append((x_value, y_value))
    if active:
        segments.append(active)
    dash_attribute = "" if not dash else f' stroke-dasharray="{dash}"'
    for segment in segments:
        if len(segment) < 2:
            continue
        mapped = [_map_point(plot, x_value, y_value) for x_value, y_value in segment]
        path = " ".join(
            ("M" if point_index == 0 else "L") + f"{x:.2f},{y:.2f}"
            for point_index, (x, y) in enumerate(mapped)
        )
        svg.append(
            f'<path d="{path}" fill="none" stroke="{color}" '
            f'stroke-width="{width}" stroke-linejoin="round" stroke-linecap="round"'
            f'{dash_attribute}/>'
        )
    for index, (x_value, y_value) in enumerate(valid_points):
        if index % 2 == 0 or index == len(valid_points) - 1:
            x_pixel, y_pixel = _map_point(plot, x_value, y_value)
            _draw_marker(svg, x_pixel, y_pixel, color, marker, 3.2)


def _draw_step_series(
    svg: list[str],
    plot: dict[str, float],
    points: list[tuple[float, float]],
    color: str,
) -> None:
    if len(points) < 2:
        return
    commands: list[str] = []
    first_x, first_y = _map_point(plot, *points[0])
    commands.append(f"M{first_x:.2f},{first_y:.2f}")
    for (previous_x, previous_y), (next_x, next_y) in zip(points, points[1:]):
        x_at_next, _ = _map_point(plot, next_x, previous_y)
        _, y_at_next = _map_point(plot, next_x, next_y)
        commands.extend((f"L{x_at_next:.2f},{_map_point(plot, next_x, previous_y)[1]:.2f}", f"L{x_at_next:.2f},{y_at_next:.2f}"))
    svg.append(
        f'<path d="{" ".join(commands)}" fill="none" stroke="{color}" '
        'stroke-width="2.4" stroke-linejoin="miter"/>'
    )


def _draw_legend(
    svg: list[str],
    x: float,
    y: float,
    entries: list[tuple[str, str, str, str]],
) -> None:
    cursor_x = x
    for label, color, dash, marker in entries:
        end_x = cursor_x + 24
        dash_attribute = "" if not dash else f' stroke-dasharray="{dash}"'
        svg.append(
            f'<line x1="{cursor_x:.2f}" y1="{y:.2f}" x2="{end_x:.2f}" y2="{y:.2f}" '
            f'stroke="{color}" stroke-width="2"{dash_attribute}/>'
        )
        _draw_marker(svg, cursor_x + 12, y, color, marker, 2.8)
        svg.append(_svg_text(end_x + 6, y + 4, label, size=10))
        cursor_x += 31 + len(label) * 5.8


def render_pilot_figure(
    scenario: Scenario,
    trajectory,
    force_series,
    force_config: ForcePlateConfig,
    lpt_series,
    lpt_config: CableLptConfig,
) -> str:
    width, height = 1600, 1260
    panel_width, panel_height = 740, 350
    x_columns = (40, 820)
    y_rows = (105, 487, 869)
    duration_s = trajectory.duration_s
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" role="img" aria-labelledby="figure-title figure-description">',
        '<title id="figure-title">I0 manufactured mechanics and simulated FORCE/LPT pilot</title>',
        (
            '<desc id="figure-description">Six panels show latent body, bar, and system center of mass; '
            'prescribed latent support force and simulated FORCE observations with missing samples; '
            'simulated cable length against latent bar height; cable rate against latent vertical bar '
            'velocity; independent FORCE and LPT clock offsets; and the declared stance-to-flight event. '
            'All records are manufactured or simulated and are not empirical validation.</desc>'
        ),
        '<rect width="100%" height="100%" fill="#FFFFFF"/>',
        _svg_text(40, 36, "I0 vertical mechanics + simulated observations", size=24, weight="bold"),
        _svg_text(
            40,
            60,
            "Manufactured scenario; explicit clocks; missing samples remain gaps",
            size=14,
            color="#4B5560",
        ),
    ]

    time_values = [sample.time_s for sample in trajectory.samples]
    state_series = [
        ("Body COM", [sample.body_com_height_m for sample in trajectory.samples], COLORS["blue"], "circle", ""),
        (
            "Bar attachment",
            [sample.bar_attachment_height_m for sample in trajectory.samples],
            COLORS["orange"],
            "square",
            "7 4",
        ),
        (
            "System COM",
            [sample.system_com_height_m for sample in trajectory.samples],
            COLORS["green"],
            "triangle",
            "2 4",
        ),
    ]
    state_values = [value for _, values, _, _, _ in state_series for value in values if value is not None]
    state_plot = _panel_frame(
        svg,
        "A",
        "Latent positions",
        x_columns[0],
        y_rows[0],
        panel_width,
        panel_height,
        (0.0, duration_s),
        _padded_range(state_values),
        "Simulation time (s)",
        "Height (m)",
    )
    _draw_legend(
        svg,
        x_columns[0] + 84,
        y_rows[0] + 44,
        [(name, color, dash, marker) for name, _, color, marker, dash in state_series],
    )
    for _, values, color, marker, dash in state_series:
        _draw_series(
            svg,
            state_plot,
            list(zip(time_values, values)),
            color=color,
            marker=marker,
            dash=dash,
        )
    liftoff_s = scenario.phases[0].duration_s
    _draw_event_line(svg, state_plot, liftoff_s, "liftoff")

    force_native = [sample.native_time_s for sample in force_series.samples]
    force_simulation_times = [
        force_config.clock.simulation_time(time_s) for time_s in force_native
    ]
    observed_force = [
        sample.total_vertical_force_n for sample in force_series.samples
    ]
    ideal_force_points = [
        (sample.time_s, sample.normal_force_n) for sample in trajectory.samples
    ]
    all_force_values = [
        value for _, value in ideal_force_points
    ] + [value for value in observed_force if value is not None]
    force_plot = _panel_frame(
        svg,
        "B",
        "Total vertical force",
        x_columns[1],
        y_rows[0],
        panel_width,
        panel_height,
        (0.0, duration_s),
        _padded_range(all_force_values, include_zero=True),
        "Simulation time (s; observed time uses declared clock map)",
        "Force (N)",
    )
    _draw_legend(
        svg,
        x_columns[1] + 84,
        y_rows[0] + 44,
        [
            ("Ideal latent support", COLORS["blue"], "", "none"),
            ("Simulated FORCE", COLORS["orange"], "6 3", "circle"),
            ("Missing sample", COLORS["black"], "", "x"),
        ],
    )
    _draw_step_series(svg, force_plot, ideal_force_points, COLORS["blue"])
    _draw_series(
        svg,
        force_plot,
        list(zip(force_simulation_times, observed_force)),
        color=COLORS["orange"],
        marker="circle",
        dash="6 3",
    )
    missing_force_times = [
        force_simulation_times[index]
        for index, sample in enumerate(force_series.samples)
        if sample.missing
    ]
    for time_s in missing_force_times:
        x_pixel, y_pixel = _map_point(
            force_plot,
            time_s,
            force_plot["ymin"] + 0.035 * (force_plot["ymax"] - force_plot["ymin"]),
        )
        _draw_marker(svg, x_pixel, y_pixel, COLORS["black"], "x", 4.0)
    _draw_event_line(svg, force_plot, liftoff_s, "liftoff")

    lpt_native = [sample.native_time_s for sample in lpt_series.samples]
    lpt_simulation_times = [
        lpt_config.clock.simulation_time(time_s) for time_s in lpt_native
    ]
    lpt_lengths = [sample.cable_length_m for sample in lpt_series.samples]
    lpt_rates = [sample.cable_rate_m_s for sample in lpt_series.samples]
    bar_heights = [
        independent_state_at(scenario, time_s)[2] for time_s in lpt_simulation_times
    ]
    bar_velocities = [
        independent_state_at(scenario, time_s)[3] for time_s in lpt_simulation_times
    ]
    panel_c_values = [
        value
        for value in lpt_lengths + bar_heights
        if value is not None
    ]
    panel_c = _panel_frame(
        svg,
        "C",
        "Cable length and bar height",
        x_columns[0],
        y_rows[1],
        panel_width,
        panel_height,
        (0.0, duration_s),
        _padded_range(panel_c_values),
        "Simulation time (s; LPT native clock mapped)",
        "Length / height (m)",
    )
    _draw_legend(
        svg,
        x_columns[0] + 84,
        y_rows[1] + 44,
        [
            ("LPT cable length", COLORS["purple"], "", "circle"),
            ("Latent bar height", COLORS["green"], "7 4", "square"),
            ("Missing sample", COLORS["black"], "", "x"),
        ],
    )
    _draw_series(
        svg,
        panel_c,
        list(zip(lpt_simulation_times, lpt_lengths)),
        color=COLORS["purple"],
        marker="circle",
    )
    _draw_series(
        svg,
        panel_c,
        list(zip(lpt_simulation_times, bar_heights)),
        color=COLORS["green"],
        marker="square",
        dash="7 4",
    )
    lpt_missing_times = [
        lpt_simulation_times[index]
        for index, sample in enumerate(lpt_series.samples)
        if sample.missing
    ]
    for time_s in lpt_missing_times:
        x_pixel, y_pixel = _map_point(
            panel_c,
            time_s,
            panel_c["ymin"] + 0.035 * (panel_c["ymax"] - panel_c["ymin"]),
        )
        _draw_marker(svg, x_pixel, y_pixel, COLORS["black"], "x", 4.0)
    _draw_event_line(svg, panel_c, liftoff_s, "liftoff")

    panel_d_values = [
        value for value in lpt_rates + bar_velocities if value is not None
    ]
    panel_d = _panel_frame(
        svg,
        "D",
        "Cable rate and vertical bar velocity",
        x_columns[1],
        y_rows[1],
        panel_width,
        panel_height,
        (0.0, duration_s),
        _padded_range(panel_d_values, include_zero=True),
        "Simulation time (s; LPT native clock mapped)",
        "Rate / velocity (m/s)",
    )
    _draw_legend(
        svg,
        x_columns[1] + 84,
        y_rows[1] + 44,
        [
            ("LPT cable rate", COLORS["purple"], "", "circle"),
            ("Latent bar vertical velocity", COLORS["orange"], "7 4", "triangle"),
            ("Missing sample", COLORS["black"], "", "x"),
        ],
    )
    _draw_series(
        svg,
        panel_d,
        list(zip(lpt_simulation_times, lpt_rates)),
        color=COLORS["purple"],
        marker="circle",
    )
    _draw_series(
        svg,
        panel_d,
        list(zip(lpt_simulation_times, bar_velocities)),
        color=COLORS["orange"],
        marker="triangle",
        dash="7 4",
    )
    for time_s in lpt_missing_times:
        x_pixel, y_pixel = _map_point(
            panel_d,
            time_s,
            panel_d["ymin"] + 0.035 * (panel_d["ymax"] - panel_d["ymin"]),
        )
        _draw_marker(svg, x_pixel, y_pixel, COLORS["black"], "x", 4.0)
    _draw_event_line(svg, panel_d, liftoff_s, "liftoff")

    force_clock_error_ms = [
        (native - simulation) * 1000.0
        for native, simulation in zip(force_native, force_simulation_times)
    ]
    lpt_clock_error_ms = [
        (native - simulation) * 1000.0
        for native, simulation in zip(lpt_native, lpt_simulation_times)
    ]
    clock_limits = _padded_range(force_clock_error_ms + lpt_clock_error_ms, include_zero=True)
    panel_e = _panel_frame(
        svg,
        "E",
        "Native clock offset from simulation time",
        x_columns[0],
        y_rows[2],
        panel_width,
        panel_height,
        (0.0, duration_s),
        clock_limits,
        "Simulation time (s; from declared clock maps)",
        "Native minus simulation time (ms)",
    )
    _draw_legend(
        svg,
        x_columns[0] + 84,
        y_rows[2] + 44,
        [
            ("FORCE clock", COLORS["blue"], "", "circle"),
            ("LPT clock", COLORS["orange"], "7 4", "square"),
        ],
    )
    _draw_series(
        svg,
        panel_e,
        list(zip(force_simulation_times, force_clock_error_ms)),
        color=COLORS["blue"],
        marker="circle",
    )
    _draw_series(
        svg,
        panel_e,
        list(zip(lpt_simulation_times, lpt_clock_error_ms)),
        color=COLORS["orange"],
        marker="square",
        dash="7 4",
    )
    _draw_zero_line(svg, panel_e)

    _draw_event_timeline(
        svg,
        "F",
        "Declared contact schedule and event",
        x_columns[1],
        y_rows[2],
        panel_width,
        panel_height,
        duration_s,
        liftoff_s,
    )

    svg.append(
        _svg_text(
            40,
            height - 20,
            "Manufactured latent mechanics and simulated observations only; gap marks missing data, not zero. Biological and device validity are not established.",
            size=12,
            color="#4B5560",
        )
    )
    svg.append("</svg>")
    return "\n".join(svg) + "\n"


def _draw_event_line(svg: list[str], plot: dict[str, float], time_s: float, label: str) -> None:
    x_pixel, _ = _map_point(plot, time_s, plot["ymin"])
    svg.append(
        f'<line x1="{x_pixel:.2f}" y1="{plot["y"]:.2f}" x2="{x_pixel:.2f}" '
        f'y2="{plot["y"] + plot["height"]:.2f}" stroke="{COLORS["black"]}" '
        'stroke-width="1.4" stroke-dasharray="3 4"/>'
    )
    svg.append(
        _svg_text(
            x_pixel + 4,
            plot["y"] + 12,
            f"{label} {time_s:.2f}s",
            size=9,
            color=COLORS["black"],
        )
    )


def _draw_zero_line(svg: list[str], plot: dict[str, float]) -> None:
    if plot["ymin"] < 0.0 < plot["ymax"]:
        _, y_pixel = _map_point(plot, plot["xmin"], 0.0)
        svg.append(
            f'<line x1="{plot["x"]:.2f}" y1="{y_pixel:.2f}" '
            f'x2="{plot["x"] + plot["width"]:.2f}" y2="{y_pixel:.2f}" '
            'stroke="#68737D" stroke-width="1.3" stroke-dasharray="2 4"/>'
        )


def _draw_event_timeline(
    svg: list[str],
    letter: str,
    title: str,
    x: float,
    y: float,
    width: float,
    height: float,
    duration_s: float,
    liftoff_s: float,
) -> None:
    svg.append(
        f'<rect x="{x}" y="{y}" width="{width}" height="{height}" '
        'rx="10" fill="#FFFFFF" stroke="#CDD4DB" stroke-width="1.2"/>'
    )
    svg.append(_svg_text(x + 16, y + 24, f"{letter}. {title}", size=16, weight="bold"))
    plot_x = x + 72
    plot_y = y + 62
    plot_width = width - 92
    for tick_index in range(5):
        fraction = tick_index / 4.0
        tick_x = plot_x + fraction * plot_width
        tick_time = fraction * duration_s
        svg.append(
            f'<line x1="{tick_x:.2f}" y1="{plot_y:.2f}" x2="{tick_x:.2f}" '
            f'y2="{plot_y + 205:.2f}" stroke="{COLORS["grid"]}" stroke-width="1"/>'
        )
        svg.append(
            _svg_text(tick_x, plot_y + 224, _format_tick(tick_time), size=10, anchor="middle")
        )
    liftoff_x = plot_x + liftoff_s / duration_s * plot_width
    stance_width = liftoff_s / duration_s * plot_width
    flight_width = plot_width - stance_width
    svg.append(
        f'<text x="{plot_x - 8}" y="{plot_y + 59}" font-family="Arial,Helvetica,sans-serif" '
        'font-size="11" text-anchor="end" fill="#20262C">Stance</text>'
    )
    svg.append(
        f'<text x="{plot_x - 8}" y="{plot_y + 146}" font-family="Arial,Helvetica,sans-serif" '
        'font-size="11" text-anchor="end" fill="#20262C">Flight</text>'
    )
    svg.append(
        f'<rect x="{plot_x:.2f}" y="{plot_y + 30:.2f}" width="{stance_width:.2f}" '
        'height="46" rx="5" fill="#D7ECF7" stroke="#0072B2" stroke-width="1.5"/>'
    )
    svg.append(
        f'<rect x="{liftoff_x:.2f}" y="{plot_y + 117:.2f}" width="{flight_width:.2f}" '
        'height="46" rx="5" fill="#FFF1C4" stroke="#E69F00" stroke-width="1.5"/>'
    )
    svg.append(
        _svg_text(
            plot_x + stance_width / 2.0,
            plot_y + 58,
            "prescribed N = 1250 N",
            size=11,
            anchor="middle",
        )
    )
    svg.append(
        _svg_text(
            liftoff_x + flight_width / 2.0,
            plot_y + 145,
            "ideal GRF = 0 N",
            size=11,
            anchor="middle",
        )
    )
    svg.append(
        f'<line x1="{liftoff_x:.2f}" y1="{plot_y + 14:.2f}" '
        f'x2="{liftoff_x:.2f}" y2="{plot_y + 177:.2f}" '
        f'stroke="{COLORS["black"]}" stroke-width="1.4" stroke-dasharray="3 4"/>'
    )
    svg.append(
        _svg_text(liftoff_x + 5, plot_y + 14, f"liftoff {liftoff_s:.2f}s", size=10)
    )
    svg.append(
        _svg_text(
            plot_x + plot_width / 2.0,
            y + height - 12,
            "Simulation time (s)",
            size=11,
            anchor="middle",
        )
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--replace-existing",
        action="store_true",
        help="replace differing run outputs after inspecting them",
    )
    args = parser.parse_args()

    run_a = build_run_artifacts()
    run_b = build_run_artifacts()
    if run_a != run_b:
        raise RuntimeError("same-seed full pilot runs produced different artifacts")
    digest_a, hashes_a = _digest_bundle(run_a)
    digest_b, hashes_b = _digest_bundle(run_b)
    if digest_a != digest_b or hashes_a != hashes_b:
        raise RuntimeError("normalized scientific artifact digests differ")

    run_directories = (
        RUN_ROOT / "end-to-end-run-a",
        RUN_ROOT / "end-to-end-run-b",
    )
    actions: list[str] = []
    for directory in run_directories:
        for name, content in sorted(run_a.items()):
            actions.append(
                _write_or_verify(directory / name, content, args.replace_existing)
            )

    reproduction_record = {
        "format_version": FORMAT_VERSION,
        "same_seed_same_config_same_version": True,
        "normalized_scientific_artifact_digest_run_a": digest_a,
        "normalized_scientific_artifact_digest_run_b": digest_b,
        "artifact_sha256": hashes_a,
        "comparison_rule": (
            "SHA-256 over sorted relative filenames and file contents; run directory "
            "names and wall-clock time are excluded."
        ),
    }
    reproduction_path = RUN_ROOT / "reproducibility.json"
    reproduction_action = _write_or_verify(
        reproduction_path,
        _json_bytes(reproduction_record),
        args.replace_existing,
    )

    svg_root = ET.fromstring(run_a["pilot-figure.svg"])
    if svg_root.tag != "{http://www.w3.org/2000/svg}svg":
        raise RuntimeError("pilot figure is not a valid SVG document")
    if not svg_root.findtext("{http://www.w3.org/2000/svg}title"):
        raise RuntimeError("pilot SVG has no accessible title")

    print("PASS: I01 mechanics, I02 observations, and I03 evaluator completed.")
    print(f"Same-seed normalized digest: {digest_a}")
    print(f"SVG title: {svg_root.findtext('{http://www.w3.org/2000/svg}title')}")
    print(f"Reproducibility record ({reproduction_action}): {reproduction_path}")
    for directory in run_directories:
        print(f"Run directory: {directory}")
    print(f"Artifacts {sorted(set(actions))}: {len(run_a)} files per run.")


if __name__ == "__main__":
    main()
