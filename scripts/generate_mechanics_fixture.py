"""Write one deterministic manufactured mechanics fixture and exact-error report."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from biosyngen.mechanics import MassProperties, Phase, PhaseMode, Scenario, simulate


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "runs" / "overnight" / "i0-queue-20261012" / "res476-mechanics-fixture.json"


def make_scenario() -> Scenario:
    return Scenario(
        name="manufactured_res476_constant_force_then_ballistic_flight",
        masses=MassProperties(body_mass_kg=78.0, bar_mass_kg=22.0, bar_offset_m=0.32),
        phases=(
            Phase(duration_s=0.37, mode=PhaseMode.STANCE, normal_force_n=1250.0),
            Phase(duration_s=0.41, mode=PhaseMode.FLIGHT, normal_force_n=0.0),
        ),
        initial_body_com_height_m=0.92,
        initial_vertical_velocity_m_s=0.05,
        gravity_m_s2=9.81,
        sample_interval_s=0.02,
        # I1/RES-481: I0 world kept as an arithmetic fixture; its flight
        # outlasts ballistic return, so it is explicitly labeled legacy.
        legacy_i0_unchecked_flight=True,
    )


def exact_reference_at(scenario: Scenario, time_s: float) -> tuple[float, float]:
    """Independent piecewise analytic reference; does not import solver helpers."""
    height_m = scenario.initial_body_com_height_m
    velocity_m_s = scenario.initial_vertical_velocity_m_s
    elapsed_s = 0.0

    for phase in scenario.phases:
        duration_in_phase_s = min(max(time_s - elapsed_s, 0.0), phase.duration_s)
        acceleration_m_s2 = (
            phase.normal_force_n / scenario.masses.total_mass_kg
            - scenario.gravity_m_s2
        )
        height_m += (
            velocity_m_s * duration_in_phase_s
            + 0.5 * acceleration_m_s2 * duration_in_phase_s**2
        )
        velocity_m_s += acceleration_m_s2 * duration_in_phase_s
        elapsed_s += phase.duration_s
        if time_s <= elapsed_s:
            break

    return height_m, velocity_m_s


def build_fixture() -> dict[str, object]:
    scenario = make_scenario()
    trajectory = simulate(scenario)
    references = [
        exact_reference_at(scenario, sample.time_s)
        for sample in trajectory.samples
    ]
    max_height_error_m = max(
        abs(sample.body_com_height_m - reference[0])
        for sample, reference in zip(trajectory.samples, references)
    )
    max_velocity_error_m_s = max(
        abs(sample.body_com_velocity_m_s - reference[1])
        for sample, reference in zip(trajectory.samples, references)
    )
    # The exact segment update should differ only by accumulated binary64
    # roundoff; this bound is numerical and is not a biomechanics tolerance.
    sample_count = len(trajectory.samples)
    height_scale = max(
        1.0,
        max(abs(sample.body_com_height_m) for sample in trajectory.samples),
    )
    velocity_scale = max(
        1.0,
        max(abs(sample.body_com_velocity_m_s) for sample in trajectory.samples),
    )
    height_roundoff_bound_m = (
        64.0 * sys.float_info.epsilon * sample_count * height_scale
    )
    velocity_roundoff_bound_m_s = (
        64.0 * sys.float_info.epsilon * sample_count * velocity_scale
    )
    if (
        max_height_error_m > height_roundoff_bound_m
        or max_velocity_error_m_s > velocity_roundoff_bound_m_s
    ):
        raise ArithmeticError("exact-reference comparison exceeded roundoff bound")

    return {
        "evidence_class": "MANUFACTURED_LATENT_MECHANICS_ONLY",
        "claim_ceiling": [
            "NUMERICAL_VERIFICATION_ONLY",
            "SIMULATED_MEASUREMENT_ONLY",
            "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED",
            "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED",
        ],
        "scenario": {
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
        },
        "reference_check": {
            "method": "independent piecewise constant-acceleration equations",
            "maximum_absolute_body_height_error_m": max_height_error_m,
            "maximum_absolute_body_velocity_error_m_s": max_velocity_error_m_s,
            "roundoff_bound_body_height_m": height_roundoff_bound_m,
            "roundoff_bound_body_velocity_m_s": velocity_roundoff_bound_m_s,
        },
        "transitions": [
            {
                "time_s": event.time_s,
                "from_mode": event.from_mode.value,
                "to_mode": event.to_mode.value,
            }
            for event in trajectory.transitions
        ],
        "samples": [
            {
                "time_s": sample.time_s,
                "mode": sample.mode.value,
                "normal_force_n": sample.normal_force_n,
                "body_com_height_m": sample.body_com_height_m,
                "body_com_velocity_m_s": sample.body_com_velocity_m_s,
                "body_com_acceleration_m_s2": sample.body_com_acceleration_m_s2,
                "bar_attachment_height_m": sample.bar_attachment_height_m,
                "bar_attachment_velocity_m_s": sample.bar_attachment_velocity_m_s,
                "system_com_height_m": sample.system_com_height_m,
                "system_com_velocity_m_s": sample.system_com_velocity_m_s,
                "system_com_acceleration_m_s2": sample.system_com_acceleration_m_s2,
            }
            for sample in trajectory.samples
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--replace-existing",
        action="store_true",
        help="replace a differing fixture after inspecting it",
    )
    args = parser.parse_args()
    fixture = build_fixture()
    serialized = json.dumps(fixture, indent=2, sort_keys=True, allow_nan=False) + "\n"
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    action = "wrote"
    if OUTPUT.exists():
        existing = OUTPUT.read_text(encoding="utf-8")
        if existing == serialized:
            action = "verified identical existing"
        elif args.replace_existing:
            OUTPUT.write_text(serialized, encoding="utf-8")
            action = "replaced inspected"
        else:
            raise FileExistsError(
                "Fixture differs from the existing file; inspect it and pass --replace-existing."
            )
    else:
        OUTPUT.write_text(serialized, encoding="utf-8")
    print(
        f"PASS: {action} one manufactured mechanics fixture "
        f"({len(fixture['samples'])} samples); "
        f"max |height error|={fixture['reference_check']['maximum_absolute_body_height_error_m']:.3e} m; "
        f"max |velocity error|={fixture['reference_check']['maximum_absolute_body_velocity_error_m_s']:.3e} m/s"
    )
    print(f"Artifact: {OUTPUT}")


if __name__ == "__main__":
    main()
