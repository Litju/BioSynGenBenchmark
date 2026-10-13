"""Write deterministic separate FORCE and cable-LPT observation series."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path

from biosyngen.mechanics import MassProperties, Phase, PhaseMode, Scenario, simulate
from biosyngen.observations import (
    AttachmentTarget,
    CableLptConfig,
    ClockModel,
    ForcePlateConfig,
    observe_cable_lpt,
    observe_force_plate,
)


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "runs" / "overnight" / "i0-queue-20261012" / "res477-observation-fixture.json"


def build_fixture() -> dict[str, object]:
    scenario = Scenario(
        name="manufactured_separate_force_lpt_observations",
        masses=MassProperties(body_mass_kg=78.0, bar_mass_kg=22.0, bar_offset_m=0.32),
        phases=(
            Phase(duration_s=0.37, mode=PhaseMode.STANCE, normal_force_n=1250.0),
            Phase(duration_s=0.41, mode=PhaseMode.FLIGHT, normal_force_n=0.0),
        ),
        initial_body_com_height_m=0.92,
        initial_vertical_velocity_m_s=0.05,
        gravity_m_s2=9.81,
        sample_interval_s=0.02,
    )
    trajectory = simulate(scenario)
    force_config = ForcePlateConfig(
        clock=ClockModel(sample_period_s=0.04, offset_s=0.003, drift_ppm=50.0),
        noise_std_n=0.15,
        quantization_n=0.1,
        saturation_n=2000.0,
        seed=20261012,
        missing_indices=frozenset({3}),
    )
    lpt_config = CableLptConfig(
        clock=ClockModel(sample_period_s=0.025, offset_s=-0.007, drift_ppm=-35.0),
        anchor_x_m=-0.2,
        anchor_z_m=0.1,
        attachment_x_m=0.4,
        attachment_target=AttachmentTarget.BAR_ATTACHMENT,
        length_offset_m=0.001,
        length_noise_std_m=0.001,
        length_quantization_m=0.0005,
        length_saturation_m=2.0,
        rate_noise_std_m_s=0.002,
        rate_quantization_m_s=0.001,
        rate_saturation_m_s=5.0,
        angle_noise_std_rad=0.001,
        angle_quantization_rad=0.001,
        seed=20261012,
        missing_indices=frozenset({4}),
    )
    force = observe_force_plate(trajectory, force_config)
    lpt = observe_cable_lpt(trajectory, lpt_config)
    return {
        "evidence_class": "MANUFACTURED_SIMULATED_OBSERVATIONS",
        "claim_ceiling": [
            "NUMERICAL_VERIFICATION_ONLY",
            "SIMULATED_MEASUREMENT_ONLY",
            "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED",
            "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED",
        ],
        "provenance": {
            "source_stage": "SIMULATED_OBSERVATION",
            "scenario_id": scenario.name,
            "latent_truth_in_sensor_records": False,
            "force_bilateral_split": "not declared; no left/right channels",
            "cross_device_alignment": "independent native time vectors",
        },
        "force_plate": {
            "metadata": asdict(force.metadata),
            "clock": asdict(force.clock),
            "duration_s": force.duration_s,
            "samples": [asdict(sample) for sample in force.samples],
        },
        "cable_lpt": {
            "metadata": asdict(lpt.metadata),
            "clock": asdict(lpt.clock),
            "duration_s": lpt.duration_s,
            "samples": [asdict(sample) for sample in lpt.samples],
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--replace-existing",
        action="store_true",
        help="replace a differing fixture after inspecting it",
    )
    args = parser.parse_args()
    serialized = json.dumps(build_fixture(), indent=2, sort_keys=True, allow_nan=False) + "\n"
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    action = "wrote"
    if OUTPUT.exists():
        if OUTPUT.read_text(encoding="utf-8") == serialized:
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

    payload = json.loads(serialized)
    force_count = len(payload["force_plate"]["samples"])
    lpt_count = len(payload["cable_lpt"]["samples"])
    print(
        f"PASS: {action} one manufactured observation fixture "
        f"(FORCE={force_count}, LPT={lpt_count} samples)"
    )
    print(f"Artifact: {OUTPUT}")


if __name__ == "__main__":
    main()
