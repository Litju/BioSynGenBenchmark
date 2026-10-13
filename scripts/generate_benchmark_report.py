"""Run small independent known-answer, falsification, and refusal cases."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

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
OUTPUT = ROOT / "runs" / "overnight" / "i0-queue-20261012" / "res478-benchmark-report.json"


def roundoff_bound(scale: float, operations: int = 1) -> float:
    return 64.0 * sys.float_info.epsilon * max(1.0, abs(scale)) * operations


def _force(
    values: tuple[float | None, ...],
    *,
    left: tuple[float | None, ...] | None = None,
    right: tuple[float | None, ...] | None = None,
    coverage: SupportCoverage = SupportCoverage.TOTAL_SYSTEM,
    split_fraction: float | None = None,
) -> ForceMeasurements:
    return ForceMeasurements(
        sample_indices=(0, 1, 2),
        native_times_s=(0.0, 0.5, 1.0),
        total_force_n=values,
        clock=ClockContract(
            sample_period_native_s=0.5,
            offset_native_s=0.0,
            rate_scale=1.0,
            qualified=True,
        ),
        support_coverage=coverage,
        source_stage="SIMULATED_OBSERVATION",
        left_force_n=left,
        right_force_n=right,
        bilateral_split_fraction=split_fraction,
    )


def _lpt(rate_m_s: float) -> LptMeasurements:
    return LptMeasurements(
        sample_indices=(0,),
        native_times_s=(0.0,),
        cable_length_m=(5.0,),
        cable_rate_m_s=(rate_m_s,),
        cable_angle_rad=(0.9272952180016122,),
        clock=ClockContract(
            sample_period_native_s=1.0,
            offset_native_s=0.0,
            rate_scale=1.0,
            qualified=True,
        ),
        source_stage="SIMULATED_OBSERVATION",
        attachment_target="bar_attachment",
    )


def _references():
    force_clock = ClockContract(0.5, 0.0, 1.0, qualified=True)
    lpt_clock = ClockContract(1.0, 0.0, 1.0, qualified=True)
    return (
        ForceImpulseReference(
            mass_kg=2.0,
            start_simulation_time_s=0.0,
            end_simulation_time_s=1.0,
            initial_velocity_m_s=0.0,
            final_velocity_m_s=-4.81,
            other_external_impulse_n_s=-19.62,
            external_impulses_complete=True,
            tolerance_n_s=roundoff_bound(20.0, 3),
            time_tolerance_s=roundoff_bound(1.0),
            expected_clock=force_clock,
        ),
        FlightReference(
            simulation_times_s=(0.5,),
            expected_force_n=0.0,
            tolerance_n=roundoff_bound(20.0),
            time_tolerance_s=roundoff_bound(1.0),
            expected_clock=force_clock,
        ),
        CableGeometryReference(
            points=(
                CableGeometryPoint(
                    simulation_time_s=0.0,
                    anchor_xz_m=(0.0, 0.0),
                    attachment_xz_m=(3.0, 4.0),
                    attachment_velocity_xz_m_s=(1.0, 0.0),
                ),
            ),
            length_tolerance_m=roundoff_bound(5.0),
            rate_tolerance_m_s=roundoff_bound(1.0),
            angle_tolerance_rad=roundoff_bound(1.0),
            time_tolerance_s=roundoff_bound(1.0),
            expected_clock=lpt_clock,
        ),
    )


def build_report() -> dict[str, object]:
    impulse_ref, flight_ref, cable_ref = _references()
    valid = evaluate(
        MeasurementSubmission(
            candidate_id="manufactured-known-answer",
            claims=(
                ClaimKind.FORCE_IMPULSE_BALANCE,
                ClaimKind.BALLISTIC_FLIGHT,
                ClaimKind.CABLE_GEOMETRY,
                ClaimKind.BIOLOGICAL_VALIDITY,
                ClaimKind.REAL_PAIRED_FORCE_LPT,
            ),
            force=_force((20.0, 0.0, 20.0)),
            lpt=_lpt(0.6),
            impulse_reference=impulse_ref,
            flight_reference=flight_ref,
            cable_reference=cable_ref,
        )
    )
    adversarial = evaluate(
        MeasurementSubmission(
            candidate_id="manufactured-adversarial-controls",
            claims=(
                ClaimKind.FORCE_IMPULSE_BALANCE,
                ClaimKind.BALLISTIC_FLIGHT,
                ClaimKind.CABLE_GEOMETRY,
                ClaimKind.BILATERAL_FORCE,
            ),
            force=_force(
                (20.0, 5.0, 40.0),
                left=(10.0, 10.0, 10.0),
                right=(10.0, 10.0, 10.0),
            ),
            lpt=_lpt(-0.6),
            impulse_reference=impulse_ref,
            flight_reference=flight_ref,
            cable_reference=cable_ref,
        )
    )
    incomplete = evaluate(
        MeasurementSubmission(
            candidate_id="manufactured-incomplete-evidence",
            claims=(
                ClaimKind.FORCE_IMPULSE_BALANCE,
                ClaimKind.CABLE_GEOMETRY,
                ClaimKind.BIOLOGICAL_VALIDITY,
                ClaimKind.REAL_PAIRED_FORCE_LPT,
            ),
            force=_force((20.0, 0.0, 20.0), coverage=SupportCoverage.PARTIAL),
            lpt=_lpt(0.6),
            impulse_reference=impulse_ref,
            flight_reference=flight_ref,
        )
    )

    if valid.counts.admitted < 3:
        raise AssertionError("independent numerical known answers did not pass")
    if adversarial.counts.rejected < 2:
        raise AssertionError("adversarial controls did not reject at least two false claims")
    if incomplete.counts.not_evaluable < 3:
        raise AssertionError("missing evidence did not produce explicit NOT_EVALUABLE results")
    return {
        "evidence_class": "MANUFACTURED_PUBLIC_OBSERVATION_ARRAYS",
        "claim_ceiling": [
            "NUMERICAL_VERIFICATION_ONLY",
            "SIMULATED_MEASUREMENT_ONLY",
            "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED",
            "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED",
        ],
        "reports": {
            "valid_known_answer": asdict(valid),
            "adversarial_false_claims": asdict(adversarial),
            "incomplete_evidence": asdict(incomplete),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--replace-existing",
        action="store_true",
        help="replace a differing report after inspecting it",
    )
    args = parser.parse_args()
    serialized = json.dumps(build_report(), indent=2, sort_keys=True, allow_nan=False) + "\n"
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
                "Report differs from the existing file; inspect it and pass --replace-existing."
            )
    else:
        OUTPUT.write_text(serialized, encoding="utf-8")

    payload = json.loads(serialized)
    print(f"PASS: {action} independent benchmark report")
    for name, report in payload["reports"].items():
        counts = report["counts"]
        print(
            f"{name}: evaluated={counts['evaluated']} eligible={counts['eligible']} "
            f"admitted={counts['admitted']} rejected={counts['rejected']} "
            f"not_evaluable={counts['not_evaluable']}"
        )
    print(f"Artifact: {OUTPUT}")


if __name__ == "__main__":
    main()
