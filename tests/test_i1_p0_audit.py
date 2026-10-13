"""RES-481 adversarial falsifiers against the I0 code.

Each test states the physically or evidentially correct behaviour. Run against the
unmodified I0 sources these tests FAIL (recorded in runs/i1/res481/); they pass
after the P0 repairs. None of them loosens an existing I0 assertion.
"""

from dataclasses import replace
import math
import sys
import unittest

from biosyngen.mechanics import MassProperties, Phase, PhaseMode, Scenario, simulate
from biosyngen_benchmark.core import (
    BilateralForcePoint,
    BilateralForceReference,
    CableGeometryPoint,
    CableGeometryReference,
    ClaimKind,
    ClockContract,
    ForceImpulseReference,
    ForceMeasurements,
    LptMeasurements,
    MeasurementSubmission,
    Outcome,
    SupportCoverage,
    evaluate,
)


def _roundoff(scale: float, operations: int = 1) -> float:
    return 64.0 * sys.float_info.epsilon * max(1.0, abs(scale)) * operations


CLOCK = ClockContract(0.5, 0.0, 1.0, qualified=True)


def _force(values, **kwargs) -> ForceMeasurements:
    return ForceMeasurements(
        sample_indices=(0, 1, 2),
        native_times_s=(0.0, 0.5, 1.0),
        total_force_n=values,
        clock=CLOCK,
        support_coverage=kwargs.pop("coverage", SupportCoverage.TOTAL_SYSTEM),
        source_stage="SIMULATED_OBSERVATION",
        **kwargs,
    )


def _impulse_reference(tolerance_n_s: float, time_tolerance_s: float | None = None):
    # 2 kg; samples 20, 0, 20 N at 0.5 s spacing: trapezoid support impulse
    # 10 N s, gravity -19.62 N s, so the velocity change is -4.81 m/s.
    return ForceImpulseReference(
        mass_kg=2.0,
        start_simulation_time_s=0.0,
        end_simulation_time_s=1.0,
        initial_velocity_m_s=0.0,
        final_velocity_m_s=-4.81,
        other_external_impulse_n_s=-19.62,
        external_impulses_complete=True,
        tolerance_n_s=tolerance_n_s,
        time_tolerance_s=_roundoff(1.0) if time_tolerance_s is None else time_tolerance_s,
        expected_clock=CLOCK,
    )


class I0MechanicsFalsifiers(unittest.TestCase):
    """P0-M1 and P0-M2: declared flight is not checked against contact physics."""

    def test_takeoff_with_downward_com_velocity_is_refused(self) -> None:
        # Zero support force for 0.2 s gives v = -1.96 m/s at the declared
        # stance-to-flight boundary. A point mass on a rigid support cannot
        # leave the support while moving into it.
        scenario = Scenario(
            name="falsifier-downward-takeoff",
            masses=MassProperties(body_mass_kg=70.0),
            phases=(
                Phase(0.2, PhaseMode.STANCE, 0.0),
                Phase(0.1, PhaseMode.FLIGHT, 0.0),
            ),
            initial_body_com_height_m=1.0,
        )
        with self.assertRaisesRegex(ValueError, "takeoff"):
            simulate(scenario)

    def test_flight_beyond_ballistic_return_to_takeoff_height_is_refused(self) -> None:
        # The I0 demonstration world: 100 kg, 1250 N for 0.37 s, v0 = 0.05 m/s.
        # Takeoff velocity 1.0453 m/s returns the COM to takeoff height after
        # 2 v / g = 0.2131 s, but the declared flight lasts 0.41 s and ends
        # 0.396 m below takeoff height with zero ground reaction.
        scenario = Scenario(
            name="i0-demo-world",
            masses=MassProperties(body_mass_kg=78.0, bar_mass_kg=22.0, bar_offset_m=0.32),
            phases=(
                Phase(0.37, PhaseMode.STANCE, 1250.0),
                Phase(0.41, PhaseMode.FLIGHT, 0.0),
            ),
            initial_body_com_height_m=0.92,
            initial_vertical_velocity_m_s=0.05,
            gravity_m_s2=9.81,
            sample_interval_s=0.02,
        )
        with self.assertRaisesRegex(ValueError, "below takeoff"):
            simulate(scenario)

    def test_flight_within_ballistic_return_is_still_accepted(self) -> None:
        scenario = Scenario(
            name="admissible-short-flight",
            masses=MassProperties(body_mass_kg=100.0),
            phases=(
                Phase(0.37, PhaseMode.STANCE, 1250.0),
                Phase(0.20, PhaseMode.FLIGHT, 0.0),
            ),
            initial_body_com_height_m=0.92,
            initial_vertical_velocity_m_s=0.05,
            gravity_m_s2=9.81,
            sample_interval_s=0.02,
        )
        trajectory = simulate(scenario)
        takeoff_height = next(
            s.body_com_height_m for s in trajectory.samples if s.mode is PhaseMode.FLIGHT
        )
        self.assertGreaterEqual(trajectory.samples[-1].body_com_height_m, takeoff_height)


class I0EvaluatorFalsifiers(unittest.TestCase):
    """P0-E1 tolerance authority and P0-E2 circular bilateral acceptance."""

    def test_corrupted_force_rejected_under_numerical_tolerance(self) -> None:
        corrupted = _force((20.0, 20.0, 20.0))  # +20 N at the flight sample
        report = evaluate(
            MeasurementSubmission(
                candidate_id="corrupted",
                claims=(ClaimKind.FORCE_IMPULSE_BALANCE,),
                force=corrupted,
                impulse_reference=_impulse_reference(_roundoff(20.0, 3)),
            )
        )
        self.assertEqual(report.findings[0].outcome, Outcome.REJECTED)

    def test_uncorrupted_force_passes_same_reference(self) -> None:
        clean = _force((20.0, 0.0, 20.0))
        report = evaluate(
            MeasurementSubmission(
                candidate_id="clean",
                claims=(ClaimKind.FORCE_IMPULSE_BALANCE,),
                force=clean,
                impulse_reference=_impulse_reference(_roundoff(20.0, 3)),
            )
        )
        self.assertEqual(report.findings[0].outcome, Outcome.PASS)

    def test_candidate_cannot_buy_a_pass_by_widening_its_own_tolerance(self) -> None:
        corrupted = _force((20.0, 20.0, 20.0))
        report = evaluate(
            MeasurementSubmission(
                candidate_id="tolerance-inflation-attack",
                claims=(ClaimKind.FORCE_IMPULSE_BALANCE,),
                force=corrupted,
                impulse_reference=_impulse_reference(1.0e6),
            )
        )
        self.assertNotEqual(report.findings[0].outcome, Outcome.PASS)
        self.assertEqual(report.findings[0].code, "REFERENCE_TOLERANCE_EXCEEDS_EVALUATOR_POLICY")

    def test_candidate_cannot_snap_interval_with_a_wide_time_tolerance(self) -> None:
        # Interval declared as [0.2, 1.0] s. With a 0.3 s time tolerance the I0
        # evaluator snaps 0.2 s to the 0.0 s sample and integrates the wrong window.
        reference = replace(
            _impulse_reference(_roundoff(20.0, 3), time_tolerance_s=0.3),
            start_simulation_time_s=0.2,
        )
        report = evaluate(
            MeasurementSubmission(
                candidate_id="time-snap-attack",
                claims=(ClaimKind.FORCE_IMPULSE_BALANCE,),
                force=_force((20.0, 0.0, 20.0)),
                impulse_reference=reference,
            )
        )
        self.assertNotEqual(report.findings[0].outcome, Outcome.PASS)

    def test_bilateral_channels_manufactured_from_a_split_fraction_never_pass(self) -> None:
        # Two different assumed partitions of the same combined force both
        # passed in I0 because the "independent" reference was built from the
        # candidate's own split fraction. A partition is an assumption, not a
        # measurement, so the claim is not evaluable.
        for fraction in (0.5, 0.7):
            left = tuple(v * fraction for v in (20.0, 0.0, 20.0))
            right = tuple(v - l for v, l in zip((20.0, 0.0, 20.0), left))
            report = evaluate(
                MeasurementSubmission(
                    candidate_id=f"assumed-split-{fraction}",
                    claims=(ClaimKind.BILATERAL_FORCE,),
                    force=_force(
                        (20.0, 0.0, 20.0),
                        coverage=SupportCoverage.TWO_FOOT_CHANNELS,
                        left_force_n=left,
                        right_force_n=right,
                        bilateral_split_fraction=fraction,
                    ),
                    bilateral_reference=BilateralForceReference(
                        points=tuple(
                            BilateralForcePoint(t, v, l, r)
                            for t, v, l, r in zip((0.0, 0.5, 1.0), (20.0, 0.0, 20.0), left, right)
                        ),
                        tolerance_n=_roundoff(20.0, 3),
                        time_tolerance_s=_roundoff(1.0),
                        expected_clock=CLOCK,
                    ),
                )
            )
            self.assertEqual(report.findings[0].outcome, Outcome.NOT_EVALUABLE, fraction)
            self.assertEqual(
                report.findings[0].code, "BILATERAL_DERIVED_FROM_ASSUMED_PARTITION"
            )

    def test_wrong_cable_anchor_rejected_and_correct_anchor_passes(self) -> None:
        lpt = LptMeasurements(
            sample_indices=(0,),
            native_times_s=(0.0,),
            cable_length_m=(5.0,),
            cable_rate_m_s=(0.6,),
            cable_angle_rad=(math.atan2(4.0, 3.0),),
            clock=ClockContract(1.0, 0.0, 1.0, qualified=True),
            source_stage="SIMULATED_OBSERVATION",
            attachment_target="bar_attachment",
        )

        def reference(anchor):
            return CableGeometryReference(
                points=(CableGeometryPoint(0.0, anchor, (3.0, 4.0), (1.0, 0.0)),),
                length_tolerance_m=_roundoff(5.0),
                rate_tolerance_m_s=_roundoff(1.0),
                angle_tolerance_rad=_roundoff(1.0),
                time_tolerance_s=_roundoff(1.0),
                expected_clock=ClockContract(1.0, 0.0, 1.0, qualified=True),
            )

        for anchor, expected in (((0.0, 0.5), Outcome.REJECTED), ((0.0, 0.0), Outcome.PASS)):
            report = evaluate(
                MeasurementSubmission(
                    candidate_id="anchor",
                    claims=(ClaimKind.CABLE_GEOMETRY,),
                    lpt=lpt,
                    cable_reference=reference(anchor),
                )
            )
            self.assertEqual(report.findings[0].outcome, expected, anchor)


if __name__ == "__main__":
    unittest.main()
