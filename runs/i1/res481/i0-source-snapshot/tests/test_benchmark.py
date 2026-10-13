import ast
from dataclasses import fields, replace
from pathlib import Path
import sys
import unittest

from biosyngen_benchmark.core import (
    BilateralForcePoint,
    BilateralForceReference,
    CableGeometryPoint,
    CableGeometryReference,
    ClaimKind,
    ClockContract,
    ForceImpulseReference,
    ForceMeasurements,
    FlightReference,
    LptMeasurements,
    MeasurementSubmission,
    Outcome,
    SupportCoverage,
    evaluate,
)
import biosyngen_benchmark.core as benchmark_core


def _roundoff_bound(scale: float, operations: int = 1) -> float:
    return 64.0 * sys.float_info.epsilon * max(1.0, abs(scale)) * operations


def valid_force(
    values: tuple[float | None, ...] = (20.0, 0.0, 20.0),
    coverage: SupportCoverage = SupportCoverage.TOTAL_SYSTEM,
    left: tuple[float | None, ...] | None = None,
    right: tuple[float | None, ...] | None = None,
    partition_fraction: float | None = None,
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
        bilateral_split_fraction=partition_fraction,
    )


def valid_lpt(rate_m_s: float = 0.6) -> LptMeasurements:
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


def valid_references():
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
            tolerance_n_s=_roundoff_bound(20.0, 3),
            time_tolerance_s=_roundoff_bound(1.0),
            expected_clock=force_clock,
        ),
        FlightReference(
            simulation_times_s=(0.5,),
            expected_force_n=0.0,
            tolerance_n=_roundoff_bound(20.0),
            time_tolerance_s=_roundoff_bound(1.0),
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
            length_tolerance_m=_roundoff_bound(5.0),
            rate_tolerance_m_s=_roundoff_bound(1.0),
            angle_tolerance_rad=_roundoff_bound(1.0),
            time_tolerance_s=_roundoff_bound(1.0),
            expected_clock=lpt_clock,
        ),
    )


class IndependentEvaluatorTests(unittest.TestCase):
    def test_valid_known_answers_pass_and_missing_empirical_references_abstain(self) -> None:
        impulse, flight, cable = valid_references()
        report = evaluate(
            MeasurementSubmission(
                candidate_id="manufactured-known-answer",
                claims=(
                    ClaimKind.FORCE_IMPULSE_BALANCE,
                    ClaimKind.BALLISTIC_FLIGHT,
                    ClaimKind.CABLE_GEOMETRY,
                    ClaimKind.BIOLOGICAL_VALIDITY,
                    ClaimKind.REAL_PAIRED_FORCE_LPT,
                ),
                force=valid_force(),
                lpt=valid_lpt(),
                impulse_reference=impulse,
                flight_reference=flight,
                cable_reference=cable,
            )
        )
        outcomes = {finding.claim: finding.outcome for finding in report.findings}
        self.assertEqual(outcomes[ClaimKind.FORCE_IMPULSE_BALANCE], Outcome.PASS)
        self.assertEqual(outcomes[ClaimKind.BALLISTIC_FLIGHT], Outcome.PASS)
        self.assertEqual(outcomes[ClaimKind.CABLE_GEOMETRY], Outcome.PASS)
        self.assertEqual(outcomes[ClaimKind.BIOLOGICAL_VALIDITY], Outcome.NOT_EVALUABLE)
        self.assertEqual(outcomes[ClaimKind.REAL_PAIRED_FORCE_LPT], Outcome.NOT_EVALUABLE)
        self.assertEqual(report.counts.evaluated, 5)
        self.assertEqual(report.counts.eligible, 3)
        self.assertEqual(report.counts.admitted, 3)
        self.assertEqual(report.counts.rejected, 0)
        self.assertEqual(report.counts.not_evaluable, 2)
        self.assertFalse(hasattr(report, "score"))

    def test_independently_wrong_controls_reject_for_their_specific_reasons(self) -> None:
        impulse, flight, cable = valid_references()
        report = evaluate(
            MeasurementSubmission(
                candidate_id="manufactured-adversarial-controls",
                claims=(
                    ClaimKind.FORCE_IMPULSE_BALANCE,
                    ClaimKind.BALLISTIC_FLIGHT,
                    ClaimKind.CABLE_GEOMETRY,
                    ClaimKind.BILATERAL_FORCE,
                ),
                force=valid_force(
                    values=(20.0, 5.0, 40.0),
                    left=(10.0, 10.0, 10.0),
                    right=(10.0, 10.0, 10.0),
                ),
                lpt=valid_lpt(rate_m_s=-0.6),
                impulse_reference=impulse,
                flight_reference=flight,
                cable_reference=cable,
            )
        )
        codes = {finding.claim: finding.code for finding in report.findings}
        self.assertEqual(codes[ClaimKind.FORCE_IMPULSE_BALANCE], "INCONSISTENT_IMPULSE")
        self.assertEqual(codes[ClaimKind.BALLISTIC_FLIGHT], "IMPOSSIBLE_FLIGHT_GRF")
        self.assertEqual(codes[ClaimKind.CABLE_GEOMETRY], "LPT_PROJECTION_MISMATCH")
        self.assertEqual(
            codes[ClaimKind.BILATERAL_FORCE],
            "INVENTED_BILATERAL_FROM_COMBINED_FORCE",
        )
        self.assertTrue(all(f.outcome is Outcome.REJECTED for f in report.findings))
        self.assertEqual(report.counts.rejected, 4)

    def test_timebase_inconsistency_is_rejected_against_declared_clock(self) -> None:
        impulse, _, _ = valid_references()
        bad_timebase = ForceMeasurements(
            sample_indices=(0, 1, 2),
            native_times_s=(0.0, 0.5, 0.25),
            total_force_n=(20.0, 0.0, 20.0),
            clock=ClockContract(0.5, 0.0, 1.0, qualified=True),
            support_coverage=SupportCoverage.TOTAL_SYSTEM,
            source_stage="SIMULATED_OBSERVATION",
        )
        report = evaluate(
            MeasurementSubmission(
                candidate_id="bad-clock",
                claims=(ClaimKind.FORCE_IMPULSE_BALANCE,),
                force=bad_timebase,
                impulse_reference=impulse,
            )
        )
        self.assertEqual(report.findings[0].outcome, Outcome.REJECTED)
        self.assertEqual(report.findings[0].code, "TIMEBASE_INCONSISTENT")

    def test_unqualified_clock_is_not_evaluable(self) -> None:
        impulse, _, _ = valid_references()
        unqualified_force = replace(
            valid_force(),
            clock=ClockContract(0.5, 0.0, 1.0, qualified=False),
        )
        report = evaluate(
            MeasurementSubmission(
                candidate_id="unqualified-clock",
                claims=(ClaimKind.FORCE_IMPULSE_BALANCE,),
                force=unqualified_force,
                impulse_reference=replace(
                    impulse,
                    expected_clock=ClockContract(0.5, 0.0, 1.0, qualified=False),
                ),
            )
        )
        self.assertEqual(report.findings[0].outcome, Outcome.NOT_EVALUABLE)
        self.assertEqual(report.findings[0].code, "CLOCK_UNQUALIFIED")

    def test_missing_independent_clock_reference_is_not_evaluable(self) -> None:
        impulse, _, _ = valid_references()
        report = evaluate(
            MeasurementSubmission(
                candidate_id="missing-independent-clock-reference",
                claims=(ClaimKind.FORCE_IMPULSE_BALANCE,),
                force=valid_force(),
                impulse_reference=replace(impulse, expected_clock=None),
            )
        )
        self.assertEqual(report.findings[0].outcome, Outcome.NOT_EVALUABLE)
        self.assertEqual(report.findings[0].code, "CLOCK_REFERENCE_MISSING")

    def test_incomplete_support_and_missing_reference_are_not_evaluable(self) -> None:
        impulse, _, _ = valid_references()
        report = evaluate(
            MeasurementSubmission(
                candidate_id="incomplete-evidence",
                claims=(
                    ClaimKind.FORCE_IMPULSE_BALANCE,
                    ClaimKind.BILATERAL_FORCE,
                    ClaimKind.REAL_PAIRED_FORCE_LPT,
                ),
                force=valid_force(coverage=SupportCoverage.PARTIAL),
                impulse_reference=impulse,
            )
        )
        self.assertEqual(report.counts.evaluated, 3)
        self.assertEqual(report.counts.admitted, 0)
        self.assertEqual(report.counts.rejected, 0)
        self.assertEqual(report.counts.not_evaluable, 3)
        self.assertEqual(
            report.findings[0].code,
            "SUPPORT_COVERAGE_INCOMPLETE",
        )
        self.assertEqual(
            report.findings[1].code,
            "COMBINED_FORCE_NOT_BILATERAL",
        )

    def test_impulse_needs_covered_external_impulses_and_observed_interval(self) -> None:
        impulse, _, _ = valid_references()
        no_external_coverage = replace(impulse, external_impulses_complete=False)
        uncovered = evaluate(
            MeasurementSubmission(
                candidate_id="uncovered-external-impulses",
                claims=(ClaimKind.FORCE_IMPULSE_BALANCE,),
                force=valid_force(),
                impulse_reference=no_external_coverage,
            )
        )
        self.assertEqual(uncovered.findings[0].outcome, Outcome.NOT_EVALUABLE)
        self.assertEqual(uncovered.findings[0].code, "OTHER_EXTERNAL_IMPULSES_INCOMPLETE")

        off_grid_interval = replace(
            impulse,
            start_simulation_time_s=0.1,
            end_simulation_time_s=0.9,
        )
        interval_result = evaluate(
            MeasurementSubmission(
                candidate_id="off-grid-impulse-interval",
                claims=(ClaimKind.FORCE_IMPULSE_BALANCE,),
                force=valid_force(),
                impulse_reference=off_grid_interval,
            )
        )
        self.assertEqual(interval_result.findings[0].outcome, Outcome.NOT_EVALUABLE)
        self.assertEqual(interval_result.findings[0].code, "IMPULSE_INTERVAL_NOT_OBSERVED")

    def test_declared_bilateral_channels_can_be_evaluated_without_claiming_fidelity(self) -> None:
        report = evaluate(
            MeasurementSubmission(
                candidate_id="declared-two-channel-known-answer",
                claims=(ClaimKind.BILATERAL_FORCE,),
                force=valid_force(
                    coverage=SupportCoverage.TWO_FOOT_CHANNELS,
                    left=(12.0, 0.0, 12.0),
                    right=(8.0, 0.0, 8.0),
                    partition_fraction=0.6,
                ),
                bilateral_reference=BilateralForceReference(
                    points=(
                        BilateralForcePoint(0.0, 20.0, 12.0, 8.0),
                        BilateralForcePoint(0.5, 0.0, 0.0, 0.0),
                        BilateralForcePoint(1.0, 20.0, 12.0, 8.0),
                    ),
                    tolerance_n=_roundoff_bound(20.0, 3),
                    time_tolerance_s=_roundoff_bound(1.0),
                    expected_clock=ClockContract(0.5, 0.0, 1.0, qualified=True),
                ),
            )
        )
        self.assertEqual(report.findings[0].outcome, Outcome.PASS)
        self.assertIn("does not establish empirical", report.findings[0].reason)
        self.assertEqual(report.findings[0].evidence_ceiling, "NUMERICAL_VERIFICATION_ONLY")

    def test_declared_channels_without_an_independent_split_reference_abstain(self) -> None:
        report = evaluate(
            MeasurementSubmission(
                candidate_id="two-channels-no-reference",
                claims=(ClaimKind.BILATERAL_FORCE,),
                force=valid_force(
                    coverage=SupportCoverage.TWO_FOOT_CHANNELS,
                    left=(12.0, 0.0, 12.0),
                    right=(8.0, 0.0, 8.0),
                    partition_fraction=0.6,
                ),
            )
        )
        self.assertEqual(report.findings[0].outcome, Outcome.NOT_EVALUABLE)
        self.assertEqual(report.findings[0].code, "BILATERAL_REFERENCE_MISSING")

    def test_wrong_declared_bilateral_partition_is_rejected(self) -> None:
        report = evaluate(
            MeasurementSubmission(
                candidate_id="wrong-assumed-partition",
                claims=(ClaimKind.BILATERAL_FORCE,),
                force=valid_force(
                    coverage=SupportCoverage.TWO_FOOT_CHANNELS,
                    left=(12.0, 0.0, 12.0),
                    right=(8.0, 0.0, 8.0),
                    partition_fraction=0.5,
                ),
                bilateral_reference=BilateralForceReference(
                    points=(
                        BilateralForcePoint(0.0, 20.0, 12.0, 8.0),
                        BilateralForcePoint(0.5, 0.0, 0.0, 0.0),
                        BilateralForcePoint(1.0, 20.0, 12.0, 8.0),
                    ),
                    tolerance_n=_roundoff_bound(20.0, 3),
                    time_tolerance_s=_roundoff_bound(1.0),
                    expected_clock=ClockContract(0.5, 0.0, 1.0, qualified=True),
                ),
            )
        )
        self.assertEqual(report.findings[0].outcome, Outcome.REJECTED)
        self.assertEqual(report.findings[0].code, "BILATERAL_PARTITION_MISMATCH")

    def test_module_is_independent_and_submission_has_no_hidden_truth_field(self) -> None:
        source_path = Path(benchmark_core.__file__)
        tree = ast.parse(source_path.read_text(encoding="utf-8"))
        imported_modules: list[str] = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported_modules.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported_modules.append(node.module)
        self.assertFalse(
            any(
                name.startswith(("biosyngen", "plate2bar"))
                for name in imported_modules
            ),
            msg=f"forbidden import in independent evaluator: {imported_modules}",
        )
        public_fields = {field.name for field in fields(MeasurementSubmission)}
        self.assertNotIn("hidden_truth", public_fields)
        self.assertNotIn("latent_truth", public_fields)
        source = source_path.read_text(encoding="utf-8")
        self.assertNotIn("hidden_truth", source)
        self.assertNotIn("latent_truth", source)

        class HiddenTruthTrap:
            @property
            def hidden_truth(self):
                raise AssertionError("evaluator attempted to read candidate hidden truth")

        with self.assertRaises(TypeError):
            evaluate(HiddenTruthTrap())


if __name__ == "__main__":
    unittest.main()
