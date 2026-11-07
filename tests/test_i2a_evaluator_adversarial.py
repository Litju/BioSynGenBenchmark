"""Fresh RES-489 falsifiers for false acceptance in the I1 public evaluator.

The base measurement fixture is frozen synthetic input. Mutations are authored
here, separately from the generator and benchmark mutation helpers.
"""

from __future__ import annotations

import copy
import json
from pathlib import Path
import unittest

from biosyngen_benchmark.controls import prismatic_push_control, two_body_free_fall_control
from biosyngen_benchmark.i1 import Finding, MetrologyAuthority, Report, evaluate_i1
from biosyngen_benchmark.suite import Case, run_suite


ROOT = Path(__file__).resolve().parents[1]


def _submission(kinematics: dict, claims: tuple[str, ...]) -> dict:
    return {"candidate_id": "fresh-adversarial", "claims": list(claims), "kinematics": kinematics}


def _fake_report(submission: dict) -> Report:
    outcome = "PASS" if submission["candidate_id"] == "clean-positive" else "NOT_EVALUABLE"
    return Report(submission["candidate_id"],
                  (Finding("NEWTON_LINEAR_MOMENTUM", outcome, "fixture", "fixture", "NONE"),), (), {})


def _calibrated_measurement_fixture() -> tuple[dict, MetrologyAuthority, MetrologyAuthority]:
    """Hand-authored, constant-velocity supported body and two exact devices."""
    mass, g, dt = 72.0, 9.80665, 0.001
    frames = []
    for i in range(401):
        t = i * dt
        x = 0.1 + 0.05 * t
        frames.append({
            "t": t,
            "body_states": [[x, 1.0, 0.0, 0.05, 0.0, 0.0]],
            "contacts": [{"name": "support", "x": x, "z": 0.0, "vx": 0.05, "vz": 0.0,
                          "fx": 0.0, "fz": mass * g}],
            "foot_support_x_m": [x - 0.1, x + 0.1],
            "joint_power_w": 0.0, "internal_dissipation_power_w": 0.0,
            "internal_elastic_energy_j": 0.0,
            "lpt_attachment_m": [x + 0.2, 0.0, 1.1],
        })
    kin = {"gravity_m_s2": g, "bodies": [{"name": "body", "mass_kg": mass, "inertia_kg_m2": 2.0}],
           "frames": frames, "declared_impulsive_events": []}
    force = {"device_class": "SIMULATED_DEVICE_HYPOTHESIS", "sample_period_s": dt,
             "native_time_offset_s": 0.0, "support_coverage": "SINGLE_PLATE", "positive_z": "up (reaction on subject)",
             "units": {"force": "N", "moment": "N m", "length": "m"},
             "samples": [{"index": i, "t": i * dt, "fx": 0.0, "fz": mass * g, "my": 0.0,
                          "cop_x": 0.0, "saturated": False} for i in range(401)]}
    anchor = [0.0, 0.0, 0.0]
    def cable_length(point):
        return sum((point[i] - anchor[i]) ** 2 for i in range(3)) ** 0.5
    initial = cable_length(frames[0]["lpt_attachment_m"])
    lpt = {"device_class": "SIMULATED_DEVICE_HYPOTHESIS", "sample_period_s": 0.002,
           "native_time_offset_s": 0.0, "anchor_m": anchor, "attachment": "hip_belt",
           "quantity": "cable displacement from first sample", "units": {"displacement": "m", "velocity": "m/s"},
           "samples": [{"index": i, "t": i * 0.002,
                        "displacement": cable_length(frames[i * 2]["lpt_attachment_m"]) - initial,
                        "provider_velocity": 0.0, "slack": False} for i in range(201)]}
    submission = {"candidate_id": "hand-authored-numerical-fixture",
                  "claims": ["PLATE_WEIGHT_CONSISTENCY", "LPT_CABLE_GEOMETRY", "SENSOR_METROLOGY"],
                  "kinematics": kin, "force": force, "lpt": lpt}
    authority_hash = "a" * 64  # test-only sentinel, not admitted calibration evidence
    force_authority = MetrologyAuthority("TEST_ONLY_FORCE", authority_hash, "FORCE", force["device_class"],
                                         dt, 0.0, 0.0, 0.0, 0.0, 0.0)
    lpt_authority = MetrologyAuthority("TEST_ONLY_LPT", authority_hash, "LPT", lpt["device_class"],
                                       0.002, 0.0, 0.0, 0.0, encoder_resolution_m=1e-8)
    return submission, force_authority, lpt_authority


class SuiteGateFalsifiers(unittest.TestCase):
    def test_selective_negative_abstention_cannot_pass(self) -> None:
        cases = [
            Case("positive", "closed_form", {"candidate_id": "clean-positive"},
                 {"NEWTON_LINEAR_MOMENTUM": "PASS"}),
            Case("negative", "unseen_family", {"candidate_id": "corrupted-negative"},
                 {"NEWTON_LINEAR_MOMENTUM": "REJECTED"}),
        ]
        report = run_suite(cases, _fake_report)
        self.assertEqual(report["gate"], "FAIL", report["gate_failures"])
        stats = report["per_claim"]["NEWTON_LINEAR_MOMENTUM"]
        self.assertEqual(stats["negative_abstain"], 1)
        self.assertEqual(stats["negative_decision_coverage"], 0.0)

    def test_unknown_model_family_is_visible_in_suite_coverage(self) -> None:
        cases = [Case("positive", "unseen_family", {"candidate_id": "clean-positive"},
                      {"NEWTON_LINEAR_MOMENTUM": "PASS"}),
                 Case("negative", "unseen_family", {"candidate_id": "corrupted-negative"},
                      {"NEWTON_LINEAR_MOMENTUM": "REJECTED"})]
        report = run_suite(cases, _fake_report)
        self.assertIn("unseen_family", report["families"])
        self.assertGreaterEqual(report["family_count"], 1)


class SchemaFalsifiers(unittest.TestCase):
    def test_nonfinite_mass_and_inertia_are_rejected(self) -> None:
        for field, value in (("mass_kg", float("nan")), ("mass_kg", float("inf")),
                             ("mass_kg", 1.0e308), ("inertia_kg_m2", float("nan")),
                             ("inertia_kg_m2", float("inf"))):
            with self.subTest(field=field, value=value):
                kin = copy.deepcopy(prismatic_push_control())
                kin["bodies"][0][field] = value
                report = evaluate_i1(_submission(kin, ("NEWTON_LINEAR_MOMENTUM",)))
                self.assertEqual(report.outcome("NEWTON_LINEAR_MOMENTUM"), "REJECTED")

    def test_non_mapping_submission_is_a_structured_rejection(self) -> None:
        report = evaluate_i1(None)
        self.assertEqual((report.findings[0].outcome, report.findings[0].code),
                         ("REJECTED", "TOP_LEVEL_NOT_MAPPING"))

    def test_duplicate_kinematic_time_is_rejected(self) -> None:
        kin = copy.deepcopy(prismatic_push_control())
        kin["frames"][1]["t"] = kin["frames"][0]["t"]
        report = evaluate_i1(_submission(kin, ("NEWTON_LINEAR_MOMENTUM",)))
        self.assertEqual(report.outcome("NEWTON_LINEAR_MOMENTUM"), "REJECTED")

    def test_malformed_contact_vector_is_rejected_without_truncation(self) -> None:
        kin = copy.deepcopy(prismatic_push_control())
        kin["frames"][0]["contacts"][0]["fz"] = float("nan")
        report = evaluate_i1(_submission(kin, ("CONTACT_ADMISSIBILITY",)))
        self.assertEqual(report.outcome("CONTACT_ADMISSIBILITY"), "REJECTED")

    def test_lpt_attachment_must_be_a_three_dimensional_vector(self) -> None:
        kin = copy.deepcopy(prismatic_push_control())
        kin["frames"][0]["lpt_attachment_m"] = [0.0, 1.0]
        report = evaluate_i1(_submission(kin, ("NEWTON_LINEAR_MOMENTUM",)))
        self.assertEqual(report.outcome("NEWTON_LINEAR_MOMENTUM"), "REJECTED")


class NumericalReferenceControls(unittest.TestCase):
    def test_unseen_two_body_family_satisfies_analytic_conservation_laws(self) -> None:
        claims = ("NEWTON_LINEAR_MOMENTUM", "EULER_ANGULAR_MOMENTUM", "BALLISTIC_FLIGHT", "WORK_ENERGY_LEDGER")
        report = evaluate_i1(_submission(two_body_free_fall_control(), claims))
        self.assertEqual([finding.outcome for finding in report.findings], ["PASS"] * len(claims), report.findings)
        self.assertTrue(all(finding.ceiling == "NUMERICAL_VERIFICATION_ONLY" for finding in report.findings))

    def test_contact_transition_is_an_event_boundary_for_quadrature_windows(self) -> None:
        from biosyngen_benchmark.i1 import _windows

        kin = prismatic_push_control()
        for frame in kin["frames"][20:]:
            frame["contacts"] = []
        windows, excluded = _windows(kin["frames"], [])
        self.assertTrue(windows)
        self.assertTrue(all(not (window[0] <= 20 <= window[-1]) for window in windows))
        self.assertEqual(excluded, 2)

    def test_aliased_alternating_load_is_refused_as_under_resolved(self) -> None:
        mass, g, dt = 10.0, 9.80665, 0.001
        frames = []
        for i in range(41):
            t = i * dt
            frames.append({"t": t, "body_states": [[0.0, 1.0, 0.0, 0.0, 0.0, 0.0]],
                           "contacts": [{"name": "support", "x": 0.0, "z": 0.0, "vx": 0.0, "vz": 0.0,
                                         "fx": 10.0 if i % 2 else -10.0, "fz": mass * g}],
                           "foot_support_x_m": [-0.1, 0.1], "joint_power_w": 0.0,
                           "internal_dissipation_power_w": 0.0, "internal_elastic_energy_j": 0.0})
        kin = {"gravity_m_s2": g, "bodies": [{"name": "body", "mass_kg": mass, "inertia_kg_m2": 1.0}],
               "frames": frames, "declared_event_boundaries": []}
        report = evaluate_i1(_submission(kin, ("NEWTON_LINEAR_MOMENTUM",)))
        finding = report.finding("NEWTON_LINEAR_MOMENTUM")
        self.assertEqual(finding.outcome, "NOT_EVALUABLE")
        self.assertEqual(finding.code, "QUADRATURE_REFINEMENT_UNSTABLE")
        self.assertGreater(finding.metrics["under_resolved_quadratures"], 0)
        self.assertFalse(finding.metrics["quadrature_error_is_bound"])


class DeviceAuthorityFalsifiers(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        path = ROOT / "runs" / "i1" / "res487" / "run" / "evaluator-input-cmj.json"
        cls.base = json.loads(path.read_text(encoding="utf-8"))

    def test_absent_independent_calibration_never_passes_metrology_claims(self) -> None:
        submission, _, _ = _calibrated_measurement_fixture()
        report = evaluate_i1(submission)
        self.assertEqual(report.outcome("PLATE_WEIGHT_CONSISTENCY"), "NOT_EVALUABLE")
        self.assertEqual(report.outcome("LPT_CABLE_GEOMETRY"), "NOT_EVALUABLE")
        self.assertEqual(report.outcome("SENSOR_METROLOGY"), "NOT_EVALUABLE")

    def test_candidate_plate_uncertainty_cannot_buy_a_pass(self) -> None:
        poisoned, force_authority, lpt_authority = _calibrated_measurement_fixture()
        poisoned = copy.deepcopy(poisoned)
        for sample in poisoned["force"]["samples"]:
            if sample["fz"] is not None:
                sample["fz"] += 100.0
        poisoned["force"]["gain_uncertainty_bound"] = float("inf")
        poisoned["force"]["zero_offset_bound_n"] = 1.0e9
        report = evaluate_i1(poisoned, force_authority=force_authority, lpt_authority=lpt_authority)
        self.assertEqual(report.outcome("PLATE_WEIGHT_CONSISTENCY"), "REJECTED")
        self.assertIn("force.gain_uncertainty_bound", report.refused_fields)
        self.assertIn("force.zero_offset_bound_n", report.refused_fields)

    def test_candidate_lpt_resolution_and_timing_cannot_buy_a_pass(self) -> None:
        poisoned, force_authority, lpt_authority = _calibrated_measurement_fixture()
        poisoned = copy.deepcopy(poisoned)
        poisoned["lpt"]["anchor_m"] = [3.0, -2.0, 4.0]
        poisoned["lpt"]["encoder_resolution_m"] = 1000.0
        poisoned["lpt"]["timing_uncertainty_bound_s"] = 1.0e9
        report = evaluate_i1(poisoned, force_authority=force_authority, lpt_authority=lpt_authority)
        self.assertEqual(report.outcome("LPT_CABLE_GEOMETRY"), "REJECTED")
        self.assertIn("lpt.encoder_resolution_m", report.refused_fields)
        self.assertIn("lpt.timing_uncertainty_bound_s", report.refused_fields)

    def test_only_out_of_band_authority_can_admit_numerical_device_checks(self) -> None:
        submission, force_authority, lpt_authority = _calibrated_measurement_fixture()
        report = evaluate_i1(submission, force_authority=force_authority, lpt_authority=lpt_authority)
        self.assertEqual(report.outcome("PLATE_WEIGHT_CONSISTENCY"), "PASS")
        self.assertEqual(report.outcome("LPT_CABLE_GEOMETRY"), "PASS")
        self.assertEqual(report.outcome("SENSOR_METROLOGY"), "NOT_EVALUABLE")
        allowed = {"NUMERICAL_VERIFICATION_ONLY", "SIMULATED_MEASUREMENT_ONLY",
                   "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED", "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED"}
        self.assertTrue(all(f.ceiling in allowed for f in report.findings))

    def test_nested_candidate_authority_canary_is_refused_and_inert(self) -> None:
        submission, _, _ = _calibrated_measurement_fixture()
        baseline = evaluate_i1(submission)
        poisoned = copy.deepcopy(submission)
        poisoned["force"]["samples"][0]["metrology_authority"] = {
            "gain_uncertainty_bound": 1.0e9, "zero_offset_bound_n": 1.0e9,
        }
        observed = evaluate_i1(poisoned)
        self.assertEqual([(f.claim, f.outcome, f.code) for f in baseline.findings],
                         [(f.claim, f.outcome, f.code) for f in observed.findings])
        self.assertIn("force.samples[*].metrology_authority", observed.refused_fields)

    def test_zero_and_negative_sample_periods_are_rejected(self) -> None:
        for period in (0.0, -0.001):
            with self.subTest(period=period):
                poisoned = copy.deepcopy(self.base)
                force = poisoned["force"]
                force["sample_period_s"] = period
                force["samples"] = [dict(s, t=force["native_time_offset_s"] + i * period)
                                     for i, s in enumerate(force["samples"])]
                report = evaluate_i1(poisoned)
                self.assertEqual(report.outcome("CLOCK_INTEGRITY"), "REJECTED")


if __name__ == "__main__":
    unittest.main()
