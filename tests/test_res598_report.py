"""Prevent a successful integration duration from becoming movement acceptance."""
from types import SimpleNamespace
import unittest
from scripts.run_movement_credibility import recovery_finding
from scripts.run_i2_benchmark import angular_control_counts


class RecoveryReportingTests(unittest.TestCase):
    def test_duration_reached_alone_cannot_certify_recovery(self):
        result = recovery_finding(SimpleNamespace(termination="DURATION_REACHED", frames=[]))
        self.assertEqual(result["measured"], "EVIDENCE_PENDING")

    def test_collapse_is_failure_and_retains_termination(self):
        result = recovery_finding(SimpleNamespace(termination="BODY_GROUND_CONTACT_NOT_MODELED: knee", frames=[]))
        self.assertEqual(result["measured"], "FAIL")
        self.assertIn("knee", result["termination"])

    def test_already_failed_baseline_cannot_count_as_fault_detection(self):
        cases = {"good": dict(angular_spin_negative=dict(eligible_positive=True, rejected=True, detected=True)),
                 "already_failed": dict(angular_spin_negative=dict(eligible_positive=False, rejected=True, detected=None)),
                 "missed_fault": dict(angular_spin_negative=dict(eligible_positive=True, rejected=False, detected=False))}
        self.assertEqual(angular_control_counts(cases), dict(detected=1, n=2, attempted=3,
                         rejected=2, ineligible_baseline_count=1))
