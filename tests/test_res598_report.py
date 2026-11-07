"""Prevent a successful integration duration from becoming movement acceptance."""
from types import SimpleNamespace
import unittest
from scripts.run_movement_credibility import recovery_finding


class RecoveryReportingTests(unittest.TestCase):
    def test_duration_reached_alone_cannot_certify_recovery(self):
        result = recovery_finding(SimpleNamespace(termination="DURATION_REACHED", frames=[]))
        self.assertEqual(result["measured"], "EVIDENCE_PENDING")

    def test_collapse_is_failure_and_retains_termination(self):
        result = recovery_finding(SimpleNamespace(termination="BODY_GROUND_CONTACT_NOT_MODELED: knee", frames=[]))
        self.assertEqual(result["measured"], "FAIL")
        self.assertIn("knee", result["termination"])
