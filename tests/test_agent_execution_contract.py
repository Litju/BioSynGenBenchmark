"""Regression guard for BioSynGenBenchmark's agent execution contract.

These tests protect documented agent behavior; they do not certify biomechanical
simulation, model validation, or any scientific outcome.
"""
from pathlib import Path
import unittest


POLICY = (Path(__file__).resolve().parents[1] / "AGENTS.md").read_text(encoding="utf-8")


class AgentExecutionContract(unittest.TestCase):
    def test_no_global_attempt_or_elapsed_time_hard_stop(self):
        self.assertNotIn(
            "Stop at a blocker, after two repair attempts, or at six elapsed hours.",
            POLICY,
        )
        self.assertIn("diagnostic pivot, not automatic termination", POLICY)
        self.assertIn("Six elapsed hours is a checkpoint, not an automatic stop", POLICY)
        self.assertIn("A blocker applies to the affected path", POLICY)

    def test_active_issue_and_checkout_are_discovered(self):
        self.assertIn("git rev-parse --show-toplevel", POLICY)
        self.assertIn("live", POLICY.lower())
        self.assertIn("Linear issue", POLICY)
        self.assertNotIn("Current/future execution authority is GPT-6.1 Solo", POLICY)
        self.assertNotIn("This workspace is the canonical project root: D:", POLICY)

    def test_scientific_failures_are_not_masked_by_continuation(self):
        for required in (
            "Hard stops remain mandatory",
            "Do not fabricate extra attempts",
            "independent final-SHA scientific review",
            "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED",
            "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED",
            "A known failed overhead grasp",
            "original RED evidence",
        ):
            with self.subTest(required=required):
                self.assertIn(required, POLICY)

    def test_atomic_commit_policy_and_owner_control_remain(self):
        for required in (
            "at least two meaningful, independently verified, atomic Git commits",
            "exact Linear issue ID",
            "git diff --check",
            "owner-only decisions",
            "do not modify Plate2bar, DynamisBench",
            "In Progress",
            "In Review",
        ):
            with self.subTest(required=required):
                self.assertIn(required, POLICY)


if __name__ == "__main__":
    unittest.main()
