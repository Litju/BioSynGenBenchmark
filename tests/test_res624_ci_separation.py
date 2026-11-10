"""RES-624 Gate I: CI keeps the frozen RED as an expected-negative canary and gates on the new qualification."""
import copy
import json
import tempfile
import unittest
from pathlib import Path

WORKFLOW = Path(".github/workflows/verification.yml").read_text(encoding="utf-8")


class WorkflowStructure(unittest.TestCase):
    def test_no_failure_masking(self):
        self.assertNotIn("continue-on-error", WORKFLOW)
        self.assertNotIn("|| true", WORKFLOW)

    def test_frozen_red_runs_only_through_canary(self):
        self.assertIn("scripts.res624_red_canary", WORKFLOW)
        self.assertNotIn("python -m scripts.res618_fullbody_qualification", WORKFLOW)

    def test_current_qualification_is_full_and_required(self):
        self.assertIn("scripts.res624_receiving_qualification --out", WORKFLOW)
        self.assertNotIn("--quick", WORKFLOW)
        self.assertIn("scripts.res624_numerical_verification", WORKFLOW)
        self.assertIn("scripts.res624_compare_platforms", WORKFLOW)
        self.assertIn("ref: ${{ github.event.pull_request.head.sha || github.sha }}", WORKFLOW)


def _report(final="RECEIVING_HOLD_QUALIFIED", disp=.0012):
    case = dict(static_verdict="STATIC_EQUILIBRIUM_CERTIFIED", outcome="RECEIVING_HOLD_QUALIFIED", all_failures=[],
                expectation="QUALIFIED", expectation_met=True,
                evidence=dict(bar_displacement_in_hand_m=disp, hand_support_ratio_mean=1.0, floor_vertical_over_weight_mean=1.0))
    q = dict(receipt=dict(head="h", mujoco="3.3.7", scipy="1.17.1", numpy="2.4.4"), final_verdict=final,
             cases=dict(nominal_2s=case), adversarial=dict(a=dict(observed="X")))
    n = dict(checks=dict(x=True), energy=dict(relative_to_gross_flux=8e-5), timestep_refinement=dict(observed_ratio=1.94),
             fingerprint=dict(sha256="f"))
    return q, n


class PlatformComparison(unittest.TestCase):
    def run_compare(self, mutate=None):
        from scripts.res624_compare_platforms import compare
        with tempfile.TemporaryDirectory() as t:
            for plat in ("ubuntu-24.04", "windows-latest"):
                q, n = _report()
                if mutate and plat == "windows-latest":
                    mutate(q, n)
                d = Path(t)/f"human-v2-receiving-{plat}"
                (d/"qualification").mkdir(parents=True)
                (d/"gateF").mkdir()
                (d/"qualification/receiving_qualification.json").write_text(json.dumps(q))
                (d/"gateF/numerical_verification.json").write_text(json.dumps(n))
            return compare(t)

    def test_small_float_differences_are_consistent(self):
        r = self.run_compare(lambda q, n: q["cases"]["nominal_2s"]["evidence"].update(bar_displacement_in_hand_m=.0012+2e-4))
        self.assertEqual(r["outcome"], "PLATFORMS_CONSISTENT", r["problems"])

    def test_verdict_difference_is_detected(self):
        r = self.run_compare(lambda q, n: q["cases"]["nominal_2s"].update(outcome="GRASP_SLIP"))
        self.assertEqual(r["outcome"], "PLATFORM_COMPARISON_FAILED")

    def test_large_numeric_difference_is_detected(self):
        r = self.run_compare(lambda q, n: q["cases"]["nominal_2s"]["evidence"].update(bar_displacement_in_hand_m=.0012+5e-3))
        self.assertEqual(r["outcome"], "PLATFORM_COMPARISON_FAILED")

    def test_version_mismatch_is_detected(self):
        r = self.run_compare(lambda q, n: q["receipt"].update(mujoco="3.4.0"))
        self.assertEqual(r["outcome"], "PLATFORM_COMPARISON_FAILED")


if __name__ == "__main__":
    unittest.main()
