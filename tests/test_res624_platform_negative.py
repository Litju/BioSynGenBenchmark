"""RES-624 Gate D: negative-case failure signatures and cross-platform falsifiers."""
import copy
import json
import tempfile
import unittest
from pathlib import Path


def positive_case():
    return dict(static_verdict="STATIC_EQUILIBRIUM_CERTIFIED", outcome="RECEIVING_HOLD_QUALIFIED",
                expectation="QUALIFIED", expectation_met=True, all_failures=[],
                evidence=dict(bar_displacement_in_hand_m=.0012, hand_support_ratio_mean=1.0,
                              hand_support_ratio_min=.998, floor_vertical_over_weight_mean=1.0))


def identity():
    return dict(head="h", mujoco="3.3.7", scipy="1.17.1", numpy="2.4.4",
                evaluator={"receiving": "e1"}, model_parameter_set_sha256="model", controller="c")


def negative_case(kind="capacity"):
    if kind == "capacity":
        return dict(static_verdict="ACTUATOR_INFEASIBLE", outcome="NUMERICAL_NOT_QUALIFIED",
                    expectation="REFUSED:ACTUATOR_INFEASIBLE", expectation_met=True, all_failures=[],
                    failure_signature=dict(expected="REFUSED:ACTUATOR_INFEASIBLE", observed="NUMERICAL_NOT_QUALIFIED",
                                           reason="SYSTEM_BALANCE_OPEN_UNDECLARED_LOAD",
                                           first_event=dict(code="NUMERICAL_NOT_QUALIFIED",
                                                            reason="SYSTEM_BALANCE_OPEN_UNDECLARED_LOAD", t_s=.012),
                                           contact_loss_s=.02, actuator_saturation_s=.012,
                                           qualified_prefix=dict(outcome="NUMERICAL_NOT_QUALIFIED", support_ratio=.68,
                                                                 support_share=dict(l=.40, r=.28),
                                                                 bar_displacement_m=.010),
                                           identity=identity()))
    return dict(static_verdict="STATIC_EQUILIBRIUM_CERTIFIED", outcome="NUMERICAL_NOT_QUALIFIED",
                expectation="pre-release QUALIFIED and post-release GRASP_SLIP", expectation_met=True, all_failures=[],
                failure_signature=dict(expected="pre-release QUALIFIED and post-release GRASP_SLIP",
                                       observed="NUMERICAL_NOT_QUALIFIED", reason="GRASP_SLIP",
                                       first_event=dict(code="GRASP_SLIP", reason="CONTACT_LOST", t_s=1.006),
                                       contact_loss_s=1.006, actuator_saturation_s=None,
                                       qualified_prefix=dict(outcome="RECEIVING_HOLD_QUALIFIED", support_ratio=1.0003,
                                                             support_share=dict(l=.4996, r=.5007),
                                                             bar_displacement_m=.0004),
                                       identity=identity()))


def adversarial():
    return {
        "external_support_hand_load_5pct": dict(
            expected="GRASP_UNSUPPORTED", observed="GRASP_UNSUPPORTED", reason="INSUFFICIENT_HAND_SUPPORT",
            signature=dict(expected="GRASP_UNSUPPORTED", observed="GRASP_UNSUPPORTED", reason="INSUFFICIENT_HAND_SUPPORT")),
        "declared_friction_violated": dict(
            expected="CONTACT_INADMISSIBLE", observed="CONTACT_INADMISSIBLE",
            reason="[('OUTSIDE_FRICTION_CONE', 7.4, 140.1)]",
            signature=dict(expected="CONTACT_INADMISSIBLE", observed="CONTACT_INADMISSIBLE", reason="OUTSIDE_FRICTION_CONE")),
    }


def report():
    q = dict(receipt=dict(head="h", mujoco="3.3.7", scipy="1.17.1", numpy="2.4.4"),
             final_verdict="OVERALL_DECLARED_SCOPE_QUALIFIED",
             cases=dict(nominal_2s=positive_case(), finger_release_at_1s=negative_case("release"),
                        arm_capacity_half_threshold=negative_case("capacity")),
             adversarial=adversarial(),
             contact_model_sensitivity=dict(pyramidal_cone="NUMERICAL_NOT_QUALIFIED"))
    n = dict(checks=dict(x=True), energy=dict(relative_to_gross_flux=8e-5),
             timestep_refinement=dict(observed_ratio=1.94, bar_traj_max_diff_1ms_vs_0p5ms_m=1.5e-5,
                                      bar_traj_max_diff_0p5ms_vs_0p25ms_m=7.9e-6),
             energy_refinement=dict(observed_order=[1.0, 1.0]), fingerprint=dict(sha256="f"))
    return q, n


class NegativeSignatureComparison(unittest.TestCase):
    def run_compare(self, mutate=None):
        from scripts.res624_compare_platforms import compare
        with tempfile.TemporaryDirectory() as t:
            for plat in ("ubuntu-24.04", "windows-latest"):
                q, n = report()
                if mutate and plat == "windows-latest":
                    mutate(q, n)
                d = Path(t)/f"human-v2-receiving-{plat}"
                (d/"qualification").mkdir(parents=True)
                (d/"gateF").mkdir()
                (d/"qualification/receiving_qualification.json").write_text(json.dumps(q))
                (d/"gateF/numerical_verification.json").write_text(json.dumps(n))
            return compare(t)

    def test_noisy_post_impact_trajectory_and_secondary_categories_pass(self):
        def mutate(q, n):
            sig = q["cases"]["finger_release_at_1s"]["failure_signature"]
            sig["post_impact_trajectory"] = [0.1, 0.2, 0.3]  # not part of the signature comparison
            sig["secondary_failures"] = ["NUMERICAL_NOT_QUALIFIED:ANGULAR_MOMENTUM_REFERENCE_MISMATCH"]
            sig["support_chatter"] = 0.123
        r = self.run_compare(mutate)
        self.assertEqual(r["outcome"], "PLATFORMS_CONSISTENT", r["problems"])

    def test_different_primary_mechanism_fails(self):
        r = self.run_compare(lambda q, n: q["cases"]["finger_release_at_1s"]["failure_signature"].update(
            reason="ACTUATOR_INFEASIBLE"))
        self.assertEqual(r["outcome"], "PLATFORM_COMPARISON_FAILED")

    def test_different_first_contact_loss_beyond_tolerance_fails(self):
        r = self.run_compare(lambda q, n: q["cases"]["finger_release_at_1s"]["failure_signature"].update(
            contact_loss_s=1.071))
        self.assertEqual(r["outcome"], "PLATFORM_COMPARISON_FAILED")

    def test_first_contact_loss_inside_tolerance_passes(self):
        r = self.run_compare(lambda q, n: q["cases"]["finger_release_at_1s"]["failure_signature"].update(
            contact_loss_s=1.016))
        self.assertEqual(r["outcome"], "PLATFORMS_CONSISTENT", r["problems"])

    def test_different_pre_failure_support_fails(self):
        def mutate(q, n):
            q["cases"]["arm_capacity_half_threshold"]["failure_signature"]["qualified_prefix"].update(support_ratio=.80)
        r = self.run_compare(mutate)
        self.assertEqual(r["outcome"], "PLATFORM_COMPARISON_FAILED")

    def test_different_source_or_solver_identity_fails(self):
        r = self.run_compare(lambda q, n: q["receipt"].update(head="different"))
        self.assertEqual(r["outcome"], "PLATFORM_COMPARISON_FAILED")
        r = self.run_compare(lambda q, n: q["cases"]["arm_capacity_half_threshold"]["failure_signature"]["identity"].update(
            mujoco="3.4.0"))
        self.assertEqual(r["outcome"], "PLATFORM_COMPARISON_FAILED")
        r = self.run_compare(lambda q, n: q["cases"]["arm_capacity_half_threshold"]["failure_signature"]["identity"].update(
            model_parameter_set_sha256="other"))
        self.assertEqual(r["outcome"], "PLATFORM_COMPARISON_FAILED")

    def test_missing_negative_evidence_fails(self):
        r = self.run_compare(lambda q, n: q["cases"]["arm_capacity_half_threshold"].pop("failure_signature"))
        self.assertEqual(r["outcome"], "PLATFORM_COMPARISON_FAILED")

    def test_adversarial_generic_refusal_with_different_mechanism_fails(self):
        def mutate(q, n):
            q["adversarial"]["external_support_hand_load_5pct"]["signature"].update(
                observed="GRASP_UNSUPPORTED", reason="CONTACT_SITE_SLIP_EXCEEDS_GRIP")
        r = self.run_compare(mutate)
        self.assertEqual(r["outcome"], "PLATFORM_COMPARISON_FAILED")

    def test_valid_nominal_numeric_differences_pass(self):
        r = self.run_compare(lambda q, n: q["cases"]["nominal_2s"]["evidence"].update(
            bar_displacement_in_hand_m=.0012+2e-4))
        self.assertEqual(r["outcome"], "PLATFORMS_CONSISTENT", r["problems"])

    def test_elliptic_cone_finding_is_not_a_platform_difference(self):
        r = self.run_compare(lambda q, n: q["contact_model_sensitivity"].update(pyramidal_cone="QUALIFIED"))
        self.assertEqual(r["outcome"], "PLATFORMS_CONSISTENT", r["problems"])


if __name__ == "__main__":
    unittest.main()
