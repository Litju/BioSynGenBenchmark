"""RES-624 Gate C falsifiers: nominal vs anthropometric scope verdicts.

A passing report is mutated five ways. Each mutation must fail the appropriate
generalization gate without falsely corrupting a legitimately qualified
nominal-only claim. A missing/failing exploratory subject must not invalidate
the narrower nominal claim.
"""
import copy
import unittest

from biosyngen_benchmark.qualification_scope import (
    OVERALL_NOMINAL_ONLY, OVERALL_VERDICT, GENERALIZATION_NOT_VERDICT, GENERALIZATION_VERDICT,
    NOMINAL_NOT_VERDICT, NOMINAL_VERDICT, qualify_scope)

REQUIRED = ("anthropometry_70kg_1p70m", "anthropometry_80kg_1p80m")


def positive_case(name, mass, stature, *, support=1.0, tilt=.4, outcome="RECEIVING_HOLD_QUALIFIED",
                  expectation_met=True, static="STATIC_EQUILIBRIUM_CERTIFIED", penetration=5e-4,
                  contacts=8, identity=None, explanation=None):
    return dict(static_verdict=static, outcome=outcome, expectation="QUALIFIED",
                expectation_met=expectation_met, reason=explanation,
                evidence=dict(hand_support_impulse_ratio=support, min_grasp_active_contacts=contacts,
                              bar_displacement_in_hand_m=.0012, max_penetration_m=penetration,
                              posture_saturation_samples=0, contact_problems=0,
                              grasp_identity_problems=identity or [],
                              relative_bar_pose=dict(max_tilt_deg={"l": tilt, "r": tilt},
                                                     max_axial_slip_m={"l": 2e-5, "r": 2e-5})),
                subject=dict(body_mass_kg=mass, stature_m=stature, model_human_mass_kg=mass,
                             mass_consistent=True, source="declared subject profile"))


def passing_report():
    cases = {name: positive_case(name, 75., 1.75) for name in
             ("nominal_0p3s_red_window", "nominal_2s", "nominal_10s")}
    cases["bar_mass_15kg"] = positive_case("bar_mass_15kg", 75., 1.75)
    cases["bar_mass_25kg"] = positive_case("bar_mass_25kg", 75., 1.75)
    cases["finger_release_at_1s"] = dict(expectation="pre-release QUALIFIED and post-release GRASP_SLIP",
                                         expectation_met=True, outcome="NUMERICAL_NOT_QUALIFIED")
    cases["arm_capacity_half_threshold"] = dict(expectation="REFUSED:ACTUATOR_INFEASIBLE",
                                                expectation_met=True, outcome="NUMERICAL_NOT_QUALIFIED")
    cases["anthropometry_70kg_1p70m"] = positive_case("anthropometry_70kg_1p70m", 70., 1.70)
    cases["anthropometry_80kg_1p80m"] = positive_case("anthropometry_80kg_1p80m", 80., 1.80)
    return dict(cases=cases, red_canary=dict(refused=True),
                adversarial={"a": dict(refused=True)}, adversarial_positive={"p": dict(qualified=True)},
                receipt=dict(models={name: dict(parameter_set_sha256=f"hash-{i}")
                                     for i, name in enumerate(REQUIRED)}))


class ScopeVerdicts(unittest.TestCase):
    def test_complete_required_scope_qualifies(self):
        v = qualify_scope(passing_report())
        self.assertEqual(v["nominal"]["verdict"], NOMINAL_VERDICT)
        self.assertEqual(v["anthropometric_generalization"]["verdict"], GENERALIZATION_VERDICT)
        self.assertEqual(v["overall"]["verdict"], OVERALL_VERDICT)
        self.assertFalse(v["overall"]["nominal_only"])

    def test_70kg_failure_does_not_corrupt_nominal_claim(self):
        report = passing_report()
        report["cases"]["anthropometry_70kg_1p70m"].update(outcome="GRASP_SLIP", expectation_met=False)
        v = qualify_scope(report)
        self.assertTrue(v["nominal"]["qualified"])
        self.assertFalse(v["anthropometric_generalization"]["qualified"])
        self.assertEqual(v["overall"]["verdict"], OVERALL_NOMINAL_ONLY)
        self.assertIn("NOT established", v["scope_statement"])

    def test_80kg_failure_does_not_corrupt_nominal_claim(self):
        report = passing_report()
        report["cases"]["anthropometry_80kg_1p80m"].update(outcome="GRASP_SLIP", expectation_met=False)
        v = qualify_scope(report)
        self.assertTrue(v["nominal"]["qualified"])
        self.assertFalse(v["anthropometric_generalization"]["qualified"])
        self.assertEqual(v["overall"]["verdict"], OVERALL_NOMINAL_ONLY)

    def test_missing_required_subject_prohibits_generalization(self):
        for missing in REQUIRED:
            report = passing_report()
            del report["cases"][missing]
            v = qualify_scope(report)
            self.assertTrue(v["nominal"]["qualified"], missing)
            self.assertFalse(v["anthropometric_generalization"]["qualified"], missing)
            self.assertTrue(any("MISSING" in p for p in v["anthropometric_generalization"]["problems"]))

    def test_subject_mass_or_stature_identity_mismatch_prohibits_generalization(self):
        report = passing_report()
        report["cases"]["anthropometry_70kg_1p70m"]["subject"]["body_mass_kg"] = 71.
        v = qualify_scope(report)
        self.assertFalse(v["anthropometric_generalization"]["qualified"])
        self.assertTrue(v["nominal"]["qualified"])
        report = passing_report()
        report["cases"]["anthropometry_80kg_1p80m"]["subject"]["stature_m"] = 1.79
        v = qualify_scope(report)
        self.assertFalse(v["anthropometric_generalization"]["qualified"])
        self.assertTrue(v["nominal"]["qualified"])

    def test_model_human_mass_must_match_the_declared_subject(self):
        report = passing_report()
        report["cases"]["anthropometry_80kg_1p80m"]["subject"]["model_human_mass_kg"] = 85.
        v = qualify_scope(report)
        self.assertFalse(v["anthropometric_generalization"]["qualified"])

    def test_unqualified_subject_relabelled_as_qualified_still_fails(self):
        report = passing_report()
        report["cases"]["anthropometry_80kg_1p80m"] = positive_case(
            "anthropometry_80kg_1p80m", 80., 1.80, support=.3, contacts=1, penetration=2e-3)
        v = qualify_scope(report)
        self.assertFalse(v["anthropometric_generalization"]["qualified"])
        self.assertTrue(any("HAND_SUPPORT" in p or "CONTACT" in p or "PENETRATION" in p
                            for p in v["anthropometric_generalization"]["problems"]))
        self.assertTrue(v["nominal"]["qualified"])

    def test_shared_subject_parameter_set_prohibits_generalization(self):
        report = passing_report()
        for name in REQUIRED:
            report["receipt"]["models"][name]["parameter_set_sha256"] = "same"
        v = qualify_scope(report)
        self.assertFalse(v["anthropometric_generalization"]["qualified"])

    def test_exploratory_subject_cannot_invalidate_nominal_or_required_scope(self):
        report = passing_report()
        report["cases"]["exploratory_90kg_1p90m"] = dict(
            expectation="QUALIFIED", expectation_met=False, outcome="GRASP_SLIP",
            explanatory="not a preregistered required subject")
        v = qualify_scope(report)
        self.assertTrue(v["nominal"]["qualified"])
        self.assertTrue(v["anthropometric_generalization"]["qualified"])
        self.assertEqual(v["overall"]["verdict"], OVERALL_VERDICT)
        self.assertIn("exploratory_90kg_1p90m", v["exploratory_cases"])

    def test_missing_red_or_adversarial_or_positive_controls_fail_nominal(self):
        for mutate in (lambda r: r["red_canary"].update(refused=False),
                       lambda r: r["adversarial"].update(b=dict(refused=False)),
                       lambda r: r["adversarial_positive"].update(q=dict(qualified=False)),
                       lambda r: r.update(adversarial={}),
                       lambda r: r.update(adversarial_positive={})):
            report = copy.deepcopy(passing_report())
            mutate(report)
            v = qualify_scope(report)
            self.assertFalse(v["nominal"]["qualified"])
            self.assertFalse(v["anthropometric_generalization"]["qualified"])
            self.assertEqual(v["overall"]["verdict"], "NOT_QUALIFIED")

    def test_nominal_negative_expectation_must_be_met(self):
        report = passing_report()
        report["cases"]["arm_capacity_half_threshold"]["expectation_met"] = False
        v = qualify_scope(report)
        self.assertFalse(v["nominal"]["qualified"])


if __name__ == "__main__":
    unittest.main()
