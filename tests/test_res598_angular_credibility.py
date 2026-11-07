"""Analytical bodies and adversaries, independently authored without generator imports."""
import copy
import unittest

from biosyngen_benchmark.i2_evaluator import evaluate_i2, energy_diagnostics


def supported_rotor():
    return dict(schema="BIOSYNGEN_I2_SUBMISSION_V1", exercise_id="MANUFACTURED_ROTOR",
        gravity_m_s2=9.81, bodies=[dict(name="rotor", mass_kg=2., inertia_kg_m2=.5)],
        initial_body_states=[[0, 1, 0, 0, 0, 0]],
        frames=[dict(t0=0, t1=1, body_states=[[0, 1, 0, 0, 0, 0]],
                     contacts=[dict(name="support", group="OTHER", x=0, z=0, fx=0, fz=19.62)])],
        impulses=[], external_support_groups=["OTHER"], friction_by_group={"OTHER": .8},
        claims=["LINEAR_MOMENTUM_CLOSURE", "ANGULAR_MOMENTUM_CLOSURE"])


def angular(sub):
    return evaluate_i2(sub)["findings"]["ANGULAR_MOMENTUM_CLOSURE"]


class AngularCredibilityTests(unittest.TestCase):
    def test_static_and_translated_origins(self):
        sub = supported_rotor()
        self.assertEqual(angular(sub)["outcome"], "PASS")
        translated = copy.deepcopy(sub)
        for state in (translated["initial_body_states"][0], translated["frames"][0]["body_states"][0]):
            state[0] += 17
            state[1] += 32
        translated["frames"][0]["contacts"][0].update(x=17, z=32)
        self.assertEqual(angular(translated)["outcome"], "PASS")
        self.assertEqual(angular(translated)["tolerance_kg_m2_s"], angular(sub)["tolerance_kg_m2_s"])

    def test_off_center_support_exposes_false_linear_pass(self):
        sub = supported_rotor()
        sub["frames"][0]["contacts"][0]["x"] = .1
        result = evaluate_i2(sub)["findings"]
        self.assertEqual(result["LINEAR_MOMENTUM_CLOSURE"]["outcome"], "PASS")
        self.assertEqual(result["ANGULAR_MOMENTUM_CLOSURE"]["outcome"], "REJECTED")
        self.assertAlmostEqual(result["ANGULAR_MOMENTUM_CLOSURE"]["residual_kg_m2_s"], 1.962)

    def test_impulse_spin_and_wrong_inertia(self):
        sub = supported_rotor()
        sub["frames"][0]["body_states"][0][4] = 1
        sub["frames"][0]["body_states"][0][5] = .8
        sub["impulses"] = [dict(t=1, name="strike", group="OTHER", x=.2, z=1, jx=0, jz=2)]
        self.assertEqual(angular(sub)["outcome"], "PASS")
        sub["bodies"][0]["inertia_kg_m2"] = .6
        self.assertEqual(angular(sub)["outcome"], "REJECTED")

    def test_intermediate_spin_error_cannot_cancel_at_end(self):
        sub = supported_rotor()
        sub["frames"].append(copy.deepcopy(sub["frames"][0]))
        sub["frames"][1].update(t0=1, t1=2)
        sub["frames"][0]["body_states"][0][5] = 1
        result = angular(sub)
        self.assertEqual(result["outcome"], "REJECTED")
        self.assertEqual(result["residual_kg_m2_s"], 0)
        self.assertEqual(result["max_prefix_residual_kg_m2_s"], .5)

    def test_empty_evidence_abstains_and_candidate_tolerance_refuses(self):
        sub = supported_rotor()
        sub["frames"] = []
        self.assertEqual(angular(sub)["outcome"], "NOT_EVALUABLE")
        sub = supported_rotor()
        sub["tolerance"] = 10
        self.assertEqual(evaluate_i2(sub)["status"], "INVALID")


class EnergyEvidenceTests(unittest.TestCase):
    def test_energy_is_recomputed_and_projection_cannot_make_a_pass(self):
        sub = supported_rotor()
        sub["frames"][0]["body_states"][0][1] += .5
        result = energy_diagnostics(sub, joint_work_j=2, contact_dissipation_j=3,
            initial_stored_energy_j=0, final_stored_energy_j=1,
            numerical_projection_work_j=11.81, rigid_contact_work_estimate_j=0)
        self.assertAlmostEqual(result["raw_energy_residual_j"], 11.81)
        self.assertAlmostEqual(result["projection_adjusted_residual_j"], 0)
        self.assertEqual(result["outcome"], "NOT_EVALUABLE")

    def test_nonfinite_or_negative_dissipation_refuses(self):
        for loss in (-1, float("nan"), True):
            result = energy_diagnostics(supported_rotor(), joint_work_j=0, contact_dissipation_j=loss,
                initial_stored_energy_j=0, final_stored_energy_j=0,
                numerical_projection_work_j=0, rigid_contact_work_estimate_j=0)
            self.assertEqual(result["outcome"], "INVALID")
