"""RES-624 Gate G: anthropometric static certification uses the subject's own measured hand-bar relation."""
import unittest

try:
    import mujoco
    import numpy as np  # noqa: F401
    import scipy  # noqa: F401
except ImportError:
    mujoco = None


@unittest.skipIf(mujoco is None, "MuJoCo/SciPy required by mandatory spatial CI")
class Anthropometry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from biosyngen.spatial.anthropometry import HumanSubjectProfile
        from biosyngen.spatial.model_v2 import HumanV2Spec
        cls.spec = {k: HumanV2Spec(profile=HumanSubjectProfile(m, h)) for k, (m, h) in
                    {"70": (70, 1.70), "80": (80, 1.80)}.items()}

    def test_subject_hand_changes_measured_relation_nominal_unchanged(self):
        from biosyngen.spatial.model_v2 import HumanV2Spec
        from biosyngen.spatial.receiving_grasp_v2 import qualified_relation
        nominal = qualified_relation()
        self.assertEqual(nominal.shaft_point_hand, qualified_relation(human=HumanV2Spec()).shaft_point_hand)
        big = qualified_relation(human=self.spec["80"])
        self.assertNotEqual(nominal.shaft_point_hand, big.shaft_point_hand)

    def test_subjects_certified_without_changing_limits_or_strength(self):
        from biosyngen.spatial.receiving_grasp_v2 import build_certified_candidate
        for key, spec in self.spec.items():
            plant, cand, static, _ = build_certified_candidate(spec)
            self.assertEqual(static["verdict"], "STATIC_EQUILIBRIUM_CERTIFIED", key)
            self.assertGreaterEqual(static["certificate"]["hand_contacts"], 4)
            self.assertLessEqual(static["certificate"]["max_penetration_m"], 1.4e-3)

    def test_nominal_relation_on_scaled_hand_is_refused_control(self):
        """The pre-repair behaviour (nominal-hand relation on an 80 kg hand) must still be refused: a
        falsifying control that the certificate, not the pose solver, decides."""
        from biosyngen.spatial.plant_v2 import HumanV2Plant
        from biosyngen.spatial.receiving_grasp_v2 import qualified_relation, qualify_static, solve_supported_candidate
        plant = HumanV2Plant.build(self.spec["80"])
        solve_supported_candidate(plant, qualified_relation(), "POWER", 2000)
        self.assertNotEqual(qualify_static(plant)["verdict"], "STATIC_EQUILIBRIUM_CERTIFIED")

    def test_runtime_bar_mass_edit_matches_compiled_dynamics(self):
        from biosyngen.spatial.model_v2 import HumanV2Spec
        from scripts.res624_receiving_campaign import _set_bar_mass, build_nominal, clone
        _, nominal = build_nominal()
        for kg in (15.0, 25.0):
            p = clone(HumanV2Spec(), nominal["qpos"])
            _set_bar_mass(p, kg)
            q, v = p.data.qpos.copy(), p.data.qvel.copy()
            mujoco.mj_setConst(p.model, p.data)
            p.data.qpos[:], p.data.qvel[:] = q, v
            mujoco.mj_forward(p.model, p.data)
            bar = p.body_id("barbell")
            dof = p.model.jnt_dofadr[p.model.body_jntadr[bar]]
            self.assertAlmostEqual(p.model.body_mass[bar], kg)
            self.assertAlmostEqual(p.model.body_subtreemass[bar], kg)
            self.assertAlmostEqual(float(p.data.qfrc_bias[dof+2]), kg*9.81, places=9)
            self.assertTrue(np.array_equal(p.data.qpos, q))


if __name__ == "__main__":
    unittest.main()
