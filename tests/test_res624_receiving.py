"""RES-624 Gates D/E: equilibrium-certified Human V2 overhead pose and bounded forward hold."""
import unittest

try:
    import mujoco
    import numpy as np
    import scipy  # noqa: F401
except ImportError:
    mujoco = None


@unittest.skipIf(mujoco is None, "MuJoCo/SciPy required by mandatory spatial CI")
class ReceivingGates(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from scripts.res624_receiving_campaign import build_nominal
        cls.plant, cls.nominal = build_nominal()

    def test_static_certificate_balances_bar_and_whole_body(self):
        st = self.nominal["static"]
        c = st["certificate"]
        self.assertEqual(st["verdict"], "STATIC_EQUILIBRIUM_CERTIFIED")
        np.testing.assert_allclose(c["bar_contact_force_n"], [0, 0, 196.2], atol=1e-6)
        self.assertAlmostEqual(c["floor_vertical_n"], c["total_weight_n"], places=6)
        self.assertGreater(c["hand_contacts"], 0)
        self.assertGreater(c["cop_hull_margin_m"], 0)
        self.assertLessEqual(c["max_utilization"], 1)
        self.assertEqual(st["self_intersection"]["penetrating"], [])
        self.assertLessEqual(c["max_penetration_m"], .0014)

    def test_static_refusals(self):
        from scripts.res624_receiving_campaign import static_adversarials
        r = static_adversarials(self.plant)
        self.assertEqual(r["frozen_res618_kinematic_candidate"], "STATIC_GRASP_INFEASIBLE")
        self.assertEqual(r["nominal_at_20pct_capacity"], "ACTUATOR_INFEASIBLE")
        self.assertEqual(r["bar_lifted_20mm_into_fingers"], "POSE_INADMISSIBLE")
        self.assertEqual(r["bar_displaced_50mm_out_of_hands"], "STATIC_GRASP_INFEASIBLE")

    def test_insufficient_arm_capacity_is_refused(self):
        from biosyngen.spatial.receiving_grasp_v2 import qualify_static
        from biosyngen.spatial.model_v2 import HumanV2Spec
        from scripts.res624_receiving_campaign import _scale_arm_capacity, clone
        t = self.nominal["arm_capacity_threshold"]
        self.assertGreater(t, 0)
        self.assertLess(t, 1)
        p = clone(HumanV2Spec(), self.nominal["qpos"])
        _scale_arm_capacity(p, .5*t)
        self.assertEqual(qualify_static(p)["verdict"], "ACTUATOR_INFEASIBLE")

    def test_forward_hold_red_window_is_contact_supported(self):
        from scripts.res624_receiving_campaign import frame_metrics, run_case
        frames, static, _ = run_case("nominal_0p3s_red_window", None, None, .3, {}, self.nominal)
        mt = frame_metrics(frames)
        self.assertGreater(mt["min_hand_contacts"], 0)
        self.assertLess(mt["bar_displacement_in_hand_m"], .014)
        self.assertAlmostEqual(mt["hand_support_mean"], 1.0, delta=.01)
        self.assertEqual(mt["posture_saturation_samples"], 0)
        self.assertAlmostEqual(mt["floor_vertical_over_weight_mean"], 1.0, delta=.01)
        bar = next(b for b in frames[0]["bodies"] if b["name"] == "barbell")
        self.assertEqual(bar["mass"], 20.0)
        self.assertFalse(any(f["external"] for f in frames))


if __name__ == "__main__":
    unittest.main()
