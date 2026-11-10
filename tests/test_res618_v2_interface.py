"""V2 actuation/export trust boundary; physical outcomes remain observable."""
import unittest

try:
    import mujoco
    import numpy as np
except ImportError:
    mujoco = None


@unittest.skipIf(mujoco is None, "MuJoCo required by mandatory spatial CI")
class V2Interface(unittest.TestCase):
    def test_exports_distinguish_rhythm_from_grip(self):
        from biosyngen.spatial.plant_v2 import HumanV2Plant
        p = HumanV2Plant.build()
        frame = p.export_frame()
        self.assertEqual(frame["grips"], [])
        self.assertEqual(frame["model_version"], "2.1.0")
        self.assertEqual(len(p.actuated), 48)
        self.assertEqual(len(frame["bodies"]), 32)
        self.assertEqual(len(frame["joint_angles"]), 52)

    def test_targets_refuse_unknown_nonfinite_and_outside_rom(self):
        from biosyngen.spatial.plant_v2 import HumanV2Plant
        p = HumanV2Plant.build()
        for targets in ({"fake": 1}, {"l_knee_flex": 3}, {"l_finger_flex": float("nan")}):
            with self.assertRaises(ValueError):
                p.set_targets(targets)
        with self.assertRaises(ValueError):
            p.set_targets({}, {"l_finger_flex": float("inf")})

    def test_motor_output_is_bounded_and_power_is_observable(self):
        from biosyngen.spatial.plant_v2 import HumanV2Plant
        p = HumanV2Plant.build()
        p.set_targets({"l_finger_flex": 2}, {"l_finger_flex": 1e9})
        mujoco.mj_forward(p.model, p.data)
        self.assertTrue(np.all(abs(p.data.actuator_force) <= p.model.actuator_forcerange[:, 1]+1e-12))
        frame = p.export_frame()
        self.assertAlmostEqual(frame["actuator_power_w"], p.data.actuator_force @ p.data.actuator_velocity)
        self.assertGreaterEqual(frame["saturation_count"], 1)

    def test_thumb_opposes_toward_palm_midline(self):
        from biosyngen.spatial.plant_v2 import HumanV2Plant
        p = HumanV2Plant.build()
        for side in ("l", "r"):
            b = p.body_id(side+"_thumb")
            hand = p.body_id(side+"_hand")
            before = abs(p.data.xipos[b, 1]-p.data.xpos[hand, 1])
            p.data.qpos[p.joint_qpos(side+"_thumb_opposition")] = .9
            mujoco.mj_forward(p.model, p.data)
            after = abs(p.data.xipos[b, 1]-p.data.xpos[hand, 1])
            self.assertLess(after, before)
