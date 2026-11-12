"""RES-624 Gate B: relative bar orientation, axial slip and grip-frame falsifiers.

A nearly stationary COM is not a stable grasp. These controls keep the bar
wrench intact and move only the reported relative pose, so the pre-repair
COM-translation test cannot explain the verdict.
"""
import unittest

try:
    import mujoco
    import numpy as np  # noqa: F401
    import scipy  # noqa: F401
except ImportError:
    mujoco = None


@unittest.skipIf(mujoco is None, "MuJoCo/SciPy required by mandatory spatial CI")
class RelativeBarPose(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from scripts.res624_receiving_campaign import build_nominal, run_case
        _, cls.nominal = build_nominal()
        cls.frames, cls.static, plant = run_case("nominal_0p3s_red_window", None, None, .3, {}, cls.nominal)
        cls.dt = plant.model.opt.timestep

    def evaluate(self, frames):
        from biosyngen_benchmark.receiving_acceptance import evaluate_receiving
        return evaluate_receiving(frames, self.dt, self.static["verdict"])

    def test_nominal_relative_pose_is_inside_declared_bounds(self):
        r = self.evaluate(self.frames)
        self.assertEqual(r["outcome"], "RECEIVING_HOLD_QUALIFIED", r.get("reason"))
        pose = r["evidence"]["relative_bar_pose"]
        limits = pose["limits"]
        for side in "lr":
            self.assertLess(pose["max_tilt_deg"][side], limits["tilt_deg"])
            self.assertLess(pose["max_omega_rel_perp_rad_s"][side], limits["omega_rel_perp_rad_s"])
            self.assertLess(pose["max_axial_slip_m"][side], limits["axial_slip_m"])
            self.assertLess(pose["max_contact_site_drift_m"][side], limits["contact_site_drift_m"])
            self.assertLess(pose["initial_grip_axis_misalignment_deg"][side], 5.)
        self.assertTrue(pose["axisymmetric"])
        self.assertFalse(pose["twist_gated"])

    def test_large_tilt_with_almost_no_com_translation_is_refused(self):
        from scripts import res624_adversarial_scenes as scenes
        r = self.evaluate(scenes.rotate_reported_bar_ramp(self.frames, tilt_deg=10.))
        self.assertNotEqual(r["outcome"], "RECEIVING_HOLD_QUALIFIED")
        self.assertTrue(any("RELATIVE_ROTATION" in str(f.get("reason")) for f in r["failures"]), r["failures"])
        self.assertLess(r["evidence"]["bar_displacement_in_hand_m"], .014)

    def test_axial_slip_is_refused(self):
        from scripts import res624_adversarial_scenes as scenes
        r = self.evaluate(scenes.translate_bar_axially(self.frames, .005))
        self.assertEqual(r["outcome"], "GRASP_SLIP", r.get("reason"))
        self.assertTrue(any("AXIAL_SLIP" in str(f.get("reason")) for f in r["failures"]))
        self.assertGreater(r["evidence"]["relative_bar_pose"]["max_axial_slip_m"]["l"], .004)

    def test_incorrect_hand_frame_orientation_is_refused(self):
        from scripts import res624_adversarial_scenes as scenes
        r = self.evaluate(scenes.offset_hand_frame(self.frames, "r", 10., "x"))
        self.assertEqual(r["outcome"], "GRASP_SLIP", r.get("reason"))
        self.assertTrue(any("HAND_FRAME_GRIP_AXIS" in str(f.get("reason")) for f in r["failures"]))

    def test_hands_rotating_in_the_grip_are_refused(self):
        from scripts import res624_adversarial_scenes as scenes
        r = self.evaluate(scenes.ramp_hand_frames(self.frames, "r", 4., "z"))
        self.assertEqual(r["outcome"], "GRASP_SLIP", r.get("reason"))
        self.assertTrue(any(("RELATIVE_ROTATION" in str(f.get("reason"))
                             or "CONTACT_SITE_SLIP" in str(f.get("reason"))
                             or "AXIAL_SLIP" in str(f.get("reason"))
                             or "BAR_MOVED_IN_HAND" in str(f.get("reason"))) for f in r["failures"]))

    def test_shaft_axis_twist_is_not_gated_by_the_relative_pose_criterion(self):
        from scripts import res624_adversarial_scenes as scenes
        from biosyngen_benchmark.grasp_contract import classify_frame
        from biosyngen_benchmark.receiving_acceptance import relative_pose_check
        twist = [f for f in scenes.rotate_reported_bar_ramp(self.frames, twist_deg=30.) if f["t"] >= .1]
        pose = relative_pose_check(twist, "barbell", [classify_frame(f) for f in twist])
        self.assertGreater(pose["max_twist_deg"]["l"], 15.)
        self.assertLess(pose["max_tilt_deg"]["l"], 1.)
        self.assertFalse(pose["twist_gated"])
        self.assertEqual(pose["first"], {})

    def test_legitimate_small_relative_motion_is_permitted(self):
        from scripts import res624_adversarial_scenes as scenes
        r = self.evaluate(scenes.small_legitimate_relative_motion(self.frames))
        self.assertEqual(r["outcome"], "RECEIVING_HOLD_QUALIFIED", r.get("reason"))
        pose = r["evidence"]["relative_bar_pose"]
        self.assertLessEqual(pose["max_tilt_deg"]["l"], .6)
        self.assertLess(pose["max_contact_site_drift_m"]["r"], .005)

    def test_one_hand_with_zero_load_is_not_bilateral(self):
        from scripts import res624_adversarial_scenes as scenes
        r = self.evaluate(scenes.redistribute_hand_load(self.frames, {"l": 2., "r": 0.}))
        self.assertEqual(r["outcome"], "GRASP_NOT_BILATERAL", r.get("reason"))


if __name__ == "__main__":
    unittest.main()
