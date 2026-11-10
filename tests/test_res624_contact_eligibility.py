"""RES-624 Gate B: compiled collision eligibility, engine detection and drop behavior."""
import unittest

try:
    import mujoco
    import numpy as np  # noqa: F401
except ImportError:
    mujoco = None


@unittest.skipIf(mujoco is None, "MuJoCo required by mandatory spatial CI")
class ContactEligibility(unittest.TestCase):
    def setUp(self):
        from biosyngen.spatial.plant_v2 import HumanV2Plant
        self.plant = HumanV2Plant.build()

    def test_policy_rule_and_engine_agree_for_every_declared_pair(self):
        from scripts.res624_contact_audit import eligibility_matrix
        rows = eligibility_matrix(self.plant)
        bad = [r for r in rows if not r["consistent"]]
        self.assertEqual(bad, [])
        self.assertTrue(any(not r["intended"] for r in rows))  # negatives are exercised

    def test_unplated_v2_bar_lands_on_floor_on_sleeves(self):
        from scripts.res624_contact_audit import drop_test
        r = drop_test(self.plant.model)
        self.assertGreater(r["max_bar_floor_contacts"], 0)
        self.assertAlmostEqual(r["final_bar_z_m"], .025, delta=5e-4)
        self.assertLess(r["final_speed_m_s"], 1e-6)

    def test_pre_repair_masks_reproduce_ineligible_floor_and_fall_through(self):
        from biosyngen.spatial.grasp_diagnostics import pair_eligibility
        from scripts.res624_contact_audit import actual_detection, drop_test
        m = self.plant.model
        for name in ("bar_shaft", "bar_sleeve_l", "bar_sleeve_r"):
            g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
            m.geom_contype[g], m.geom_conaffinity[g] = (2, 4) if name == "bar_shaft" else (0, 0)
        self.assertFalse(pair_eligibility(m, "bar_shaft", "floor")["eligible"])
        self.assertFalse(actual_detection(self.plant, "bar_shaft", "floor"))
        self.assertLess(drop_test(m)["final_bar_z_m"], -1.0)

    def test_broken_pad_mask_is_detected_as_inconsistent(self):
        from scripts.res624_contact_audit import eligibility_matrix
        m = self.plant.model
        g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "l_palm_collision")
        m.geom_contype[g] = m.geom_conaffinity[g] = 0
        row = next(r for r in eligibility_matrix(self.plant) if r["pair"] == "barbell-palm")
        self.assertFalse(row["engine_detected"])
        self.assertFalse(row["consistent"])

    def test_v1_masks_are_untouched(self):
        from biosyngen.spatial.grasp_diagnostics import pair_eligibility
        from biosyngen.spatial.model import AthleteBarbellSpec, build_mjcf
        v1 = mujoco.MjModel.from_xml_string(build_mjcf(AthleteBarbellSpec(plate_mass_kg=0)))
        g = mujoco.mj_name2id(v1, mujoco.mjtObj.mjOBJ_GEOM, "bar_shaft")
        self.assertEqual((int(v1.geom_contype[g]), int(v1.geom_conaffinity[g])), (2, 4))
        self.assertFalse(pair_eligibility(v1, "bar_shaft", "floor")["eligible"])


if __name__ == "__main__":
    unittest.main()
