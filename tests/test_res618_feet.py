"""Foot candidate compilation/FK/ballistic falsifiers; no receiving PASS."""
import unittest
import mujoco
import numpy as np
from biosyngen.spatial.feet_v2 import ShoeSpec, build_foot_candidate


class Feet(unittest.TestCase):
    def test_partition_and_bounds(self):
        m = mujoco.MjModel.from_xml_string(build_foot_candidate())
        self.assertAlmostEqual(sum(m.body_mass), 135, places=11)
        self.assertEqual(m.nv, 49)
        self.assertEqual(m.nu, 37)
        for side in ("l", "r"):
            b = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, side + "_" + tag)
                 for tag in ("foot", "forefoot", "toes")]
            np.testing.assert_allclose(m.body_mass[b] / sum(m.body_mass[b]), [.55, .30, .15], atol=1e-15)
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, side + "_ankle_dorsi")
            np.testing.assert_array_equal(m.jnt_range[j], [-.9, .75])

    def test_shoe_changes_support_geometry(self):
        heights = []
        for heel in (0, .02):
            m = mujoco.MjModel.from_xml_string(build_foot_candidate(shoe=ShoeSpec(heel)))
            g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "l_hindfoot_sole")
            heights.append(m.geom_pos[g, 2] - m.geom_size[g, 2])
        self.assertAlmostEqual(heights[0] - heights[1], .02)

    def test_mtp_moves_toe_contact(self):
        m = mujoco.MjModel.from_xml_string(build_foot_candidate())
        d = mujoco.MjData(m)
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "l_mtp_flex")
        g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "l_mtp_sole")
        mujoco.mj_forward(m, d)
        before = d.geom_xpos[g].copy()
        d.qpos[m.jnt_qposadr[j]] = .5
        mujoco.mj_forward(m, d)
        self.assertGreater(d.geom_xpos[g, 2], before[2])

    def test_airborne_momentum(self):
        m = mujoco.MjModel.from_xml_string(build_foot_candidate())
        d = mujoco.MjData(m)
        # Isolated free flight, disable grip constraints and all contacts.
        m.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        d.eq_active[:] = False
        d.qpos[2] += 3
        mujoco.mj_forward(m, d)
        before = np.sum(m.body_mass[:, None] * d.xipos, axis=0) / sum(m.body_mass)
        for _ in range(100):
            mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        after = np.sum(m.body_mass[:, None] * d.xipos, axis=0) / sum(m.body_mass)
        # Semi-implicit constant-gravity discrete displacement, exact analytical budget.
        np.testing.assert_allclose(after - before, [0, 0, -9.81 * .001**2 * 100*101/2], atol=2e-8)

    def test_invalid_shoe(self):
        for kw in ({"heel_elevation_m": -.01}, {"friction": float("nan")}, {"sole_thickness_m": True}):
            with self.assertRaises(ValueError):
                ShoeSpec(**kw)
