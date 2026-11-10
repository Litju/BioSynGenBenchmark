"""Executable V2 anatomy checks with independent engine/analytical witnesses."""
from dataclasses import replace
import unittest
import xml.etree.ElementTree as ET

try:
    import mujoco
    import numpy as np
except ImportError:
    mujoco = None


@unittest.skipIf(mujoco is None, "MuJoCo required by mandatory spatial CI")
class FullBody(unittest.TestCase):
    def setUp(self):
        from biosyngen.spatial.model_v2 import HumanV2Spec, build_mjcf_v2
        self.Spec, self.builder = HumanV2Spec, build_mjcf_v2
        self.m = mujoco.MjModel.from_xml_string(self.builder())
        self.d = mujoco.MjData(self.m)

    def body(self, name):
        return mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, name)

    def test_version_topology_and_no_bar_connect(self):
        root = ET.fromstring(self.builder())
        self.assertTrue(root.get("model").startswith("HUMAN_V2_TORQUE_FULLBODY_"))
        self.assertEqual((self.m.nbody-1, self.m.nv, self.m.nu), (32, 64, 48))
        self.assertFalse(root.findall("./equality/connect"))
        self.assertFalse(root.findall("./contact/exclude"))
        for name in ("lumbar", "thorax", "cervical", "head", "l_clavicle", "r_scapula", "l_thumb", "r_fingers_distal"):
            self.assertGreater(self.body(name), 0)
        self.assertTrue(np.all(self.m.actuator_forcelimited))

    def test_fullbody_mass_symmetry_and_realizable_inertia(self):
        for mass, stature in ((40, 1.4), (75, 1.75), (180, 2.1)):
            from biosyngen.spatial.anthropometry import HumanSubjectProfile
            m = mujoco.MjModel.from_xml_string(self.builder(self.Spec(profile=HumanSubjectProfile(mass, stature))))
            self.assertAlmostEqual(sum(m.body_mass)-20, mass, places=11)
            for b in range(1, m.nbody):
                values = sorted(m.body_inertia[b])
                self.assertGreater(values[0], 0)
                self.assertLessEqual(values[-1], values[0]+values[1]+1e-12)
            for b in range(1, m.nbody):
                name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
                if name.startswith("l_"):
                    right = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "r_"+name[2:])
                    self.assertEqual(m.body_mass[b], m.body_mass[right])
                    np.testing.assert_allclose(m.body_inertia[b], m.body_inertia[right], atol=1e-14)

    def test_cervical_head_and_trunk_are_articulated(self):
        mujoco.mj_forward(self.m, self.d)
        before = self.d.xipos[self.body("head")].copy()
        for name in ("cervical_flex", "thoracic_flex"):
            joint = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.d.qpos[self.m.jnt_qposadr[joint]] = .3
        mujoco.mj_forward(self.m, self.d)
        self.assertGreater(np.linalg.norm(self.d.xipos[self.body("head")]-before), .05)
        # Independent finite difference of head COM against engine Jacobian.
        b = self.body("head")
        jp, jr = np.zeros((3, self.m.nv)), np.zeros((3, self.m.nv))
        mujoco.mj_jac(self.m, self.d, jp, jr, self.d.xipos[b], b)
        j = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, "cervical_flex")
        before = self.d.xipos[b].copy()
        self.d.qpos[self.m.jnt_qposadr[j]] += 1e-6
        mujoco.mj_forward(self.m, self.d)
        np.testing.assert_allclose((self.d.xipos[b]-before)/1e-6, jp[:, self.m.jnt_dofadr[j]], atol=1e-7)

    def test_finger_and_thumb_contact_geometry_moves(self):
        mujoco.mj_forward(self.m, self.d)
        for joint_name, body_name in (("l_finger_flex", "l_fingers_distal"), ("r_thumb_opposition", "r_thumb")):
            b = self.body(body_name)
            before = self.d.xipos[b].copy()
            j = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            self.d.qpos[self.m.jnt_qposadr[j]] = 1
            mujoco.mj_forward(self.m, self.d)
            self.assertGreater(np.linalg.norm(self.d.xipos[b]-before), .01)

    def test_scapular_equations_are_independently_observable(self):
        for side in ("l", "r"):
            for name, value in (("sh_abd", .8), ("scap_up", .32), ("sh_flex", 1), ("scap_tilt", .1)):
                j = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, side+"_"+name)
                self.d.qpos[self.m.jnt_qposadr[j]] = value
        mujoco.mj_forward(self.m, self.d)
        indices = self.d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
        self.assertLess(np.max(np.abs(self.d.efc_pos[indices])), 1e-14)
        j = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, "r_scap_up")
        self.d.qpos[self.m.jnt_qposadr[j]] += .2
        mujoco.mj_forward(self.m, self.d)
        indices = self.d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
        self.assertGreater(np.max(np.abs(self.d.efc_pos[indices])), .19)

    def test_anthropometric_lengths_drive_actual_geometry(self):
        from biosyngen.spatial.anthropometry import HumanSubjectProfile
        p = HumanSubjectProfile()
        p = replace(p, dimensions=replace(p.dimensions, upper_arm_m=.36, shoulder_breadth_m=.44))
        m = mujoco.MjModel.from_xml_string(self.builder(self.Spec(profile=p)))
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "l_forearm")
        self.assertAlmostEqual(m.body_pos[b, 2], -.36)

    def test_free_root_forward_standing_and_replay(self):
        outcomes = []
        for _ in range(2):
            d = mujoco.MjData(self.m)
            for _ in range(1000):
                mujoco.mj_step(self.m, d)
            self.assertTrue(np.isfinite(d.qpos).all())
            self.assertGreater(d.qpos[2], .8)
            outcomes.append(d.qpos.copy())
        np.testing.assert_array_equal(*outcomes)
