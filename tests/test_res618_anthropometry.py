"""Independent analytical checks; does not qualify anatomy or receiving."""
from dataclasses import replace
import unittest

try:
    import mujoco
    import numpy as np
    from biosyngen.spatial.anthropometry import HumanSubjectProfile, SegmentInertia, inertial_migration
    from biosyngen.spatial.model import build_mjcf
except ImportError:
    mujoco = None


@unittest.skipIf(mujoco is None, "MuJoCo required; mandatory in spatial CI")
class Anthropometry(unittest.TestCase):
    def test_compiled_mass_and_inertia_across_profiles(self):
        for mass, height in ((40, 1.4), (75, 1.75), (180, 2.1)):
            m = mujoco.MjModel.from_xml_string(inertial_migration(HumanSubjectProfile(mass, height)))
            bar = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "barbell")
            self.assertAlmostEqual(sum(m.body_mass) - m.body_mass[bar], mass, places=11)
            self.assertTrue(np.all(m.body_mass[1:] > 0))
            self.assertTrue(np.all(m.body_inertia[1:] > 0))
            for side in ("foot", "hand", "girdle", "thigh", "shank", "upper_arm", "forearm"):
                l, r = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, s + "_" + side) for s in ("l", "r")]
                np.testing.assert_allclose(m.body_inertia[l], m.body_inertia[r], atol=0)
                np.testing.assert_allclose(m.body_ipos[l] * (1, -1, 1), m.body_ipos[r], atol=0)

    def test_v1_mass_bug_is_preserved_as_historical_witness(self):
        m = mujoco.MjModel.from_xml_string(build_mjcf())
        self.assertAlmostEqual(sum(m.body_mass) - 60, 75.105, places=10)

    def test_cylinder_analytical_inertia(self):
        m = mujoco.MjModel.from_xml_string(inertial_migration())
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "l_thigh")
        mass = m.body_mass[b]
        expected = sorted([mass * (.43**2 + 3 * .07**2) / 12] * 2 + [mass * .07**2 / 2])
        np.testing.assert_allclose(sorted(m.body_inertia[b]), expected, rtol=2e-9)
        np.testing.assert_allclose(m.body_ipos[b], [0, 0, -.43 * .43], atol=1e-10)

    def test_profile_faults(self):
        for kw in ({"body_mass_kg": True}, {"body_mass_kg": float("nan")}, {"stature_m": 175},
                   {"units": "g,cm,s,deg,N"}, {"parameter_status": "MEASURED"}):
            with self.assertRaises(ValueError):
                replace(HumanSubjectProfile(), **kw)

    def test_inertia_faults(self):
        good = SegmentInertia("a", 1, (0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, 1)))
        for kw in ({"mass_kg": -1}, {"frame": "WORLD"}, {"com_m": (float("nan"), 0, 0)},
                   {"tensor_kg_m2": ((-1, 0, 0), (0, 1, 0), (0, 0, 1))},
                   {"tensor_kg_m2": ((1, .1, 0), (0, 1, 0), (0, 0, 1))},
                   {"tensor_kg_m2": ((3, 0, 0), (0, 1, 0), (0, 0, 1))}):
            with self.assertRaises(ValueError):
                replace(good, **kw)
