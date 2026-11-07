"""Independent constitutive identities and physical negative controls."""
import math
import unittest
from biosyngen.compliant_contact import HuntCrossleyContact
from biosyngen.multibody import DynamicsError


class CompliantContactTests(unittest.TestCase):
    def test_open_contact_has_no_force_or_energy(self):
        self.assertEqual(HuntCrossleyContact().evaluate(.01, -3).normal_n, 0)
        self.assertEqual(HuntCrossleyContact().evaluate(.01, -3).stored_energy_j, 0)

    def test_elastic_force_is_potential_gradient(self):
        law = HuntCrossleyContact(damping_s_m=0)
        d, h = .01, 1e-7
        gradient = (law.evaluate(-d-h, 0).stored_energy_j - law.evaluate(-d+h, 0).stored_energy_j)/(2*h)
        self.assertAlmostEqual(gradient, law.evaluate(-d, 0).normal_n, delta=1e-7)

    def test_nonadhesive_dissipative_and_friction_cone(self):
        law = HuntCrossleyContact()
        for vn in (-10, -1, 0, 1, 10):
            for vt in (-2, 0, 2):
                r = law.evaluate(-.01, vn, vt)
                self.assertGreaterEqual(r.normal_n, 0)
                self.assertGreaterEqual(r.dissipated_power_w, -1e-12)
                self.assertLessEqual(abs(r.tangent_n), law.friction*r.normal_n)
                self.assertLessEqual(r.tangent_n*vt, 0)

    def test_invalid_parameters_and_nonfinite_states_refuse(self):
        for kwargs in ({"stiffness": -1}, {"damping_s_m": -1}, {"friction": -1},
                       {"exponent": .5}, {"stiffness": True}, {"stiffness": math.inf}):
            with self.assertRaises(DynamicsError):
                HuntCrossleyContact(**kwargs)
        for gap in (math.nan, math.inf, True):
            with self.assertRaises(DynamicsError):
                HuntCrossleyContact().evaluate(gap, 0)
