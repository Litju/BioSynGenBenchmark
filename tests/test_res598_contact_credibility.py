"""Distinct RK4 reference versus production multibody symplectic stepping.

Reference derives the scalar ODE directly, without importing the production
constitutive law or linear solver. Target: manufactured 1 kg contact, no human
or device inference. Initial conditions and parameters match the declared
verification fixture; tolerance covers first-order discretization at 0.1 ms.
"""
import unittest
from tests.test_res597_compliant_contact import drop


def independent_reference(h=.00001):
    def rhs(z, v):
        compression = max(0, -z)
        force = 10000*compression*max(0, 1-.3*v)
        return v, force-9.81
    z, v = .1, 0
    for _ in range(round(.3/h)):
        a = rhs(z, v)
        b = rhs(z+h*a[0]/2, v+h*a[1]/2)
        c = rhs(z+h*b[0]/2, v+h*b[1]/2)
        d = rhs(z+h*c[0], v+h*c[1])
        z += h*(a[0]+2*b[0]+2*c[0]+d[0])/6
        v += h*(a[1]+2*b[1]+2*c[1]+d[1])/6
    return z, v


class ContactCredibilityTests(unittest.TestCase):
    def test_distinct_integrator_velocity_agreement_and_broken_force_control(self):
        _, reference_v = independent_reference()
        velocity, impulse, _, _, _ = drop(.0001, .3)
        self.assertAlmostEqual(velocity, reference_v, delta=.0004)
        # Wrong mass in the independent force ledger must break closure.
        bad_mass = 1.02
        bad_residual = bad_mass*velocity-(impulse-bad_mass*9.81*.3)
        self.assertGreater(abs(bad_residual), .01)

    def test_reference_refinement_is_smaller_than_candidate_error(self):
        _, coarse = independent_reference(.00002)
        _, fine = independent_reference(.00001)
        velocity, *_ = drop(.0001, .3)
        self.assertLess(abs(fine-coarse), abs(velocity-fine)/10)
