"""Distinct DOP853 reference for the default nonlinear contact and friction law.

Manufactured 1 kg point body only. The 0.5 mm/s velocity budget at 0.01 ms
is a numerical fixture requirement, never a footwear/device accuracy claim.
"""
import math
import json
import unittest

try:
    from scipy.integrate import solve_ivp
except ImportError:
    solve_ivp = None

from biosyngen.compliant_contact import CompliantPlanarStepper, HuntCrossleyContact
from biosyngen.multibody import PlanarMultibodySystem, RigidBodyState
from tests.test_i2c_multibody import body, point_contact


def reference(damping=.5, tolerance=1e-10):
    def rhs(t, state):
        x, z, vx, vz = state
        force = 500000*max(0, -z)**1.5*max(0, 1-damping*vz)
        friction = -.8*force*math.tanh(vx/.01)
        return vx, vz, friction, force-9.81
    result = solve_ivp(rhs, (0, .3), (0, .15, .5, 0), method="DOP853",
                       rtol=tolerance, atol=tolerance/100, max_step=.0001)
    assert result.success
    return result.y[2:4, -1]


def production(dt):
    system = PlanarMultibodySystem((body("mass", 1, 1),), (),
        (point_contact("floor", "mass", (0, 0), (0, 0), side="LEFT", channel="FZ_LEFT"),),
        gravity_m_s2=9.81, clock_id="manufactured-nonlinear-drop")
    stepper = CompliantPlanarStepper(system, {"floor": HuntCrossleyContact()})
    state = system.make_state({"mass": RigidBodyState((0, .15), 0, velocity_m_s=(.5, 0))})
    for _ in range(round(.3/dt)):
        step, _ = stepper.advance(state, dt)
        state = step.state
    return state.bodies["mass"].velocity_m_s


@unittest.skipIf(solve_ivp is None, "optional SciPy; compulsory in the mandatory solver job")
class NonlinearContactScipyTests(unittest.TestCase):
    def test_default_material_and_friction_match_distinct_integrator(self):
        reference_v = reference()
        finer_reference = reference(tolerance=1e-12)
        coarse, fine = production(.00002), production(.00001)
        coarse_error = math.dist(coarse, reference_v)
        fine_error = math.dist(fine, reference_v)
        self.assertLess(fine_error, coarse_error)
        self.assertLess(fine_error, .0005)
        self.assertLess(math.dist(reference_v, finer_reference), fine_error/10)
        # A wrong damping coefficient must not fall inside the numerical budget.
        wrong_damping_error = math.dist(reference_v, reference(damping=1))
        self.assertGreater(wrong_damping_error, .0005)
        print("NONLINEAR_CONTACT_REFERENCE "+json.dumps(dict(
            coarse_velocity_error_m_s=coarse_error, fine_velocity_error_m_s=fine_error,
            reference_refinement_m_s=math.dist(reference_v, finer_reference),
            wrong_damping_discrepancy_m_s=wrong_damping_error, budget_m_s=.0005)), flush=True)
