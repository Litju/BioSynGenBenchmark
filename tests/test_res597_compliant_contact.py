"""Independent constitutive identities and physical negative controls."""
import math
import unittest
from biosyngen.compliant_contact import HuntCrossleyContact, CompliantPlanarStepper
from biosyngen.multibody import DynamicsError, PlanarMultibodySystem, RigidBodyState
from tests.test_i2c_multibody import body, point_contact


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


def drop(dt, damping):
    system = PlanarMultibodySystem((body("mass", 1, 1),), (),
        (point_contact("floor", "mass", (0, 0), (0, 0), side="LEFT", channel="FZ_LEFT"),),
        gravity_m_s2=9.81, clock_id="manufactured-drop")
    law = HuntCrossleyContact(stiffness=10_000, exponent=1, damping_s_m=damping)
    stepper = CompliantPlanarStepper(system, {"floor": law})
    state = system.make_state({"mass": RigidBodyState((0, .1), 0)})
    impulse = dissipated = 0.0
    peak = 0.0
    for _ in range(round(.3/dt)):
        step, responses = stepper.advance(state, dt)
        r = responses["floor"]
        impulse += r.normal_n*dt
        dissipated += r.dissipated_power_w*dt
        peak = max(peak, r.normal_n)
        if any(e.event_id == "CONTACT_IMPACT" for e in step.events):
            raise AssertionError("compliant surface applied rigid impact")
        state = step.state
    b = state.bodies["mass"]
    energy = .5*b.velocity_m_s[1]**2 + 9.81*b.position_m[1] + law.evaluate(b.position_m[1], b.velocity_m_s[1]).stored_energy_j
    return b.velocity_m_s[1], impulse, energy, dissipated, peak


class CompliantIntegrationTests(unittest.TestCase):
    def test_momentum_and_energy_refinement(self):
        errors = []
        for dt in (.0004, .0002, .0001):
            velocity, impulse, energy, dissipation, peak = drop(dt, .3)
            self.assertAlmostEqual(velocity, impulse-9.81*.3, delta=1e-10)
            self.assertGreater(dissipation, 0)
            self.assertGreater(peak, 0)
            errors.append(abs(energy + dissipation - .981))
        self.assertLess(errors[1], errors[0])
        self.assertLess(errors[2], errors[1])
        self.assertLess(errors[-1], .01*.981)

    def test_elastic_impact_cannot_create_finite_energy(self):
        _, _, energy, dissipation, _ = drop(.0001, 0)
        self.assertEqual(dissipation, 0)
        self.assertAlmostEqual(energy, .981, delta=.003)
