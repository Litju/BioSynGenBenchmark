"""RES-504 analytical mechanics written independently of prior test helpers."""
from dataclasses import replace
import math
import unittest
from biosyngen.multibody import (
    AppliedWrench, ContactKind, ContactNormalFrame, ContactSpec, ContactState, ContactMode,
    ParameterProvenance, ParameterStatus, PlanarMultibodySystem, RigidBodyDefinition, RigidBodyState,
)


def definition(name, mass, inertia):
    def provenance(parameter):
        return ParameterProvenance(name + parameter, ParameterStatus.MANUFACTURED_TEST_FIXTURE,
                                   "independent RES-504 analytical equation", 0.0)
    return RigidBodyDefinition(name, mass, inertia, provenance(":mass"), provenance(":inertia"))


class IndependentMechanicsTests(unittest.TestCase):
    def test_asymmetric_support_reactions_balance_a_known_external_couple(self):
        mass, width, g, torque = 3.7, 0.42, 9.80665, 2.1
        contacts = tuple(ContactSpec(name, "body", (x, -0.8), (0, 1), 0.0, 0.0, ContactKind.BENCH,
                                     surface_point_m=(x, 0), friction_group="one_plane")
                         for name, x in (("left", -width), ("right", width)))
        system = PlanarMultibodySystem((definition("body", mass, 0.3),), (), contacts,
                                       gravity_m_s2=g, clock_id="ANALYTICAL")
        state = system.make_state({"body": RigidBodyState((0, 0.8), 0)}, active_contacts=("left", "right"))
        result = system.solve_accelerations(state, external_wrenches=(AppliedWrench("body", "couple", moment_n_m=torque),))
        self.assertAlmostEqual(result.contact_wrenches["left"].normal_force_n, mass * g / 2 + torque / (2 * width), places=11)
        self.assertAlmostEqual(result.contact_wrenches["right"].normal_force_n, mass * g / 2 - torque / (2 * width), places=11)
        self.assertLess(max(abs(v) for a in result.accelerations.values() for v in (*a.linear_m_s2, a.angular_rad_s2)), 1e-11)

    def test_coulomb_capture_and_sliding_match_closed_form_impulses(self):
        mass, mu, g, dt = 2.3, 0.37, 9.80665, 0.02
        contact = ContactSpec("floor", "body", (0, 0), (0, 1), mu, 0, ContactKind.BAR_FLOOR, surface_point_m=(0, 0))
        system = PlanarMultibodySystem((definition("body", mass, 0.1),), (), (contact,), gravity_m_s2=g, clock_id="ANALYTICAL")
        for speed in (0.01, 0.5, -0.01, -0.5):
            with self.subTest(speed=speed):
                state = system.make_state({"body": RigidBodyState((0, 0), 0, (speed, 0))}, active_contacts=("floor",))
                step = system.advance(state, dt)
                expected = math.copysign(max(0, abs(speed) - mu * g * dt), speed)
                actual = step.state.bodies["body"].velocity_m_s[0]
                self.assertAlmostEqual(actual, expected, places=12)
                normal, tangent = step.contact_impulses_n_s["floor"]
                self.assertAlmostEqual(normal, mass * g * dt, places=12)
                self.assertAlmostEqual(tangent, mass * (expected - speed), places=12)
                self.assertLessEqual(abs(tangent), mu * normal + 1e-12)
                self.assertLessEqual(system.kinetic_energy(step.state), system.kinetic_energy(state) + 1e-12)

    def test_restitution_impulse_and_exact_impact_energy(self):
        mass, speed = 3.2, 1.7
        for restitution in (0.0, 0.3, 1.0):
            with self.subTest(restitution=restitution):
                contact = ContactSpec("floor", "body", (0, 0), (0, 1), 0, restitution,
                                      ContactKind.BAR_FLOOR, surface_point_m=(0, 0))
                system = PlanarMultibodySystem((definition("body", mass, 0.2),), (), (contact,),
                                               gravity_m_s2=9.80665, clock_id="ANALYTICAL")
                state = system.make_state({"body": RigidBodyState((0, 0), 0, (0, -speed))})
                after, event = system.resolve_impact(state, ("floor",))
                self.assertAlmostEqual(after.bodies["body"].velocity_m_s[1], restitution * speed, places=12)
                self.assertAlmostEqual(event.normal_impulses_n_s["floor"], mass * (1 + restitution) * speed, places=12)
                self.assertAlmostEqual(event.contact_work_j, -mass * (1 - restitution**2) * speed**2 / 2, places=12)

    def test_free_fall_energy_error_refines_at_the_declared_integrator_order(self):
        mass, g, duration = 2.3, 9.80665, 0.2
        system = PlanarMultibodySystem((definition("body", mass, 0.2),), (), (), gravity_m_s2=g, clock_id="ANALYTICAL")
        errors = []
        for dt in (0.02, 0.01, 0.005):
            state = system.make_state({"body": RigidBodyState((0, 2), 0)})
            e0 = mass * g * 2
            for _ in range(round(duration / dt)):
                state = system.advance(state, dt).state
            actual = system.kinetic_energy(state) + system.potential_energy(state)
            error = e0 - actual
            self.assertAlmostEqual(error, mass * g**2 * duration * dt / 2, places=10)
            self.assertAlmostEqual(state.bodies["body"].velocity_m_s[1], -g * duration, places=12)
            errors.append(error)
        self.assertAlmostEqual(errors[0] / errors[1], 2, places=10)
        self.assertAlmostEqual(errors[1] / errors[2], 2, places=10)

    def test_rotating_normal_velocity_matches_an_independent_gap_difference(self):
        contact = ContactSpec("plane", "block", (0.2, -0.1), (0, 1), 0.2, 0, ContactKind.CATCH,
                              body_b_id="plane", local_point_b_m=(0.4, 0.2), normal_frame=ContactNormalFrame.BODY_B)
        system = PlanarMultibodySystem((definition("block", 1, 0.2), definition("plane", 5, 0.5)), (), (contact,),
                                       gravity_m_s2=9.80665, clock_id="ANALYTICAL")
        state = system.make_state({"block": RigidBodyState((0.7, 1.2), 0.3, (0.2, -0.1), 0.7),
                                   "plane": RigidBodyState((0, 0), -0.2, (-0.1, 0.2), -0.8)})
        eps = 1e-6
        def gap(t):
            moved = {name: replace(b, position_m=tuple(p + t * v for p, v in zip(b.position_m, b.velocity_m_s)),
                                   angle_rad=b.angle_rad + t * b.angular_velocity_rad_s) for name, b in state.bodies.items()}
            return system.contact_kinematics(system.make_state(moved), "plane").gap_m
        derivative = (gap(eps) - gap(-eps)) / (2 * eps)
        self.assertAlmostEqual(system.contact_kinematics(state, "plane").normal_velocity_m_s, derivative, delta=1e-9)

    def test_stationary_contact_work_and_com_pseudo_work_are_different(self):
        mass, a, height, omega = 3.0, 0.2, 0.8, 0.3
        contact = ContactSpec("pivot", "body", (-a, -height), (0, 1), 2, 0, ContactKind.BENCH, surface_point_m=(0, 0))
        system = PlanarMultibodySystem((definition("body", mass, 0.5),), (), (contact,), gravity_m_s2=9.80665, clock_id="ANALYTICAL")
        state = system.make_state({"body": RigidBodyState((a, height), 0, (-omega * height, omega * a), omega)},
                                  active_contacts=("pivot",))
        result = system.solve_accelerations(state)
        self.assertAlmostEqual(result.contact_power_w, 0.0, places=12)
        wrench = result.contact_wrenches["pivot"].force_a_world_n
        velocity = state.bodies["body"].velocity_m_s
        pseudo_power = sum(f * v for f, v in zip(wrench, velocity))
        self.assertGreater(abs(pseudo_power), 0.1)


if __name__ == "__main__":
    unittest.main()
