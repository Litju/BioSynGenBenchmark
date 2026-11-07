"""RES-507 independent mechanical wrench/power and interval reporting controls."""
import unittest
from biosyngen.multibody import (ActuatorHypothesis, ContactKind, ContactNormalFrame, ContactSpec, ParameterProvenance, ParameterStatus, PlanarJoint, PlanarMultibodySystem, RigidBodyDefinition, RigidBodyState)

def provenance(name, uncertainty=0):
    return ParameterProvenance(name, ParameterStatus.MANUFACTURED_TEST_FIXTURE, "independent RES-507 reporting falsifier", uncertainty)

def body(name, mass, inertia):
    return RigidBodyDefinition(name, mass, inertia, provenance(name+":mass"), provenance(name+":inertia"))

class ReportingDefectTests(unittest.TestCase):
    def test_moving_plane_reaction_point_moment_and_power_match_physical_contact(self):
        # A plane pivots at its COM, carrying a block at x=0.3. The material
        # contact point is at the block, not the plane's reference anchor.
        pin = ContactSpec("pin", "plane", (0, 0), (0, 1), 2, 0, ContactKind.RACK, surface_point_m=(0, 0))
        seat = ContactSpec("seat", "block", (0, 0), (0, 1), 0, 0, ContactKind.CATCH,
                           body_b_id="plane", local_point_b_m=(0, 0), normal_frame=ContactNormalFrame.BODY_B)
        system = PlanarMultibodySystem((body("plane", 5, 1), body("block", 1, 0.1)), (), (pin, seat),
                                       gravity_m_s2=9.80665, clock_id="CONTACT_REPORT")
        state = system.make_state({"plane": RigidBodyState((0, 0), 0, (0, 0), 0.5),
                                   "block": RigidBodyState((0.3, 0), 0, (0, 0.15))}, active_contacts=("pin", "seat"))
        result = system.solve_accelerations(state)
        wrench = result.contact_wrenches["seat"]
        with self.subTest(quantity="material contact point"):
            self.assertEqual(wrench.point_b_world_m, wrench.point_a_world_m)
        with self.subTest(quantity="reaction moment"):
            self.assertAlmostEqual(wrench.moment_on_b_about_com_n_m, result.accelerations["plane"].angular_rad_s2, places=11)
        with self.subTest(quantity="contact power"):
            self.assertAlmostEqual(result.contact_power_w, 0, places=11)

    def test_signed_torque_uncertainty_is_an_ordered_interval(self):
        joint = PlanarJoint("hinge", "a", "b", (0, 0), (0, 0))
        actuator = ActuatorHypothesis("torque", "hinge", 10, 0.02, 0.05, provenance("capacity", 0.2))
        system = PlanarMultibodySystem((body("a", 1, 1), body("b", 1, 1)), (joint,), (),
                                       gravity_m_s2=9.80665, clock_id="UNCERTAINTY", actuators=(actuator,))
        state = system.make_state({"a": RigidBodyState((0, 1), 0), "b": RigidBodyState((0, 1), 0)},
                                   activations={"torque": -0.3})
        result = system.solve_accelerations(state)
        lower, upper = result.actuator_capacity_bounds_n_m["torque"]
        self.assertAlmostEqual(lower, -3.6, places=12)
        self.assertAlmostEqual(upper, -2.4, places=12)
