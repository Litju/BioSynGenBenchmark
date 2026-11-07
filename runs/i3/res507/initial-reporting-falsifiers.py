"""RES-507 independent falsifiers for misleading reporting/domain acceptance."""
import math
from types import SimpleNamespace
import unittest
from biosyngen.multibody import (
    ActuatorHypothesis, ContactKind, ContactNormalFrame, ContactSpec, ParameterProvenance,
    ParameterStatus, PlanarJoint, PlanarMultibodySystem, RigidBodyDefinition, RigidBodyState,
)
from biosyngen.i2.bench import BenchRefused, system_kinetics_from_plates
from biosyngen.i2.power_clean import PowerCleanRefused, front_rack_feasibility
from biosyngen.i2.squat import analyze_squat


def provenance(name, uncertainty=0):
    return ParameterProvenance(name, ParameterStatus.MANUFACTURED_TEST_FIXTURE,
                               "independent RES-507 reporting falsifier", uncertainty)


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

    def test_work_names_identify_the_actual_com_pseudo_work_and_vertical_estimator(self):
        # A deliberately tiny reporting fixture: force 20 N, bar vz 2 m/s,
        # system vz 1 m/s for 0.5 s. No generator is used as the work oracle.
        load_state = SimpleNamespace(position_m=(0, 1), velocity_m_s=(0, 2))
        state = SimpleNamespace(bodies={"load": load_state}, contacts={})
        system = SimpleNamespace(linear_momentum=lambda _: (0, 4))
        model = SimpleNamespace(system=system, gravity_m_s2=10, total_mass_kg=lambda: 4, load=SimpleNamespace(mass_kg=1),
                                load_body_id="load", load_contact_id="shelf", contact_ids={})
        sample = SimpleNamespace(force_world_n=(0, 20))
        frame = SimpleNamespace(state=state, t_start_s=0, t_end_s=0.5, contact_forces={"shelf": sample}, joint_power_w=7)
        trajectory = SimpleNamespace(model=model, frames=[frame], initial_state=state, termination="DURATION_REACHED")
        analysis = analyze_squat(trajectory)
        # No feet in this fixture: foot-to-COM pseudo-work is zero. Bar work
        # is only its vertical force/velocity product, not total contact work.
        self.assertEqual(analysis.system_com_pseudo_work_j, 0)
        self.assertEqual(analysis.bar_contact_vertical_work_estimate_j, 20)
        self.assertIn("pseudo", analysis.work_definitions["system_com_pseudo_work_j"])

    def test_bench_whole_system_force_requires_explicit_rack_reaction(self):
        with self.assertRaisesRegex(BenchRefused, "rack"):
            system_kinetics_from_plates(100, 600)
        self.assertEqual(system_kinetics_from_plates(100, 600, rack_reaction_n=300), 1000)
        self.assertEqual(system_kinetics_from_plates(100, 600, rack_reaction_n=0), 700)


class ReachDomainTests(unittest.TestCase):
    def test_over_reach_is_not_clipped_into_a_feasible_straight_arm(self):
        result = front_rack_feasibility(rack_offset_m=2.0)
        self.assertFalse(result.feasible)
        self.assertIsNone(result.required_elbow_flexion_rad)

    def test_nonfinite_or_invalid_projection_geometry_is_refused(self):
        for arguments in ({"rack_offset_m": math.nan}, {"rack_offset_m": math.inf}, {"rack_offset_m": True},
                          {"grip_projection": 0}, {"grip_projection": -1}, {"grip_projection": 1.2}):
            with self.subTest(arguments=arguments), self.assertRaises(PowerCleanRefused):
                front_rack_feasibility(**arguments)


if __name__ == "__main__":
    unittest.main()
