"""RES-507 independent work-semantics fixture."""
from types import SimpleNamespace
import unittest
from biosyngen.i2.squat import analyze_squat

class WorkReportingTests(unittest.TestCase):
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
