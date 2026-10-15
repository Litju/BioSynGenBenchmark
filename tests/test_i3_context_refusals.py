"""RES-507 independently authored context canaries with no human data."""
from types import SimpleNamespace as NS
import unittest
from biosyngen.multibody import ContactMode
from biosyngen.i2.human import LoadKind
from biosyngen.i2.qualification import qualify_context


class ContextRefusalTests(unittest.TestCase):
    def test_shelf_only_bar_never_becomes_a_hand_stabilized_jump_claim(self):
        state = NS(contacts={"bar": NS(mode=ContactMode.OPEN)})
        model = NS(load=NS(kind=LoadKind.BACK_BAR), load_body_id="load", load_contact_id="bar", system=NS(joints=()))
        trajectory = NS(model=model, exercise_id="LOADED_JUMP_SQUAT", frames=[NS(state=state, t_end_s=0.7)], termination="DURATION_REACHED")
        result = qualify_context(trajectory)
        self.assertEqual(result["findings"]["ordinary_hand_stabilized_jump_squat"]["code"], "HAND_STABILIZATION_NOT_IMPLEMENTED")
        self.assertEqual(result["findings"]["ordinary_hand_stabilized_jump_squat"]["outcome"], "NOT_EVALUABLE")
        self.assertEqual(result["findings"]["ordinary_hand_stabilized_jump_squat"]["first_bar_open_state_s"], 0.7)

    def test_pelvis_loss_outside_repetitions_is_still_reported(self):
        state = NS(contacts={"bench_upper_back": NS(mode=ContactMode.STICK), "bench_pelvis": NS(mode=ContactMode.OPEN)})
        system = NS(contact_kinematics=lambda state, contact: NS(gap_m=0.002 if contact=="bench_pelvis" else 0))
        trajectory = NS(model=NS(system=system), exercise_id="BENCH_PRESS", termination="DURATION_REACHED",
                        frames=[NS(state=state, controller_stage="UNRACK")])
        finding = qualify_context(trajectory)["findings"]["stable_supine_support_whole_record"]
        self.assertEqual(finding["outcome"], "NOT_EVALUABLE")
        self.assertEqual(finding["open_intervals_by_stage"], {"UNRACK": {"bench_pelvis": 1}})
        self.assertEqual(finding["maximum_modeled_gap_m"]["bench_pelvis"], 0.002)

    def test_numerical_contact_retention_does_not_admit_empirical_force_rfd(self):
        state = NS(contacts={c: NS(mode=ContactMode.STICK) for c in ("bench_upper_back", "bench_pelvis")})
        system = NS(contact_kinematics=lambda *args: NS(gap_m=0))
        trajectory = NS(model=NS(system=system), exercise_id="BENCH_PRESS", termination="DURATION_REACHED",
                        frames=[NS(state=state, controller_stage="PRESS")])
        result = qualify_context(trajectory)
        self.assertEqual(result["findings"]["stable_supine_support_whole_record"]["outcome"], "NUMERICAL_SUPPORT_MAINTAINED")
        for key in ("biological_validation", "real_paired_force_lpt", "physiological_peak_force_rfd"):
            self.assertEqual(result["findings"][key]["outcome"], "NOT_EVALUABLE")
