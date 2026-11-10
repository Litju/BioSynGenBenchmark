"""Analytically authored contact histories; no controller or generator oracle."""
from types import SimpleNamespace as NS
import unittest

from biosyngen.i2.phases import analyze_jump
from biosyngen.multibody import ContactMode


def trajectory(modes, events=()):
    def state(pair):
        return NS(contacts={side: NS(mode=ContactMode.OPEN if opened else ContactMode.STICK)
                            for side, opened in zip(("left", "right"), pair)})
    system = NS(linear_momentum=lambda s: (0.0, 0.0), center_of_mass=lambda s: (0.0, 1.0))
    model = NS(system=system, total_mass_kg=lambda: 1.0, gravity_m_s2=9.81,
               contact_ids={"left": ("left",), "right": ("right",)})
    frames = [NS(state=state(pair), t_start_s=i * .001, t_end_s=(i + 1) * .001,
                 contact_forces={}) for i, pair in enumerate(modes)]
    return NS(model=model, frames=frames, initial_state=state((False, False)),
              exercise_id="CMJ", termination="DURATION_REACHED", events=events,
              impacts=(), control_commands=())


class BilateralReleaseTests(unittest.TestCase):
    def test_alternating_unilateral_release_is_not_flight(self):
        trial = trajectory([(True, False)] * 3 + [(False, True)] * 3)
        self.assertIsNone(analyze_jump(trial).time("takeoff"))

    def test_truncated_release_cannot_establish_persistence(self):
        for count in (1, 2):
            with self.subTest(count=count):
                self.assertIsNone(analyze_jump(trajectory([(True, True)] * count)).time("takeoff"))

    def test_takeoff_begins_at_first_complete_bilateral_window(self):
        trial = trajectory([(True, False)] * 3 + [(False, True)] * 3 + [(True, True)] * 3)
        self.assertEqual(analyze_jump(trial).time("takeoff"), .007)

    def test_terminal_touchdown_does_not_invent_landing_reversal(self):
        event = NS(time_s=.004, event_id="CONTACT_IMPACT", contact_ids=("left",))
        trial = trajectory([(True, True)] * 3 + [(False, False)], (event,))
        analysis = analyze_jump(trial)
        self.assertEqual(analysis.time("touchdown"), .004)
        self.assertIsNone(analysis.time("landing_lowest_point"))
        self.assertIsNone(analysis.phases[-1].end_s)
