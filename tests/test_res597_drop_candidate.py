"""Experimental drop candidate checks; no completed-protocol admission."""
import unittest
from biosyngen.i2.drop_jump import build_drop_jump_candidate


class DropCandidateTests(unittest.TestCase):
    def test_drop_contact_reversal_release_and_external_momentum(self):
        results = []
        for dt in (.0004, .0002):
            scenario = build_drop_jump_candidate(duration_s=.5)
            trial = scenario.run(dt)
            self.assertEqual(trial.termination, "DURATION_REACHED")
            self.assertFalse(trial.impacts)
            commands = {name: time for time, name, _ in trial.control_commands}
            self.assertLess(commands["BRAKING"], commands["REACTIVE_PROPULSION"])
            self.assertLess(commands["REACTIVE_PROPULSION"], commands["FLIGHT"])
            mass = scenario.model.total_mass_kg()
            impulse = -mass*scenario.model.gravity_m_s2*.5
            peak = 0
            for f in trial.frames:
                total = sum(c.normal_force_n for c in f.contact_forces.values())
                self.assertGreaterEqual(total, 0)
                impulse += sum(c.force_world_n[1] for c in f.contact_forces.values())*(f.t_end_s-f.t_start_s)
                peak = max(peak, total)
            final_p = scenario.model.system.linear_momentum(trial.frames[-1].state)[1]
            self.assertAlmostEqual(final_p, impulse, delta=1e-7)
            results.append((commands["BRAKING"], commands["FLIGHT"], peak))
        self.assertLess(abs(results[0][0]-results[1][0]), .0008)
        self.assertLess(abs(results[0][1]-results[1][1]), .005)

    def test_incomplete_terminal_recovery_remains_visible(self):
        trial = build_drop_jump_candidate().run()
        self.assertTrue(trial.termination.startswith("BODY_GROUND_CONTACT_NOT_MODELED"))
        self.assertEqual(trial.scenario["status"], "EXPERIMENTAL_NOT_QUALIFIED")
