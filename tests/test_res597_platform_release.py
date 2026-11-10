"""Analytical support/release witnesses; no completed-protocol admission."""
import unittest

from biosyngen.i2.drop_jump import build_drop_jump_candidate
from biosyngen.multibody import ContactMode, DynamicsError


class PlatformReleaseTests(unittest.TestCase):
    def test_static_support_release_and_ballistic_momentum(self):
        release = .12345  # Deliberately off the integration grid.
        scenario = build_drop_jump_candidate(platform_release_s=release, duration_s=.2)
        trial = scenario.run(.0004)
        model = scenario.model
        mass, gravity = model.total_mass_kg(), model.gravity_m_s2
        self.assertEqual(trial.termination, "DURATION_REACHED")
        self.assertFalse(trial.impacts)
        releases = [e for e in trial.events if e.time_s == release]
        self.assertEqual({c for e in releases for c in e.contact_ids}, set(scenario.platform_ids))
        for frame in trial.frames:
            platform_force = sum(frame.contact_forces[c].normal_force_n
                                 for c in scenario.platform_ids if c in frame.contact_forces)
            if frame.t_end_s <= release:
                self.assertAlmostEqual(platform_force, mass*gravity, delta=1e-7)
                self.assertAlmostEqual(model.system.kinetic_energy(frame.state), 0, delta=1e-15)
            else:
                self.assertEqual(platform_force, 0)
                self.assertTrue(all(frame.state.contacts[c].mode is ContactMode.OPEN
                                    for c in scenario.platform_ids))
            # There is no ground contact during this short descent.
            self.assertEqual(sum(f.normal_force_n for c, f in frame.contact_forces.items()
                                 if c not in scenario.platform_ids), 0)
            expected_p = -mass*gravity*max(0, frame.t_end_s-release)
            self.assertAlmostEqual(model.system.linear_momentum(frame.state)[1], expected_p, delta=1e-8)

    def test_release_time_is_not_a_body_reset(self):
        trial = build_drop_jump_candidate(platform_release_s=.1, duration_s=.1004).run(.0004)
        before, after = trial.frames[-2:]
        self.assertEqual(before.t_end_s, .1)
        self.assertEqual(after.t_start_s, .1)
        self.assertLess(after.state.bodies["hat"].velocity_m_s[1], 0)
        self.assertLess(abs(after.state.bodies["hat"].position_m[1] -
                            before.state.bodies["hat"].position_m[1]), 2e-6)

    def test_invalid_release_times_refuse(self):
        for time in (-1, 0, True, float("nan"), 1.5, 2):
            with self.subTest(time=time), self.assertRaises(DynamicsError):
                build_drop_jump_candidate(platform_release_s=time)

    def test_contact_energy_ledger_preserves_constitutive_response(self):
        scenario = build_drop_jump_candidate(duration_s=.3)
        trial = scenario.run(.0004)
        previous = trial.initial_state
        dissipated = 0
        for frame in trial.frames:
            for cid, energy in frame.contact_stored_energy_j.items():
                kin = scenario.model.system.contact_kinematics(previous, cid)
                response = scenario.contact_law.evaluate(kin.gap_m, kin.normal_velocity_m_s,
                                                         kin.tangential_velocity_m_s)
                self.assertEqual(energy, response.stored_energy_j)
                self.assertEqual(frame.contact_dissipated_power_w[cid], response.dissipated_power_w)
                self.assertGreaterEqual(response.dissipated_power_w, -1e-12)
                dissipated += response.dissipated_power_w*(frame.t_end_s-frame.t_start_s)
            previous = frame.state
        self.assertGreater(dissipated, 0)

    def test_articulated_energy_residual_refines(self):
        errors = []
        for dt in (.0004, .0002, .0001):
            scenario = build_drop_jump_candidate(duration_s=.5)
            trial = scenario.run(dt)
            system = scenario.model.system
            final = trial.frames[-1].state
            law = scenario.contact_law
            # Independent integral of k*d**n, evaluated at the final geometry.
            stored = sum(law.stiffness*max(0, -system.contact_kinematics(final, cid).gap_m)**
                         (law.exponent+1)/(law.exponent+1) for cid in final.contacts)
            work = sum(f.joint_power_w*(f.t_end_s-f.t_start_s) for f in trial.frames)
            loss = sum(sum(f.contact_dissipated_power_w.values())*(f.t_end_s-f.t_start_s)
                       for f in trial.frames)
            residual = (system.kinetic_energy(final)+system.potential_energy(final)+stored-
                        system.kinetic_energy(trial.initial_state)-system.potential_energy(trial.initial_state)-
                        work+loss)
            errors.append(abs(residual))
        # First-order force evaluation must converge; this is not a small-residual admission.
        self.assertLess(errors[1], errors[0])
        self.assertLess(errors[2], errors[1])
