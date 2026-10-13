"""RES-482/483 verification of the simulated reference CMJ (forward dynamics).

The CMJ is simulated once per test class. Checks are physical invariants or
independent recomputations from exported states, never the solver's own
intermediate values.
"""

from __future__ import annotations

from dataclasses import replace
import math
import unittest

from biosyngen.i1.body import PlanarBody
from biosyngen.i1.phases import analyze_phases
from biosyngen.i1.scenarios import reference_cmj
from biosyngen.i1.simulate import ContactMode, SolverSettings, simulate_jump


_CACHE: dict = {}


def reference():
    if "cmj" not in _CACHE:
        _CACHE["cmj"] = simulate_jump(reference_cmj())
    return _CACHE["cmj"]


class EmergentContactTests(unittest.TestCase):
    def test_takeoff_emerges_from_contact_force_not_from_a_scheduled_time(self) -> None:
        traj = reference()
        self.assertEqual(traj.termination, "LANDING_CONTACT_NOT_MODELED")
        takeoff = traj.event_time("TAKEOFF")
        self.assertIsNotNone(takeoff)
        breakpoints = [e.time_s for e in traj.events if e.kind == "CONTROL_BREAKPOINT"]
        self.assertGreater(min(abs(takeoff - b) for b in breakpoints), 0.05)
        # Last contact sample before release carries ~zero normal force.
        last_contact = [s for s in traj.samples if s.time_s <= takeoff and s.mode is not ContactMode.FLIGHT][-1]
        weight = traj.samples[0].grf_z_n
        self.assertLess(abs(last_contact.grf_z_n), 1e-6 * weight)
        self.assertGreater(traj.sample_at_or_after(takeoff).system_com_velocity_m_s[1], 0.0)

    def test_contact_forces_are_nonadhesive_frictionally_admissible_and_cop_is_on_the_foot(self) -> None:
        traj = reference()
        weight = traj.samples[0].grf_z_n
        mu = traj.scenario.body.friction_coefficient
        for s in traj.samples:
            if s.mode is ContactMode.FLIGHT:
                self.assertEqual((s.grf_x_n, s.grf_z_n), (0.0, 0.0))
                continue
            self.assertGreaterEqual(s.heel_force_n[1], -1e-6 * weight)
            self.assertGreaterEqual(s.toe_force_n[1], -1e-6 * weight)
            self.assertLessEqual(abs(s.grf_x_n), mu * s.grf_z_n + 1e-6 * weight)
            if s.cop_x_m is not None and s.mode is ContactMode.FLAT:
                self.assertGreaterEqual(s.cop_x_m, s.heel_m[0] - 1e-9)
                self.assertLessEqual(s.cop_x_m, s.toe_m[0] + 1e-9)
            self.assertLess(s.constraint_residual_m, 1e-9)

    def test_no_body_point_other_than_the_foot_penetrates_the_ground(self) -> None:
        # Regression for defect 8 (VANDV.md): only foot points carry contact, so
        # every other point must stay above ground in every recorded sample of
        # every reference scenario that ends in a foot landing.
        from biosyngen.i1.scenarios import reference_loaded_cmj, reference_sj

        for traj in (reference(), simulate_jump(reference_loaded_cmj()), simulate_jump(reference_sj())):
            body = PlanarBody(traj.scenario.body, traj.scenario.bar)
            self.assertEqual(traj.termination, "LANDING_CONTACT_NOT_MODELED", traj.scenario.name)
            for s in traj.samples:
                for name in ("knee", "hip", "shoulder", "head_top"):
                    self.assertGreaterEqual(body.point(list(s.q), body.points[name])[1], -1e-9, name)

    def test_activations_stay_within_declared_excitation_bounds(self) -> None:
        traj = reference()
        u_min = traj.scenario.nm.minimum_excitation
        for s in traj.samples:
            for a in s.activation:
                self.assertGreaterEqual(a, u_min - 1e-12)
                self.assertLessEqual(a, 1.0 + 1e-12)


class ConservationTests(unittest.TestCase):
    def test_work_energy_identity_holds_over_the_whole_jump(self) -> None:
        traj = reference()
        e0 = traj.samples[0].kinetic_energy_j + traj.samples[0].potential_energy_j
        worst = max(
            abs(s.kinetic_energy_j + s.potential_energy_j - e0
                - (s.work_joint_j + s.work_coupling_damping_j + s.work_contact_j))
            for s in traj.samples
        )
        peak_work = max(abs(s.work_joint_j) for s in traj.samples)
        self.assertLess(worst, 1e-6 * peak_work)
        # Sticking contact does (almost) no work; it is bounded, not tuned away.
        self.assertLess(max(abs(s.work_contact_j) for s in traj.samples), 1e-2)

    def test_flight_is_ballistic_and_conserves_angular_momentum_about_the_com(self) -> None:
        traj = reference()
        body = PlanarBody(traj.scenario.body, traj.scenario.bar)
        g = traj.scenario.body.gravity_m_s2
        flight = [s for s in traj.samples if s.mode is ContactMode.FLIGHT]
        self.assertGreater(len(flight), 50)
        # Exact identity v(t) = v(t0) - g (t - t0); the bound is the integrator
        # tolerance scale (rtol 1e-8 on states of order 1), not a tuned value.
        t0, v0 = flight[0].time_s, flight[0].system_com_velocity_m_s
        for s in flight:
            self.assertAlmostEqual(s.system_com_velocity_m_s[1], v0[1] - g * (s.time_s - t0), delta=1e-7)
            self.assertAlmostEqual(s.system_com_velocity_m_s[0], v0[0], delta=1e-7)

        def h_com(s):
            h_origin = body.angular_momentum(list(s.q), list(s.qd), (0.0, 0.0))
            (x, z), (vx, vz) = s.system_com_m, s.system_com_velocity_m_s
            return h_origin - body.total_mass_kg * (x * vz - z * vx)

        h0 = h_com(flight[0])
        joints_move = max(abs(w) for s in flight for w in s.joint_velocity_rad_s)
        self.assertGreater(joints_move, 1.0)  # internal motion is not frozen
        for s in flight:
            self.assertAlmostEqual(h_com(s), h0, delta=1e-6)

    def test_stance_impulse_equals_momentum_change(self) -> None:
        traj = reference()
        takeoff = traj.event_time("TAKEOFF")
        stance = [s for s in traj.samples if s.time_s <= takeoff and s.mode is not ContactMode.FLIGHT]
        weight = traj.samples[0].grf_z_n
        mass = weight / traj.scenario.body.gravity_m_s2
        impulse = sum(
            0.5 * (a.grf_z_n + b.grf_z_n - 2 * weight) * (b.time_s - a.time_s)
            for a, b in zip(stance, stance[1:])
        )
        dp = mass * (stance[-1].system_com_velocity_m_s[1] - stance[0].system_com_velocity_m_s[1])
        # Trapezoid error on <= 1 ms samples; bound is 0.1 % of the momentum change.
        self.assertLess(abs(impulse - dp), 1e-3 * abs(dp))


class PhaseTests(unittest.TestCase):
    def test_cmj_phases_emerge_in_physical_order(self) -> None:
        traj = reference()
        pa = analyze_phases(traj, "CMJ")
        self.assertEqual(pa.refusals, ())
        command = pa.event("control_command").time_s
        onset = pa.event("kinetic_onset").time_s
        vmin = pa.event("peak_downward_velocity").time_s
        reversal = pa.event("com_velocity_reversal").time_s
        takeoff = pa.event("takeoff").time_s
        touchdown = pa.event("touchdown").time_s
        self.assertTrue(command <= onset < vmin < reversal < takeoff < touchdown)
        stance = [s for s in traj.samples if s.time_s <= takeoff]
        lowest = min(stance, key=lambda s: s.system_com_m[1])
        self.assertLess(abs(lowest.time_s - reversal), 2e-3)
        weight = pa.weight_n
        for s in traj.samples:
            label_time = s.time_s
            if onset + 2e-3 < label_time < vmin - 2e-3:
                self.assertLess(s.grf_z_n, weight)
                self.assertLess(s.system_com_velocity_m_s[1], 0.0)
            if vmin + 2e-3 < label_time < reversal - 2e-3:
                self.assertGreater(s.grf_z_n, weight)
                self.assertLess(s.system_com_velocity_m_s[1], 0.0)
            if reversal + 2e-3 < label_time < takeoff - 2e-3:
                self.assertGreater(s.system_com_velocity_m_s[1], 0.0)
        self.assertGreater(pa.countermovement_depth_m, 0.05)


class NumericalConvergenceAndScalingTests(unittest.TestCase):
    def test_takeoff_converges_under_tolerance_refinement(self) -> None:
        coarse = simulate_jump(replace(reference_cmj(), settings=SolverSettings(rtol=1e-6, atol=1e-8)))
        fine = reference()
        dt = abs(coarse.event_time("TAKEOFF") - fine.event_time("TAKEOFF"))
        dv = abs(
            coarse.sample_at_or_after(coarse.event_time("TAKEOFF")).system_com_velocity_m_s[1]
            - fine.sample_at_or_after(fine.event_time("TAKEOFF")).system_com_velocity_m_s[1]
        )
        _CACHE["refinement"] = (dt, dv)
        self.assertLess(dt, 1e-4)
        self.assertLess(dv, 1e-4)

    def test_uniform_mass_and_strength_scaling_preserves_kinematics(self) -> None:
        base = reference_cmj()
        factor = 1.2
        scaled = replace(
            base,
            body=replace(base.body, body_mass_kg=base.body.body_mass_kg * factor),
            nm=base.nm.scaled_strength(factor),
        )
        a, b = reference(), simulate_jump(scaled)
        ta, tb = a.event_time("TAKEOFF"), b.event_time("TAKEOFF")
        self.assertAlmostEqual(ta, tb, delta=1e-7)
        sa, sb = a.sample_at_or_after(ta), b.sample_at_or_after(tb)
        for x, y in zip(sa.joint_angle_rad, sb.joint_angle_rad):
            self.assertAlmostEqual(x, y, delta=1e-6)
        self.assertAlmostEqual(sa.system_com_velocity_m_s[1], sb.system_com_velocity_m_s[1], delta=1e-6)
        peak_a = max(s.grf_z_n for s in a.samples)
        peak_b = max(s.grf_z_n for s in b.samples)
        self.assertAlmostEqual(peak_b / peak_a, factor, delta=1e-5)

    def test_too_weak_actuators_do_not_produce_a_fake_takeoff(self) -> None:
        base = reference_cmj()
        weak = simulate_jump(replace(base, nm=base.nm.scaled_strength(0.45)))
        self.assertIsNone(weak.event_time("TAKEOFF"))
        self.assertNotEqual(weak.termination, "LANDING_CONTACT_NOT_MODELED")
        self.assertIn("NO_TAKEOFF", " ".join(analyze_phases(weak).refusals))


if __name__ == "__main__":
    unittest.main()
