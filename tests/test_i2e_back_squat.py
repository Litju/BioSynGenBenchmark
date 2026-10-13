"""RES-493 back squat: continuous bilateral support, multi-repetition dynamics.

Positive runs check the mechanics and the latent repetition structure. Each
negative control must surface as an explicit refusal or a measurable
discrepancy, never as a valid-looking repetition.
"""

from __future__ import annotations

from dataclasses import replace
import unittest

from biosyngen.i2.instruments import CableLptModel, observe_dual_plates, observe_lpt
from biosyngen.i2.squat import SquatProtocol, SquatRefused, analyze_squat, bar_state, build_back_squat, squat_lpt
from biosyngen.i2.simulate import body_points_below_floor
from biosyngen.multibody import ContactMode

_CACHE: dict[str, tuple] = {}


def run(**kwargs):
    key = repr(sorted(kwargs.items()))
    if key not in _CACHE:
        scenario = build_back_squat(replace(SquatProtocol(), **kwargs))
        trajectory = scenario.run()
        _CACHE[key] = (scenario, trajectory, analyze_squat(trajectory))
    return _CACHE[key]


BASE = {"repetitions": 2}


class BackSquatMechanicsTests(unittest.TestCase):
    def test_loaded_multi_rep_set_has_eccentric_reversal_concentric_and_lockout(self) -> None:
        scenario, trajectory, analysis = run(**BASE)
        self.assertEqual(trajectory.termination, "DURATION_REACHED")
        self.assertEqual(analysis.refusals, ())
        self.assertFalse(analysis.flight_detected)
        self.assertEqual(len(analysis.reps), 2)
        for rep in analysis.reps:
            self.assertEqual(rep.refusals, ())
            self.assertLess(rep.descent_onset_s, rep.bottom_reversal_s)
            self.assertLessEqual(rep.bottom_reversal_s, rep.ascent_onset_s)
            self.assertLess(rep.ascent_onset_s, rep.rep_end_s)
            self.assertGreater(rep.bar_displacement_m, 0.30)
            self.assertLess(rep.peak_descent_velocity_m_s, -0.3)
            self.assertGreater(rep.peak_ascent_velocity_m_s, 0.3)
        first, second = analysis.reps
        # Identical declared repetitions reproduce: the set is mechanically repeatable.
        self.assertAlmostEqual(first.bar_displacement_m, second.bar_displacement_m, delta=0.005)
        self.assertAlmostEqual(first.rep_end_s - first.descent_onset_s, second.rep_end_s - second.descent_onset_s, delta=0.02)

    def test_continuous_support_no_penetration_no_adhesion_and_momentum_closure(self) -> None:
        scenario, trajectory, _ = run(**BASE)
        model = scenario.model
        system = model.system
        mass = model.total_mass_kg()
        foot = {c for side in model.contact_ids for c in model.contact_ids[side]}
        impulse = [0.0, 0.0]
        for frame in trajectory.frames:
            self.assertEqual(body_points_below_floor(model, frame.state), [])
            for side in model.contact_ids:
                self.assertTrue(any(frame.state.contacts[c].mode is not ContactMode.OPEN for c in model.contact_ids[side]))
            for spec in system.contacts:
                self.assertGreater(system.contact_kinematics(frame.state, spec.contact_id).gap_m, -1e-8)
            h = frame.t_end_s - frame.t_start_s
            for contact_id, sample in frame.contact_forces.items():
                self.assertGreaterEqual(sample.normal_force_n, -1e-6)
                if contact_id in foot:
                    impulse[0] += sample.force_world_n[0] * h
                    impulse[1] += sample.force_world_n[1] * h
        for impact in trajectory.impacts:
            if impact.contact_id in foot:
                impulse[0] += impact.impulse_world_n_s[0]
                impulse[1] += impact.impulse_world_n_s[1]
        span = trajectory.frames[-1].t_end_s
        impulse[1] -= mass * model.gravity_m_s2 * span
        p0 = system.linear_momentum(trajectory.initial_state)
        p1 = system.linear_momentum(trajectory.frames[-1].state)
        scale = mass * model.gravity_m_s2 * span
        self.assertLess(abs(p1[0] - p0[0] - impulse[0]), 1e-7 * scale)
        self.assertLess(abs(p1[1] - p0[1] - impulse[1]), 1e-7 * scale)

    def test_system_com_bar_and_joint_work_are_distinct_quantities(self) -> None:
        scenario, trajectory, analysis = run(**BASE)
        # The bar returns to its start: net work on the bar and on the system COM is small,
        # while the per-phase bar power and COM power differ (body segments move differently).
        system = scenario.model.system
        mass = scenario.model.total_mass_kg()
        bar_mass = scenario.model.load.mass_kg
        rep = analysis.reps[0]
        bar_p, com_p = [], []
        for frame in trajectory.frames:
            if rep.ascent_onset_s <= frame.t_end_s <= rep.rep_end_s:
                bar_v = bar_state(trajectory, frame.state)[1]
                com_v = system.linear_momentum(frame.state)[1] / mass
                bar_p.append(bar_v)
                com_p.append(com_v)
        ratio = max(bar_p) / max(com_p)
        self.assertGreater(ratio, 1.05)  # bar moves faster than the system COM during ascent
        self.assertLess(abs(analysis.bar_work_on_bar_j), 0.05 * bar_mass * 9.81 * rep.bar_displacement_m)
        self.assertNotAlmostEqual(analysis.joint_work_j, analysis.system_com_work_j, delta=1e-6)

    def test_heavier_load_slows_ascent_and_shows_a_sticking_region(self) -> None:
        light = run(repetitions=1)[2].reps[0]
        heavy = run(repetitions=1, load_mass_kg=160.0)[2].reps[0]
        self.assertIsNone(light.sticking_region_s)
        self.assertIsNotNone(heavy.sticking_region_s)
        self.assertLess(heavy.mean_ascent_velocity_m_s, light.mean_ascent_velocity_m_s)
        start, end = heavy.sticking_region_s
        self.assertGreater(end - start, 0.05)


class BackSquatNegativeControlTests(unittest.TestCase):
    def test_overload_failure_is_refused_not_reported_as_a_rep(self) -> None:
        _, trajectory, analysis = run(repetitions=1, load_mass_kg=200.0, ascent_time_s=1.6)
        self.assertTrue(analysis.refusals)
        self.assertTrue(any("REP_NOT_COMPLETED" in r for r in analysis.reps[0].refusals))

    def test_statically_infeasible_torque_demand_is_refused_at_setup(self) -> None:
        with self.assertRaisesRegex(SquatRefused, "UNSAFE_OR_INFEASIBLE_TORQUE_DEMAND"):
            build_back_squat(replace(SquatProtocol(), load_mass_kg=900.0))

    def test_bar_slippage_is_refused_statically_or_detected_dynamically(self) -> None:
        with self.assertRaisesRegex(SquatRefused, "BAR_SLIPPAGE_AT_SETUP"):
            build_back_squat(replace(SquatProtocol(), bar_friction=0.15))
        _, _, analysis = run(repetitions=1, bar_friction=0.40)
        self.assertTrue(any(r.startswith("BAR_SLIPPAGE") for r in analysis.reps[0].refusals))
        self.assertLess(run(repetitions=1)[2].reps[0].bar_slip_m, 1e-4)

    def test_safety_pin_contact_makes_the_rep_support_incomplete(self) -> None:
        _, trajectory, analysis = run(repetitions=1, safety_pin_height_m=1.10)
        rep = analysis.reps[0]
        self.assertTrue(rep.support_incomplete)
        self.assertTrue(any(r.startswith("SUPPORT_INCOMPLETE") for r in rep.refusals))
        self.assertIn("safety_pins", [c for e in trajectory.events for c in e.contact_ids])

    def test_tempo_pause_and_partial_depth_change_latent_rep_structure(self) -> None:
        base = run(repetitions=1)[2].reps[0]
        paused = run(repetitions=1, bottom_pause_s=0.6)[2].reps[0]
        partial = run(repetitions=1, bottom_ankle_knee=(0.35, -1.0))[2].reps[0]
        self.assertGreater(paused.ascent_onset_s - paused.bottom_reversal_s, 0.4)
        self.assertLess(base.ascent_onset_s - base.bottom_reversal_s, 0.15)
        self.assertLess(partial.bar_displacement_m, 0.5 * base.bar_displacement_m)
        self.assertEqual(partial.refusals, ())

    def test_plate_load_identification_requires_declared_body_mass(self) -> None:
        scenario, trajectory, analysis = run(**BASE)
        export = observe_dual_plates(trajectory)
        g = scenario.model.gravity_m_s2
        quiet = [s.fz_total_n for s in export.samples if s.native_time_s < 0.45]
        plate_weight = sum(quiet) / len(quiet)
        self.assertAlmostEqual(plate_weight / analysis.system_weight_n, 1.0, delta=0.01)
        # Plates see athlete + bar; the bar mass follows only with a declared body mass.
        self.assertAlmostEqual((plate_weight - analysis.body_weight_n) / g, scenario.model.load.mass_kg, delta=1.5)
        self.assertGreater(abs(plate_weight - analysis.body_weight_n), 0.4 * analysis.system_weight_n)

    def test_cable_angle_error_and_lpt_rep_segmentation(self) -> None:
        scenario, trajectory, analysis = run(**BASE)
        vertical = observe_lpt(trajectory, squat_lpt(trajectory))
        attachment = squat_lpt(trajectory).attachment
        oblique = observe_lpt(trajectory, CableLptModel(attachment, anchor_m=(0.9, 0.6, 0.0)))
        bar_drop = analysis.reps[0].bar_displacement_m
        vertical_span = max(s.displacement_m for s in vertical.samples) - min(s.displacement_m for s in vertical.samples)
        oblique_span = max(s.displacement_m for s in oblique.samples) - min(s.displacement_m for s in oblique.samples)
        self.assertLess(abs(vertical_span - bar_drop), 0.03 * bar_drop)
        self.assertGreater(abs(oblique_span - bar_drop), 0.10 * bar_drop)
        # Independent LPT segmentation: cable shortening minima (anchor below the bar).
        disp = [s.displacement_m for s in vertical.samples]
        minima = []
        for k in range(1, len(disp) - 1):
            if disp[k] < -0.2 and disp[k] <= disp[k - 1] and disp[k] < disp[k + 1]:
                if not minima or vertical.samples[k].native_time_s - minima[-1] > 0.5:
                    minima.append(vertical.samples[k].native_time_s)
        self.assertEqual(len(minima), len(analysis.reps))
        offset = vertical.metadata["native_time_offset_s"]
        for t, rep in zip(minima, analysis.reps):
            self.assertLess(abs((t - offset) - rep.bottom_reversal_s), 0.05)


if __name__ == "__main__":
    unittest.main()
