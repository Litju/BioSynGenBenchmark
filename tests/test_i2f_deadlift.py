"""RES-494 deadlift: pre-tension, bar-floor support transfer, lift, lockout and return."""

from __future__ import annotations

from dataclasses import replace
import unittest

from biosyngen.i2.deadlift import (
    DeadliftProtocol,
    DeadliftRefused,
    analyze_deadlift,
    build_deadlift,
    deadlift_lpt,
    system_force_from_plates,
)
from biosyngen.i2.instruments import observe_dual_plates, observe_lpt
from biosyngen.i2.phases import foot_force_z
from biosyngen.i2.simulate import body_points_below_floor
from biosyngen.multibody import ContactMode

_CACHE: dict[str, tuple] = {}


def run(**kwargs):
    key = repr(sorted(kwargs.items()))
    if key not in _CACHE:
        scenario = build_deadlift(replace(DeadliftProtocol(), **kwargs))
        trajectory = scenario.run()
        _CACHE[key] = (scenario, trajectory, analyze_deadlift(trajectory))
    return _CACHE[key]


def bar_floor_force(trajectory, frame) -> float:
    sample = frame.contact_forces.get(trajectory.model.bar_floor_contact_id)
    return 0.0 if sample is None else sample.force_world_n[1]


class DeadliftSupportTransferTests(unittest.TestCase):
    def test_setup_is_exactly_static_with_bar_weight_on_the_floor_not_the_plates(self) -> None:
        scenario, trajectory, analysis = run()
        bar_weight = analysis.system_weight_n - analysis.body_weight_n
        for frame in trajectory.frames:
            if frame.t_end_s >= scenario.protocol.setup_hold_s - 0.002:
                break
            self.assertAlmostEqual(foot_force_z(trajectory, frame), analysis.body_weight_n, delta=1e-6)
            self.assertAlmostEqual(bar_floor_force(trajectory, frame), bar_weight, delta=1e-6)

    def test_plate_force_rises_while_the_bar_is_still_on_the_floor(self) -> None:
        scenario, trajectory, analysis = run()
        self.assertEqual(analysis.refusals, ())
        bar_weight = analysis.system_weight_n - analysis.body_weight_n
        self.assertGreater(analysis.lift_off_s - analysis.pretension_onset_s, 0.3)
        start_z = trajectory.initial_state.bodies["load"].position_m[1]
        for frame in trajectory.frames:
            if frame.t_end_s >= analysis.lift_off_s - 0.002:
                break
            # Before lift-off: the bar has not moved, and plate rise equals the
            # bar-floor reaction drop (quasi-static transfer, small inertia).
            self.assertLess(abs(frame.state.bodies["load"].position_m[1] - start_z), 1e-6)
            transfer = (foot_force_z(trajectory, frame) - analysis.body_weight_n) + (bar_floor_force(trajectory, frame) - bar_weight)
            self.assertLess(abs(transfer), 0.03 * bar_weight)
        self.assertAlmostEqual(analysis.plate_rise_before_lift_n, bar_weight, delta=0.05 * bar_weight)

    def test_lift_off_is_the_bar_floor_contact_opening_and_phases_are_ordered(self) -> None:
        scenario, trajectory, analysis = run()
        events = (analysis.pretension_onset_s, analysis.lift_off_s, analysis.bar_passes_knee_s,
                  analysis.lockout_s, analysis.descent_onset_s, analysis.bar_floor_return_s)
        self.assertTrue(all(t is not None for t in events))
        self.assertEqual(list(events), sorted(events))
        lift_frame = next(f for f in trajectory.frames if f.t_end_s >= analysis.lift_off_s)
        self.assertIs(lift_frame.state.contacts["bar_floor"].mode, ContactMode.OPEN)
        self.assertTrue(any(e.event_id == "CONTACT_RELEASE" and "bar_floor" in e.contact_ids for e in trajectory.events))
        self.assertGreater(analysis.peak_bar_height_m, 0.45)
        final = trajectory.frames[-1].state
        self.assertIsNot(final.contacts["bar_floor"].mode, ContactMode.OPEN)

    def test_momentum_closes_with_feet_and_bar_floor_as_external_supports(self) -> None:
        scenario, trajectory, analysis = run()
        model = scenario.model
        system = model.system
        external = {c for v in model.contact_ids.values() for c in v} | {"bar_floor"}
        mass = model.total_mass_kg()
        impulse = [0.0, -mass * model.gravity_m_s2 * trajectory.frames[-1].t_end_s]
        for frame in trajectory.frames:
            h = frame.t_end_s - frame.t_start_s
            self.assertEqual(body_points_below_floor(model, frame.state), [])
            for spec in system.contacts:
                self.assertGreater(system.contact_kinematics(frame.state, spec.contact_id).gap_m, -1e-8)
            for contact_id, sample in frame.contact_forces.items():
                self.assertGreaterEqual(sample.normal_force_n, -1e-6)
                if contact_id in external:
                    impulse[0] += sample.force_world_n[0] * h
                    impulse[1] += sample.force_world_n[1] * h
        for impact in trajectory.impacts:
            if impact.contact_id in external:
                impulse[0] += impact.impulse_world_n_s[0]
                impulse[1] += impact.impulse_world_n_s[1]
        p0 = system.linear_momentum(trajectory.initial_state)
        p1 = system.linear_momentum(trajectory.frames[-1].state)
        scale = mass * model.gravity_m_s2 * trajectory.frames[-1].t_end_s
        self.assertLess(abs(p1[0] - p0[0] - impulse[0]), 1e-7 * scale)
        self.assertLess(abs(p1[1] - p0[1] - impulse[1]), 1e-7 * scale)


class DeadliftNegativeControlTests(unittest.TestCase):
    def test_load_not_lifted_is_refused(self) -> None:
        _, trajectory, analysis = run(load_mass_kg=320.0)
        self.assertIsNone(analysis.lift_off_s)
        self.assertIn("LOAD_NOT_LIFTED: the bar-floor reaction never reached zero", analysis.refusals)
        self.assertTrue(all(f.state.contacts["bar_floor"].mode is not ContactMode.OPEN for f in trajectory.frames))

    def test_system_wrench_needs_an_independent_bar_floor_reaction(self) -> None:
        scenario, trajectory, analysis = run()
        export = observe_dual_plates(trajectory)
        quiet = [s.fz_total_n for s in export.samples if s.native_time_s < 0.45]
        plate = sum(quiet) / len(quiet)
        # Constant loaded-mass assumption is false before lift-off.
        self.assertAlmostEqual(plate / analysis.body_weight_n, 1.0, delta=0.01)
        self.assertLess(plate, 0.6 * analysis.system_weight_n)
        with self.assertRaisesRegex(DeadliftRefused, "NOT_EVALUABLE"):
            system_force_from_plates(plate, None)
        floor = bar_floor_force(trajectory, trajectory.frames[100])
        self.assertAlmostEqual(system_force_from_plates(plate, floor) / analysis.system_weight_n, 1.0, delta=0.01)
        lockout = [s.fz_total_n for s in export.samples
                   if analysis.lockout_s + 0.1 < s.native_time_s - export.metadata["native_time_offset_s"] < analysis.lockout_s + 0.5]
        self.assertAlmostEqual(sum(lockout) / len(lockout) / analysis.system_weight_n, 1.0, delta=0.02)

    def test_misrouted_lpt_shows_a_false_onset_before_bar_lift_off(self) -> None:
        scenario, trajectory, analysis = run()

        def onset(export, threshold):
            base = export.samples[0].displacement_m
            first = next(s for s in export.samples if abs(s.displacement_m - base) > threshold)
            return first.native_time_s - export.metadata["native_time_offset_s"]

        bar = observe_lpt(trajectory, deadlift_lpt(trajectory))
        hip = observe_lpt(trajectory, deadlift_lpt(trajectory, routed_to="HIP_BELT"))
        count = 1.0e-4  # one encoder count
        lags = [onset(bar, threshold) - analysis.lift_off_s for threshold in (count, 0.001, 0.003)]
        # A correctly routed bar LPT never fires before lift-off; threshold
        # estimators lag it, more so for larger thresholds (estimator limit).
        self.assertTrue(all(lag >= 0.0 for lag in lags))
        self.assertEqual(lags, sorted(lags))
        self.assertLess(lags[0], 0.1)
        # The hip belt moves during pre-tension: a false onset well before lift-off.
        self.assertLess(onset(hip, count), analysis.lift_off_s - 0.2)
        with self.assertRaises(DeadliftRefused):
            deadlift_lpt(trajectory, routed_to="KNEE")

    def test_cable_slack_is_flagged_when_the_reel_cannot_follow_the_lowering(self) -> None:
        scenario, trajectory, analysis = run()
        normal = observe_lpt(trajectory, deadlift_lpt(trajectory))
        slow = observe_lpt(trajectory, deadlift_lpt(trajectory, max_retraction_speed_m_s=0.2))
        self.assertEqual(sum(s.slack for s in normal.samples), 0)
        slack_times = [s.native_time_s - slow.metadata["native_time_offset_s"] for s in slow.samples if s.slack]
        self.assertTrue(slack_times)
        self.assertGreater(min(slack_times), analysis.descent_onset_s - 0.05)

    def test_bar_never_penetrates_the_floor_on_return(self) -> None:
        scenario, trajectory, analysis = run()
        radius = scenario.model.load.plate_radius_m
        self.assertGreater(min(f.state.bodies["load"].position_m[1] for f in trajectory.frames), radius - 1e-8)
        returns = [e for e in trajectory.events if "bar_floor" in e.contact_ids and e.event_id in ("CONTACT_IMPACT", "CONTACT_TOUCHDOWN")]
        self.assertTrue(returns)
        for event in returns:
            if event.event_id == "CONTACT_IMPACT":
                self.assertLessEqual(event.contact_work_j, 1e-9)

    def test_setup_that_cannot_reach_the_bar_is_refused(self) -> None:
        with self.assertRaises(DeadliftRefused):
            build_deadlift(replace(DeadliftProtocol(), bar_over_ankle_x_m=0.9))


if __name__ == "__main__":
    unittest.main()
