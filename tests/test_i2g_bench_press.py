"""RES-495 bench press: supine pressing with bench, feet, chest and rack supports."""

from __future__ import annotations

from dataclasses import replace
import unittest

from biosyngen.i1.instruments import cable_length
from biosyngen.i2.bench import (
    BENCH_CONTACTS,
    BenchProtocol,
    BenchRefused,
    analyze_bench,
    bench_lpt,
    build_bench_press,
    claim,
    support_reaction,
    system_kinetics_from_plates,
)
from biosyngen.i2.instruments import attachment_world, observe_lpt
from biosyngen.multibody import ContactMode

_CACHE: dict[str, tuple] = {}


def run(**kwargs):
    key = repr(sorted(kwargs.items()))
    if key not in _CACHE:
        scenario = build_bench_press(replace(BenchProtocol(), **kwargs))
        trajectory = scenario.run()
        _CACHE[key] = (scenario, trajectory, analyze_bench(trajectory))
    return _CACHE[key]


class BenchPressMechanicsTests(unittest.TestCase):
    def test_setup_is_exactly_static_with_separate_bench_foot_and_rack_reactions(self) -> None:
        scenario = build_bench_press()
        model = scenario.model
        state = scenario.initial_state
        result = model.system.solve_accelerations(state, excitations=dict(state.activations))
        worst = max(abs(v) for a in result.accelerations.values() for v in (*a.linear_m_s2, a.angular_rad_s2))
        self.assertLess(worst, 1e-9)
        g = model.gravity_m_s2
        bar_weight = model.load.mass_kg * g
        body_weight = model.total_mass_kg() * g - bar_weight
        wrenches = result.contact_wrenches
        feet = sum(wrenches[c].normal_force_n for side in model.contact_ids for c in model.contact_ids[side])
        bench = sum(wrenches[c].normal_force_n for c in BENCH_CONTACTS)
        self.assertAlmostEqual(feet + bench, body_weight, delta=1e-6)
        self.assertAlmostEqual(wrenches["rack_hooks"].normal_force_n, bar_weight, delta=1e-6)
        self.assertGreater(bench, 3.0 * feet)  # a supine lifter is mostly bench-supported

    def test_multi_rep_set_has_unrack_touch_press_lockout_and_rerack(self) -> None:
        scenario, trajectory, analysis = run()
        self.assertEqual(trajectory.termination, "DURATION_REACHED")
        self.assertEqual(analysis.refusals, ())
        self.assertIsNotNone(analysis.unrack_s)
        self.assertIsNotNone(analysis.rerack_s)
        self.assertEqual(len(analysis.reps), 2)
        for rep in analysis.reps:
            self.assertEqual(rep.refusals, ())
            self.assertLess(analysis.unrack_s, rep.lowering_onset_s)
            self.assertLess(rep.lowering_onset_s, rep.chest_touch_s)
            self.assertLessEqual(abs(rep.chest_touch_s - rep.bottom_reversal_s), 0.02)
            self.assertLess(rep.bottom_reversal_s, rep.lockout_s)
            self.assertLess(rep.lockout_s, analysis.rerack_s)
            self.assertGreater(rep.bar_travel_m, 0.4)
            self.assertLess(rep.chest_impulse_n_s, 5.0)
            self.assertEqual(rep.bench_contact_lost, ())
            self.assertFalse(rep.rack_contact_during_rep)

    def test_momentum_closes_with_feet_bench_and_rack_as_external_supports(self) -> None:
        scenario, trajectory, analysis = run()
        model = scenario.model
        system = model.system
        external = {c for v in model.contact_ids.values() for c in v} | set(BENCH_CONTACTS) | {"rack_hooks"}
        mass = model.total_mass_kg()
        impulse = [0.0, -mass * model.gravity_m_s2 * trajectory.frames[-1].t_end_s]
        for frame in trajectory.frames:
            h = frame.t_end_s - frame.t_start_s
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
        p0, p1 = system.linear_momentum(trajectory.initial_state), system.linear_momentum(trajectory.frames[-1].state)
        scale = mass * model.gravity_m_s2 * trajectory.frames[-1].t_end_s
        self.assertLess(abs(p1[0] - p0[0] - impulse[0]), 1e-7 * scale)
        self.assertLess(abs(p1[1] - p0[1] - impulse[1]), 1e-7 * scale)

    def test_feet_only_plates_are_incomplete_and_bench_reaction_closes_the_balance(self) -> None:
        scenario, trajectory, analysis = run()
        rep = analysis.reps[0]
        hold = [f for f in trajectory.frames if rep.lockout_s + 0.1 < f.t_end_s < rep.lockout_s + 0.35]
        feet = sum(support_reaction(trajectory, f)["feet"] for f in hold) / len(hold)
        bench = sum(support_reaction(trajectory, f)["bench"] for f in hold) / len(hold)
        self.assertLess(feet, 0.3 * analysis.system_weight_n)
        with self.assertRaisesRegex(BenchRefused, "NOT_EVALUABLE"):
            system_kinetics_from_plates(feet, None)
        self.assertAlmostEqual(system_kinetics_from_plates(feet, bench) / analysis.system_weight_n, 1.0, delta=0.03)

    def test_lpt_measures_the_bar_attachment_not_the_system_com(self) -> None:
        scenario, trajectory, analysis = run()
        model = bench_lpt(trajectory)
        export = observe_lpt(trajectory, model)
        span = max(s.displacement_m for s in export.samples) - min(s.displacement_m for s in export.samples)
        lengths = [cable_length(attachment_world(trajectory, s, model.attachment), model.anchor_m) for s in trajectory.states()]
        self.assertAlmostEqual(span, max(lengths) - min(lengths), delta=0.002)
        # The bar is a large share of system mass, so the system COM moves too,
        # but its excursion is not what the cable measures.
        com_z = [scenario.model.system.center_of_mass(s)[1] for s in trajectory.states()]
        self.assertGreater(span - (max(com_z) - min(com_z)), 0.1)

    def test_unsupported_internal_and_frontal_plane_claims_are_refused(self) -> None:
        for name in ("WRIST_INTERNAL_FORCE", "SHOULDER_INTERNAL_FORCE", "BAR_TILT", "ANYTHING_ELSE"):
            with self.assertRaisesRegex(BenchRefused, "NOT_EVALUABLE"):
                claim(name)


class BenchPressNegativeControlTests(unittest.TestCase):
    def test_missing_bench_support_is_refused_at_setup(self) -> None:
        with self.assertRaisesRegex(BenchRefused, "SUPPORT_INCOMPLETE"):
            build_bench_press(replace(BenchProtocol(), with_bench=False))

    def test_hooks_in_the_bar_path_are_refused_not_reported_as_a_valid_rep(self) -> None:
        _, trajectory, analysis = run(repetitions=1, rack_headward_m=0.0)
        rep_refused = bool(analysis.reps) and any(
            r.startswith(("SUPPORT_INCOMPLETE", "REP_NOT_COMPLETED")) for r in analysis.reps[0].refusals
        )
        run_refused = any(r.startswith("RUN_TERMINATED") for r in analysis.refusals)
        self.assertTrue(rep_refused or run_refused)
        self.assertFalse(any(r.refusals == () for r in analysis.reps))

    def test_pause_keeps_the_bar_on_the_chest_and_touch_is_a_contact_state(self) -> None:
        scenario, trajectory, analysis = run(repetitions=1, pause_s=0.8)
        rep = analysis.reps[0]
        self.assertEqual(rep.refusals, ())
        on_chest = [f.t_end_s for f in trajectory.frames
                    if rep.chest_touch_s <= f.t_end_s <= rep.lockout_s
                    and f.state.contacts["bar_chest"].mode is not ContactMode.OPEN]
        self.assertGreater(max(on_chest) - min(on_chest), 0.6)

    def test_excessive_leg_drive_lifts_the_pelvis_and_is_reported(self) -> None:
        scenario = build_bench_press(replace(BenchProtocol(), repetitions=1, leg_drive_n_m=150.0))
        scenario.duration_s = 1.0
        trajectory = scenario.run()
        analysis = analyze_bench(trajectory)
        self.assertTrue(any("bench_pelvis" in contacts and event == "CONTACT_RELEASE"
                            for _, event, contacts in analysis.support_events))


if __name__ == "__main__":
    unittest.main()
