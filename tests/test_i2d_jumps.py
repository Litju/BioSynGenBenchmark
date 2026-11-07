"""RES-492 jump family: CMJ, SJ, loaded CMJ, loaded jump squat and landing.

Expected values are independent physical identities (impulse-momentum, ballistic
flight, no penetration, nonadhesive contact, friction cones, symmetry) or
explicit refusals, never values copied from a previous run. Runs are cached per
class to bound the suite time.
"""

from __future__ import annotations

from dataclasses import replace
import math
import unittest

from biosyngen.i2.human import Anthropometry, LoadKind, StrengthHypothesis
from biosyngen.i2.instruments import (
    LANDING_PEAK_FORCE_STATUS,
    attachment_world,
    default_lpt,
    observe_dual_plates,
    observe_lpt,
)
from biosyngen.i2.jumps import REFUSED_SUBTYPES, JumpRefused, JumpStrategy, build_jump
from biosyngen.i2.phases import analyze_jump, foot_force_z
from biosyngen.i2.simulate import body_points_below_floor
from biosyngen.i1.instruments import cable_length
from biosyngen.multibody import ContactMode

_CACHE: dict[str, tuple] = {}


def run(subtype: str, **kwargs):
    key = subtype + repr(sorted(kwargs.items()))
    if key not in _CACHE:
        scenario = build_jump(subtype, **kwargs)
        trajectory = scenario.run()
        _CACHE[key] = (scenario, trajectory, analyze_jump(trajectory))
    return _CACHE[key]


SUBTYPES = ("CMJ", "SJ", "LOADED_CMJ", "LOADED_JUMP_SQUAT")


class JumpMechanicsTests(unittest.TestCase):
    def test_every_registered_subtype_completes_stance_takeoff_flight_and_landing(self) -> None:
        for subtype in SUBTYPES:
            with self.subTest(subtype=subtype):
                scenario, trajectory, analysis = run(subtype)
                self.assertEqual(trajectory.termination, "DURATION_REACHED")
                self.assertEqual(analysis.refusals, ())
                for name in ("kinetic_onset", "takeoff", "touchdown", "landing_lowest_point"):
                    self.assertIsNotNone(analysis.time(name), name)
                self.assertLess(analysis.time("kinetic_onset"), analysis.time("takeoff"))
                self.assertLess(analysis.time("takeoff"), analysis.time("touchdown"))
                self.assertLess(analysis.time("touchdown"), analysis.time("landing_lowest_point"))
                final = trajectory.frames[-1].state
                self.assertTrue(all(final.contacts[c].mode is not ContactMode.OPEN
                                    for side in scenario.model.contact_ids for c in scenario.model.contact_ids[side]))

    def test_no_penetration_no_adhesion_and_patch_friction_cones_hold_every_interval(self) -> None:
        for subtype in SUBTYPES:
            with self.subTest(subtype=subtype):
                scenario, trajectory, _ = run(subtype)
                system = scenario.model.system
                mu = scenario.model.anthropometry.friction_coefficient
                for frame in trajectory.frames:
                    self.assertEqual(body_points_below_floor(scenario.model, frame.state), [])
                    for spec in system.contacts:
                        self.assertGreater(system.contact_kinematics(frame.state, spec.contact_id).gap_m, -1e-8)
                    for side, (heel, toe) in scenario.model.contact_ids.items():
                        normal = tangential = 0.0
                        for c in (heel, toe):
                            sample = frame.contact_forces.get(c)
                            if sample is not None:
                                self.assertGreaterEqual(sample.normal_force_n, -1e-6)
                                normal += sample.normal_force_n
                                tangential += sample.tangential_force_n
                        self.assertLessEqual(abs(tangential), mu * max(normal, 0.0) + 1e-6)
                for impact in trajectory.impacts:
                    self.assertGreaterEqual(impact.impulse_world_n_s[1], -1e-12)

    def test_system_linear_momentum_closes_with_foot_impulses_and_gravity(self) -> None:
        for subtype in SUBTYPES:
            with self.subTest(subtype=subtype):
                scenario, trajectory, _ = run(subtype)
                model = scenario.model
                mass = model.total_mass_kg()
                foot = {c for side in model.contact_ids for c in model.contact_ids[side]}
                states = trajectory.states()
                p0 = model.system.linear_momentum(states[0])
                p1 = model.system.linear_momentum(states[-1])
                impulse = [0.0, -mass * model.gravity_m_s2 * (trajectory.frames[-1].t_end_s - states[0].time_s)]
                for frame in trajectory.frames:
                    h = frame.t_end_s - frame.t_start_s
                    for contact_id, sample in frame.contact_forces.items():
                        if contact_id in foot:
                            impulse[0] += sample.force_world_n[0] * h
                            impulse[1] += sample.force_world_n[1] * h
                for impact in trajectory.impacts:
                    if impact.contact_id in foot:
                        impulse[0] += impact.impulse_world_n_s[0]
                        impulse[1] += impact.impulse_world_n_s[1]
                scale = mass * model.gravity_m_s2 * trajectory.frames[-1].t_end_s
                self.assertLess(abs(p1[0] - p0[0] - impulse[0]), 1e-7 * scale)
                self.assertLess(abs(p1[1] - p0[1] - impulse[1]), 1e-7 * scale)

    def test_flight_is_ballistic_with_conserved_angular_momentum_about_the_com(self) -> None:
        for subtype in SUBTYPES:
            with self.subTest(subtype=subtype):
                scenario, trajectory, analysis = run(subtype)
                system = scenario.model.system
                mass = scenario.model.total_mass_kg()
                g = scenario.model.gravity_m_s2
                takeoff, touchdown = analysis.time("takeoff"), analysis.time("touchdown")
                flight = [f for f in trajectory.frames if takeoff < f.t_start_s and f.t_end_s < touchdown
                          and not any(s.normal_force_n for s in f.contact_forces.values()
                                      if s.contact_id in {c for v in scenario.model.contact_ids.values() for c in v})]
                self.assertGreater(len(flight), 100)
                h0 = None
                for previous, frame in zip(flight, flight[1:]):
                    dt = frame.t_end_s - previous.t_end_s
                    dv = (system.linear_momentum(frame.state)[1] - system.linear_momentum(previous.state)[1]) / mass
                    self.assertAlmostEqual(dv, -g * dt, delta=1e-9)
                    com = system.center_of_mass(frame.state)
                    h = system.angular_momentum(frame.state, com)
                    h0 = h if h0 is None else h0
                    # Symplectic Euler preserves angular momentum about the COM only to O(dt).
                    self.assertAlmostEqual(h, h0, delta=0.02 * (1.0 + abs(h0)))
                self.assertAlmostEqual(analysis.takeoff_impulse_n_s / mass, analysis.takeoff_velocity_m_s, places=9)
                rise = analysis.takeoff_velocity_m_s ** 2 / (2.0 * g)
                self.assertAlmostEqual(analysis.com_rise_after_takeoff_m, rise, delta=0.01)


class JumpPhaseTests(unittest.TestCase):
    def test_cmj_phases_follow_force_and_com_predicates_in_order(self) -> None:
        scenario, trajectory, analysis = run("CMJ")
        order = ("kinetic_onset", "peak_downward_velocity", "com_velocity_reversal", "takeoff", "touchdown")
        times = [analysis.time(name) for name in order]
        self.assertTrue(all(t is not None for t in times))
        self.assertEqual(times, sorted(times))
        command = next(t for t, stage, _ in analysis.control_commands if stage == "COUNTERMOVEMENT")
        self.assertGreaterEqual(analysis.time("kinetic_onset"), command - 1e-12)
        self.assertGreater(analysis.countermovement_depth_m, 0.10)
        self.assertLess(analysis.peak_downward_velocity_m_s, -0.5)
        # Unweighting: Fz < W just after onset; braking: Fz >= W while descending.
        weight = analysis.weight_n
        unweighting = [foot_force_z(trajectory, f) for f in trajectory.frames
                       if analysis.time("kinetic_onset") < f.t_start_s < analysis.time("peak_downward_velocity") - 0.002]
        self.assertTrue(unweighting and max(unweighting) < weight)

    def test_squat_jump_is_concentric_only_without_fake_countermovement(self) -> None:
        _, trajectory, analysis = run("SJ")
        self.assertIsNone(analysis.time("peak_downward_velocity"))
        self.assertIsNone(analysis.time("com_velocity_reversal"))
        self.assertGreater(analysis.peak_downward_velocity_m_s, -1e-3)
        self.assertEqual([p.name for p in analysis.phases][:2], ["STATIC_HOLD", "PROPULSION"])
        # The static hold is an exact equilibrium before the command.
        weight = analysis.weight_n
        before = [foot_force_z(trajectory, f) for f in trajectory.frames if f.t_end_s < 0.29]
        self.assertLess(max(abs(v - weight) for v in before), 1e-6 * weight)

    def test_loads_change_topology_and_reduce_takeoff_velocity(self) -> None:
        cmj = run("CMJ")[2].takeoff_velocity_m_s
        vest_scenario, _, vest = run("LOADED_CMJ")
        bar_scenario, bar_trajectory, bar = run("LOADED_JUMP_SQUAT")
        self.assertIs(vest_scenario.model.load.kind, LoadKind.VEST)
        self.assertIs(bar_scenario.model.load.kind, LoadKind.BACK_BAR)
        self.assertIsNone(vest_scenario.model.load_contact_id)
        self.assertEqual(bar_scenario.model.load_contact_id, "bar_on_back")
        self.assertLess(vest.takeoff_velocity_m_s, cmj)
        self.assertLess(bar.takeoff_velocity_m_s, vest.takeoff_velocity_m_s)
        # The bar rests on a unilateral shoulder contact: it may separate and re-impact.
        bar_events = [e.event_id for e in bar_trajectory.events if "bar_on_back" in e.contact_ids]
        self.assertIn("CONTACT_RELEASE", bar_events)
        self.assertIn("CONTACT_IMPACT", bar_events)
        for frame in bar_trajectory.frames:
            sample = frame.contact_forces.get("bar_on_back")
            if sample is not None:
                self.assertGreaterEqual(sample.normal_force_n, -1e-6)

    def test_unsupported_and_mislabeled_jump_requests_are_refused(self) -> None:
        for subtype in REFUSED_SUBTYPES:
            with self.assertRaisesRegex(JumpRefused, "NOT_IMPLEMENTED"):
                build_jump(subtype)
        with self.assertRaisesRegex(JumpRefused, "unloaded protocol"):
            build_jump("CMJ", load_mass_kg=20.0)
        with self.assertRaisesRegex(JumpRefused, "not a registered"):
            build_jump("BOX_JUMP")


class BilateralAndVariationTests(unittest.TestCase):
    def test_symmetric_body_gives_identical_left_right_plate_forces(self) -> None:
        _, trajectory, analysis = run("CMJ")
        for frame in trajectory.frames:
            left = sum(frame.contact_forces[c].force_world_n[1] for c in ("left_heel", "left_toe") if c in frame.contact_forces)
            right = sum(frame.contact_forces[c].force_world_n[1] for c in ("right_heel", "right_toe") if c in frame.contact_forces)
            self.assertAlmostEqual(left, right, delta=1e-6 * (1.0 + abs(left)))
        self.assertEqual(analysis.time("left_foot_release"), analysis.time("right_foot_release"))

    def test_strength_asymmetry_separates_plate_channels_without_changing_their_sum_identity(self) -> None:
        scenario, trajectory, analysis = run("SJ", strength=StrengthHypothesis(right_scale=0.80))
        self.assertEqual(trajectory.termination, "DURATION_REACHED")
        differences = []
        for frame in trajectory.frames:
            if frame.t_start_s < analysis.time("kinetic_onset") or frame.t_end_s > analysis.time("takeoff"):
                continue
            left = sum(frame.contact_forces[c].force_world_n[1] for c in ("left_heel", "left_toe") if c in frame.contact_forces)
            right = sum(frame.contact_forces[c].force_world_n[1] for c in ("right_heel", "right_toe") if c in frame.contact_forces)
            differences.append(left - right)
        self.assertGreater(max(abs(d) for d in differences), 0.02 * analysis.weight_n)
        symmetric = run("SJ")[2]
        self.assertLess(analysis.takeoff_velocity_m_s, symmetric.takeoff_velocity_m_s)

    def test_anthropometric_variations_run_with_exact_static_start(self) -> None:
        for mass, stature in ((60.0, 1.65), (90.0, 1.88)):
            with self.subTest(mass=mass):
                # Declared hypothesis: torque capacity scales with body weight x stature.
                factor = (mass / 75.0) * (stature / 1.78)
                scenario, trajectory, analysis = run(
                    "CMJ", anthropometry=Anthropometry().scaled(body_mass_kg=mass, stature_m=stature),
                    strength=StrengthHypothesis(left_scale=factor, right_scale=factor),
                )
                self.assertAlmostEqual(analysis.weight_n, mass * scenario.model.gravity_m_s2, places=9)
                before = [foot_force_z(trajectory, f) for f in trajectory.frames if f.t_end_s < 0.29]
                self.assertLess(max(abs(v - analysis.weight_n) for v in before), 1e-6 * analysis.weight_n)
                self.assertIsNotNone(analysis.time("takeoff"))
                self.assertEqual(analysis.refusals, ())


    def test_relatively_weak_heavy_jumper_landing_collapse_is_refused_not_hidden(self) -> None:
        scenario, trajectory, analysis = run("CMJ", anthropometry=Anthropometry().scaled(body_mass_kg=90.0, stature_m=1.88))
        self.assertIsNotNone(analysis.time("takeoff"))
        self.assertTrue(trajectory.termination.startswith("BODY_GROUND_CONTACT_NOT_MODELED"))
        self.assertTrue(any(r.startswith("RUN_TERMINATED") for r in analysis.refusals))


class JumpInstrumentTests(unittest.TestCase):
    def test_dual_plate_export_has_independent_channels_and_declared_limits(self) -> None:
        scenario, trajectory, analysis = run("CMJ")
        export = observe_dual_plates(trajectory)
        weight = analysis.weight_n
        quiet = [s for s in export.samples if s.native_time_s < 0.25]
        mean_total = sum(s.fz_total_n for s in quiet) / len(quiet)
        self.assertAlmostEqual(mean_total / weight, 1.0, delta=0.01)
        self.assertTrue(all(s.fz_total_n == s.fz_left_n + s.fz_right_n for s in export.samples))
        # Calibration error differs per plate, so equal latent loads export unequal values.
        self.assertGreater(abs(sum(s.fz_left_n - s.fz_right_n for s in quiet) / len(quiet)), 0.5)
        self.assertEqual(export.metadata["landing_peak_force"], LANDING_PEAK_FORCE_STATUS)
        # A threshold estimator on the device clock finds takeoff after latent takeoff by about the clock offset.
        detected = next(s.native_time_s for s in export.samples if s.native_time_s > 0.5 and s.fz_total_n < 20.0)
        offset = export.metadata["native_time_offset_s"]
        self.assertLess(abs(detected - offset - analysis.time("takeoff")), 0.004)
        flight = [s.fz_total_n for s in export.samples
                  if analysis.time("takeoff") + 0.02 < s.native_time_s - offset < analysis.time("touchdown") - 0.01]
        self.assertLess(max(abs(v) for v in flight), 15.0)

    def test_lpt_cable_displacement_matches_attachment_geometry_within_encoder_and_clock(self) -> None:
        for subtype in ("CMJ", "LOADED_JUMP_SQUAT"):
            with self.subTest(subtype=subtype):
                _, trajectory, _ = run(subtype)
                model = default_lpt(trajectory)
                export = observe_lpt(trajectory, model)
                self.assertEqual(model.attachment.label, "BAR_COLLAR" if subtype == "LOADED_JUMP_SQUAT" else "HIP_BELT")
                times, states = trajectory.times(), trajectory.states()
                lengths = [cable_length(attachment_world(trajectory, s, model.attachment), model.anchor_m) for s in states]
                worst = 0.0
                for sample in export.samples[1:-1]:
                    t = (sample.native_time_s - model.clock.offset_s) / 1.0  # nominal mapping, drift ignored
                    i = max(0, min(len(times) - 2, next((k for k in range(len(times) - 1) if times[k + 1] >= t), len(times) - 2)))
                    w = (t - times[i]) / (times[i + 1] - times[i]) if times[i + 1] > times[i] else 0.0
                    latent = lengths[i] + w * (lengths[i + 1] - lengths[i]) - export.zero_reference_m
                    if not sample.slack:
                        worst = max(worst, abs(sample.displacement_m - latent))
                # Encoder step + drift/jitter of the undeclared true clock: an estimate, not a bound.
                self.assertLess(worst, 0.01)


class TimeStepRefinementTests(unittest.TestCase):
    def test_takeoff_converges_under_time_step_refinement(self) -> None:
        results = {}
        for dt in (0.002, 0.001, 0.0005):
            scenario = build_jump("SJ", duration_s=0.75)
            trajectory = scenario.run(dt_s=dt)
            analysis = analyze_jump(trajectory)
            results[dt] = (analysis.time("takeoff"), analysis.takeoff_velocity_m_s)
        coarse = abs(results[0.002][1] - results[0.0005][1])
        fine = abs(results[0.001][1] - results[0.0005][1])
        self.assertLess(fine, coarse)
        self.assertLess(fine, 0.02 * results[0.0005][1])
        self.assertLess(abs(results[0.001][0] - results[0.0005][0]), 0.003)


if __name__ == "__main__":
    unittest.main()
