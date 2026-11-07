"""RES-496 Olympic pulling derivatives: clean pull and snatch pull from the floor."""

from __future__ import annotations

from dataclasses import replace
import math
import unittest

from biosyngen.i1.instruments import cable_length
from biosyngen.i2.instruments import attachment_world, observe_lpt
from biosyngen.i2.olympic import (
    PULL_VARIANTS,
    REFUSED_VARIANTS,
    PullProtocol,
    PullRefused,
    analyze_pull,
    build_pull,
    claim_from_sensors,
    grip_force_z,
    pull_lpt,
)

_CACHE: dict[str, tuple] = {}


def run(variant: str, **kwargs):
    key = variant + repr(sorted(kwargs.items()))
    if key not in _CACHE:
        scenario = build_pull(replace(PullProtocol(), variant=variant, **kwargs))
        trajectory = scenario.run()
        _CACHE[key] = (scenario, trajectory, analyze_pull(trajectory))
    return _CACHE[key]


class OlympicPullMechanicsTests(unittest.TestCase):
    def test_both_variants_lift_pull_and_return_the_bar(self) -> None:
        for variant in PULL_VARIANTS:
            with self.subTest(variant=variant):
                scenario, trajectory, analysis = run(variant)
                self.assertEqual(trajectory.termination, "DURATION_REACHED")
                self.assertEqual(analysis.refusals, ())
                order = (analysis.lift_off_s, analysis.first_pull_end_s, analysis.second_pull_end_s, analysis.bar_return_s)
                self.assertTrue(all(t is not None for t in order))
                self.assertEqual(list(order), sorted(order))
                self.assertGreater(analysis.peak_bar_velocity_m_s, 1.0)
                final = trajectory.frames[-1].state
                self.assertIsNot(final.contacts["bar_floor"].mode.value, "OPEN")

    def test_variants_are_distinct_physical_topologies(self) -> None:
        clean_s, clean_t, clean = run("CLEAN_PULL")
        snatch_s, snatch_t, snatch = run("SNATCH_PULL")
        self.assertLess(snatch_s.metadata["grip_projection_factor"], clean_s.metadata["grip_projection_factor"])
        self.assertAlmostEqual(snatch_s.metadata["grip_projection_factor"], math.cos(math.radians(30.0)), places=12)
        # Shorter projected reach: a deeper setup and a higher bar at the top of the pull.
        self.assertLess(snatch_s.setup_posture.knee, clean_s.setup_posture.knee)
        self.assertGreater(snatch.bar_peak_height_m, clean.bar_peak_height_m + 0.03)
        self.assertNotEqual(snatch_s.metadata["variant_id"], clean_s.metadata["variant_id"])

    def test_momentum_closes_with_feet_and_bar_floor_supports_and_nothing_penetrates(self) -> None:
        for variant in PULL_VARIANTS:
            with self.subTest(variant=variant):
                scenario, trajectory, _ = run(variant)
                model = scenario.model
                system = model.system
                external = {c for v in model.contact_ids.values() for c in v} | {"bar_floor"}
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

    def test_grip_tension_from_bar_newton_balance(self) -> None:
        scenario, trajectory, analysis = run("CLEAN_PULL")
        bar_weight = scenario.model.load.mass_kg * scenario.model.gravity_m_s2
        setup = [grip_force_z(trajectory, i) for i, f in enumerate(trajectory.frames) if 0.01 < f.t_end_s < 0.35]
        self.assertLess(max(abs(v) for v in setup), 1e-6 * bar_weight)  # bar resting on the floor: no grip load
        pull = [grip_force_z(trajectory, i) for i, f in enumerate(trajectory.frames)
                if analysis.lift_off_s + 0.01 < f.t_end_s < analysis.first_pull_end_s]
        self.assertTrue(all(v > 0.0 for v in pull))              # the grip pulls up, never pushes
        self.assertGreater(max(pull), 1.2 * bar_weight)          # the bar is accelerated upward

    def test_transition_is_reported_only_when_the_knee_actually_re_flexes(self) -> None:
        for variant in PULL_VARIANTS:
            with self.subTest(variant=variant):
                scenario, trajectory, analysis = run(variant)
                knee_rates = [scenario.model.system.joint_state(f.state, "left_knee")[1] for f in trajectory.frames
                              if analysis.first_pull_end_s <= f.t_end_s <= analysis.second_pull_end_s]
                re_flexes = any(rate < -0.05 for rate in knee_rates)
                self.assertEqual(analysis.transition_s is not None, re_flexes)

    def test_lpt_is_bar_collar_geometry(self) -> None:
        _, trajectory, _ = run("SNATCH_PULL")
        model = pull_lpt(trajectory)
        export = observe_lpt(trajectory, model)
        span = max(s.displacement_m for s in export.samples) - min(s.displacement_m for s in export.samples)
        lengths = [cable_length(attachment_world(trajectory, s, model.attachment), model.anchor_m) for s in trajectory.states()]
        self.assertAlmostEqual(span, max(lengths) - min(lengths), delta=0.002)


class OlympicPullRefusalTests(unittest.TestCase):
    def test_hang_and_block_variants_are_refused_per_the_inventory(self) -> None:
        for variant in REFUSED_VARIANTS:
            with self.assertRaisesRegex(PullRefused, "NOT_IMPLEMENTED"):
                build_pull(replace(PullProtocol(), variant=variant))
        with self.assertRaisesRegex(PullRefused, "not a registered"):
            build_pull(replace(PullProtocol(), variant="POWER_SNATCH_FROM_KNEE"))

    def test_setup_outside_the_declared_joint_ranges_is_refused(self) -> None:
        with self.assertRaisesRegex(PullRefused, "SETUP_OUT_OF_RANGE"):
            build_pull(replace(PullProtocol(), variant="SNATCH_PULL", setup_ankle=0.45))

    def test_joint_phase_claims_from_force_or_lpt_are_refused(self) -> None:
        for name in ("DOUBLE_KNEE_BEND", "TRIPLE_EXTENSION", "TECHNIQUE_PROFICIENCY", "OTHER"):
            with self.assertRaisesRegex(PullRefused, "NOT_EVALUABLE"):
                claim_from_sensors(name)


if __name__ == "__main__":
    unittest.main()
