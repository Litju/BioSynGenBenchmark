from dataclasses import replace
import math
import unittest

from biosyngen.mechanics import MassProperties, Phase, PhaseMode, Scenario, simulate
from biosyngen.observations import (
    AttachmentTarget,
    CableLptConfig,
    ClockModel,
    ForcePlateConfig,
    cable_kinematics,
    observe_cable_lpt,
    observe_force_plate,
    partition_support_force,
    state_at_time,
    validate_observation_clock,
)


def make_trajectory():
    scenario = Scenario(
        name="manufactured_force_lpt_observations",
        masses=MassProperties(body_mass_kg=75.0, bar_mass_kg=15.0, bar_offset_m=0.25),
        phases=(
            Phase(0.37, PhaseMode.STANCE, 1300.0),
            Phase(0.31, PhaseMode.FLIGHT, 0.0),
        ),
        initial_body_com_height_m=0.9,
        initial_vertical_velocity_m_s=0.1,
        gravity_m_s2=9.81,
        sample_interval_s=0.04,
    )
    return scenario, simulate(scenario)


class CableGeometryTests(unittest.TestCase):
    def test_vertical_oblique_and_horizontal_projection_cases(self) -> None:
        vertical = cable_kinematics((0.0, 0.0), (0.0, 2.0), (0.0, 1.0))
        self.assertEqual(vertical.length_m, 2.0)
        self.assertEqual(vertical.rate_m_s, 1.0)
        self.assertAlmostEqual(vertical.angle_rad, math.pi / 2.0, places=15)

        oblique = cable_kinematics((0.0, 0.0), (3.0, 4.0), (3.0, 4.0))
        self.assertEqual(oblique.length_m, 5.0)
        self.assertEqual(oblique.rate_m_s, 5.0)

        horizontal_motion = cable_kinematics(
            (0.0, 0.0), (3.0, 4.0), (1.0, 0.0)
        )
        self.assertEqual(horizontal_motion.length_m, 5.0)
        self.assertAlmostEqual(horizontal_motion.rate_m_s, 0.6, places=15)
        self.assertNotEqual(horizontal_motion.rate_m_s, 0.0)

    def test_cable_rate_matches_finite_difference_and_wrong_geometry_is_detectable(self) -> None:
        anchor = (0.0, 0.0)
        attachment = (3.0, 4.0)
        velocity = (1.0, -0.2)
        geometry = cable_kinematics(anchor, attachment, velocity)
        step_s = 1e-6
        plus = cable_kinematics(
            anchor,
            (attachment[0] + velocity[0] * step_s, attachment[1] + velocity[1] * step_s),
            (0.0, 0.0),
        )
        minus = cable_kinematics(
            anchor,
            (attachment[0] - velocity[0] * step_s, attachment[1] - velocity[1] * step_s),
            (0.0, 0.0),
        )
        independent_rate = (plus.length_m - minus.length_m) / (2.0 * step_s)
        self.assertAlmostEqual(geometry.rate_m_s, independent_rate, places=8)

        wrong_anchor = cable_kinematics((0.0, 1.0), attachment, velocity)
        self.assertNotAlmostEqual(wrong_anchor.length_m, geometry.length_m, places=4)
        self.assertNotAlmostEqual(geometry.rate_m_s, velocity[1], places=4)

    def test_coincident_anchor_and_attachment_is_refused(self) -> None:
        with self.assertRaisesRegex(ValueError, "must not coincide"):
            cable_kinematics((1.0, 2.0), (1.0, 2.0), (0.0, 0.0))


class ForcePlateObservationTests(unittest.TestCase):
    def test_partition_requires_explicit_two_foot_channels(self) -> None:
        left, right = partition_support_force(1250.0, 0.6)
        self.assertEqual(left, 750.0)
        self.assertEqual(right, 500.0)
        self.assertEqual(left + right, 1250.0)

        with self.assertRaisesRegex(ValueError, "two declared foot-contact"):
            ForcePlateConfig(
                clock=ClockModel(0.02),
                assumed_left_fraction=0.5,
                two_foot_channels_declared=False,
            )

    def test_declared_plate_partition_sums_to_ideal_force_before_noise(self) -> None:
        _, trajectory = make_trajectory()
        series = observe_force_plate(
            trajectory,
            ForcePlateConfig(
                clock=ClockModel(0.02),
                two_foot_channels_declared=True,
                assumed_left_fraction=0.6,
            ),
        )
        first = series.samples[0]
        self.assertEqual(first.total_vertical_force_n, 1300.0)
        self.assertEqual(first.left_vertical_force_n, 780.0)
        self.assertEqual(first.right_vertical_force_n, 520.0)
        self.assertEqual(first.left_vertical_force_n + first.right_vertical_force_n, 1300.0)

    def test_independent_native_clock_missing_samples_and_signed_flight_noise(self) -> None:
        _, trajectory = make_trajectory()
        clock = ClockModel(sample_period_s=0.02, offset_s=0.003, drift_ppm=80.0)
        series = observe_force_plate(
            trajectory,
            ForcePlateConfig(
                clock=clock,
                offset_n=-0.3,
                missing_indices=frozenset({2}),
            ),
        )
        validate_observation_clock(series, expected_clock=clock)
        self.assertEqual(series.metadata.source_stage, "SIMULATED_OBSERVATION")
        self.assertEqual(series.metadata.units, "N")
        self.assertEqual(series.metadata.positive_direction, "upward")
        self.assertEqual(series.samples[0].native_time_s, 0.003)
        self.assertTrue(series.samples[2].missing)
        self.assertIsNone(series.samples[2].total_vertical_force_n)

        flight_samples = [
            sample
            for sample in series.samples
            if clock.simulation_time(sample.native_time_s) > 0.37
            and not sample.missing
        ]
        self.assertTrue(flight_samples)
        self.assertTrue(all(sample.total_vertical_force_n == -0.3 for sample in flight_samples))

    def test_offset_quantization_saturation_and_seeded_repeatability(self) -> None:
        _, trajectory = make_trajectory()
        config = ForcePlateConfig(
            clock=ClockModel(0.02),
            offset_n=8.0,
            noise_std_n=0.25,
            quantization_n=0.5,
            saturation_n=1000.0,
            seed=44,
        )
        first = observe_force_plate(trajectory, config)
        second = observe_force_plate(trajectory, config)
        self.assertEqual(first, second)
        self.assertTrue(first.samples[0].saturated)
        self.assertEqual(first.samples[0].total_vertical_force_n, 1000.0)
        self.assertTrue(all(sample.left_vertical_force_n is None for sample in first.samples))


class LptObservationTests(unittest.TestCase):
    def test_separate_clock_and_explicit_bar_attachment_geometry(self) -> None:
        _, trajectory = make_trajectory()
        clock = ClockModel(sample_period_s=0.015, offset_s=-0.004, drift_ppm=-35.0)
        config = CableLptConfig(
            clock=clock,
            anchor_x_m=0.0,
            anchor_z_m=0.0,
            attachment_x_m=0.4,
            attachment_target=AttachmentTarget.BAR_ATTACHMENT,
        )
        series = observe_cable_lpt(trajectory, config)
        validate_observation_clock(series, expected_clock=clock)
        first = series.samples[0]
        expected = cable_kinematics((0.0, 0.0), (0.4, 1.15), (0.0, 0.1))
        self.assertAlmostEqual(first.cable_length_m, expected.length_m, places=14)
        self.assertAlmostEqual(first.cable_rate_m_s, expected.rate_m_s, places=14)
        self.assertAlmostEqual(first.cable_angle_rad, expected.angle_rad, places=14)
        self.assertEqual(first.native_time_s, -0.004)
        self.assertEqual(series.metadata.source_stage, "SIMULATED_OBSERVATION")
        self.assertEqual(series.metadata.units, "m;m/s;rad")

    def test_lpt_zero_reference_angle_quantization_missing_and_saturation(self) -> None:
        _, trajectory = make_trajectory()
        initial = trajectory.samples[0]
        geometry = cable_kinematics(
            (0.0, 0.0),
            (0.4, initial.bar_attachment_height_m),
            (0.0, initial.bar_attachment_velocity_m_s),
        )
        series = observe_cable_lpt(
            trajectory,
            CableLptConfig(
                clock=ClockModel(0.02),
                anchor_x_m=0.0,
                anchor_z_m=0.0,
                attachment_x_m=0.4,
                attachment_target=AttachmentTarget.BAR_ATTACHMENT,
                length_zero_reference_m=geometry.length_m,
                angle_zero_rad=geometry.angle_rad,
                length_offset_m=0.003,
                length_quantization_m=0.001,
                length_saturation_m=0.002,
                missing_indices=frozenset({1}),
                seed=8,
            ),
        )
        self.assertEqual(series.samples[0].cable_length_m, 0.002)
        self.assertTrue(series.samples[0].saturated)
        self.assertAlmostEqual(series.samples[0].cable_angle_rad, 0.0, places=14)
        self.assertTrue(series.samples[1].missing)
        self.assertIsNone(series.samples[1].cable_length_m)
        self.assertIsNone(series.samples[1].cable_rate_m_s)
        self.assertIsNone(series.samples[1].cable_angle_rad)

    def test_missing_bar_target_and_out_of_range_missing_index_are_refused(self) -> None:
        body_only = Scenario(
            name="manufactured_body_only",
            masses=MassProperties(body_mass_kg=80.0),
            phases=(Phase(0.2, PhaseMode.STANCE, 900.0),),
            initial_body_com_height_m=0.9,
        )
        with self.assertRaisesRegex(ValueError, "requires a declared bar"):
            observe_cable_lpt(
                simulate(body_only),
                CableLptConfig(
                    clock=ClockModel(0.02),
                    anchor_x_m=0.0,
                    anchor_z_m=0.0,
                    attachment_x_m=0.0,
                    attachment_target=AttachmentTarget.BAR_ATTACHMENT,
                ),
            )

        _, trajectory = make_trajectory()
        with self.assertRaisesRegex(ValueError, "outside the sample schedule"):
            observe_force_plate(
                trajectory,
                ForcePlateConfig(
                    clock=ClockModel(0.2),
                    missing_indices=frozenset({100}),
                ),
            )

    def test_different_clock_rates_produce_independent_time_vectors(self) -> None:
        _, trajectory = make_trajectory()
        force_clock = ClockModel(0.02, offset_s=0.003, drift_ppm=50.0)
        lpt_clock = ClockModel(0.015, offset_s=-0.004, drift_ppm=-35.0)
        force = observe_force_plate(
            trajectory, ForcePlateConfig(clock=force_clock)
        )
        lpt = observe_cable_lpt(
            trajectory,
            CableLptConfig(
                clock=lpt_clock,
                anchor_x_m=0.0,
                anchor_z_m=0.0,
                attachment_x_m=0.2,
                attachment_target=AttachmentTarget.BAR_ATTACHMENT,
            ),
        )
        validate_observation_clock(force, expected_clock=force_clock)
        validate_observation_clock(lpt, expected_clock=lpt_clock)
        self.assertNotEqual(len(force.samples), len(lpt.samples))
        self.assertNotEqual(
            [sample.native_time_s for sample in force.samples],
            [sample.native_time_s for sample in lpt.samples],
        )
        self.assertNotEqual(force.clock, lpt.clock)

    def test_corrupted_or_wrong_timebase_metadata_is_detected(self) -> None:
        _, trajectory = make_trajectory()
        clock = ClockModel(0.02, offset_s=0.003, drift_ppm=50.0)
        series = observe_force_plate(trajectory, ForcePlateConfig(clock=clock))
        corrupted_sample = replace(
            series.samples[1],
            native_time_s=series.samples[1].native_time_s + 0.001,
        )
        corrupted = replace(
            series,
            samples=(series.samples[0], corrupted_sample, *series.samples[2:]),
        )
        with self.assertRaisesRegex(ValueError, "native timestamps"):
            validate_observation_clock(corrupted, expected_clock=clock)

        wrong_clock = ClockModel(0.02, offset_s=0.003, drift_ppm=500.0)
        with self.assertRaisesRegex(ValueError, "differs from the declared clock"):
            validate_observation_clock(series, expected_clock=wrong_clock)

    def test_same_seed_repeats_both_observation_models(self) -> None:
        _, trajectory = make_trajectory()
        force_config = ForcePlateConfig(
            clock=ClockModel(0.02), noise_std_n=0.5, seed=91
        )
        lpt_config = CableLptConfig(
            clock=ClockModel(0.015),
            anchor_x_m=-0.2,
            anchor_z_m=0.1,
            attachment_x_m=0.4,
            attachment_target=AttachmentTarget.BAR_ATTACHMENT,
            length_noise_std_m=0.002,
            rate_noise_std_m_s=0.003,
            angle_noise_std_rad=0.001,
            seed=91,
        )
        self.assertEqual(
            observe_force_plate(trajectory, force_config),
            observe_force_plate(trajectory, force_config),
        )
        self.assertEqual(
            observe_cable_lpt(trajectory, lpt_config),
            observe_cable_lpt(trajectory, lpt_config),
        )

    def test_off_grid_lpt_samples_use_exact_phase_state(self) -> None:
        _, trajectory = make_trajectory()
        sample_time_s = 0.355
        state = state_at_time(trajectory, sample_time_s)
        expected_acceleration = 1300.0 / 90.0 - 9.81
        expected_velocity = 0.1 + expected_acceleration * sample_time_s
        expected_height = (
            0.9
            + 0.1 * sample_time_s
            + 0.5 * expected_acceleration * sample_time_s**2
        )
        self.assertAlmostEqual(state.body_com_velocity_m_s, expected_velocity, places=13)
        self.assertAlmostEqual(state.body_com_height_m, expected_height, places=13)
        self.assertEqual(state.mode, PhaseMode.STANCE)
        self.assertEqual(state_at_time(trajectory, 0.37).mode, PhaseMode.FLIGHT)
        with self.assertRaisesRegex(ValueError, "outside the latent trajectory"):
            state_at_time(trajectory, 1.0)


if __name__ == "__main__":
    unittest.main()
