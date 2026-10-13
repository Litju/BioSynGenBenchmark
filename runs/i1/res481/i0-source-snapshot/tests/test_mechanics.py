import sys
import unittest

from biosyngen.mechanics import (
    MassProperties,
    Phase,
    PhaseMode,
    Scenario,
    simulate,
)


class MechanicsKnownAnswerTests(unittest.TestCase):
    def assert_roundoff_equal(
        self,
        actual: float,
        expected: float,
        operation_count: int = 1,
    ) -> None:
        scale = max(1.0, abs(actual), abs(expected))
        tolerance = 64.0 * sys.float_info.epsilon * scale * max(1, operation_count)
        self.assertLessEqual(
            abs(actual - expected),
            tolerance,
            msg=f"absolute error exceeds binary64 roundoff bound {tolerance:.3e}",
        )

    def test_constant_stance_force_matches_independent_kinematic_solution(self) -> None:
        mass_kg = 80.0
        gravity_m_s2 = 9.81
        force_n = 1200.0
        duration_s = 0.37
        initial_height_m = 0.91
        initial_velocity_m_s = -0.12
        scenario = Scenario(
            name="manufactured_constant_stance",
            masses=MassProperties(body_mass_kg=mass_kg),
            phases=(Phase(duration_s, PhaseMode.STANCE, force_n),),
            initial_body_com_height_m=initial_height_m,
            initial_vertical_velocity_m_s=initial_velocity_m_s,
            gravity_m_s2=gravity_m_s2,
            sample_interval_s=0.1,
        )

        result = simulate(scenario)
        final = result.samples[-1]
        acceleration = force_n / mass_kg - gravity_m_s2
        expected_velocity = initial_velocity_m_s + acceleration * duration_s
        expected_height = (
            initial_height_m
            + initial_velocity_m_s * duration_s
            + 0.5 * acceleration * duration_s**2
        )

        self.assert_roundoff_equal(final.body_com_acceleration_m_s2, acceleration)
        self.assert_roundoff_equal(final.body_com_velocity_m_s, expected_velocity, len(result.samples))
        self.assert_roundoff_equal(final.body_com_height_m, expected_height, len(result.samples))
        self.assertEqual(final.mode, PhaseMode.STANCE)

    def test_ballistic_flight_matches_independent_reference_from_liftoff(self) -> None:
        gravity_m_s2 = 9.80665
        stance_s = 0.43
        flight_s = 0.37
        scenario = Scenario(
            name="manufactured_stance_then_flight",
            masses=MassProperties(body_mass_kg=72.0),
            phases=(
                Phase(stance_s, PhaseMode.STANCE, 1250.0),
                Phase(flight_s, PhaseMode.FLIGHT, 0.0),
            ),
            initial_body_com_height_m=0.88,
            initial_vertical_velocity_m_s=0.05,
            gravity_m_s2=gravity_m_s2,
            sample_interval_s=0.1,
        )

        result = simulate(scenario)
        liftoff = next(sample for sample in result.samples if sample.time_s == stance_s)
        final = result.samples[-1]
        expected_velocity = liftoff.body_com_velocity_m_s - gravity_m_s2 * flight_s
        expected_height = (
            liftoff.body_com_height_m
            + liftoff.body_com_velocity_m_s * flight_s
            - 0.5 * gravity_m_s2 * flight_s**2
        )

        self.assert_roundoff_equal(final.body_com_acceleration_m_s2, -gravity_m_s2)
        self.assert_roundoff_equal(final.body_com_velocity_m_s, expected_velocity, len(result.samples))
        self.assert_roundoff_equal(final.body_com_height_m, expected_height, len(result.samples))
        self.assertEqual(final.normal_force_n, 0.0)
        self.assertEqual(final.mode, PhaseMode.FLIGHT)

    def test_net_impulse_equals_total_mass_times_velocity_change(self) -> None:
        mass_kg = 83.0
        gravity_m_s2 = 9.8
        phases = (
            Phase(0.23, PhaseMode.STANCE, 1150.0),
            Phase(0.41, PhaseMode.STANCE, 980.0),
            Phase(0.52, PhaseMode.FLIGHT, 0.0),
        )
        scenario = Scenario(
            name="manufactured_impulse_balance",
            masses=MassProperties(body_mass_kg=mass_kg),
            phases=phases,
            initial_body_com_height_m=0.95,
            gravity_m_s2=gravity_m_s2,
            sample_interval_s=0.07,
        )

        result = simulate(scenario)
        delta_momentum = mass_kg * (
            result.samples[-1].body_com_velocity_m_s
            - result.samples[0].body_com_velocity_m_s
        )
        net_external_impulse = sum(
            (phase.normal_force_n - mass_kg * gravity_m_s2) * phase.duration_s
            for phase in phases
        )

        self.assert_roundoff_equal(delta_momentum, net_external_impulse, len(phases) * 4)

    def test_contact_force_is_nonnegative_in_stance_and_zero_in_flight(self) -> None:
        scenario = Scenario(
            name="manufactured_contact_boundary",
            masses=MassProperties(body_mass_kg=80.0),
            phases=(
                Phase(0.25, PhaseMode.STANCE, 900.0),
                Phase(0.25, PhaseMode.FLIGHT, 0.0),
            ),
            initial_body_com_height_m=1.0,
            sample_interval_s=0.03,
        )

        result = simulate(scenario)
        stance = [sample for sample in result.samples if sample.mode is PhaseMode.STANCE]
        flight = [sample for sample in result.samples if sample.mode is PhaseMode.FLIGHT]
        self.assertTrue(stance)
        self.assertTrue(flight)
        self.assertTrue(all(sample.normal_force_n >= 0.0 for sample in stance))
        self.assertTrue(all(sample.normal_force_n == 0.0 for sample in flight))

    def test_mass_and_force_scaling_preserves_acceleration(self) -> None:
        def initial_acceleration(body_mass_kg: float, normal_force_n: float) -> float:
            scenario = Scenario(
                name="manufactured_scaling",
                masses=MassProperties(body_mass_kg=body_mass_kg),
                phases=(Phase(0.2, PhaseMode.STANCE, normal_force_n),),
                initial_body_com_height_m=1.0,
            )
            return simulate(scenario).samples[0].body_com_acceleration_m_s2

        self.assert_roundoff_equal(
            initial_acceleration(80.0, 1100.0),
            initial_acceleration(160.0, 2200.0),
            operation_count=2,
        )

    def test_support_force_equal_to_weight_has_zero_vertical_acceleration(self) -> None:
        mass_kg = 75.0
        gravity_m_s2 = 9.81
        scenario = Scenario(
            name="manufactured_static_support",
            masses=MassProperties(body_mass_kg=mass_kg),
            phases=(Phase(0.5, PhaseMode.STANCE, mass_kg * gravity_m_s2),),
            initial_body_com_height_m=1.1,
            gravity_m_s2=gravity_m_s2,
            sample_interval_s=0.1,
        )

        result = simulate(scenario)
        for sample in result.samples:
            self.assert_roundoff_equal(sample.body_com_acceleration_m_s2, 0.0)
            self.assert_roundoff_equal(sample.body_com_velocity_m_s, 0.0)
            self.assert_roundoff_equal(sample.body_com_height_m, 1.1)

    def test_body_bar_and_constrained_system_coordinates_are_distinct(self) -> None:
        masses = MassProperties(body_mass_kg=80.0, bar_mass_kg=20.0, bar_offset_m=0.3)
        scenario = Scenario(
            name="manufactured_rigid_bar_offset",
            masses=masses,
            phases=(Phase(0.2, PhaseMode.STANCE, 1200.0),),
            initial_body_com_height_m=0.9,
            sample_interval_s=0.1,
        )

        sample = simulate(scenario).samples[0]
        self.assert_roundoff_equal(sample.body_com_height_m, 0.9)
        self.assert_roundoff_equal(sample.bar_attachment_height_m, 1.2)
        self.assert_roundoff_equal(sample.system_com_height_m, 0.96, operation_count=3)
        self.assertEqual(sample.body_com_velocity_m_s, sample.bar_attachment_velocity_m_s)
        self.assertEqual(masses.total_mass_kg, 100.0)

    def test_boundary_is_included_once_and_contact_mode_is_right_continuous(self) -> None:
        transition_s = 0.35
        scenario = Scenario(
            name="manufactured_off_grid_transition",
            masses=MassProperties(body_mass_kg=75.0),
            phases=(
                Phase(transition_s, PhaseMode.STANCE, 1000.0),
                Phase(0.2, PhaseMode.FLIGHT, 0.0),
            ),
            initial_body_com_height_m=1.0,
            sample_interval_s=0.1,
        )

        result = simulate(scenario)
        at_transition = [sample for sample in result.samples if sample.time_s == transition_s]
        self.assertEqual(len(at_transition), 1)
        self.assertEqual(at_transition[0].mode, PhaseMode.FLIGHT)
        self.assertEqual(at_transition[0].normal_force_n, 0.0)
        stance_acceleration = 1000.0 / 75.0 - 9.80665
        self.assert_roundoff_equal(
            at_transition[0].body_com_velocity_m_s,
            stance_acceleration * transition_s,
            len(result.samples),
        )
        self.assert_roundoff_equal(
            at_transition[0].body_com_height_m,
            1.0 + 0.5 * stance_acceleration * transition_s**2,
            len(result.samples),
        )
        self.assertEqual(
            at_transition[0].body_com_acceleration_m_s2,
            -scenario.gravity_m_s2,
        )
        self.assertEqual(result.transitions[0].time_s, transition_s)
        self.assertEqual(result.transitions[0].from_mode, PhaseMode.STANCE)
        self.assertEqual(result.transitions[0].to_mode, PhaseMode.FLIGHT)
        self.assertTrue(
            all(
                left.time_s < right.time_s
                for left, right in zip(result.samples, result.samples[1:])
            )
        )

    def test_same_declared_scenario_is_deterministic(self) -> None:
        scenario = Scenario(
            name="manufactured_repeatability",
            masses=MassProperties(body_mass_kg=80.0),
            phases=(
                Phase(0.3, PhaseMode.STANCE, 1000.0),
                Phase(0.4, PhaseMode.FLIGHT, 0.0),
            ),
            initial_body_com_height_m=0.9,
            sample_interval_s=0.017,
        )
        self.assertEqual(simulate(scenario), simulate(scenario))


class MechanicsRefusalTests(unittest.TestCase):
    def test_negative_stance_force_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "nonnegative"):
            Phase(0.2, PhaseMode.STANCE, -1.0)

    def test_nonzero_flight_force_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "zero ground reaction"):
            Phase(0.2, PhaseMode.FLIGHT, 1.0)

    def test_untyped_phase_mode_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "PhaseMode"):
            Phase(0.2, "STANCE")  # type: ignore[arg-type]

    def test_landing_impact_is_refused(self) -> None:
        with self.assertRaisesRegex(ValueError, "impact is unsupported"):
            Scenario(
                name="unsupported_landing",
                masses=MassProperties(body_mass_kg=80.0),
                phases=(
                    Phase(0.2, PhaseMode.STANCE, 1000.0),
                    Phase(0.2, PhaseMode.FLIGHT, 0.0),
                    Phase(0.2, PhaseMode.STANCE, 1000.0),
                ),
                initial_body_com_height_m=1.0,
            )

    def test_massless_bar_offset_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "explicitly declared"):
            MassProperties(body_mass_kg=80.0, bar_offset_m=0.1)


if __name__ == "__main__":
    unittest.main()
