"""Native sampling controls with independent closed-form source signals."""

from dataclasses import replace
import math
from types import SimpleNamespace
import unittest

from biosyngen.i1.instruments import DeviceClock
from biosyngen.i2.acquisition import DUAL_1KHZ_V1, LEGACY_MIXED_RATE_V1
from biosyngen.i2.instruments import (
    CableLptModel, DualPlateModel, LptAttachment, PlateChannelModel,
    observe_dual_plates, observe_lpt,
)
from scripts.probe_acquisition_geometry import geometry_diagnostics


def analytical_world():
    # Exactly radial z=1+t^2, at 1ms physics knots, with constant 100N per foot.
    times = [i / 1000 for i in range(101)]
    contacts = [SimpleNamespace(contact_id=s, plate_frame=SimpleNamespace(
        side=s.upper(), origin_world_m=(0, 0))) for s in ("left", "right")]
    model = SimpleNamespace(
        system=SimpleNamespace(contacts=contacts),
        point_world=lambda t, body, local: (0, 1 + t * t),
    )
    frame = SimpleNamespace(t_start_s=0, t_end_s=0.1, contact_forces={
        s: SimpleNamespace(force_world_n=(0, 100), point_world_m=(0, 0))
        for s in ("left", "right")})
    return SimpleNamespace(model=model, times=lambda: times, states=lambda: times,
                           frames=[frame], impacts=[])


def sensor():
    return CableLptModel(LptAttachment("load", (0, 0), 0, "BAR_COLLAR"),
                         clock=DeviceClock(0.002), anchor_m=(0, 0, 0),
                         encoder_resolution_m=1e-9)


class AcquisitionProfileTests(unittest.TestCase):
    def test_legacy_defaults_and_explicit_profile_are_preserved(self):
        force, lpt = LEGACY_MIXED_RATE_V1.instruments(sensor())
        self.assertEqual(force.clock.sample_period_s, 0.001)
        self.assertEqual(lpt.clock.sample_period_s, 0.002)
        self.assertEqual(CableLptModel(sensor().attachment).clock.sample_period_s, 0.002)
        self.assertEqual(len(observe_lpt(analytical_world(), lpt).samples), 51)

    def test_profile_preserves_separate_clock_and_sensor_parameters(self):
        original_force = DualPlateModel()
        original_lpt = replace(sensor(), clock=DeviceClock(0.002, -0.017, -60, 5e-6))
        force, lpt = DUAL_1KHZ_V1.instruments(original_lpt, original_force)
        self.assertEqual(force, original_force)
        self.assertEqual(replace(lpt, clock=original_lpt.clock), original_lpt)
        self.assertNotEqual(force.clock.offset_s, lpt.clock.offset_s)
        self.assertNotEqual(force.clock.drift_ppm, lpt.clock.drift_ppm)
        self.assertNotEqual(force.seed, lpt.seed)

    def test_both_native_grids_have_101_samples_in_closed_100ms_window(self):
        force, lpt = DUAL_1KHZ_V1.instruments(sensor(), DualPlateModel(clock=DeviceClock(0.001)))
        for stream in (observe_dual_plates(analytical_world(), force), observe_lpt(analytical_world(), lpt)):
            self.assertEqual(len(stream.samples), 101)
            self.assertEqual([s.index for s in stream.samples], list(range(101)))
            for a, b in zip(stream.samples, stream.samples[1:]):
                self.assertGreater(b.native_time_s, a.native_time_s)
                self.assertAlmostEqual(b.native_time_s - a.native_time_s, 0.001, places=14)

    def test_new_odd_samples_are_not_interpolated_from_legacy_500hz(self):
        _, lpt = DUAL_1KHZ_V1.instruments(sensor())
        samples = observe_lpt(analytical_world(), lpt).samples
        # A 500Hz interpolation would give 2e-6 at 1ms; the source is 1e-6.
        self.assertAlmostEqual(samples[1].displacement_m, 1e-6, delta=1e-9)
        self.assertGreater(abs(samples[1].displacement_m - 2e-6), 0.9e-6)
        for s in samples:
            # Floor quantization is within one encoder count plus float roundoff.
            self.assertAlmostEqual(s.displacement_m, s.native_time_s**2, delta=1e-9 + 1e-14)
            if s.provider_velocity_m_s is not None:
                self.assertAlmostEqual(s.provider_velocity_m_s, 2 * s.native_time_s, delta=1e-6)

    def test_encoder_quantization_and_velocity_use_one_ms_spacing(self):
        _, lpt = DUAL_1KHZ_V1.instruments(replace(sensor(), encoder_resolution_m=1e-4))
        samples = observe_lpt(analytical_world(), lpt).samples
        for k, s in enumerate(samples):
            self.assertAlmostEqual(s.displacement_m / 1e-4, round(s.displacement_m / 1e-4), places=8)
            self.assertLessEqual(abs(s.displacement_m - s.native_time_s**2), 1.01e-4)
            if s.provider_velocity_m_s is not None:
                expected = (samples[k + 1].displacement_m - samples[k - 1].displacement_m) / 0.002
                self.assertAlmostEqual(s.provider_velocity_m_s, expected, places=10)
        self.assertIsNone(samples[0].provider_velocity_m_s)
        self.assertIsNone(samples[-1].provider_velocity_m_s)

    def test_different_drifts_produce_different_native_counts_and_origins(self):
        force, lpt = DUAL_1KHZ_V1.instruments(
            replace(sensor(), clock=DeviceClock(0.002, -0.02, -10000)),
            DualPlateModel(clock=DeviceClock(0.001, 0.03, 10000)),
        )
        f = observe_dual_plates(analytical_world(), force).samples
        l = observe_lpt(analytical_world(), lpt).samples
        self.assertEqual((len(f), len(l)), (102, 100))
        self.assertEqual((f[0].native_time_s, l[0].native_time_s), (0.03, -0.02))

    def test_force_saturation_is_retained_at_native_1khz(self):
        channel = PlateChannelModel(gain=((1, 0, 0), (0, 1, 0), (0, 0, 1)),
                                    offset=(0, 0, 0), noise_sd=(0, 0, 0))
        force, _ = DUAL_1KHZ_V1.instruments(sensor(), DualPlateModel(
            clock=DeviceClock(0.001), left=channel, right=channel,
            adc_range=(10, 50, 10)))
        samples = observe_dual_plates(analytical_world(), force).samples
        self.assertTrue(all(s.saturated for s in samples))
        self.assertTrue(all(math.isclose(s.fz_total_n, 100) for s in samples))


class CableGeometryDiagnosticTests(unittest.TestCase):
    def test_fixed_vertical_path_has_no_coordinate_error(self):
        report = geometry_diagnostics([(0, 0, 1), (0, 0, 2)], (0, 0, 0))
        self.assertEqual(report["max_angle_deg"], 0)
        self.assertEqual(report["max_abs_cable_minus_vertical_travel_m"], 0)

    def test_initial_alignment_does_not_establish_full_path_alignment(self):
        # Cable lengths 4 and 5 by the independent 3-4-5 triangle;
        # vertical travel is zero, cable travel is one metre.
        report = geometry_diagnostics([(0, 0, 4), (3, 0, 4)], (0, 0, 0))
        self.assertEqual(report["initial_angle_deg"], 0)
        self.assertEqual(report["max_abs_cable_minus_vertical_travel_m"], 1)
        self.assertEqual(report["max_horizontal_offset_m"], 3)
        self.assertGreater(report["max_angle_deg"], 36)


if __name__ == "__main__":
    unittest.main()
