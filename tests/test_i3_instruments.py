"""RES-504 transduction controls from hand-authored wrenches/cable paths."""
from dataclasses import replace
import math
from types import SimpleNamespace
import unittest
from biosyngen.i1.instruments import DeviceClock
from biosyngen.i2.instruments import (
    CableLptModel, DualPlateModel, LptAttachment, PlateChannelModel, observe_dual_plates, observe_lpt,
)


class IndependentInstrumentTests(unittest.TestCase):
    def test_plate_force_moment_cop_and_bilateral_sum_on_independent_wrenches(self):
        contacts = [SimpleNamespace(contact_id=side, plate_frame=SimpleNamespace(side=side.upper(), origin_world_m=(0, 0)))
                    for side in ("left", "right")]
        samples = {"left": SimpleNamespace(force_world_n=(2, 10), point_world_m=(0.2, 0)),
                   "right": SimpleNamespace(force_world_n=(-3, 20), point_world_m=(-0.1, 0))}
        frame = SimpleNamespace(t_start_s=0, t_end_s=0.1, contact_forces=samples)
        trajectory = SimpleNamespace(model=SimpleNamespace(system=SimpleNamespace(contacts=contacts)), frames=[frame], impacts=[])
        channel = PlateChannelModel(gain=((1, 0, 0), (0, 1, 0), (0, 0, 1)), offset=(0, 0, 0), noise_sd=(0, 0, 0))
        model = DualPlateModel(clock=DeviceClock(0.001, 0.023, 100), left=channel, right=channel,
                               adc_bits=24, origin_depth_m=0.04)
        exported = observe_dual_plates(trajectory, model)
        self.assertGreater(len(exported.samples), 90)
        for sample in exported.samples:
            self.assertAlmostEqual(sample.fz_left_n, 10, delta=0.001)
            self.assertAlmostEqual(sample.fz_right_n, 20, delta=0.001)
            self.assertEqual(sample.fz_total_n, sample.fz_left_n + sample.fz_right_n)
            self.assertAlmostEqual(sample.my_left_n_m, 1.92, delta=0.001)
            self.assertAlmostEqual(sample.my_right_n_m, -1.88, delta=0.001)
            self.assertAlmostEqual((sample.my_left_n_m + 0.04 * sample.fx_left_n) / sample.fz_left_n, 0.2, delta=0.001)
            self.assertAlmostEqual(sample.native_time_s, 0.023 + sample.index * 0.001, places=12)
        self.assertEqual(exported.metadata["stage_status"], "ASSUMED_UNCALIBRATED")

    def test_cable_geometry_clock_drift_and_provider_velocity_on_a_radial_path(self):
        # Hand path in 3-D: (3*(1+t), 4, 0), anchor (0,4,0).
        # True cable length is exactly 3*(1+t); nominal rate has 1000 ppm drift.
        states = [0.0, 0.1]
        model = SimpleNamespace(point_world=lambda state, body, local: (3 * (1 + state), 0))
        trajectory = SimpleNamespace(model=model, times=lambda: states, states=lambda: states)
        sensor = CableLptModel(LptAttachment("hand", (0, 0), 4, "HAND"), clock=DeviceClock(0.002, -0.017, 1000),
                               anchor_m=(0, 4, 0), encoder_resolution_m=1e-6)
        exported = observe_lpt(trajectory, sensor)
        for sample in exported.samples:
            true_time = sample.index * 0.002 / 1.001
            self.assertAlmostEqual(sample.displacement_m, 3 * true_time, delta=2e-6)
            self.assertAlmostEqual(sample.native_time_s, -0.017 + sample.index * 0.002, places=12)
            if sample.provider_velocity_m_s is not None:
                self.assertAlmostEqual(sample.provider_velocity_m_s, 3 / 1.001, delta=0.001)
        self.assertEqual(exported.metadata["stage_status"], "ASSUMED_UNCALIBRATED")

    def test_slow_reel_exposes_slack_instead_of_tracking_true_shortening(self):
        states = [0.0, 0.1]
        trajectory = SimpleNamespace(model=SimpleNamespace(point_world=lambda t, body, local: (1 - 3 * t, 0)),
                                     times=lambda: states, states=lambda: states)
        fast = CableLptModel(LptAttachment("hand", (0, 0), 0, "HAND"), clock=DeviceClock(0.002),
                             anchor_m=(0, 0, 0), encoder_resolution_m=1e-6, max_retraction_speed_m_s=4)
        normal = observe_lpt(trajectory, fast)
        slow = observe_lpt(trajectory, replace(fast, max_retraction_speed_m_s=0.1))
        self.assertFalse(any(s.slack for s in normal.samples))
        self.assertTrue(all(s.slack for s in slow.samples[1:]))
        self.assertGreater(slow.samples[-1].displacement_m - normal.samples[-1].displacement_m, 0.25)


if __name__ == "__main__":
    unittest.main()
