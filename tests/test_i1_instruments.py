"""RES-484 instrument-chain and RES-483 detector/observability tests.

References are closed forms (Butterworth magnitude, Euclidean cable length,
projection identities, quantization bounds) or explicit refusal semantics.
"""

from __future__ import annotations

from dataclasses import replace
import math
import unittest

from biosyngen.i1.instruments import (
    DeviceClock,
    ForcePlateModel,
    LptModel,
    _filter_stream,
    butterworth2_coefficients,
    cable_length,
    latent_plate_wrench,
    lpt_attachment_point,
    observe_force_plate,
    observe_lpt,
)
from biosyngen.i1.observability import study
from biosyngen.i1.phases import analyze_phases
from biosyngen.i1.scenarios import reference_cmj
from biosyngen.i1.simulate import ContactMode, simulate_jump
from biosyngen_benchmark.detectors import detect_force_events, detect_lpt_events

_CACHE: dict = {}


def cmj():
    if "cmj" not in _CACHE:
        _CACHE["cmj"] = simulate_jump(reference_cmj())
    return _CACHE["cmj"]


IDEAL_PLATE = ForcePlateModel(
    clock=DeviceClock(0.001),
    gain=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    offset=(0.0, 0.0, 0.0),
    noise_sd=(0.0, 0.0, 0.0),
    filter_cutoff_hz=4000.0,
    adc_bits=24,
)


class FilterTests(unittest.TestCase):
    def test_butterworth_magnitude_matches_closed_form(self) -> None:
        rate, fc = 10_000.0, 250.0
        b, a = butterworth2_coefficients(fc, rate)
        for f in (50.0, 250.0, 1000.0, 2500.0):
            n = int(rate * 0.2)
            xs = [math.sin(2 * math.pi * f * i / rate) for i in range(n)]
            ys = _filter_stream(xs, b, a, 0.0)
            amplitude = max(abs(y) for y in ys[n // 2 :])
            # Bilinear transform with pre-warping: |H| = 1/sqrt(1 + (w_d/w_c)^4),
            # w = tan(pi f / rate).
            wd = math.tan(math.pi * f / rate) / math.tan(math.pi * fc / rate)
            expected = 1.0 / math.sqrt(1.0 + wd**4)
            self.assertAlmostEqual(amplitude, expected, delta=2e-3)

    def test_anti_alias_filter_suppresses_a_component_above_nyquist(self) -> None:
        rate, fc = 10_000.0, 250.0
        b, a = butterworth2_coefficients(fc, rate)
        n = int(rate * 0.2)
        xs = [math.sin(2 * math.pi * 900.0 * i / rate) for i in range(n)]
        ys = _filter_stream(xs, b, a, 0.0)
        aliased = [ys[i] for i in range(n // 2, n, 10)]  # 1 kHz device sampling
        self.assertLess(max(abs(v) for v in aliased), 0.08)
        raw = [xs[i] for i in range(n // 2, n, 10)]
        self.assertGreater(max(abs(v) for v in raw), 0.5)


class ForcePlateTests(unittest.TestCase):
    def test_ideal_plate_reproduces_latent_force_and_cop(self) -> None:
        traj = cmj()
        export = observe_force_plate(traj, IDEAL_PLATE)
        latent_times = [s.time_s for s in traj.samples]
        for s in export.samples[:: 25]:
            t = s.native_time_s
            # Nearest latent sample at or after t, compared within the change
            # over one latent interval (<= 1 ms) plus the ADC step.
            k = next(i for i, lt in enumerate(latent_times) if lt >= t - 1e-12)
            a = traj.samples[max(0, k - 1)]
            b = traj.samples[k]
            lo, hi = sorted((a.grf_z_n, b.grf_z_n))
            step = IDEAL_PLATE.adc_range[1] / 2 ** 23
            self.assertTrue(lo - 1.0 - step <= s.fz_n <= hi + 1.0 + step, (t, s.fz_n, lo, hi))
        quiet = export.samples[100]
        self.assertAlmostEqual(quiet.cop_x_m, traj.samples[0].cop_x_m, delta=1e-4)

    def test_cop_is_missing_in_flight_and_bilateral_channels_are_not_offered(self) -> None:
        traj = cmj()
        export = observe_force_plate(traj, ForcePlateModel())
        takeoff = traj.event_time("TAKEOFF")
        flight = [s for s in export.samples if s.native_time_s - export.model.clock.offset_s > takeoff + 0.01]
        self.assertTrue(flight)
        self.assertTrue(all(s.cop_x_m is None for s in flight))
        self.assertTrue(all(abs(s.fz_n) < 10.0 for s in flight))  # noise/offset around zero, signed
        self.assertTrue(any(s.fz_n < 0.0 for s in flight))
        self.assertIn("NOT_AVAILABLE", export.metadata["bilateral_channels"])

    def test_adc_saturation_is_flagged_and_quantization_is_bounded(self) -> None:
        traj = cmj()
        low_range = replace(IDEAL_PLATE, adc_range=(2500.0, 1500.0, 1500.0), adc_bits=12)
        export = observe_force_plate(traj, low_range)
        self.assertTrue(any(s.saturated for s in export.samples))
        self.assertLessEqual(max(s.fz_n for s in export.samples), 1500.0)
        step = 1500.0 / 2 ** 11
        for s in export.samples[:200]:
            self.assertAlmostEqual(s.fz_n / step, round(s.fz_n / step), delta=1e-6)

    def test_native_timestamps_are_nominal_and_independent_of_drift(self) -> None:
        traj = cmj()
        export = observe_force_plate(traj, ForcePlateModel())
        clock = export.model.clock
        for s in export.samples[:50]:
            self.assertEqual(s.native_time_s, clock.offset_s + s.index * clock.sample_period_s)

    def test_moment_and_cop_round_trip_through_the_plate_origin(self) -> None:
        traj = cmj()
        model = ForcePlateModel()
        for s in traj.samples[::200]:
            if s.cop_x_m is None:
                continue
            fx, fz, my = latent_plate_wrench(s, model)
            cop = model.plate_center_x_m + (my + model.origin_depth_m * fx) / fz
            self.assertAlmostEqual(cop, s.cop_x_m, delta=1e-12)


class LptTests(unittest.TestCase):
    def test_cable_displacement_is_euclidean_and_differs_from_vertical_motion(self) -> None:
        traj = cmj()
        model = LptModel(clock=DeviceClock(0.002), encoder_resolution_m=1e-6, anchor_m=(-0.6, 0.3, 0.0))
        export = observe_lpt(traj, model)
        p0 = lpt_attachment_point(traj.samples[0], traj, model)
        l0 = cable_length(p0, model.anchor_m)
        worst_cable = 0.0
        worst_vertical_gap = 0.0
        for s in export.samples[::20]:
            latent = traj.sample_at_or_after(s.native_time_s)
            p = lpt_attachment_point(latent, traj, model)
            expected = cable_length(p, model.anchor_m) - l0
            worst_cable = max(worst_cable, abs(s.displacement_m - expected))
            worst_vertical_gap = max(worst_vertical_gap, abs(s.displacement_m - (p[2] - p0[2])))
        self.assertLess(worst_cable, 5e-3)  # <= one latent interval of motion + encoder step
        self.assertGreater(worst_vertical_gap, 0.02)  # oblique cable is not vertical displacement

    def test_provider_velocity_is_derived_and_not_vertical_velocity(self) -> None:
        traj = cmj()
        model = LptModel(clock=DeviceClock(0.002), anchor_m=(-0.6, 0.3, 0.0))
        export = observe_lpt(traj, model)
        self.assertIn("PROVIDER_DERIVED", export.metadata["velocity_stage"])
        s = export.samples[400]
        expected = (export.samples[401].displacement_m - export.samples[399].displacement_m) / (2 * 0.002)
        self.assertAlmostEqual(s.provider_velocity_m_s, expected, delta=1e-12)

    def test_slow_reel_produces_flagged_slack(self) -> None:
        traj = cmj()
        export = observe_lpt(traj, LptModel(max_retraction_speed_m_s=0.5))
        self.assertGreater(export.stage_trace["slack_samples"], 0)
        export_fast = observe_lpt(traj, LptModel())
        self.assertEqual(export_fast.stage_trace["slack_samples"], 0)


class DetectorAndObservabilityTests(unittest.TestCase):
    def test_detectors_refuse_unsupported_inputs(self) -> None:
        times = [i * 0.001 for i in range(600)]
        flat = [735.0] * 600
        self.assertEqual(detect_force_events(times, flat)["onset_threshold"].refusal, "NO_THRESHOLD_CROSSING")
        gap = list(flat)
        gap[10] = None
        self.assertEqual(detect_force_events(times, gap)["takeoff"].refusal, "MISSING_SAMPLE_IN_QUIET_WINDOW")
        moving = [735.0 + 100.0 * math.sin(20 * t) for t in times]
        self.assertEqual(detect_force_events(times, moving)["body_weight"].refusal, "NO_QUIET_STANCE_BASELINE")
        lpt = detect_lpt_events(times, [0.0] * 600, [0.0] * 600)
        self.assertIn("NOT_IDENTIFIABLE", lpt["lpt_takeoff"].refusal)

    def test_threshold_onset_lags_latent_onset_and_backdating_is_a_convention(self) -> None:
        traj = cmj()
        pa = analyze_phases(traj)
        rows, fdet, _ = study(pa, observe_force_plate(traj, ForcePlateModel()), observe_lpt(traj, LptModel()))
        by = {(r.latent_event, r.observed_event): r for r in rows}
        onset = by[("kinetic_onset", "force:onset_threshold")]
        self.assertEqual(onset.status, "COMPARED")
        self.assertGreater(onset.error_ms, 0.0)  # causal: detection cannot precede the force change
        takeoff = by[("takeoff", "force:takeoff")]
        self.assertLess(abs(takeoff.error_ms), 3 * takeoff.resolution_ms)
        self.assertEqual(by[("takeoff", "lpt:lpt_takeoff")].status, "NOT_EVALUABLE")
        self.assertEqual(by[("touchdown", "force:landing")].status, "NOT_EVALUABLE")
        # Hip-belt cable reversal is not the COM reversal (attachment != COM).
        lpt_rev = by[("com_velocity_reversal", "lpt:lpt_velocity_reversal")]
        self.assertGreater(abs(lpt_rev.error_ms), 5.0)

    def test_squat_jump_has_no_countermovement_and_detector_refuses_a_reversal(self) -> None:
        # Degenerate-reversal challenge: before the noise-floor rule the
        # integrated-force detector reported a spurious reversal here.
        from biosyngen.i1.scenarios import reference_sj

        traj = simulate_jump(reference_sj())
        pa = analyze_phases(traj, "SJ")
        self.assertEqual(pa.refusals, ())
        self.assertGreater(pa.peak_downward_velocity_m_s, -1e-6)
        export = observe_force_plate(traj, ForcePlateModel())
        times = [s.native_time_s - export.model.clock.offset_s for s in export.samples]
        det = detect_force_events(times, [s.fz_n for s in export.samples])
        self.assertEqual(det["velocity_reversal"].refusal, "NO_SIGNIFICANT_COUNTERMOVEMENT")
        self.assertIsNone(det["takeoff"].refusal)


if __name__ == "__main__":
    unittest.main()
