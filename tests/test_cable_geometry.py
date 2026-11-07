"""Closed-form geometry, interval uncertainty and source-separation controls."""

from dataclasses import replace
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from biosyngen.i1.instruments import DeviceClock
from biosyngen.i2.cable_geometry import (CableAngleSample, InclinationModel,
    observe_cable_angle, reconstruct_vertical)
from biosyngen.i2.instruments import CableLptModel, LptAttachment
from biosyngen.i2.paired_export import export_case, guided_vertical_world, sha


class GeometryObservationTests(unittest.TestCase):
    def test_full_oblique_trajectory_and_interval_containment(self):
        times = [i / 1000 for i in range(1001)]
        point = lambda t: (0.8 * t, 1.4 + 0.3 * t)
        world = SimpleNamespace(model=SimpleNamespace(point_world=lambda t, body, local: point(t)),
                                times=lambda: times, states=lambda: times)
        cable = CableLptModel(LptAttachment("bar", (0, 0), 0, "BAR_COLLAR"),
            clock=DeviceClock(0.001, -0.02, 300), anchor_m=(0, 0, 0))
        angle = InclinationModel()
        observations = observe_cable_angle(world, cable, angle)
        derived = reconstruct_vertical(observations.samples, length_resolution_m=1e-4,
            angle_error_bound_rad=angle.error_bound_rad, angle_resolution_rad=angle.resolution_rad)
        self.assertGreater(len(derived), 1000)
        for sample, t, (value, low, high) in zip(observations.samples, observations.true_times_s, derived):
            expected = 0.3 * t
            self.assertLessEqual(low, expected)
            self.assertGreaterEqual(high, expected)
            self.assertLessEqual(abs(value - expected), high - low)
            self.assertAlmostEqual(sample.native_time_s, -0.02 + sample.index / 1000)
        self.assertGreater(abs(observations.samples[-1].displacement_m - 0.3), 0.1)

    def test_three_four_five_reconstruction_uses_absolute_length_and_angle(self):
        samples = (CableAngleSample(0, 0, 4, 0, 0, False),
                   CableAngleSample(1, 1, 5, 1, math.acos(0.8), False))
        derived = reconstruct_vertical(samples, length_resolution_m=1e-8,
                                        angle_error_bound_rad=0, angle_resolution_rad=0)
        self.assertAlmostEqual(derived[-1][0], 0, places=14)
        self.assertLessEqual(derived[-1][1], 0)
        self.assertGreaterEqual(derived[-1][2], 0)

    def test_unobserved_angle_bias_is_not_silently_corrected_from_truth(self):
        samples = (CableAngleSample(0, 0, 1, 0, 0, False),
                   CableAngleSample(1, 1, 2, 1, math.pi / 3, False))
        derived = reconstruct_vertical(samples, length_resolution_m=1e-6,
            angle_error_bound_rad=0, angle_resolution_rad=0)
        self.assertAlmostEqual(derived[-1][0], 0, places=14)
        # If the source falsely declares theta=pi/3 for a vertical path, true
        # displacement=1 lies outside the declared measurement interval.
        self.assertLess(derived[-1][2], 0.001)

    def test_slack_and_nonfinite_values_refuse_reconstruction(self):
        for sample in (CableAngleSample(0, 0, 1, 0, 0, True),
                       CableAngleSample(0, 0, 1, 0, float("nan"), False)):
            with self.assertRaises(ValueError):
                reconstruct_vertical([sample], length_resolution_m=1e-4,
                                      angle_error_bound_rad=0.001, angle_resolution_rad=1e-5)

    def test_angle_error_sequence_is_separate_and_seeded(self):
        world, cable, _ = guided_vertical_world()
        cable = replace(cable, clock=DeviceClock(0.001, 0, 0))
        a = observe_cable_angle(world, cable, InclinationModel(seed=1))
        b = observe_cable_angle(world, cable, InclinationModel(seed=2))
        self.assertEqual([s.encoder_length_m for s in a.samples], [s.encoder_length_m for s in b.samples])
        self.assertNotEqual([s.inclination_rad for s in a.samples], [s.inclination_rad for s in b.samples])
        self.assertTrue(all(abs(s.inclination_rad) <= 0.000505 for s in a.samples))


class SourcePackageTests(unittest.TestCase):
    def test_guided_apparatus_interval_reactions_close_analytic_impulse(self):
        world, _, _ = guided_vertical_world()
        total_impulse = 0.0
        for frame in world.frames:
            reaction = sum(c.force_world_n[1] for c in frame.contact_forces.values())
            self.assertGreater(reaction, 0)
            total_impulse += (reaction - 95 * 9.81) * (frame.t_end_s - frame.t_start_s)
        # Both endpoint velocities are exactly zero; the interval average force
        # is obtained from analytical velocity differences, not midpoint force.
        self.assertAlmostEqual(total_impulse, 0, delta=1e-10)

    def test_vertical_control_export_inventory_hashes_and_truth_separation(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parents[1] / "runs") as tmp:
            root = Path(tmp)
            case = export_case(root, protocol="guided_vertical", seed=524, case_id="trial-test")
            source = root / "SOURCE/trial-test"
            files = {p.relative_to(source).as_posix() for p in source.rglob("*") if p.is_file()}
            self.assertEqual(files, {"manifest.json", "checksums.sha256", "streams/force.csv", "streams/lpt.csv"})
            manifest = json.loads((source / "manifest.json").read_text())
            self.assertEqual(manifest["lpt_stream"]["frame_id"], "LAB_VERTICAL_Z_UP")
            for record in manifest["source_files"]:
                self.assertEqual(record["sha256"], sha(source / record["path"]))
            text = "".join(p.read_text() for p in source.rglob("*") if p.is_file())
            for forbidden in ("physics_times_s", "bar_vertical_m", "events", "true_sample_times"):
                self.assertNotIn(forbidden, text)
            self.assertNotEqual(case["force_count"], case["lpt_count"])
            self.assertTrue((root / "TRUTH/trial-test/reference.json").exists())
            with self.assertRaises(FileExistsError):
                export_case(root, protocol="guided_vertical", seed=524, case_id="trial-test")


if __name__ == "__main__":
    unittest.main()
