"""Configuration, replay, source authority and semantic adversaries."""
import csv
import copy
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

from biosyngen.i2.corpus_config import validate_parameters, ConfigurationRefused
from biosyngen.i2.paired_export import export_case, sha
from biosyngen_benchmark.paired_source import verify_source, SourceRefused
from biosyngen_benchmark.paired_events import verify_events
from scripts.verify_paired_corpus import verify_reference
from scripts.compare_paired_platforms import compare

ROOT = Path(__file__).resolve().parents[1]


class ConfigTests(unittest.TestCase):
    def test_platform_comparison_rejects_moved_events_and_wrong_rates(self):
        case = {"case_id": "case", "protocol": "cmj", "reference_resolution_s": 0.001,
            "source": {"source": "PASS", "synchronization": "BOUNDED_CAPTURE_MAPPING",
                "nominal_rates_hz": {"force": 1000, "lpt": 1000}, "dropped_sample_counts": {},
                "force_saturated_sample_count": 0},
            "mechanics": {"findings": {"LINEAR_MOMENTUM_CLOSURE": {"outcome": "PASS"}}},
            "geometry": {"status": "PASS", "max_error_m": 0.0001},
            "independent_reference_events": {"findings": [{"event": "takeoff", "outcome": "PASS", "declared_time_s": 1.5}]}}
        fixture = {"requested_count": 16, "qualified_count": 16, "cases": [copy.deepcopy(case) for _ in range(16)]}
        for i, c in enumerate(fixture["cases"]):
            c["case_id"] = f"case-{i}"
        self.assertEqual(compare(fixture, fixture)["status"], "PASS")
        with self.assertRaisesRegex(ValueError, "PLATFORM_CASE_INVENTORY_MISMATCH"):
            compare(fixture, {**fixture, "cases": fixture["cases"][:15]})
        moved = copy.deepcopy(fixture)
        moved["cases"][0]["independent_reference_events"]["findings"][0]["declared_time_s"] += 0.01
        with self.assertRaisesRegex(ValueError, "PLATFORM_EVENT_TIME_DIFFERENCE"):
            compare(fixture, moved)
        wrong_rate = copy.deepcopy(fixture)
        wrong_rate["cases"][0]["source"]["nominal_rates_hz"]["lpt"] = 500
        with self.assertRaisesRegex(ValueError, "PLATFORM_SOURCE_DECISION_MISMATCH"):
            compare(fixture, wrong_rate)

    def test_independent_event_checks_reject_shifted_takeoff(self):
        ref = {"protocol": "sj", "mass_kg": 1, "physics_times_s": [0, 1, 2, 3, 4, 5],
            "contacts": [{"left": m, "right": m} for m in ("STICK", "STICK", "OPEN", "OPEN", "OPEN", "STICK")],
            "contact_specs": [{"contact_id": side, "plate_frame": {}, "kind": "FOOT"} for side in ("left", "right")],
            "vertical_momentum_kg_m_s": [0, 1, 2, 1, -1, 0]}
        declared = {"events": {"takeoff": {"time_s": 2}, "touchdown": {"time_s": 5}}}
        self.assertTrue(verify_events(ref, declared)["passed"])
        declared["events"]["takeoff"]["time_s"] = 20
        self.assertFalse(verify_events(ref, declared)["passed"])

    def test_domains_and_inapplicable_parameters_fail_closed(self):
        for protocol, parameters in (("back_squat", {"body_mass_kg": 0}),
                ("deadlift", {"effort": 0.8}), ("cmj", {"load_mass_kg": 10}),
                ("guided_vertical", {"stature_m": 1.8}), ("deadlift", {"friction": float("nan")}),
                ("back_squat", {"contact_configuration": "INVENTED"}),
                ("deadlift", {"force_drop_indices": [1, 1]}), ("drop_jump", {}),
                ("power_clean", {}), ("guided_vertical", {"secret_truth": 1})):
            with self.subTest(protocol=protocol, parameters=parameters), self.assertRaises(ConfigurationRefused):
                validate_parameters(parameters, protocol)

    def test_config_changes_actual_force_motion_and_recording(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "runs") as tmp:
            root = Path(tmp)
            a = export_case(root, protocol="guided_vertical", seed=1, case_id="a")
            b = export_case(root, protocol="guided_vertical", seed=1, case_id="b", parameters={
                "body_mass_kg": 90, "load_mass_kg": 30, "tempo_scale": 1.2,
                "force_gain_scale": 0.95, "lpt_start_s": 0.2, "lpt_drop_indices": [500]})
            self.assertNotEqual(a["force_count"], b["force_count"])
            self.assertNotEqual((root / "SOURCE/a/streams/force.csv").read_bytes(),
                                (root / "SOURCE/b/streams/force.csv").read_bytes())
            ref = json.loads((root / "TRUTH/b/reference.json").read_text())
            self.assertEqual(ref["mass_kg"], 120)
            self.assertAlmostEqual(ref["events"]["bottom_s"], 2.4)
            with (root / "SOURCE/b/streams/lpt.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["sample_index"], "200")
            self.assertNotIn("500", [r["sample_index"] for r in rows])

    def test_fresh_process_replay_and_fresh_directory_requirement(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "runs") as tmp:
            parent = Path(tmp)
            for name in ("a", "b"):
                result = subprocess.run([sys.executable, "-m", "scripts.generate_paired_corpus",
                    "--protocol", "guided_vertical", "--count", "4", "--seed", "42", "--out", str(parent / name)],
                    cwd=ROOT, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual((parent / "a/inventory.json").read_bytes(), (parent / "b/inventory.json").read_bytes())
            result = subprocess.run([sys.executable, "-m", "scripts.generate_paired_corpus", "--protocol",
                "guided_vertical", "--out", str(parent / "a")], cwd=ROOT, capture_output=True)
            self.assertNotEqual(result.returncode, 0)

    def test_refusals_remain_in_requested_denominator(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "runs") as tmp:
            out = Path(tmp) / "refused"
            result = subprocess.run([sys.executable, "-m", "scripts.generate_paired_corpus", "--protocol",
                "drop_jump", "--count", "4", "--out", str(out)], cwd=ROOT, capture_output=True)
            self.assertEqual(result.returncode, 1)
            report = json.loads((out / "run.json").read_text())
            self.assertEqual((report["requested_count"], report["generated_count"], report["refused_count"]), (4, 0, 4))

    def test_independent_clocks_partial_overlap_gaps_and_jitter_policy(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "runs") as tmp:
            root = Path(tmp)
            for case_id, p in (("partial", {"force_offset_s": 0.3, "lpt_offset_s": -0.5,
                    "force_drift_ppm": 300, "lpt_drift_ppm": -300, "lpt_start_s": 0.5,
                    "force_drop_indices": [500, 501]}), ("jitter", {"jitter_sd_s": 5e-6}),
                    ("none", {"force_end_s": 1, "lpt_start_s": 2}), ("legacy", {"acquisition_profile": "legacy_mixed"})):
                export_case(root, protocol="guided_vertical", seed=10, case_id=case_id, parameters=p)
                authority = json.loads((root / "DIAGNOSTICS" / case_id / "capture_authority.json").read_text())
                if case_id == "none":
                    with self.assertRaisesRegex(SourceRefused, "NO_CLOCK_OVERLAP"):
                        verify_source(root / "SOURCE" / case_id, authority)
                else:
                    result = verify_source(root / "SOURCE" / case_id, authority)
                    if case_id == "partial":
                        self.assertEqual(result["dropped_sample_counts"]["force"], 2)
                    if case_id == "jitter":
                        self.assertEqual(result["synchronization"], "REFUSED_UNBOUNDED_SAMPLE_JITTER")
                    if case_id == "legacy":
                        self.assertEqual(result["nominal_rates_hz"], {"force": 1000, "lpt": 500})


class SourceAdversaries(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(dir=ROOT / "runs")
        cls.root = Path(cls.tmp.name)
        export_case(cls.root, protocol="back_squat", seed=1, case_id="fixture")
        cls.original = cls.root / "SOURCE/fixture"
        cls.capture = json.loads((cls.root / "DIAGNOSTICS/fixture/capture_authority.json").read_text())

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def setUp(self):
        self.source = self.root / self._testMethodName
        shutil.copytree(self.original, self.source)
        self.authority = json.loads(json.dumps(self.capture))

    def reseal(self):
        # Test-only operator explicitly endorses altered bytes, to exercise
        # semantic validation beyond integrity. Never used in production.
        manifest_path = self.source / "manifest.json"
        manifest = json.loads(manifest_path.read_text())
        notes = json.loads(manifest["operator_notes"])
        for name in ("force", "lpt"):
            notes["sync_capture"]["recording_hashes"][name] = sha(self.source / f"streams/{name}.csv")
        self.authority.update(notes["sync_capture"])
        manifest["operator_notes"] = json.dumps(notes, sort_keys=True)
        for record in manifest["source_files"]:
            path = self.source / record["path"]
            if path.exists():
                record.update(sha256=sha(path), byte_length=path.stat().st_size)
        manifest_path.write_text(json.dumps(manifest, sort_keys=True))
        checksums = "".join(f"{sha(p)}  {p.relative_to(self.source).as_posix()}\n"
            for p in sorted(self.source.rglob("*")) if p.is_file() and p.name != "checksums.sha256")
        (self.source / "checksums.sha256").write_text(checksums)
        self.authority["source_manifest_sha256"] = sha(manifest_path)

    def change_manifest(self, function):
        path = self.source / "manifest.json"
        manifest = json.loads(path.read_text())
        function(manifest)
        path.write_text(json.dumps(manifest))
        self.reseal()

    def test_clean_physics_geometry_and_baseline(self):
        self.assertEqual(verify_source(self.source, self.authority)["source"], "PASS")
        result = verify_reference(self.source, self.root / "TRUTH/fixture")
        self.assertTrue(result["passed"], result)

    def test_completion_after_record_is_rejected(self):
        truth = self.root / "TRUTH/fixture"
        reference = json.loads((truth / "reference.json").read_text())
        analysis = json.loads((truth / "model_events.json").read_text())["analysis"]
        analysis["reps"][0]["rep_end_s"] = reference["physics_times_s"][-1] + 1
        self.assertFalse(verify_events(reference, analysis)["passed"])

    def test_no_independent_authority(self):
        with self.assertRaisesRegex(SourceRefused, "INDEPENDENT_CAPTURE_AUTHORITY_REQUIRED"):
            verify_source(self.source, None)

    def test_corrupted_bytes(self):
        with (self.source / "streams/force.csv").open("a") as stream:
            stream.write("corrupt\n")
        with self.assertRaisesRegex(SourceRefused, "CHECKSUM_MISMATCH"):
            verify_source(self.source, self.authority)

    def test_self_consistently_rehashed_cross_trial_swap(self):
        other = self.root / "SOURCE/other"
        export_case(self.root, protocol="guided_vertical", seed=2, case_id="other")
        shutil.copyfile(other / "streams/force.csv", self.source / "streams/force.csv")
        original_authority = json.loads(json.dumps(self.authority))
        self.reseal()
        with self.assertRaisesRegex(SourceRefused, "CAPTURE_AUTHORITY_MANIFEST_MISMATCH"):
            verify_source(self.source, original_authority)

    def test_wrong_processing_attachment_frame_and_clock(self):
        for key, field, value, code in (("lpt_stream", "processing", {"state": "DERIVED"}, "PROCESSING_STATE_UNSUPPORTED"),
                ("lpt_stream", "attachment_point", "SYSTEM_COM", "ATTACHMENT_UNSUPPORTED"),
                ("lpt_stream", "frame_id", "LAB_VERTICAL_Z_UP", "FRAME_UNSUPPORTED"),
                ("force_stream", "timebase", {"clock_id": "lpt"}, "CLOCK_ID_MISMATCH")):
            shutil.copyfile(self.original / "manifest.json", self.source / "manifest.json")
            self.change_manifest(lambda m: m[key].update({field: value}))
            with self.subTest(code=code), self.assertRaisesRegex(SourceRefused, code):
                verify_source(self.source, self.authority)

    def test_missing_angle(self):
        (self.source / "streams/angle.csv").unlink()
        self.reseal()
        with self.assertRaisesRegex(SourceRefused, "MANIFEST_HASH_MISMATCH"):
            verify_source(self.source, self.authority)

    def test_inconsistent_absolute_encoder_reference(self):
        self.change_manifest(lambda m: m.update(operator_notes=json.dumps({**json.loads(m["operator_notes"]),
            "observed_encoder_zero_m": 9})))
        with self.assertRaisesRegex(SourceRefused, "ENCODER_ABSOLUTE_REFERENCE_MISMATCH"):
            verify_source(self.source, self.authority)

    def test_wrong_capture_mapping(self):
        self.change_manifest(lambda m: m["synchronization"]["stream_mappings"]["lpt"].update(offset_s=100))
        with self.assertRaisesRegex(SourceRefused, "DECLARED_MAPPING_DISAGREES_WITH_CAPTURE"):
            verify_source(self.source, self.authority)

    def test_held_out_capture_is_independently_checked(self):
        def corrupt(m):
            notes = json.loads(m["operator_notes"])
            notes["sync_capture"]["devices"]["lpt"]["edge_native_times_s"][1] += 0.01
            m["operator_notes"] = json.dumps(notes)
        self.change_manifest(corrupt)
        with self.assertRaisesRegex(SourceRefused, "HELD_OUT_CAPTURE_RESIDUAL"):
            verify_source(self.source, self.authority)

    def test_clock_reset_even_with_operator_rehash(self):
        path = self.source / "streams/lpt.csv"
        lines = path.read_text().splitlines()
        fields = lines[10].split(",")
        fields[1] = "-1"
        lines[10] = ",".join(fields)
        path.write_text("\n".join(lines) + "\n")
        self.reseal()
        with self.assertRaisesRegex(SourceRefused, "CLOCK_RESET_OR_NONMONOTONIC"):
            verify_source(self.source, self.authority)

    def test_wrong_angle_is_falsified_by_reference_comparison(self):
        path = self.source / "derived/vertical.csv"
        with path.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        rows[1000]["lower_m"], rows[1000]["upper_m"] = "5", "6"
        with path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        self.reseal()
        self.assertEqual(verify_reference(self.source, self.root / "TRUTH/fixture")["geometry"]["status"], "REJECTED")

    def test_slack_never_produces_a_qualified_vertical_coordinate(self):
        path = self.source / "streams/angle.csv"
        lines = path.read_text().splitlines()
        fields = lines[10].split(",")
        fields[-1] = "1"
        lines[10] = ",".join(fields)
        path.write_text("\n".join(lines) + "\n")
        self.reseal()
        with self.assertRaisesRegex(SourceRefused, "SLACK_DERIVED_FALSE_ACCEPT"):
            verify_source(self.source, self.authority)


if __name__ == "__main__":
    unittest.main()
