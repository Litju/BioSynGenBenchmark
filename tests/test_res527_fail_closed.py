"""RES-527 fail-closed paired qualification regressions for RES-525 F01-F08.

Every adversary is a disposable manufactured copy. Test-only "reseal" helpers
play an operator who coherently re-endorses altered bytes, so each refusal is
proved to come from the semantic admission contract, not from a stale hash.
"""
import csv
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import textwrap
import unittest
from unittest.mock import patch

from biosyngen.i2.paired_export import export_case, sha
from biosyngen_benchmark.paired_source import SourceRefused, verify_source
import scripts.verify_paired_corpus as verifier
from scripts.verify_paired_corpus import ReferenceRefused, verify_reference
from scripts.compare_paired_platforms import compare

ROOT = Path(__file__).resolve().parents[1]
CASES = {"guide": ("guided_vertical", {}), "jitter": ("guided_vertical", {"jitter_sd_s": 5e-6}),
         "legacy_guide": ("guided_vertical", {"acquisition_profile": "legacy_mixed"}),
         "heavy_guide": ("guided_vertical", {"body_mass_kg": 90, "load_mass_kg": 30}),
         "cmj": ("cmj", {}), "squat": ("back_squat", {}), "deadlift": ("deadlift", {}),
         "legacy_cmj": ("cmj", {"acquisition_profile": "legacy_mixed"})}


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


class FailClosedQualification(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(dir=ROOT / "runs")
        cls.corpus = Path(cls.tmp.name) / "corpus"
        cls.receipts = {}
        for seed, (case_id, (protocol, parameters)) in enumerate(CASES.items(), 52700):
            cls.receipts[case_id] = {"case_id": case_id, "protocol": protocol,
                "status": "GENERATED_NOT_EMPIRICALLY_QUALIFIED",
                **export_case(cls.corpus, protocol=protocol, seed=seed, case_id=case_id, parameters=parameters)}

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def setUp(self):
        self.work = Path(self.tmp.name) / self._testMethodName
        self.serial = 0

    # ----- disposable bundles and test-only operator re-endorsement -----
    def bundle(self, *case_ids):
        self.serial += 1
        root = self.work / f"bundle-{self.serial}"
        for case_id in case_ids:
            for namespace in ("SOURCE", "TRUTH", "DIAGNOSTICS"):
                shutil.copytree(self.corpus / namespace / case_id, root / namespace / case_id)
        dump(root / "run.json", {"requested_count": len(case_ids), "generated_count": len(case_ids),
            "refused_count": 0, "cases": [copy.deepcopy(self.receipts[c]) for c in case_ids]})
        return root

    def paths(self, root, case_id):
        return root / "SOURCE" / case_id, root / "TRUTH" / case_id, root / "DIAGNOSTICS" / case_id / "capture_authority.json"

    def authority(self, root, case_id):
        return json.loads(self.paths(root, case_id)[2].read_text())

    def edit_manifest(self, root, case_id, function):
        path = self.paths(root, case_id)[0] / "manifest.json"
        manifest = json.loads(path.read_text())
        notes = json.loads(manifest["operator_notes"])
        function(manifest, notes)
        manifest["operator_notes"] = json.dumps(notes, sort_keys=True)
        dump(path, manifest)

    def reseal(self, root, case_id, *, bind_truth=True):
        source, truth, authority_path = self.paths(root, case_id)
        manifest_path = source / "manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["source_files"] = [dict(r, sha256=sha(source / r["path"]), byte_length=(source / r["path"]).stat().st_size)
                                    for r in manifest["source_files"] if (source / r["path"]).is_file()]
        notes_text = manifest["operator_notes"]
        notes = json.loads(notes_text)
        for name in ("force", "lpt"):
            notes["sync_capture"]["recording_hashes"][name] = sha(source / f"streams/{name}.csv")
        manifest["operator_notes"] = json.dumps(notes, sort_keys=True)
        dump(manifest_path, manifest)
        (source / "checksums.sha256").write_text("".join(f"{sha(p)}  {p.relative_to(source).as_posix()}\n"
            for p in sorted(source.rglob("*")) if p.is_file() and p.name != "checksums.sha256"))
        # Python's json reparses its own Infinity output, as the audit witness did.
        authority_path.write_text(json.dumps({**json.loads(manifest["operator_notes"])["sync_capture"],
            "source_manifest_sha256": sha(manifest_path)}, indent=2) + "\n")
        if bind_truth:
            reference = json.loads((truth / "reference.json").read_text())
            reference["source_manifest_sha256"] = sha(manifest_path)
            dump(truth / "reference.json", reference)
        self.reseal_truth(root, case_id)

    def reseal_truth(self, root, case_id):
        source, truth, _ = self.paths(root, case_id)
        (truth / "checksums.sha256").write_text("".join(f"{sha(p)}  {p.name}\n"
            for p in sorted(truth.iterdir()) if p.name != "checksums.sha256"))
        run = json.loads((root / "run.json").read_text())
        for case in run["cases"]:
            if case["case_id"] == case_id:
                case.update(source_manifest_sha256=sha(source / "manifest.json"),
                            truth_inventory_sha256=sha(truth / "checksums.sha256"))
        dump(root / "run.json", run)

    def cli(self, root):
        out = root / f"verification-{self.serial}.json"
        result = subprocess.run([sys.executable, "-m", "scripts.verify_paired_corpus", "--root", str(root),
                                 "--out", str(out)], cwd=ROOT, capture_output=True, text=True)
        self.assertNotIn("Traceback", result.stderr)
        return result.returncode, json.loads(out.read_text())

    def assert_denied(self, root, case_id, code, *, source_code=None):
        source, truth, _ = self.paths(root, case_id)
        with self.assertRaisesRegex(ReferenceRefused, f"^{code}$"):
            verify_reference(source, truth)
        if source_code:
            with self.assertRaisesRegex(SourceRefused, f"^{source_code}$"):
                verify_source(source, self.authority(root, case_id))
        exit_code, report = self.cli(root)
        self.assertEqual(exit_code, 1)
        self.assertEqual(report["model_reference_qualified_count"], 0)
        self.assertEqual(report["cases"][0]["evidence_refusal"], source_code or code)
        return report

    # ----- legitimate positive controls -----
    def test_positive_controls_guide_cmj_squat_deadlift_and_legacy_500hz(self):
        root = self.bundle("guide", "cmj", "squat", "deadlift", "legacy_guide", "legacy_cmj")
        for case_id in ("guide", "cmj", "squat", "deadlift", "legacy_guide", "legacy_cmj"):
            source, truth, _ = self.paths(root, case_id)
            with self.subTest(case=case_id):
                checked = verify_source(source, self.authority(root, case_id))
                self.assertEqual((checked["source"], checked["synchronization"]), ("PASS", "BOUNDED_CAPTURE_MAPPING"))
                self.assertEqual(checked["nominal_rates_hz"]["lpt"], 500 if case_id.startswith("legacy") else 1000)
                result = verify_reference(source, truth)
                self.assertTrue(result["passed"], result)
                self.assertEqual(result["model_reference_status"], "MODEL_REFERENCE_QUALIFIED")
        guide = verify_reference(*self.paths(root, "guide")[:2])
        self.assertEqual(guide["independent_reference_events"]["scope"], "ANALYTICAL_GUIDE_EVENTS")
        self.assertEqual(set(guide["required_mechanics_outcomes"]), {"LINEAR_MOMENTUM_CLOSURE", "UNILATERAL_SUPPORT_SIGN"})
        self.assertEqual(guide["geometry"]["status"], "NOT_APPLICABLE_CONSTRAINED_VERTICAL_GUIDE")
        cmj = verify_reference(*self.paths(root, "cmj")[:2])
        self.assertEqual(set(cmj["required_mechanics_outcomes"]), {"LINEAR_MOMENTUM_CLOSURE", "CONTACT_ADMISSIBILITY"})
        self.assertEqual(cmj["geometry"]["status"], "PASS")
        exit_code, report = self.cli(root)
        self.assertEqual(exit_code, 0)
        self.assertEqual((report["model_reference_qualified_count"], report["full_case_qualified_count"],
                          report["evidence_refused_count"]), (6, 6, 0))

    # ----- F01 missing / malformed / resealed event evidence -----
    def test_f01_missing_model_events_coherently_resealed_refuses(self):
        for case_id in ("cmj", "squat"):
            with self.subTest(case=case_id):
                root = self.bundle(case_id)
                (self.paths(root, case_id)[1] / "model_events.json").unlink()
                self.reseal_truth(root, case_id)
                report = self.assert_denied(root, case_id, "TRUTH_INVENTORY_INCOMPLETE")
                self.assertEqual(report["missing_evidence_count"], 1)

    def test_f01_empty_malformed_and_incomplete_event_claims_refuse(self):
        for case_id, mutate, code in (
                ("cmj", lambda d: {}, "MODEL_EVENTS_MALFORMED"),
                ("cmj", lambda d: {**d, "analysis": {k: v for k, v in d["analysis"].items() if k != "events"}},
                 "MODEL_EVENTS_INCOMPLETE"),
                ("squat", lambda d: {**d, "analysis": {**d["analysis"], "reps": []}}, "MODEL_EVENTS_INCOMPLETE"),
                ("deadlift", lambda d: {**d, "analysis": {k: v for k, v in d["analysis"].items()
                                                           if k != "lift_off_s"}}, "MODEL_EVENTS_INCOMPLETE")):
            with self.subTest(case=case_id, code=code):
                root = self.bundle(case_id)
                path = self.paths(root, case_id)[1] / "model_events.json"
                dump(path, mutate(json.loads(path.read_text())))
                self.reseal_truth(root, case_id)
                self.assert_denied(root, case_id, code)
        root = self.bundle("cmj")
        (self.paths(root, "cmj")[1] / "model_events.json").write_text('{"analysis": NaN}')
        self.reseal_truth(root, "cmj")
        self.assert_denied(root, "cmj", "TRUTH_NONFINITE_JSON_NUMBER")

    def test_f01_guide_exception_is_exact_and_scoped(self):
        root = self.bundle("guide")
        truth = self.paths(root, "guide")[1]
        shutil.copyfile(self.corpus / "TRUTH/cmj/model_events.json", truth / "model_events.json")
        self.reseal_truth(root, "guide")
        self.assert_denied(root, "guide", "TRUTH_INVENTORY_UNEXPECTED")
        root = self.bundle("guide")
        truth = self.paths(root, "guide")[1]
        reference = json.loads((truth / "reference.json").read_text())
        del reference["events"]
        dump(truth / "reference.json", reference)
        self.reseal_truth(root, "guide")
        self.assert_denied(root, "guide", "GUIDE_EVENTS_INCOMPLETE")
        root = self.bundle("guide")
        run = json.loads((root / "run.json").read_text())
        run["cases"][0]["protocol"] = "cmj"
        dump(root / "run.json", run)
        exit_code, report = self.cli(root)
        self.assertEqual((exit_code, report["cases"][0]["evidence_refusal"]), (1, "TRUTH_PROTOCOL_MISMATCH"))

    # ----- F02 taut geometry must be evaluated -----
    def test_f02_missing_taut_derivation_refuses_in_source_and_reference(self):
        root = self.bundle("cmj")
        source = self.paths(root, "cmj")[0]
        (source / "derived/vertical.csv").unlink()
        self.edit_manifest(root, "cmj", lambda m, n: m["derived_observation"].update(path=None, refusal=None))
        self.reseal(root, "cmj")
        report = self.assert_denied(root, "cmj", "TAUT_DERIVED_OBSERVATION_MISSING",
                                    source_code="TAUT_DERIVED_OBSERVATION_MISSING")
        self.assertEqual(report["missing_evidence_count"], 1)

    def test_f02_empty_derivation_and_undeclared_sealed_file_refuse(self):
        root = self.bundle("squat")
        path = self.paths(root, "squat")[0] / "derived/vertical.csv"
        path.write_text(path.read_text().splitlines()[0] + "\n")
        self.reseal(root, "squat")
        self.assert_denied(root, "squat", "DERIVED_SAMPLES_INSUFFICIENT", source_code="INSUFFICIENT_SAMPLES")
        root = self.bundle("squat")
        self.edit_manifest(root, "squat", lambda m, n: m["derived_observation"].update(path=None))
        self.reseal(root, "squat")
        with self.assertRaisesRegex(SourceRefused, "^SOURCE_ROLE_INVENTORY_MISMATCH$"):
            verify_source(self.paths(root, "squat")[0], self.authority(root, "squat"))
        root = self.bundle("squat")
        self.edit_manifest(root, "squat", lambda m, n: m["derived_observation"].update(
            source_processing_state="DERIVED_GEOMETRY_RECONSTRUCTION_V0"))
        self.reseal(root, "squat")
        with self.assertRaisesRegex(SourceRefused, "^DERIVED_STAGE_INVALID$"):
            verify_source(self.paths(root, "squat")[0], self.authority(root, "squat"))

    def make_slack(self, root, case_id, refusal):
        source = self.paths(root, case_id)[0]
        lines = (source / "streams/angle.csv").read_text().splitlines()
        fields = lines[10].split(",")
        fields[-1] = "1"
        lines[10] = ",".join(fields)
        (source / "streams/angle.csv").write_text("\n".join(lines) + "\n")
        (source / "derived/vertical.csv").unlink()
        self.edit_manifest(root, case_id, lambda m, n: m["derived_observation"].update(path=None, refusal=refusal))
        self.reseal(root, case_id)

    def test_f02_legitimate_slack_refusal_is_typed_not_evaluable_never_full_case(self):
        root = self.bundle("cmj")
        self.make_slack(root, "cmj", "SLACK_CABLE_VERTICAL_RECONSTRUCTION_REFUSED")
        source, truth, _ = self.paths(root, "cmj")
        self.assertEqual(verify_source(source, self.authority(root, "cmj"))["source"], "PASS")
        result = verify_reference(source, truth)
        self.assertEqual(result["geometry"], {"status": "NOT_EVALUABLE_SLACK_CABLE_REFUSED",
                                              "refusal": "SLACK_CABLE_VERTICAL_RECONSTRUCTION_REFUSED"})
        self.assertNotIn("samples", result["geometry"])
        exit_code, report = self.cli(root)
        self.assertEqual(exit_code, 0)
        self.assertEqual((report["model_reference_qualified_count"], report["full_case_qualified_count"],
                          report["not_evaluable_count"], report["expected_refusal_count"]), (1, 0, 1, 1))
        root = self.bundle("cmj")
        self.make_slack(root, "cmj", None)
        self.assert_denied(root, "cmj", "TAUT_DERIVED_OBSERVATION_MISSING", source_code="SLACK_DERIVED_FALSE_ACCEPT")

    # ----- F03 non-guide mechanics are mandatory; no endpoint fallback -----
    def test_f03_original_balanced_tensile_witness_without_mechanics_refuses(self):
        root = self.bundle("cmj")
        truth = self.paths(root, "cmj")[1]
        (truth / "mechanics_submission.json").unlink()
        reference = json.loads((truth / "reference.json").read_text())
        with (truth / "support_intervals.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        dt = lambda r: float(r["end_s"]) - float(r["start_s"])
        target = (reference["mass_kg"] * reference["gravity_m_s2"] * reference["physics_times_s"][-1] +
                  reference["vertical_momentum_kg_m_s"][-1] - reference["vertical_momentum_kg_m_s"][0])
        rows[-1]["fz_N"] = str(float(rows[-1]["fz_N"]) + (target - sum(float(r["fz_N"]) * dt(r) for r in rows)) / dt(rows[-1]))
        rows[0]["fz_N"] = str(float(rows[0]["fz_N"]) - 100000)
        rows[1]["fz_N"] = str(float(rows[1]["fz_N"]) + 100000 * dt(rows[0]) / dt(rows[1]))
        self.assertLess(min(float(r["fz_N"]) for r in rows), -99000)
        with (truth / "support_intervals.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        self.reseal_truth(root, "cmj")
        self.assert_denied(root, "cmj", "TRUTH_INVENTORY_INCOMPLETE")

    def test_f03_tampered_empty_and_tensile_mechanics_refuse(self):
        root = self.bundle("cmj")
        path = self.paths(root, "cmj")[1] / "mechanics_submission.json"
        path.write_text(path.read_text().replace('"fz"', '"fz" ', 1))
        self.assert_denied(root, "cmj", "TRUTH_CHECKSUM_MISMATCH")
        root = self.bundle("cmj")
        path = self.paths(root, "cmj")[1] / "mechanics_submission.json"
        dump(path, {})
        self.reseal_truth(root, "cmj")
        self.assert_denied(root, "cmj", "MECHANICS_SCHEMA_INVALID")
        root = self.bundle("squat")
        path = self.paths(root, "squat")[1] / "mechanics_submission.json"
        submission = json.loads(path.read_text())
        frame = next(f for f in submission["frames"] if f["t1"] > f["t0"])
        first, second = [c for c in frame["contacts"] if c["group"] in submission["external_support_groups"]][:2]
        first["fz"] -= 100000.0
        second["fz"] += 100000.0  # Net vertical force preserved; one normal becomes tensile.
        dump(path, submission)
        self.reseal_truth(root, "squat")
        result = verify_reference(*self.paths(root, "squat")[:2])
        self.assertEqual(result["required_mechanics_outcomes"],
                         {"LINEAR_MOMENTUM_CLOSURE": "PASS", "CONTACT_ADMISSIBILITY": "REJECTED"})
        self.assertFalse(result["passed"])
        exit_code, report = self.cli(root)
        self.assertEqual((exit_code, report["model_reference_qualified_count"]), (1, 0))

    def test_f03_required_findings_cannot_be_omitted(self):
        root = self.bundle("cmj")
        real = verifier.evaluate_i2
        def without_contact(submission):
            result = real(submission)
            result["findings"].pop("CONTACT_ADMISSIBILITY")
            return result
        with patch.object(verifier, "evaluate_i2", without_contact), \
                self.assertRaisesRegex(ReferenceRefused, "^REQUIRED_MECHANICS_FINDING_MISSING$"):
            verify_reference(*self.paths(root, "cmj")[:2])

    def test_f03_guide_fallback_stays_scoped_and_checks_support_sign(self):
        root = self.bundle("guide")
        truth = self.paths(root, "guide")[1]
        with (truth / "support_intervals.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        rows[0]["fz_N"] = str(float(rows[0]["fz_N"]) - 100000)
        rows[2]["fz_N"] = str(float(rows[2]["fz_N"]) + 100000)  # Same interval duration; impulse kept.
        with (truth / "support_intervals.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        self.reseal_truth(root, "guide")
        result = verify_reference(*self.paths(root, "guide")[:2])
        self.assertEqual(result["required_mechanics_outcomes"],
                         {"LINEAR_MOMENTUM_CLOSURE": "PASS", "UNILATERAL_SUPPORT_SIGN": "REJECTED"})
        self.assertFalse(result["passed"])

    # ----- F04 finite, versioned capture authority -----
    def capture_denied(self, function, code):
        root = self.bundle("cmj")
        self.edit_manifest(root, "cmj", function)
        self.reseal(root, "cmj")
        source = self.paths(root, "cmj")[0]
        with self.assertRaisesRegex(SourceRefused, f"^{code}$"):
            verify_source(source, self.authority(root, "cmj"))
        exit_code, report = self.cli(root)
        self.assertEqual((exit_code, report["cases"][0]["evidence_refusal"]), (1, code))

    def test_f04_original_infinite_bound_and_shifted_edge_refuses(self):
        def attack(m, n):
            n["sync_capture"]["capture_error_bound_s"] = float("inf")
            n["sync_capture"]["devices"]["lpt"]["edge_native_times_s"][1] += 0.1
        self.capture_denied(attack, "NONFINITE_JSON_NUMBER")

    def test_f04_bound_outside_versioned_envelope_or_malformed_refuses(self):
        for value in (1.0, 2.1e-6, 0, -1e-6, "2e-6", True, None):
            def attack(m, n, value=value):
                n["sync_capture"]["capture_error_bound_s"] = value
                n["sync_capture"]["devices"]["lpt"]["edge_native_times_s"][1] += 0.1
            with self.subTest(bound=value):
                self.capture_denied(attack, "CAPTURE_UNCERTAINTY_INVALID")

    def test_f04_edges_roles_contract_and_jitter_policy_are_validated(self):
        def shuffle(m, n):
            edges = n["sync_capture"]["devices"]["force"]["edge_native_times_s"]
            edges[1], edges[2] = edges[2], edges[1]
        for function, code in (
                (shuffle, "CLOCK_CAPTURE_INVALID"),
                (lambda m, n: n["sync_capture"]["devices"]["lpt"]["edge_native_times_s"].append(9.0),
                 "CLOCK_CAPTURE_INVALID"),
                (lambda m, n: n["sync_capture"]["reference_edge_times_s"].pop(1), "CAPTURE_ANCHORS_INSUFFICIENT"),
                (lambda m, n: n["sync_capture"].update(contract_id="BSMB-FOUR-EDGE-CAPTURE-9.9"),
                 "CAPTURE_CONTRACT_UNSUPPORTED"),
                (lambda m, n: n["sync_capture"].update(fit_edge_indices=[0, 1]), "CAPTURE_EDGE_ROLES_UNSUPPORTED"),
                (lambda m, n: n["acquisition_jitter_policy"].update(finite_error_bound_s=-1.0), "JITTER_POLICY_INVALID")):
            with self.subTest(code=code):
                self.capture_denied(function, code)

    def test_f04_stricter_finite_bound_remains_admissible(self):
        root = self.bundle("cmj")
        self.edit_manifest(root, "cmj", lambda m, n: n["sync_capture"].update(capture_error_bound_s=1.9e-6))
        self.reseal(root, "cmj")
        self.assertEqual(verify_source(self.paths(root, "cmj")[0], self.authority(root, "cmj"))["source"], "PASS")

    # ----- F05 TRUTH <-> SOURCE binding -----
    def test_f05_original_95kg_source_with_120kg_truth_refuses(self):
        root = self.bundle("guide")
        truth = self.paths(root, "guide")[1]
        shutil.rmtree(truth)
        shutil.copytree(self.corpus / "TRUTH/heavy_guide", truth)
        self.assertEqual(json.loads((truth / "reference.json").read_text())["mass_kg"], 120)
        self.reseal_truth(root, "guide")
        self.assertEqual(verify_source(self.paths(root, "guide")[0], self.authority(root, "guide"))["source"], "PASS")
        self.assert_denied(root, "guide", "TRUTH_SOURCE_BINDING_MISMATCH")
        matching = self.bundle("heavy_guide")
        self.assertTrue(verify_reference(*self.paths(matching, "heavy_guide")[:2])["passed"])

    def test_f05_identity_schema_and_stale_run_receipt_refuse(self):
        root = self.bundle("guide")
        source, truth, _ = self.paths(root, "guide")
        renamed = root / "TRUTH" / "other-trial"
        shutil.copytree(truth, renamed)
        with self.assertRaisesRegex(ReferenceRefused, "^TRUTH_SOURCE_IDENTITY_MISMATCH$"):
            verify_reference(source, renamed)
        reference = json.loads((truth / "reference.json").read_text())
        reference["reference_schema_id"] = "BSMB-PAIRED-TRUTH-0.9"
        dump(truth / "reference.json", reference)
        self.reseal_truth(root, "guide")
        self.assert_denied(root, "guide", "TRUTH_SCHEMA_UNSUPPORTED")
        root = self.bundle("guide")
        run = json.loads((root / "run.json").read_text())
        run["cases"][0]["truth_inventory_sha256"] = self.receipts["heavy_guide"]["truth_inventory_sha256"]
        dump(root / "run.json", run)
        exit_code, report = self.cli(root)
        self.assertEqual((exit_code, report["cases"][0]["evidence_refusal"]), (1, "RUN_RECEIPT_BINDING_MISMATCH"))

    def test_f05_unsafe_or_extra_truth_entries_refuse(self):
        root = self.bundle("guide")
        truth = self.paths(root, "guide")[1]
        with (truth / "checksums.sha256").open("a") as stream:
            stream.write(f"{'0' * 64}  ../SOURCE/guide/manifest.json\n")
        with self.assertRaisesRegex(ReferenceRefused, "^UNSAFE_TRUTH_PATH$"):
            verify_reference(*self.paths(root, "guide")[:2])
        root = self.bundle("guide")
        truth = self.paths(root, "guide")[1]
        (truth / "extra.json").write_text("{}")
        with self.assertRaisesRegex(ReferenceRefused, "^TRUTH_INVENTORY_UNEXPECTED$"):
            verify_reference(*self.paths(root, "guide")[:2])

    # ----- F08 manifest-controlled paths are untrusted -----
    def test_f08_derived_path_variants_refuse_before_filesystem_use(self):
        for path in ("../../TRUTH/cmj/audit-unsealed-vertical.csv", "/etc/passwd", "C:/Windows/win.ini",
                     "\\\\server\\share\\vertical.csv", "derived\\vertical.csv", "derived/../derived/vertical.csv",
                     "./derived/vertical.csv", "derived/other.csv", "", 7):
            with self.subTest(path=path):
                root = self.bundle("cmj")
                self.edit_manifest(root, "cmj", lambda m, n: m["derived_observation"].update(path=path))
                self.reseal(root, "cmj")
                self.assert_denied(root, "cmj", "DERIVED_PATH_UNSUPPORTED", source_code="DERIVED_PATH_UNSUPPORTED")

    def test_f08_original_truth_sentinel_attack_makes_no_read_outside_source(self):
        root = self.bundle("cmj")
        source, truth, _ = self.paths(root, "cmj")
        shutil.copyfile(source / "derived/vertical.csv", truth / "audit-unsealed-vertical.csv")
        self.edit_manifest(root, "cmj", lambda m, n: m["derived_observation"].update(
            path="../../TRUTH/cmj/audit-unsealed-vertical.csv"))
        self.reseal(root, "cmj", bind_truth=False)
        program = textwrap.dedent("""
            import json, pathlib, sys
            from biosyngen_benchmark.paired_source import SourceRefused, verify_source
            source, authority = pathlib.Path(sys.argv[1]).resolve(), json.loads(pathlib.Path(sys.argv[2]).read_text())
            outside, busy = [], []
            def hook(event, args):
                # Record every attempted data read or listing outside SOURCE;
                # only Python's own code/extension imports are exempt.
                if busy or event not in ("open", "os.listdir", "os.scandir") or not args:
                    return
                if not isinstance(args[0], (str, bytes, pathlib.PurePath)):
                    return
                busy.append(1)
                try:
                    raw = args[0].decode() if isinstance(args[0], bytes) else args[0]
                    path = pathlib.Path(raw).resolve()
                    if not path.is_relative_to(source) and path.suffix not in (".py", ".pyc", ".pyd", ".so", ".dll"):
                        outside.append(str(path))
                finally:
                    busy.clear()
            sys.addaudithook(hook)
            try:
                verify_source(source, authority)
                code = "PASS"
            except SourceRefused as exc:
                code = str(exc)
            print(json.dumps({"code": code, "outside_reads": outside}))
        """)
        result = subprocess.run([sys.executable, "-c", program, str(source), str(self.paths(root, "cmj")[2])],
                                cwd=ROOT, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        record = json.loads(result.stdout)
        self.assertEqual(record, {"code": "DERIVED_PATH_UNSUPPORTED", "outside_reads": []})

    def test_f08_unsafe_checksum_entries_and_case_ids_refuse(self):
        for name in ("../../TRUTH/cmj/reference.json", "derived\\vertical.csv", "C:/x", "./streams/lpt.csv"):
            with self.subTest(name=name):
                root = self.bundle("cmj")
                source = self.paths(root, "cmj")[0]
                with (source / "checksums.sha256").open("a") as stream:
                    stream.write(f"{'0' * 64}  {name}\n")
                with self.assertRaisesRegex(SourceRefused, "^UNSAFE_CHECKSUM_PATH$"):
                    verify_source(source, self.authority(root, "cmj"))
        root = self.bundle("cmj")
        run = json.loads((root / "run.json").read_text())
        run["cases"][0]["case_id"] = "../SOURCE/cmj"
        dump(root / "run.json", run)
        result = subprocess.run([sys.executable, "-m", "scripts.verify_paired_corpus", "--root", str(root),
                                 "--out", str(root / "out.json")], cwd=ROOT, capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("RUN_RECEIPT_CASE_INVENTORY_INVALID", result.stderr)
        self.assertFalse((root / "out.json").exists())

    def test_f08_links_inside_source_refuse(self):
        root = self.bundle("cmj")
        source, truth, _ = self.paths(root, "cmj")
        target = truth / "audit-unsealed-vertical.csv"
        shutil.copyfile(source / "derived/vertical.csv", target)
        (source / "derived/vertical.csv").unlink()
        try:
            os.symlink(target, source / "derived/vertical.csv")
        except (OSError, NotImplementedError):
            linked = False
        else:
            linked = True
            with self.assertRaisesRegex(SourceRefused, "^SOURCE_LINK_REFUSED$"):
                verify_source(source, self.authority(root, "cmj"))
        if sys.platform == "win32":
            import _winapi
            root = self.bundle("cmj")
            source = self.paths(root, "cmj")[0]
            outside = self.work / "junction-target"
            shutil.copytree(source / "derived", outside)
            shutil.rmtree(source / "derived")
            _winapi.CreateJunction(str(outside), str(source / "derived"))
            linked = True
            with self.assertRaisesRegex(SourceRefused, "^SOURCE_LINK_REFUSED$"):
                verify_source(source, self.authority(root, "cmj"))
        if not linked:
            self.skipTest("platform does not permit creating a symbolic link")

    # ----- F06 separate qualification denominators -----
    def test_f06_report_separates_model_reference_sync_geometry_and_full_case(self):
        root = self.bundle("guide", "jitter", "cmj")
        exit_code, report = self.cli(root)
        self.assertEqual(exit_code, 0)
        self.assertEqual(report["report_schema_id"], "BSMB-PAIRED-QUALIFICATION-REPORT-2.0")
        self.assertNotIn("qualified_count", report)
        self.assertEqual({k: report[k] for k in ("requested_count", "source_integrity_pass_count",
            "model_reference_qualified_count", "bounded_synchronization_count", "synchronization_refused_count",
            "geometry_pass_count", "geometry_not_applicable_guide_count", "full_case_qualified_count",
            "expected_refusal_count", "evidence_refused_count", "missing_evidence_count", "not_evaluable_count")},
            {"requested_count": 3, "source_integrity_pass_count": 3, "model_reference_qualified_count": 3,
             "bounded_synchronization_count": 2, "synchronization_refused_count": 1, "geometry_pass_count": 1,
             "geometry_not_applicable_guide_count": 2, "full_case_qualified_count": 2, "expected_refusal_count": 1,
             "evidence_refused_count": 0, "missing_evidence_count": 0, "not_evaluable_count": 0})
        jitter = next(c for c in report["cases"] if c["case_id"] == "jitter")
        self.assertEqual((jitter["model_reference_status"], jitter["full_case_status"], jitter["expected_refusals"]),
                         ("MODEL_REFERENCE_QUALIFIED", "NOT_FULL_CASE_QUALIFIED", ["REFUSED_UNBOUNDED_SAMPLE_JITTER"]))

    def test_f06_comparator_refuses_old_receipts_and_denominator_drift(self):
        root = self.bundle("guide")
        _, report = self.cli(root)
        case = report["cases"][0]
        fixture = {**report, "cases": [dict(copy.deepcopy(case), case_id=f"c{i}") for i in range(16)]}
        for key in ("requested_count", "model_reference_qualified_count", "source_integrity_pass_count",
                    "bounded_synchronization_count", "full_case_qualified_count", "geometry_not_applicable_guide_count"):
            fixture[key] = 16
        self.assertEqual(compare(fixture, fixture)["qualification_denominators"]["full_case_qualified_count"], 16)
        legacy = {k: v for k, v in fixture.items() if k != "report_schema_id"}
        with self.assertRaisesRegex(ValueError, "PLATFORM_REPORT_SCHEMA_UNSUPPORTED"):
            compare(legacy, legacy)
        drift = copy.deepcopy(fixture)
        drift["full_case_qualified_count"] = 15
        with self.assertRaisesRegex(ValueError, "PLATFORM_QUALIFICATION_DENOMINATOR_MISMATCH"):
            compare(fixture, drift)
        status = copy.deepcopy(fixture)
        status["cases"][3]["full_case_status"] = "NOT_FULL_CASE_QUALIFIED"
        with self.assertRaisesRegex(ValueError, "PLATFORM_CASE_STATUS_MISMATCH:full_case_status"):
            compare(fixture, status)

    # ----- F07 partial writes are typed refusals and never qualify -----
    def test_f07_injected_write_failure_is_receipted_and_never_qualified(self):
        import biosyngen.i2.paired_export as export
        import scripts.generate_paired_corpus as generator
        destination = self.work / "write-failure"
        real = export.write_csv
        def failing(path, header, rows):
            if str(path).replace("\\", "/").endswith("streams/lpt.csv"):
                raise OSError("INJECTED_DISK_WRITE_FAILURE")
            return real(path, header, rows)
        with patch.object(export, "write_csv", failing), patch("builtins.print"):
            code = generator.main(["--protocol", "guided_vertical", "--out", str(destination)])
        self.assertEqual(code, 1)
        run = json.loads((destination / "run.json").read_text())
        case = run["cases"][0]
        self.assertEqual(case["status"], "REFUSED")
        self.assertTrue(case["reason"].startswith("OS_WRITE_FAILURE:OSError"))
        self.assertIn("SOURCE/trial-00000524-0000/streams/force.csv", case["partial_artifacts"])
        self.assertTrue((destination / "SOURCE/trial-00000524-0000/streams/force.csv").is_file())
        exit_code, report = self.cli(destination)
        self.assertEqual((exit_code, report["model_reference_qualified_count"], report["generation_refusal_count"],
                          report["expected_refusal_count"]), (1, 0, 1, 0))


if __name__ == "__main__":
    unittest.main()
