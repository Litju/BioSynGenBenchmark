"""RES-535 regressions for the independent re-audit of PR #4 (4fb0a75).

Contact/impulse application points and declared normals bound to reference
geometry, closed claim-scope authorities in model events and context, a
structurally admitted simulator event log, and bounded capture mappings
checked against protected TRUTH sample times. Every adversary is a disposable
manufactured copy that a test-only operator coherently re-endorses, so each
refusal comes from semantic admission rather than a stale checksum.
"""
import copy
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

from biosyngen.i2.paired_export import export_case, sha
from scripts.verify_paired_corpus import ReferenceRefused, verify_reference

ROOT = Path(__file__).resolve().parents[1]
CASES = {"guide": ("guided_vertical", {}), "jitter": ("guided_vertical", {"jitter_sd_s": 5e-6}),
         "overlap_guide": ("guided_vertical", {"force_drift_ppm": 300, "lpt_drift_ppm": -200, "lpt_start_s": 0.5,
                                               "force_offset_s": 0.3, "lpt_offset_s": -0.2, "lpt_drop_indices": [800]}),
         "cmj": ("cmj", {}), "deadlift": ("deadlift", {})}


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


class Res535Admission(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(dir=ROOT / "runs")
        cls.corpus = Path(cls.tmp.name) / "corpus"
        cls.receipts = {}
        for seed, (case_id, (protocol, parameters)) in enumerate(CASES.items(), 53500):
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

    def truth(self, root, case_id):
        return root / "TRUTH" / case_id

    def load(self, root, case_id, name):
        return json.loads((self.truth(root, case_id) / name).read_text())

    def store(self, root, case_id, name, value):
        dump(self.truth(root, case_id) / name, value)

    def reseal_truth(self, root, case_id):
        source, truth = root / "SOURCE" / case_id, self.truth(root, case_id)
        (truth / "checksums.sha256").write_text("".join(f"{sha(p)}  {p.name}\n"
            for p in sorted(truth.iterdir()) if p.name != "checksums.sha256"))
        run = json.loads((root / "run.json").read_text())
        for case in run["cases"]:
            if case["case_id"] == case_id:
                case.update(source_manifest_sha256=sha(source / "manifest.json"),
                            truth_inventory_sha256=sha(truth / "checksums.sha256"))
        dump(root / "run.json", run)

    def reseal_source(self, root, case_id, edit):
        """Edit manifest and notes, then re-endorse SOURCE, authority and TRUTH binding."""
        source, truth = root / "SOURCE" / case_id, self.truth(root, case_id)
        path = source / "manifest.json"
        manifest = json.loads(path.read_text())
        notes = json.loads(manifest["operator_notes"])
        edit(manifest, notes)
        manifest["operator_notes"] = json.dumps(notes, sort_keys=True)
        dump(path, manifest)
        (source / "checksums.sha256").write_text("".join(f"{sha(p)}  {p.relative_to(source).as_posix()}\n"
            for p in sorted(source.rglob("*")) if p.is_file() and p.name != "checksums.sha256"))
        dump(root / "DIAGNOSTICS" / case_id / "capture_authority.json",
             {**notes["sync_capture"], "source_manifest_sha256": sha(path)})
        reference = json.loads((truth / "reference.json").read_text())
        reference["source_manifest_sha256"] = sha(path)
        dump(truth / "reference.json", reference)
        self.reseal_truth(root, case_id)

    def mutate(self, case_id, edit):
        root = self.bundle(case_id)
        edit(root)
        self.reseal_truth(root, case_id)
        return root

    def cli(self, root):
        out = root.parent / f"verification-{root.name}.json"
        result = subprocess.run([sys.executable, "-m", "scripts.verify_paired_corpus", "--root", str(root),
                                 "--out", str(out)], cwd=ROOT, capture_output=True, text=True)
        self.assertNotIn("Traceback", result.stderr)
        return result.returncode, json.loads(out.read_text())

    def assert_cli_denied(self, root, code):
        exit_code, report = self.cli(root)
        self.assertEqual((exit_code, report["model_reference_qualified_count"], report["full_case_qualified_count"],
                          report["cases"][0].get("evidence_refusal")), (1, 0, 0, code))

    def assert_denied(self, root, case_id, code):
        with self.assertRaisesRegex(ReferenceRefused, f"^{code}$"):
            verify_reference(root / "SOURCE" / case_id, self.truth(root, case_id))
        self.assert_cli_denied(root, code)

    def first_contact(self, case_id, kind):
        """First submission contact on a declared spec kind, with its frame index."""
        ref = json.loads((self.corpus / "TRUTH" / case_id / "reference.json").read_text())
        names = {s["contact_id"] for s in ref["contact_specs"] if s["kind"] in kind}
        sub = json.loads((self.corpus / "TRUTH" / case_id / "mechanics_submission.json").read_text())
        return next((i, c["name"]) for i, f in enumerate(sub["frames"]) for c in f["contacts"] if c["name"] in names)

    # ----- positive controls -----
    def test_positive_controls_qualify_with_versioned_bindings(self):
        cases = ("guide", "overlap_guide", "cmj", "deadlift")
        root = self.bundle(*cases)
        for case_id in ("cmj", "deadlift"):
            with self.subTest(case=case_id):
                result = verify_reference(root / "SOURCE" / case_id, self.truth(root, case_id))
                self.assertTrue(result["passed"], result)
                binding = result["mechanics_reference_binding"]
                self.assertEqual((binding["contract"], binding["status"], binding["contact_geometry"]),
                                 ("BSMB-PAIRED-MECHANICS-BINDING-1.1", "BOUND_TO_REFERENCE_TRIAL",
                                  "APPLICATION_POINTS_AND_NORMAL_SIGN_FROM_REFERENCE_STATES_AND_SPECS"))
                self.assertEqual(binding["per_body_inertia"], "NOT_INDEPENDENTLY_BOUND_NO_TRUTH_AUTHORITY")
                self.assertEqual(result["truth_admission_contract"], "BSMB-PAIRED-TRUTH-ADMISSION-1.1")
                self.assertEqual(result["physical_event_log"], "STRUCTURE_ADMITTED_ARCHIVAL_SIMULATOR_LOG_NOT_RECOMPUTED")
        exit_code, report = self.cli(root)
        self.assertEqual((exit_code, report["full_case_qualified_count"]), (0, len(cases)))
        for case in report["cases"]:
            check = case["synchronization_truth_check"]
            self.assertEqual((check["contract"], check["status"]), ("BSMB-SYNC-TRUTH-BINDING-1.0",
                             "BOUNDED_MAPPING_CONSISTENT_WITH_TRUTH_SAMPLE_TIMES"))
            for stream in check["streams"].values():
                self.assertLessEqual(stream["max_mapped_minus_true_s"], stream["admissible_s"])

    def test_jitter_case_keeps_typed_refusal_without_truth_promotion(self):
        exit_code, report = self.cli(self.bundle("jitter"))
        case = report["cases"][0]
        self.assertEqual((exit_code, case["synchronization"], case["full_case_status"],
                          case["synchronization_truth_check"]["status"]),
                         (0, "REFUSED_UNBOUNDED_SAMPLE_JITTER", "NOT_FULL_CASE_QUALIFIED",
                          "NOT_APPLICABLE_SYNCHRONIZATION_REFUSED"))

    # ----- contact and impulse application points -----
    def test_contact_point_mutations_refuse(self):
        for case_id, kind, dx, dz in (("cmj", ("HEEL", "TOE"), 0.75, 0.0), ("cmj", ("HEEL", "TOE"), 0.0, 1.5),
                                      ("deadlift", ("BAR_FLOOR",), 0.0, 0.3), ("deadlift", ("TOE",), 1e-6, 0.0)):
            index, name = self.first_contact(case_id, kind)

            def edit(root, dx=dx, dz=dz, name=name):
                sub = self.load(root, case_id, "mechanics_submission.json")
                for frame in sub["frames"]:
                    for c in frame["contacts"]:
                        if c["name"] == name:
                            c["x"] += dx
                            c["z"] += dz
                self.store(root, case_id, "mechanics_submission.json", sub)
            with self.subTest(case=case_id, contact=name, dx=dx, dz=dz):
                self.assert_denied(self.mutate(case_id, edit), case_id, "MECHANICS_CONTACT_POINT_MISMATCH")

        def single(root):
            sub = self.load(root, "cmj", "mechanics_submission.json")
            next(f for f in sub["frames"] if f["contacts"])["contacts"][0]["x"] = 1e3
            self.store(root, "cmj", "mechanics_submission.json", sub)
        self.assert_denied(self.mutate("cmj", single), "cmj", "MECHANICS_CONTACT_POINT_MISMATCH")

    def test_impulse_point_and_time_mutations_refuse(self):
        for case_id in ("cmj", "deadlift"):
            for field, delta in (("point", 2.0), ("time", 1e-4)):
                def edit(root, field=field, delta=delta, case_id=case_id):
                    ref = self.load(root, case_id, "reference.json")
                    sub = self.load(root, case_id, "mechanics_submission.json")
                    if field == "point":
                        ref["impulses_N_s"][-1]["point_world_m"][0] += delta
                        sub["impulses"][-1]["x"] += delta
                    else:
                        ref["impulses_N_s"][-1]["time_s"] += delta
                        sub["impulses"][-1]["t"] += delta
                    self.store(root, case_id, "reference.json", ref)
                    self.store(root, case_id, "mechanics_submission.json", sub)
                with self.subTest(case=case_id, field=field):
                    self.assert_denied(self.mutate(case_id, edit), case_id, "MECHANICS_IMPULSE_POINT_MISMATCH")

    # ----- contact specification geometry -----
    def test_contact_spec_geometry_mutations_refuse(self):
        def spec_edit(change):
            def edit(root):
                ref = self.load(root, "cmj", "reference.json")
                change(next(s for s in ref["contact_specs"] if s["kind"] == "HEEL"))
                self.store(root, "cmj", "reference.json", ref)
            return edit
        cases = {"local-point": (lambda s: s.update(local_point_a_m=[s["local_point_a_m"][0] + 0.5,
                                                                      s["local_point_a_m"][1]]),
                                 "MECHANICS_CONTACT_POINT_MISMATCH"),
                 "flipped-normal": (lambda s: s.update(normal_direction=[0.0, -1.0]), "SUPPORT_NORMAL_INCONSISTENT"),
                 "non-unit-normal": (lambda s: s.update(normal_direction=[0.0, 2.0]), "REFERENCE_CONTACT_SPECS_INVALID"),
                 "unknown-frame": (lambda s: s.update(normal_frame="SENSOR"), "REFERENCE_CONTACT_SPECS_INVALID"),
                 "world-body-frame": (lambda s: s.update(normal_frame="BODY_A"), "REFERENCE_CONTACT_SPECS_INVALID"),
                 "unknown-kind": (lambda s: s.update(kind="HAND"), "REFERENCE_CONTACT_SPECS_INVALID"),
                 "missing-surface": (lambda s: s.update(surface_point_m=None), "REFERENCE_CONTACT_SPECS_INVALID")}
        for label, (change, code) in cases.items():
            with self.subTest(mutation=label):
                self.assert_denied(self.mutate("cmj", spec_edit(change)), "cmj", code)

    def test_tensile_ledger_forgery_stays_evaluator_rejection(self):
        """The evaluator, not the normal binding, rejects physically tensile support."""
        def edit(root):
            sub = self.load(root, "cmj", "mechanics_submission.json")
            contact = next(c for f in sub["frames"] for c in f["contacts"])
            contact["fz"] = -5000.0
            self.store(root, "cmj", "mechanics_submission.json", sub)
            truth = self.truth(root, "cmj")
            rows = [",".join(["start_s", "end_s", "contact_id", "fx_N", "fz_N"])] + [
                ",".join(map(repr, (f["t0"], f["t1"]))) + f",{c['name']},{c['fx']!r},{c['fz']!r}"
                for f in sub["frames"] for c in f["contacts"]]
            (truth / "support_intervals.csv").write_text("\n".join(rows) + "\n", encoding="utf-8")
        root = self.mutate("cmj", edit)
        result = verify_reference(root / "SOURCE" / "cmj", self.truth(root, "cmj"))
        self.assertFalse(result["passed"])
        self.assertEqual(result["required_mechanics_outcomes"]["CONTACT_ADMISSIBILITY"], "REJECTED")

    # ----- inertial data: reported as unbound, never silently claimed -----
    def test_inertia_and_mirrored_mass_split_are_reported_unbound(self):
        def edit(root):
            sub = self.load(root, "cmj", "mechanics_submission.json")
            bodies = {b["name"]: b for b in sub["bodies"]}
            bodies["hat"]["inertia_kg_m2"] *= 10
            bodies["left_thigh"]["mass_kg"] -= 3.0
            bodies["right_thigh"]["mass_kg"] += 3.0
            self.store(root, "cmj", "mechanics_submission.json", sub)
        root = self.mutate("cmj", edit)
        binding = verify_reference(root / "SOURCE" / "cmj", self.truth(root, "cmj"))["mechanics_reference_binding"]
        self.assertEqual((binding["per_body_inertia"], binding["per_body_mass"]),
                         ("NOT_INDEPENDENTLY_BOUND_NO_TRUTH_AUTHORITY",
                          "TOTAL_MASS_AND_VERTICAL_MOMENTUM_IDENTITY_ONLY_MIRRORED_SEGMENTS_NOT_SEPARABLE"))

        def unmirrored(root):
            sub = self.load(root, "cmj", "mechanics_submission.json")
            bodies = {b["name"]: b for b in sub["bodies"]}
            bodies["hat"]["mass_kg"] -= 5.0
            bodies["left_thigh"]["mass_kg"] += 5.0
            self.store(root, "cmj", "mechanics_submission.json", sub)
        self.assert_denied(self.mutate("cmj", unmirrored), "cmj", "REFERENCE_MOMENTUM_INCONSISTENT")

    # ----- closed claim-scope authorities -----
    def test_elevated_event_authority_and_anatomical_claims_refuse(self):
        for key, value in (("event_authority", "INDEPENDENT_PRODUCTION_SENSOR_EVENT_DETECTOR_VALIDATED"),
                           ("anatomical_events_from_sensors", "ESTABLISHED_EMPIRICALLY_VALIDATED"),
                           ("anatomical_events_from_sensors", True)):
            def edit(root, key=key, value=value):
                doc = self.load(root, "cmj", "model_events.json")
                doc[key] = value
                self.store(root, "cmj", "model_events.json", doc)
            with self.subTest(key=key, value=value):
                self.assert_denied(self.mutate("cmj", edit), "cmj", "MODEL_EVENTS_AUTHORITY_INVALID")

    def test_elevated_supported_context_refuses(self):
        def edit(root):
            doc = self.load(root, "cmj", "context_qualification.json")
            doc["supported_context"] = "EMPIRICALLY_VALIDATED_HUMAN_DEVICE_CONTEXT"
            self.store(root, "cmj", "context_qualification.json", doc)
        self.assert_denied(self.mutate("cmj", edit), "cmj", "CONTEXT_LEDGER_INVALID")

    def test_physical_event_log_structure_is_admitted_and_reported_archival(self):
        def change(fn):
            def edit(root):
                ref = self.load(root, "cmj", "reference.json")
                fn(ref["physical_events"])
                self.store(root, "cmj", "reference.json", ref)
            return edit
        for label, fn in (("unknown-id", lambda e: e[0].update(event_id="ANATOMICAL_TAKEOFF")),
                          ("out-of-domain", lambda e: e[0].update(time_s=1e6)),
                          ("unknown-contact", lambda e: e[0].update(contact_ids=["left_hand"])),
                          ("not-a-record", lambda e: e.append("CONTACT_RELEASE"))):
            with self.subTest(mutation=label):
                self.assert_denied(self.mutate("cmj", change(fn)), "cmj", "REFERENCE_PHYSICAL_EVENTS_INVALID")
        root = self.mutate("cmj", change(lambda e: e.clear()))
        result = verify_reference(root / "SOURCE" / "cmj", self.truth(root, "cmj"))
        self.assertEqual(result["physical_event_log"], "STRUCTURE_ADMITTED_ARCHIVAL_SIMULATOR_LOG_NOT_RECOMPUTED")

    # ----- bounded synchronization against protected TRUTH sample times -----
    def test_declared_jitter_bound_on_jittered_recording_refuses(self):
        for bound in (0, 1e-9):
            def edit(manifest, notes, bound=bound):
                notes["acquisition_jitter_policy"].update(distribution="ZERO", standard_deviation_s=0,
                                                          finite_error_bound_s=bound)
            root = self.bundle("jitter")
            self.reseal_source(root, "jitter", edit)
            with self.subTest(bound=bound):
                self.assert_cli_denied(root, "SYNCHRONIZATION_TRUTH_INCONSISTENT")

    def test_capture_consistent_but_shifted_clock_map_refuses(self):
        def edit(manifest, notes, shift=4e-4):
            device = notes["sync_capture"]["devices"]["lpt"]
            device["edge_native_times_s"] = [t - shift for t in device["edge_native_times_s"]]
            mapping = manifest["synchronization"]["stream_mappings"]["lpt"]
            mapping["offset_s"] += mapping["drift_scale"] * shift
        root = self.bundle("guide")
        self.reseal_source(root, "guide", edit)
        self.assert_cli_denied(root, "SYNCHRONIZATION_TRUTH_INCONSISTENT")


if __name__ == "__main__":
    unittest.main()
