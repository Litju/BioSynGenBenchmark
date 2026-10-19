"""RES-532 regressions for the RES-525 re-audit P1 groups R01-R04 (+R05).

R01 cross-artifact mechanics binding, R02 pinned-root link admission with a
hooked zero-outside-read monitor, R03 closed TRUTH structural admission and R04
interval-qualified clock fits. Every adversary is a disposable manufactured
copy that a test-only operator coherently re-endorses, so each refusal comes
from semantic admission rather than a stale checksum.
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
import biosyngen_benchmark.paired_source as paired_source
from biosyngen_benchmark.paired_source import SourceRefused, verify_source
from scripts.verify_paired_corpus import ReferenceRefused, verify_reference

ROOT = Path(__file__).resolve().parents[1]
CASES = {"guide": ("guided_vertical", {}), "jitter": ("guided_vertical", {"jitter_sd_s": 5e-6}),
         "drift_guide": ("guided_vertical", {"force_drift_ppm": 500, "lpt_drift_ppm": -500}),
         "overlap_guide": ("guided_vertical", {"force_drift_ppm": 300, "lpt_drift_ppm": -200, "lpt_start_s": 0.5,
                                               "force_offset_s": 0.3, "lpt_offset_s": -0.2, "lpt_drop_indices": [800]}),
         "cmj": ("cmj", {}), "sj": ("sj", {}), "cmj_variant": ("cmj", {"effort": 0.9})}

MONITOR = textwrap.dedent("""
    import json, os, pathlib, sys
    import scripts.verify_paired_corpus as verifier
    root, out = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
    allowed = [pathlib.Path(os.path.realpath(root)), out.resolve()]
    outside, busy = [], []
    def hook(event, args):
        # Record every attempted data open or directory listing outside the
        # pinned bundle; only Python code/extension imports are exempt.
        if busy or event not in ("open", "os.listdir", "os.scandir") or not args:
            return
        if not isinstance(args[0], (str, bytes, pathlib.PurePath)):
            return
        busy.append(1)
        try:
            path = pathlib.Path(os.fsdecode(args[0])).resolve()
            if path.suffix not in (".py", ".pyc", ".pyd", ".so", ".dll") and not any(
                    path == a or path.is_relative_to(a) for a in allowed):
                outside.append(str(path))
        finally:
            busy.clear()
    sys.addaudithook(hook)
    try:
        code = verifier.main(["--root", str(root), "--out", str(out)])
        report = json.loads(out.read_text())
        refusal = report["cases"][0].get("evidence_refusal")
    except (verifier.ReferenceRefused, verifier.SourceRefused) as exc:
        code, refusal = None, str(exc)
    print(json.dumps({"exit": code, "refusal": refusal, "outside_reads": outside}))
""")


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def make_link(target, link):
    """Directory link: a junction on Windows, a symbolic link elsewhere."""
    if sys.platform == "win32":
        import _winapi
        _winapi.CreateJunction(str(target), str(link))
    else:
        os.symlink(target, link, target_is_directory=True)


class Res532Admission(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(dir=ROOT / "runs")
        cls.corpus = Path(cls.tmp.name) / "corpus"
        cls.receipts = {}
        for seed, (case_id, (protocol, parameters)) in enumerate(CASES.items(), 53200):
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

    def load(self, root, case_id, name):
        return json.loads((self.paths(root, case_id)[1] / name).read_text())

    def store(self, root, case_id, name, value):
        dump(self.paths(root, case_id)[1] / name, value)

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

    def reseal_source(self, root, case_id, edit):
        """Edit manifest/notes, then re-endorse SOURCE, authority and TRUTH binding."""
        source, truth, authority = self.paths(root, case_id)
        path = source / "manifest.json"
        manifest = json.loads(path.read_text())
        notes = json.loads(manifest["operator_notes"])
        edit(manifest, notes)
        manifest["operator_notes"] = json.dumps(notes, sort_keys=True)
        dump(path, manifest)
        (source / "checksums.sha256").write_text("".join(f"{sha(p)}  {p.relative_to(source).as_posix()}\n"
            for p in sorted(source.rglob("*")) if p.is_file() and p.name != "checksums.sha256"))
        dump(authority, {**notes["sync_capture"], "source_manifest_sha256": sha(path)})
        reference = json.loads((truth / "reference.json").read_text())
        reference["source_manifest_sha256"] = sha(path)
        dump(truth / "reference.json", reference)
        self.reseal_truth(root, case_id)

    def cli(self, root):
        out = root.parent / f"verification-{root.name}.json"
        result = subprocess.run([sys.executable, "-m", "scripts.verify_paired_corpus", "--root", str(root),
                                 "--out", str(out)], cwd=ROOT, capture_output=True, text=True)
        self.assertNotIn("Traceback", result.stderr)
        return result.returncode, json.loads(out.read_text())

    def assert_denied(self, root, case_id, code):
        source, truth, _ = self.paths(root, case_id)
        with self.assertRaisesRegex(ReferenceRefused, f"^{code}$"):
            verify_reference(source, truth)
        exit_code, report = self.cli(root)
        self.assertEqual((exit_code, report["model_reference_qualified_count"], report["full_case_qualified_count"],
                          report["cases"][0].get("evidence_refusal")), (1, 0, 0, code))

    # ----- positive controls -----
    def test_positive_controls_guides_cmj_sj_and_variant_qualify(self):
        cases = ("guide", "drift_guide", "overlap_guide", "cmj", "sj", "cmj_variant")
        root = self.bundle(*cases)
        for case_id in cases:
            with self.subTest(case=case_id):
                source, truth, _ = self.paths(root, case_id)
                checked = verify_source(source, self.authority(root, case_id))
                self.assertEqual((checked["source"], checked["synchronization"]), ("PASS", "BOUNDED_CAPTURE_MAPPING"))
                self.assertEqual(checked["clock_admission_contract"], "BSMB-CLOCK-ADMISSION-1.0")
                self.assertTrue(all(0 < v < 5e-4 for v in checked["worst_case_mapping_error_s"].values()))
                result = verify_reference(source, truth)
                self.assertTrue(result["passed"], result)
                self.assertEqual(result["truth_admission_contract"], "BSMB-PAIRED-TRUTH-ADMISSION-1.0")
                binding = result["mechanics_reference_binding"]["status"]
                self.assertEqual(binding, "NOT_APPLICABLE_GUIDE_LEDGER_ONLY" if "guide" in case_id
                                 else "BOUND_TO_REFERENCE_TRIAL")
        exit_code, report = self.cli(root)
        self.assertEqual((exit_code, report["full_case_qualified_count"]), (0, len(cases)))

    def test_jitter_guide_keeps_typed_synchronization_refusal(self):
        root = self.bundle("jitter")
        checked = verify_source(self.paths(root, "jitter")[0], self.authority(root, "jitter"))
        self.assertEqual((checked["source"], checked["synchronization"]), ("PASS", "REFUSED_UNBOUNDED_SAMPLE_JITTER"))
        exit_code, report = self.cli(root)
        self.assertEqual((exit_code, report["model_reference_qualified_count"], report["full_case_qualified_count"],
                          report["synchronization_refused_count"]), (0, 1, 0, 1))

    # ----- R01 cross-artifact mechanics binding -----
    def substitute(self, root, donor_case, edit=None):
        donor = json.loads((self.corpus / "TRUTH" / donor_case / "mechanics_submission.json").read_text())
        if edit:
            edit(donor)
        self.store(root, "cmj", "mechanics_submission.json", donor)
        self.reseal_truth(root, "cmj")

    def test_r01_original_sj_donor_into_cmj_refuses(self):
        root = self.bundle("cmj")
        shutil.copyfile(self.corpus / "TRUTH/sj/mechanics_submission.json",
                        self.paths(root, "cmj")[1] / "mechanics_submission.json")
        self.reseal_truth(root, "cmj")
        self.assertEqual(verify_source(self.paths(root, "cmj")[0], self.authority(root, "cmj"))["source"], "PASS")
        self.assert_denied(root, "cmj", "MECHANICS_PROTOCOL_MISMATCH")

    def test_r01_relabelled_donor_and_same_protocol_other_trial_refuse(self):
        root = self.bundle("cmj")
        self.substitute(root, "sj", lambda d: d.update(exercise_id="CMJ"))
        self.assert_denied(root, "cmj", "MECHANICS_TIME_GRID_MISMATCH")
        variant = json.loads((self.corpus / "TRUTH/cmj_variant/mechanics_submission.json").read_text())
        self.assertEqual(variant["exercise_id"], "CMJ")
        root = self.bundle("cmj")
        self.substitute(root, "cmj_variant")
        with self.assertRaises(ReferenceRefused) as caught:
            verify_reference(*self.paths(root, "cmj")[:2])
        self.assertIn(str(caught.exception), {"MECHANICS_TIME_GRID_MISMATCH", "MECHANICS_STATE_MISMATCH"})
        exit_code, report = self.cli(root)
        self.assertEqual((exit_code, report["full_case_qualified_count"]), (1, 0))

    def test_r01_value_mutations_of_submission_refuse(self):
        def state(d):
            d["frames"][1000]["body_states"][0][1] += 1e-9

        def initial(d):
            d["initial_body_states"][2][3] = 0.5

        def mass(d):
            d["bodies"][0]["mass_kg"] += 1.0

        def force(d):
            frame = next(f for f in d["frames"] if f["contacts"])
            frame["contacts"][0]["fz"] += 1e-6

        def group(d):
            d["frames"][0]["contacts"][0]["group"] = "BENCH"

        def impulse(d):
            d["impulses"][0]["jz"] *= 1.01

        def friction(d):
            d["friction_by_group"]["FOOT_LEFT"] = 0.9

        def grid(d):
            d["frames"][5]["t1"] += 1e-7
        for edit, code in ((state, "MECHANICS_STATE_MISMATCH"), (initial, "MECHANICS_STATE_MISMATCH"),
                           (mass, "MECHANICS_BODY_MISMATCH"), (force, "SUPPORT_LEDGER_MECHANICS_MISMATCH"),
                           (group, "MECHANICS_CONTACT_MISMATCH"), (impulse, "MECHANICS_IMPULSE_MISMATCH"),
                           (friction, "MECHANICS_CONTACT_MISMATCH"), (grid, "MECHANICS_TIME_GRID_MISMATCH")):
            with self.subTest(code=code, edit=edit.__name__):
                root = self.bundle("cmj")
                self.substitute(root, "cmj", edit)
                self.assert_denied(root, "cmj", code)

    def test_r01_consistent_force_forgery_still_meets_independent_evaluator(self):
        # Binding is identity only: a coherent ledger+submission forgery binds,
        # and the independent I2 evaluator, not the binding, rejects its physics.
        root = self.bundle("cmj")
        _, truth, _ = self.paths(root, "cmj")
        submission = self.load(root, "cmj", "mechanics_submission.json")
        for frame in submission["frames"][100:200]:
            for contact in frame["contacts"]:
                contact["fz"] += 400.0
        self.store(root, "cmj", "mechanics_submission.json", submission)
        rows = [(f["t0"], f["t1"], c["name"], c["fx"], c["fz"]) for f in submission["frames"] for c in f["contacts"]]
        with (truth / "support_intervals.csv").open("w", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(("start_s", "end_s", "contact_id", "fx_N", "fz_N"))
            writer.writerows(rows)
        self.reseal_truth(root, "cmj")
        result = verify_reference(*self.paths(root, "cmj")[:2])
        self.assertEqual(result["mechanics_reference_binding"]["status"], "BOUND_TO_REFERENCE_TRIAL")
        self.assertEqual(result["required_mechanics_outcomes"]["LINEAR_MOMENTUM_CLOSURE"], "REJECTED")
        self.assertFalse(result["passed"])
        exit_code, report = self.cli(root)
        self.assertEqual((exit_code, report["model_reference_qualified_count"]), (1, 0))

    def test_r01_reference_internal_inconsistencies_refuse(self):
        def mode(r):
            r["contacts"][50]["left_toe"] = "OPEN" if r["contacts"][50]["left_toe"] != "OPEN" else "STICK"

        def momentum(r):
            r["vertical_momentum_kg_m_s"][2000] += 1e-3

        def attachment(r):
            r["reference_attachment"]["body_id"] = "phantom"
        for edit, code in ((mode, "REFERENCE_CONTACT_MODES_INCONSISTENT"),
                           (momentum, "REFERENCE_MOMENTUM_INCONSISTENT"), (attachment, "REFERENCE_SCALAR_INVALID")):
            with self.subTest(code=code):
                root = self.bundle("cmj")
                reference = self.load(root, "cmj", "reference.json")
                edit(reference)
                self.store(root, "cmj", "reference.json", reference)
                self.reseal_truth(root, "cmj")
                self.assert_denied(root, "cmj", code)

    # ----- R02 pinned-root link admission, zero outside reads -----
    def monitored(self, root):
        out = root.parent / f"monitored-{root.name}-{self.serial}.json"
        result = subprocess.run([sys.executable, "-c", MONITOR, str(root), str(out)], cwd=ROOT,
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        return json.loads(result.stdout.splitlines()[-1])  # last line: the monitor record

    def test_r02_clean_bundle_monitored_has_no_outside_reads(self):
        root = self.bundle("guide", "cmj")
        record = self.monitored(root)
        self.assertEqual((record["exit"], record["refusal"], record["outside_reads"]), (0, None, []))

    def test_r02_namespace_and_case_root_links_refuse_before_any_outside_read(self):
        codes = {"SOURCE": "SOURCE_ROOT_LINK_REFUSED", "TRUTH": "TRUTH_ROOT_LINK_REFUSED",
                 "DIAGNOSTICS": "DIAGNOSTICS_ROOT_LINK_REFUSED"}
        for level in ("case", "namespace"):
            for namespace, code in codes.items():
                with self.subTest(level=level, namespace=namespace):
                    root = self.bundle("guide")
                    linked = root / namespace / "guide" if level == "case" else root / namespace
                    target = self.work / f"sentinel-{level}-{namespace}-{self.serial}"
                    shutil.move(str(linked), target)
                    make_link(target, linked)
                    record = self.monitored(root)
                    self.assertEqual((record["exit"], record["refusal"], record["outside_reads"]), (1, code, []))
                    source, truth, authority = self.paths(root, "guide")
                    if namespace == "DIAGNOSTICS":
                        continue
                    with self.assertRaisesRegex((SourceRefused, ReferenceRefused), f"^{code}$"):
                        if namespace == "SOURCE":
                            verify_source(source, json.loads(authority.read_text()))
                        else:
                            verify_reference(source, truth)

    def test_r02_linked_bundle_root_and_internal_truth_link_refuse(self):
        root = self.bundle("guide")
        alias = self.work / "linked-root"
        make_link(root, alias)
        record = self.monitored(alias)
        self.assertEqual((record["exit"], record["refusal"], record["outside_reads"]),
                         (None, "BUNDLE_ROOT_LINK_REFUSED", []))
        root = self.bundle("guide")
        target = self.work / "truth-internal-target"
        target.mkdir()
        (target / "reference.json").write_text("{}")
        make_link(target, self.paths(root, "guide")[1] / "extra")
        record = self.monitored(root)
        self.assertEqual((record["exit"], record["refusal"], record["outside_reads"]),
                         (1, "TRUTH_LINK_OR_DIRECTORY_REFUSED", []))

    def test_r02_lexical_case_and_namespace_names_refuse(self):
        root = self.bundle("guide")
        for parts in ((".."), ("SOURCE", ".."), ("SOURCE", "a/b"), ("SOURCE", "C:x"), ("SOURCE", "")):
            with self.subTest(parts=parts):
                with self.assertRaises(SourceRefused):
                    paired_source.admitted_directory(root, parts if type(parts) is tuple else (parts,), "LINK", "MISSING")

    # ----- R03 closed TRUTH structural admission -----
    def mutate_reference(self, case_id, edit, code):
        root = self.bundle(case_id)
        reference = self.load(root, case_id, "reference.json")
        edit(reference)
        self.store(root, case_id, "reference.json", reference)
        self.reseal_truth(root, case_id)
        self.assert_denied(root, case_id, code)

    def test_r03_original_duplicate_time_and_truncated_states_refuse(self):
        def duplicate(r):
            r["physics_times_s"][100] = r["physics_times_s"][99]
        for case_id in ("guide", "cmj"):
            with self.subTest(case=case_id):
                self.mutate_reference(case_id, duplicate, "REFERENCE_TIME_GRID_INVALID")
        self.mutate_reference("cmj", lambda r: r.update(states=r["states"][:1]), "REFERENCE_STATES_MISALIGNED")

    def test_r03_reference_array_and_state_corruption_refuse(self):
        def reversed_time(r):
            r["physics_times_s"][10], r["physics_times_s"][11] = r["physics_times_s"][11], r["physics_times_s"][10]

        def state_time(r):
            r["states"][7]["time_s"] += 1e-6

        def body_value(r):
            r["states"][9]["bodies"]["hat"]["velocity_m_s"] = [0.0]

        def bad_mode(r):
            r["states"][3]["contacts"]["left_toe"]["mode"] = "GLUED"
        for case_id, edit, code in (
                ("guide", lambda r: r["vertical_momentum_kg_m_s"].pop(), "REFERENCE_ARRAY_MISALIGNED"),
                ("guide", lambda r: r["attachment_vertical_m"].__setitem__(5, None), "REFERENCE_ARRAY_MISALIGNED"),
                ("guide", reversed_time, "REFERENCE_TIME_GRID_INVALID"),
                ("guide", lambda r: r.update(mass_kg=0), "REFERENCE_SCALAR_INVALID"),
                ("guide", lambda r: r["lpt_true_sample_times_s"].__setitem__(-1, 99.0), "REFERENCE_SAMPLE_TIMES_INVALID"),
                ("cmj", lambda r: r["vertical_momentum_kg_m_s"].pop(), "REFERENCE_ARRAY_MISALIGNED"),
                ("cmj", state_time, "REFERENCE_STATES_MISALIGNED"),
                ("cmj", body_value, "REFERENCE_STATES_MALFORMED"),
                ("cmj", bad_mode, "REFERENCE_STATES_MALFORMED"),
                ("cmj", lambda r: r.update(contacts=[]), "REFERENCE_STATES_MISALIGNED"),
                ("cmj", lambda r: r["impulses_N_s"][0].update(time_s=-1.0), "REFERENCE_IMPULSES_INVALID"),
                ("cmj", lambda r: r["contact_specs"].append(dict(r["contact_specs"][0])),
                 "REFERENCE_CONTACT_SPECS_INVALID")):
            with self.subTest(case=case_id, code=code):
                self.mutate_reference(case_id, edit, code)

    def write_ledger(self, root, case_id, rows):
        with (self.paths(root, case_id)[1] / "support_intervals.csv").open("w", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(("start_s", "end_s", "contact_id", "fx_N", "fz_N"))
            writer.writerows(rows)
        self.reseal_truth(root, case_id)

    def ledger(self, case_id):
        with (self.corpus / "TRUTH" / case_id / "support_intervals.csv").open(newline="") as stream:
            return [list(r.values()) for r in csv.DictReader(stream)]

    def test_r03_original_reversed_compensated_guide_interval_refuses(self):
        rows = self.ledger("guide")
        rows[0][1], rows[1][1] = "-0.001", "0.003"
        root = self.bundle("guide")
        self.write_ledger(root, "guide", rows)
        self.assert_denied(root, "guide", "SUPPORT_INTERVAL_INVALID")

    def test_r03_support_ledger_mutations_refuse(self):
        def zero(rows):
            rows[4][1] = rows[4][0]

        def offgrid(rows):
            rows[6][1] = str(float(rows[6][1]) + 1e-7)

        def swap(rows):
            rows[10], rows[12] = rows[12], rows[10]

        def duplicate(rows):
            rows.insert(3, list(rows[2]))

        def drop(rows):
            del rows[40]

        def unknown(rows):
            rows[0][2] = "phantom"

        def nan(rows):
            rows[8][4] = "nan"
        for case_id, edit, code in (("guide", zero, "SUPPORT_INTERVAL_INVALID"),
                                    ("guide", offgrid, "SUPPORT_INTERVAL_INVALID"),
                                    ("guide", swap, "SUPPORT_INTERVAL_INVALID"),
                                    ("guide", duplicate, "SUPPORT_INTERVAL_INVALID"),
                                    ("guide", drop, "SUPPORT_LEDGER_COVERAGE_INCOMPLETE"),
                                    ("guide", unknown, "SUPPORT_LEDGER_MALFORMED"),
                                    ("guide", nan, "SUPPORT_LEDGER_MALFORMED"),
                                    ("guide", lambda rows: rows.clear(), "SUPPORT_LEDGER_MALFORMED"),
                                    ("cmj", lambda rows: rows.clear(), "SUPPORT_LEDGER_MALFORMED"),
                                    ("cmj", drop, "SUPPORT_LEDGER_MECHANICS_MISMATCH"),
                                    ("cmj", zero, "SUPPORT_INTERVAL_INVALID")):
            with self.subTest(case=case_id, code=code, edit=getattr(edit, "__name__", "clear")):
                rows = self.ledger(case_id)
                edit(rows)
                root = self.bundle(case_id)
                self.write_ledger(root, case_id, rows)
                self.assert_denied(root, case_id, code)
        root = self.bundle("guide")
        path = self.paths(root, "guide")[1] / "support_intervals.csv"
        path.write_text(path.read_text().replace("fz_N", "fz_kN", 1))
        self.reseal_truth(root, "guide")
        self.assert_denied(root, "guide", "SUPPORT_LEDGER_MALFORMED")

    def test_r03_context_ledger_must_parse_and_retain_refusals(self):
        def drop_refusal(c):
            del c["findings"]["real_paired_force_lpt"]

        def promote(c):
            c["findings"]["biological_validation"]["outcome"] = "PASS"
        for label, text in (("original-malformed", "{garbage"), ("empty", "{}\n"), ("list", "[]\n"),
                            ("duplicate-key", '{"termination": "X", "termination": "DURATION_REACHED"}')):
            with self.subTest(label=label):
                root = self.bundle("cmj")
                (self.paths(root, "cmj")[1] / "context_qualification.json").write_text(text)
                self.reseal_truth(root, "cmj")
                expected = {"original-malformed": "TRUTH_MALFORMED_JSON",
                            "duplicate-key": "TRUTH_DUPLICATE_JSON_KEY"}.get(label, "CONTEXT_LEDGER_INVALID")
                self.assert_denied(root, "cmj", expected)
        for edit in (drop_refusal, promote, lambda c: c.update(termination="EARLY_FAILURE"),
                     lambda c: c.update(extra=1)):
            with self.subTest(edit=getattr(edit, "__name__", "lambda")):
                root = self.bundle("cmj")
                context = self.load(root, "cmj", "context_qualification.json")
                edit(context)
                self.store(root, "cmj", "context_qualification.json", context)
                self.reseal_truth(root, "cmj")
                self.assert_denied(root, "cmj", "CONTEXT_LEDGER_INVALID")
        result = verify_reference(*self.paths(self.bundle("cmj"), "cmj")[:2])
        self.assertEqual(result["context_evidence"]["protocol_findings"], "ARCHIVAL_MODEL_OUTPUT_NOT_RECOMPUTED")

    def test_r03_model_events_resolution_and_envelope_are_closed(self):
        for edit in (lambda e: e.update(time_resolution_s=0.002), lambda e: e.update(extra=True),
                     lambda e: e.pop("event_authority")):
            with self.subTest():
                root = self.bundle("cmj")
                events = self.load(root, "cmj", "model_events.json")
                edit(events)
                self.store(root, "cmj", "model_events.json", events)
                self.reseal_truth(root, "cmj")
                self.assert_denied(root, "cmj", "MODEL_EVENTS_MALFORMED")

    # ----- R05 duplicate JSON authority keys -----
    def test_r05_duplicate_json_keys_refuse_in_truth_and_source(self):
        root = self.bundle("guide")
        path = self.paths(root, "guide")[1] / "reference.json"
        path.write_text(path.read_text().replace("{", '{"protocol": "cmj",', 1))
        self.reseal_truth(root, "guide")
        self.assert_denied(root, "guide", "TRUTH_DUPLICATE_JSON_KEY")
        with self.assertRaisesRegex(SourceRefused, "^DUPLICATE_JSON_KEY$"):
            paired_source.strict_json('{"a": {"b": 1, "b": 1}}')
        self.assertEqual(paired_source.strict_json('{"a": {"b": 1}, "c": [1, {"d": 2}]}'), {"a": {"b": 1}, "c": [1, {"d": 2}]})

    # ----- R04 interval-qualified clock admission -----
    def clock_denied(self, edit, code, case_id="cmj"):
        root = self.bundle(case_id)
        self.reseal_source(root, case_id, edit)
        with self.assertRaisesRegex(SourceRefused, f"^{code}$"):
            verify_source(self.paths(root, case_id)[0], self.authority(root, case_id))
        exit_code, report = self.cli(root)
        self.assertEqual((exit_code, report["cases"][0].get("evidence_refusal"), report["bounded_synchronization_count"],
                          report["full_case_qualified_count"]), (1, code, 0, 0))

    @staticmethod
    def set_edges(manifest, notes, name, edges):
        """Operator re-declares edges and the matching point-estimate mapping."""
        capture = notes["sync_capture"]
        capture["devices"][name]["edge_native_times_s"] = edges
        ref = capture["reference_edge_times_s"]
        scale = (ref[-1] - ref[0]) / (edges[-1] - edges[0])
        manifest["synchronization"]["stream_mappings"][name].update(offset_s=ref[0] - scale * edges[0], drift_scale=scale)

    def test_r04_original_near_zero_native_span_refuses(self):
        self.clock_denied(lambda m, n: self.set_edges(m, n, "lpt", [0.0, 1e-300, 2e-300, 3e-300]),
                          "CLOCK_ANCHOR_SPAN_UNRESOLVED")

    def test_r04_span_near_error_bound_and_excess_drift_refuse(self):
        self.clock_denied(lambda m, n: self.set_edges(m, n, "lpt", [0.0, 1.3e-6, 2.6e-6, 3.9e-6]),
                          "CLOCK_ANCHOR_SPAN_UNRESOLVED")

        def near(m, n):
            ref = n["sync_capture"]["reference_edge_times_s"]
            self.set_edges(m, n, "lpt", [v * 2e-6 for v in ref])
        self.clock_denied(near, "CLOCK_SCALE_OUTSIDE_ADMISSIBLE_DRIFT")

        def drift(m, n):
            ref = n["sync_capture"]["reference_edge_times_s"]
            self.set_edges(m, n, "force", [v / 1.0015 for v in ref])
        self.clock_denied(drift, "CLOCK_SCALE_OUTSIDE_ADMISSIBLE_DRIFT")

    def test_r04_huge_offset_and_samples_outside_fit_window_refuse(self):
        def offset(m, n):
            edges = n["sync_capture"]["devices"]["force"]["edge_native_times_s"]
            self.set_edges(m, n, "force", [v + 1e5 for v in edges])
        self.clock_denied(offset, "CLOCK_OFFSET_OUTSIDE_ADMISSIBLE")

        def window(m, n):
            capture = n["sync_capture"]
            old = capture["reference_edge_times_s"]
            fits = {}
            for name in ("force", "lpt"):
                e = capture["devices"][name]["edge_native_times_s"]
                s = (old[-1] - old[0]) / (e[-1] - e[0])
                fits[name] = (s, old[0] - s * e[0])
            # Pulses now span only the first half of the record; samples extrapolate.
            capture["reference_edge_times_s"] = [v / 2 for v in old]
            for name, (s, o) in fits.items():
                self.set_edges(m, n, name, [(v / 2 - o) / s for v in old])
        self.clock_denied(window, "CLOCK_SAMPLES_OUTSIDE_FIT_WINDOW")

    def test_r04_invalid_uncertainty_values_and_held_out_conflict_refuse(self):
        root = self.bundle("cmj")
        capture = {k: v for k, v in self.authority(root, "cmj").items() if k != "source_manifest_sha256"}
        for value in (0, -1e-6, float("nan"), float("inf"), 3e-6, True, None):
            with self.subTest(bound=value):
                # In-memory values reach the check; serialized NaN/Infinity is
                # already refused at parse time (NONFINITE_JSON_NUMBER).
                with self.assertRaisesRegex(SourceRefused, "^CAPTURE_UNCERTAINTY_INVALID$"):
                    paired_source.validate_capture({**capture, "capture_error_bound_s": value})
        self.clock_denied(lambda m, n: n["sync_capture"]["devices"]["lpt"]["edge_native_times_s"].__setitem__(
            1, n["sync_capture"]["devices"]["lpt"]["edge_native_times_s"][1] + 1e-5), "HELD_OUT_CAPTURE_RESIDUAL")

    def test_r04_mapping_uncertainty_must_resolve_native_sampling(self):
        # Defensive invariant: unreachable with the 2 us contract envelope, so a
        # widened envelope is patched in to show the propagated bound refuses.
        root = self.bundle("guide")
        self.reseal_source(root, "guide", lambda m, n: n["sync_capture"].update(capture_error_bound_s=3e-4))
        with patch.object(paired_source, "CAPTURE_ERROR_BOUND_MAX_S", 1e-3):
            with self.assertRaisesRegex(SourceRefused, "^CLOCK_MAPPING_UNCERTAINTY_UNRESOLVED$"):
                verify_source(self.paths(root, "guide")[0], self.authority(root, "guide"))


if __name__ == "__main__":
    unittest.main()
