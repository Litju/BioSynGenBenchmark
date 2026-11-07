"""RES-498 coverage matrix and independent I2 evaluator."""

from __future__ import annotations

import copy
import importlib
import pathlib
import unittest

from biosyngen.i2.export import export_submission
from biosyngen.i2.jumps import build_jump
from biosyngen_benchmark.i2_coverage import (
    IMPLEMENTED,
    NOT_IMPLEMENTED,
    SCOPED,
    CoverageGap,
    coverage_matrix,
)
from biosyngen_benchmark.i2_evaluator import FAULTS, drop_group, evaluate_i2, fault_outcome
from biosyngen_benchmark.protocol_inventory import load_inventory

_SUB: dict = {}


def cmj_submission() -> dict:
    if not _SUB:
        _SUB.update(export_submission(build_jump("CMJ").run()))
    return copy.deepcopy(_SUB)


class CoverageMatrixTests(unittest.TestCase):
    def test_every_inventory_row_and_deferred_candidate_has_an_explicit_status(self) -> None:
        inventory = load_inventory()
        rows = {r["exercise_id"]: r for r in coverage_matrix(inventory)}
        for row in inventory["exercise_matrix"]:
            self.assertIn(row["exercise_id"], rows)
        for candidate in inventory["deferred_protocol_candidates"]:
            self.assertEqual(rows[candidate["exercise_id"]]["status"], NOT_IMPLEMENTED)
            self.assertTrue(rows[candidate["exercise_id"]]["reason"])
        self.assertEqual(rows["POWER_CLEAN"]["status"], SCOPED)
        for row in rows.values():
            self.assertEqual(row["external_reference"], "EXTERNAL_REFERENCE_NOT_EVALUABLE")
            if row["status"] in (IMPLEMENTED, SCOPED):
                importlib.import_module(row["tests"])  # the cited test module exists

    def test_an_unlisted_inventory_row_or_variant_cannot_be_silently_omitted(self) -> None:
        inventory = copy.deepcopy(load_inventory())
        inventory["exercise_matrix"].append(dict(inventory["exercise_matrix"][0], exercise_id="NEW_EXERCISE"))
        with self.assertRaises(CoverageGap):
            coverage_matrix(inventory)
        inventory = copy.deepcopy(load_inventory())
        inventory["exercise_matrix"][0] = dict(inventory["exercise_matrix"][0], variant_ids=["UNMAPPED_VARIANT"])
        with self.assertRaises(CoverageGap):
            coverage_matrix(inventory)


class IndependentEvaluatorTests(unittest.TestCase):
    def test_evaluator_imports_nothing_from_the_generator(self) -> None:
        source = pathlib.Path("biosyngen_benchmark/i2_evaluator.py").read_text(encoding="utf-8")
        self.assertNotIn("import biosyngen.", source.replace("biosyngen_benchmark", ""))
        self.assertNotIn("from biosyngen", source.replace("biosyngen_benchmark", ""))

    def test_generator_export_passes_and_bilateral_or_anatomical_claims_abstain(self) -> None:
        result = evaluate_i2(cmj_submission())
        self.assertEqual(result["status"], "EVALUATED")
        self.assertEqual(result["findings"]["LINEAR_MOMENTUM_CLOSURE"]["outcome"], "PASS")
        self.assertEqual(result["findings"]["CONTACT_ADMISSIBILITY"]["outcome"], "PASS")
        self.assertEqual(result["findings"]["BILATERAL_FORCE"]["outcome"], "NOT_EVALUABLE")
        self.assertEqual(result["findings"]["ANATOMICAL_PHASES"]["outcome"], "NOT_EVALUABLE")

    def test_every_fault_injection_and_support_omission_is_refused(self) -> None:
        submission = cmj_submission()
        for name, (inject, expected, claim) in FAULTS.items():
            with self.subTest(fault=name):
                self.assertIn(fault_outcome(evaluate_i2(inject(submission)), claim), expected)
        for group in submission["external_support_groups"]:
            with self.subTest(omit=group):
                result = evaluate_i2(drop_group(submission, group))
                self.assertEqual(result["findings"]["LINEAR_MOMENTUM_CLOSURE"]["outcome"], "REJECTED")

    def test_strict_schema_refuses_unknown_and_malformed_fields(self) -> None:
        submission = cmj_submission()
        submission["latent_events"] = {"takeoff": 0.8}
        self.assertEqual(evaluate_i2(submission)["status"], "INVALID")
        submission = cmj_submission()
        submission["frames"][3]["t0"] += 0.01
        self.assertEqual(evaluate_i2(submission)["status"], "INVALID")
        submission = cmj_submission()
        submission["gravity_m_s2"] = 3.71
        self.assertEqual(evaluate_i2(submission)["status"], "INVALID")
        self.assertEqual(evaluate_i2([])["status"], "INVALID")


if __name__ == "__main__":
    unittest.main()
