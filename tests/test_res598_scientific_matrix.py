import unittest

from biosyngen_benchmark.scientific_matrix import (
    DOMAINS, PRIMARY_MOVEMENTS, qualification_matrix, qualification_counts,
)


class ScientificMatrixTests(unittest.TestCase):
    def test_exact_cartesian_inventory_without_implicit_passes(self):
        cells = qualification_matrix()["primary_cells"]
        self.assertEqual(len(cells), 120)
        self.assertEqual({(c["movement"], c["domain"]) for c in cells},
                         {(m, d) for m in PRIMARY_MOVEMENTS for d in DOMAINS})
        self.assertFalse(any(c["disposition"] == "PASS" for c in cells))

    def test_missing_catch_is_not_promoted_by_pull_coverage(self):
        report = qualification_matrix()
        for cell in report["primary_cells"]:
            if cell["movement"] == "POWER_CLEAN" and cell["domain"] == "MOVEMENT":
                self.assertEqual(cell["disposition"], "NOT_IMPLEMENTED")
        self.assertTrue(report["retained_variants"])
        self.assertTrue(all(c["disposition"] == "NOT_EVALUABLE"
                            for c in report["primary_cells"] if c["domain"] == "BIOLOGY"))

    def test_denominators_keep_refusals_and_pending_separate(self):
        result = qualification_counts(qualification_matrix()["primary_cells"])
        self.assertEqual(result["total"], 120)
        self.assertEqual(result["evaluated"], 0)
        self.assertEqual(result["counts"]["NOT_IMPLEMENTED"], 20)
        self.assertEqual(result["counts"]["NOT_EVALUABLE"], 10)
        self.assertEqual(result["counts"]["EVIDENCE_PENDING"], 90)

    def test_forged_inventory_or_disposition_fails_closed(self):
        cells = qualification_matrix()["primary_cells"]
        for bad in (cells[:-1], cells + cells[:1],
                    [dict(cells[0], disposition="SUCCESS")] + cells[1:],
                    [dict(cells[0], movement="FAKE")] + cells[1:]):
            with self.assertRaises(ValueError):
                qualification_counts(bad)
