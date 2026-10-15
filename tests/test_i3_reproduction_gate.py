"""RES-504 attack the reproduction gate with independently authored receipts."""
import copy
import unittest
from scripts.verify_i3_reproducibility import benchmark_gate


class ReproductionGateTests(unittest.TestCase):
    def test_all_pass_report_and_failed_counts_have_distinct_exit_gates(self):
        faults = {"SCALED_FORCE_1.02": 10, "NAN_FORCE": 10, "TENSILE_FOOT_FORCE": 10,
                  "CANDIDATE_TOLERANCE_FIELD": 10, "ANATOMICAL_LABEL_FIELD": 10, "OMIT": 26}
        report = {"positive_controls": {"passed": 10, "n": 10},
                  "fault_detection": {k: {"detected": n, "n": n} for k, n in faults.items()}}
        self.assertTrue(benchmark_gate(report))
        mutations = []
        bad = copy.deepcopy(report)
        bad["positive_controls"]["passed"] = 9
        mutations.append(bad)
        bad = copy.deepcopy(report)
        bad["fault_detection"]["OMIT"]["detected"] = 25
        mutations.append(bad)
        bad = copy.deepcopy(report)
        del bad["fault_detection"]["NAN_FORCE"]
        mutations.append(bad)
        bad = copy.deepcopy(report)
        bad["positive_controls"] = {"passed": 9, "n": 9}
        mutations.append(bad)
        for bad in mutations:
            self.assertFalse(benchmark_gate(bad))
        self.assertFalse(benchmark_gate({}))


if __name__ == "__main__":
    unittest.main()
