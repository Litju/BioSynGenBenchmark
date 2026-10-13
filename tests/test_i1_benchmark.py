"""RES-485 tests of the candidate-neutral I1 evaluator and its challenge suite."""

from __future__ import annotations

import ast
from pathlib import Path
import unittest

import biosyngen_benchmark
from biosyngen_benchmark import controls as C
from biosyngen_benchmark.i1 import CLAIMS, evaluate_i1
from biosyngen_benchmark.suite import Case, accept_everything, refuse_everything, run_suite, wilson


def _sub(name, kin, claims):
    return {"candidate_id": name, "claims": list(claims), "kinematics": kin}


MECH = ("NEWTON_LINEAR_MOMENTUM", "EULER_ANGULAR_MOMENTUM", "CONTACT_ADMISSIBILITY", "WORK_ENERGY_LEDGER")


class IndependenceTests(unittest.TestCase):
    def test_benchmark_modules_never_import_the_generator(self) -> None:
        root = Path(biosyngen_benchmark.__file__).parent
        for path in root.glob("*.py"):
            tree = ast.parse(path.read_text(encoding="utf-8"))
            for node in ast.walk(tree):
                names = []
                if isinstance(node, ast.Import):
                    names = [a.name for a in node.names]
                elif isinstance(node, ast.ImportFrom) and node.module:
                    names = [node.module]
                for name in names:
                    self.assertFalse(
                        name == "biosyngen" or name.startswith("biosyngen.") or name.startswith("plate2bar"),
                        f"{path.name} imports {name}",
                    )

    def test_hidden_truth_canary_is_refused_and_cannot_change_outcomes(self) -> None:
        sub = _sub("push", C.prismatic_push_control(), MECH)
        clean = evaluate_i1(sub)
        poisoned = evaluate_i1(C.hidden_truth_canary(sub))
        self.assertEqual(
            [(f.claim, f.outcome, f.code) for f in clean.findings],
            [(f.claim, f.outcome, f.code) for f in poisoned.findings],
        )
        self.assertIn("latent_events", poisoned.refused_fields)
        self.assertIn("kinematics.frames[*].phase", poisoned.refused_fields)


class PositiveAndNegativeControlTests(unittest.TestCase):
    def test_closed_form_positive_controls_pass_for_the_right_reason(self) -> None:
        push = evaluate_i1(_sub("push", C.prismatic_push_control(), MECH + ("BALLISTIC_FLIGHT",)))
        for claim in MECH:
            self.assertEqual(push.outcome(claim), "PASS", claim)
        self.assertEqual(push.outcome("BALLISTIC_FLIGHT"), "NOT_EVALUABLE")  # no flight frames
        tumble = evaluate_i1(_sub("tumble", C.tumbling_ballistic_control(),
                                  ("NEWTON_LINEAR_MOMENTUM", "EULER_ANGULAR_MOMENTUM", "BALLISTIC_FLIGHT", "WORK_ENERGY_LEDGER")))
        self.assertTrue(all(f.outcome == "PASS" for f in tumble.findings), tumble.findings)
        self.assertLess(push.finding("NEWTON_LINEAR_MOMENTUM").metrics["max_residual_over_tolerance"], 1.0)

    def test_distinct_mutations_are_rejected_on_their_target_claim(self) -> None:
        push = _sub("push", C.prismatic_push_control(), MECH)
        tumble = _sub("tumble", C.tumbling_ballistic_control(), ("BALLISTIC_FLIGHT", "WORK_ENERGY_LEDGER"))
        expectations = [
            (C.scale_contact_forces(push, 0.9), "NEWTON_LINEAR_MOMENTUM", "MOMENTUM_IMBALANCE"),
            (C.scale_body_masses(push, 1.1), "NEWTON_LINEAR_MOMENTUM", "MOMENTUM_IMBALANCE"),
            (C.inject_tensile_contact(push), "CONTACT_ADMISSIBILITY", "TENSILE_CONTACT"),
            (C.shift_contacts_outside_support(push), "CONTACT_ADMISSIBILITY", "COP_OUTSIDE_SUPPORT"),
            (C.excessive_tangential_force(push), "CONTACT_ADMISSIBILITY", "FRICTION_CONE_VIOLATED"),
            (C.inject_energy_in_flight(tumble), "BALLISTIC_FLIGHT", "NON_BALLISTIC_COM"),
        ]
        for mutated, claim, code in expectations:
            finding = evaluate_i1(mutated).finding(claim)
            self.assertEqual((finding.outcome, finding.code), ("REJECTED", code), claim)

    def test_empirical_and_non_identifiable_claims_abstain_with_their_ceilings(self) -> None:
        report = evaluate_i1({"candidate_id": "x", "claims": ["BIOLOGICAL_VALIDITY", "REAL_PAIRED_FORCE_LPT",
                                                               "SENSOR_METROLOGY", "CROSS_DEVICE_SYNC", "BILATERAL_FORCE",
                                                               "MADE_UP_CLAIM"]})
        outcome = {f.claim: (f.outcome, f.ceiling) for f in report.findings}
        self.assertEqual(outcome["BIOLOGICAL_VALIDITY"], ("NOT_EVALUABLE", "EMPIRICAL_BIOLOGICAL_VALIDATION"))
        self.assertEqual(outcome["REAL_PAIRED_FORCE_LPT"][0], "NOT_EVALUABLE")
        self.assertEqual(outcome["SENSOR_METROLOGY"][0], "NOT_EVALUABLE")
        self.assertEqual(outcome["CROSS_DEVICE_SYNC"][0], "NOT_EVALUABLE")
        self.assertEqual(outcome["BILATERAL_FORCE"][0], "NOT_EVALUABLE")
        self.assertEqual(outcome["MADE_UP_CLAIM"][0], "OUT_OF_CONTEXT")
        self.assertEqual(report.counts["admitted"], 0)
        self.assertEqual(report.counts["attempted"], 6)


class AccountingTests(unittest.TestCase):
    def test_wilson_interval_known_values(self) -> None:
        lo, hi = wilson(0, 10)
        self.assertEqual(lo, 0.0)
        self.assertAlmostEqual(hi, 0.2775, places=4)
        self.assertIsNone(wilson(0, 0))

    def test_refuse_all_and_accept_all_policies_fail_the_gate(self) -> None:
        push = _sub("push", C.prismatic_push_control(), MECH)
        cases = [
            Case("pos", "closed_form", push, {c: "PASS" for c in MECH}),
            Case("neg", "mutation", C.scale_contact_forces(push, 0.9), {"NEWTON_LINEAR_MOMENTUM": "REJECTED"}),
        ]
        self.assertEqual(run_suite(cases)["gate"], "PASS")
        self.assertEqual(run_suite(cases, refuse_everything)["gate"], "FAIL")
        self.assertEqual(run_suite(cases, accept_everything)["gate"], "FAIL")


class FullSuiteTest(unittest.TestCase):
    def test_full_adjudicated_suite_over_three_model_families(self) -> None:
        from scripts.i1_benchmark_suite import build_cases, canary_invariance

        cases, base = build_cases()
        result = run_suite(cases)
        self.assertEqual(result["gate"], "PASS", result["gate_failures"])
        self.assertEqual(result["totals"]["disagreements"], [])
        self.assertGreaterEqual(len({c.family for c in cases}), 4)
        self.assertTrue(canary_invariance(base["cmj"])["outcomes_identical"])
        for claim in CLAIMS:
            self.assertEqual(result["per_claim"][claim]["false_accept"], 0, claim)


if __name__ == "__main__":
    unittest.main()
