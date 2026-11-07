"""Adjudicated challenge suite and claim-level accounting (RES-489).

A case is a public submission plus adjudicated expectations
{claim: "PASS" | "REJECTED" | "NOT_EVALUABLE"}. For each claim the accounting
counts:

* positives: cases expected to PASS, and how many did (true accepts),
  were falsely rejected, or abstained;
* negatives: cases expected to be REJECTED, and how many passed (false
  accepts), were rejected, or abstained;
* abstention controls: cases expected NOT_EVALUABLE;
* decision coverage, refusal rates and Wilson intervals for every denominator.

Wilson intervals describe the finite adjudicated cases; they are not evidence
of performance on unseen candidates. The gate requires every adjudicated
positive and negative to receive its expected decision and every abstention
control to abstain. Refuse-all and accept-all policies must fail.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable

from .i1 import CLAIMS, NUMERICAL, Finding, Report, evaluate_i1


@dataclass(frozen=True, slots=True)
class Case:
    name: str
    family: str
    submission: dict
    expected: dict[str, str]


def wilson(successes: int, n: int, z: float = 1.959963984540054) -> tuple[float, float] | None:
    if n == 0:
        return None
    p = successes / n
    denom = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return max(0.0, centre - half), min(1.0, centre + half)


def refuse_everything(submission: dict) -> Report:
    findings = tuple(Finding(c, "NOT_EVALUABLE", "POLICY_REFUSE_ALL", "baseline", NUMERICAL) for c in submission.get("claims", []))
    return Report(submission.get("candidate_id", ""), findings, (), {})


def accept_everything(submission: dict) -> Report:
    findings = tuple(Finding(c, "PASS", "POLICY_ACCEPT_ALL", "baseline", NUMERICAL) for c in submission.get("claims", []))
    return Report(submission.get("candidate_id", ""), findings, (), {})


def run_suite(cases: list[Case], evaluator: Callable[[dict], Report] = evaluate_i1) -> dict:
    per_claim = {
        c: {"positives": 0, "true_accept": 0, "false_reject": 0, "positive_abstain": 0,
            "negatives": 0, "true_reject": 0, "false_accept": 0, "negative_abstain": 0,
            "abstention_controls": 0, "correct_abstentions": 0}
        for c in CLAIMS
    }
    rows = []
    input_failures = []
    families = {}
    for case in cases:
        families[case.family] = families.get(case.family, 0) + 1
        try:
            report = evaluator(case.submission)
            outcomes = {f.claim: f.outcome for f in report.findings}
        except Exception as exc:  # A crash is a missing decision, never an implicit accept.
            outcomes = {}
            input_failures.append({"case": case.name, "error": type(exc).__name__})
        for claim, expected in case.expected.items():
            if claim not in per_claim:
                input_failures.append({"case": case.name, "error": "unknown adjudicated claim", "claim": claim})
                continue
            if expected not in ("PASS", "REJECTED", "NOT_EVALUABLE"):
                input_failures.append({"case": case.name, "error": "invalid expected outcome", "claim": claim,
                                       "expected": expected})
                continue
            got = outcomes.get(claim, "MISSING")
            stats = per_claim[claim]
            if expected == "PASS":
                stats["positives"] += 1
                stats["true_accept"] += got == "PASS"
                stats["false_reject"] += got == "REJECTED"
                stats["positive_abstain"] += got not in ("PASS", "REJECTED")
            elif expected == "REJECTED":
                stats["negatives"] += 1
                stats["true_reject"] += got == "REJECTED"
                stats["false_accept"] += got == "PASS"
                stats["negative_abstain"] += got not in ("PASS", "REJECTED")
            else:
                stats["abstention_controls"] += 1
                stats["correct_abstentions"] += got == "NOT_EVALUABLE"
            rows.append({"case": case.name, "family": case.family, "claim": claim,
                         "expected": expected, "observed": got, "agree": got == expected})
    gate_failures = []
    if input_failures:
        gate_failures.append(f"{len(input_failures)} invalid suite/evaluator input(s)")
    for claim, s in per_claim.items():
        s["false_accept_rate_wilson95"] = wilson(s["false_accept"], s["negatives"])
        s["false_reject_rate_wilson95"] = wilson(s["false_reject"], s["positives"])
        s["true_accept_rate_wilson95"] = wilson(s["true_accept"], s["positives"])
        decided_negative = s["true_reject"] + s["false_accept"]
        decided_positive = s["true_accept"] + s["false_reject"]
        s["positive_decision_coverage"] = decided_positive / s["positives"] if s["positives"] else None
        s["positive_decision_coverage_wilson95"] = wilson(decided_positive, s["positives"])
        s["positive_acceptance_accuracy"] = s["true_accept"] / decided_positive if decided_positive else None
        s["positive_acceptance_accuracy_wilson95"] = wilson(s["true_accept"], decided_positive)
        s["negative_decision_coverage"] = decided_negative / s["negatives"] if s["negatives"] else None
        s["negative_decision_coverage_wilson95"] = wilson(decided_negative, s["negatives"])
        s["negative_rejection_accuracy"] = s["true_reject"] / decided_negative if decided_negative else None
        s["negative_rejection_accuracy_wilson95"] = wilson(s["true_reject"], decided_negative)
        s["negative_abstention_rate"] = s["negative_abstain"] / s["negatives"] if s["negatives"] else None
        s["negative_abstention_rate_wilson95"] = wilson(s["negative_abstain"], s["negatives"])
        s["abstention_control_accuracy"] = (
            s["correct_abstentions"] / s["abstention_controls"] if s["abstention_controls"] else None
        )
        s["abstention_control_accuracy_wilson95"] = wilson(s["correct_abstentions"], s["abstention_controls"])
        if s["positives"] and s["true_accept"] != s["positives"]:
            gate_failures.append(f"{claim}: positive decisions incomplete ({s['true_accept']}/{s['positives']} accepted)")
        if s["negatives"] and s["true_reject"] != s["negatives"]:
            gate_failures.append(f"{claim}: negative decisions incomplete ({s['true_reject']}/{s['negatives']} rejected; "
                                 f"{s['negative_abstain']} abstained, {s['false_accept']} falsely accepted)")
        if s["abstention_controls"] and s["correct_abstentions"] != s["abstention_controls"]:
            gate_failures.append(f"{claim}: abstention controls incomplete "
                                 f"({s['correct_abstentions']}/{s['abstention_controls']} abstained)")
    totals = {
        "cases": len(cases),
        "adjudicated_claim_instances": len(rows),
        "agreements": sum(r["agree"] for r in rows),
        "disagreements": [r for r in rows if not r["agree"]],
    }
    return {"per_claim": per_claim, "rows": rows, "totals": totals,
            "families": sorted(families), "family_count": len(families),
            "family_case_counts": families, "input_failures": input_failures,
            "finite_suite_confidence_scope": "DESCRIPTIVE_FOR_ADJUDICATED_CASES_ONLY",
            "gate": "PASS" if not gate_failures else "FAIL", "gate_failures": gate_failures}
