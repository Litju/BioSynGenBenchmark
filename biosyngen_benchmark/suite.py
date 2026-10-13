"""Adjudicated challenge suite and claim-level accounting (RES-485).

A case is a public submission plus adjudicated expectations
{claim: "PASS" | "REJECTED" | "NOT_EVALUABLE"}. For each claim the accounting
counts:

* positives: cases expected to PASS, and how many did (true accepts),
  were falsely rejected, or abstained;
* negatives: cases expected to be REJECTED, and how many passed (false
  accepts), were rejected, or abstained;
* abstention controls: cases expected NOT_EVALUABLE.

Rates carry Wilson 95% intervals. The suite fails if any claim with a
positive control admits none of them, or if any negative control is falsely
accepted. A refuse-everything policy and an accept-everything policy are run
as baselines and must fail this gate.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable

from .i1 import CLAIMS, Finding, Report, evaluate_i1


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
    findings = tuple(Finding(c, "NOT_EVALUABLE", "POLICY_REFUSE_ALL", "baseline", "NONE") for c in submission.get("claims", []))
    return Report(submission.get("candidate_id", ""), findings, (), {})


def accept_everything(submission: dict) -> Report:
    findings = tuple(Finding(c, "PASS", "POLICY_ACCEPT_ALL", "baseline", "NONE") for c in submission.get("claims", []))
    return Report(submission.get("candidate_id", ""), findings, (), {})


def run_suite(cases: list[Case], evaluator: Callable[[dict], Report] = evaluate_i1) -> dict:
    per_claim = {
        c: {"positives": 0, "true_accept": 0, "false_reject": 0, "positive_abstain": 0,
            "negatives": 0, "true_reject": 0, "false_accept": 0, "negative_abstain": 0,
            "abstention_controls": 0, "correct_abstentions": 0}
        for c in CLAIMS
    }
    rows = []
    for case in cases:
        report = evaluator(case.submission)
        outcomes = {f.claim: f.outcome for f in report.findings}
        for claim, expected in case.expected.items():
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
    for claim, s in per_claim.items():
        s["false_accept_rate_wilson95"] = wilson(s["false_accept"], s["negatives"])
        s["false_reject_rate_wilson95"] = wilson(s["false_reject"], s["positives"])
        s["true_accept_rate_wilson95"] = wilson(s["true_accept"], s["positives"])
        if s["positives"] and s["true_accept"] == 0:
            gate_failures.append(f"{claim}: no positive control admitted")
        if s["false_accept"]:
            gate_failures.append(f"{claim}: {s['false_accept']} false accept(s)")
    totals = {
        "cases": len(cases),
        "adjudicated_claim_instances": len(rows),
        "agreements": sum(r["agree"] for r in rows),
        "disagreements": [r for r in rows if not r["agree"]],
    }
    return {"per_claim": per_claim, "rows": rows, "totals": totals,
            "gate": "PASS" if not gate_failures else "FAIL", "gate_failures": gate_failures}
