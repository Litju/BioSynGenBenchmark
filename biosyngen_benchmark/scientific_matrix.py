"""Claim-specific campaign inventory; pending cells never count as passes.

No generator imports or inferred acceptance from historical coverage labels.
"""
from .i2_coverage import coverage_matrix

PRIMARY_MOVEMENTS = (
    "COUNTERMOVEMENT_JUMP", "BACK_SQUAT", "DEADLIFT", "BENCH_PRESS", "SQUAT_JUMP",
    "DROP_JUMP", "POWER_CLEAN", "POWER_SNATCH", "CLEAN", "SNATCH",
)
DOMAINS = (
    "MATHEMATICS", "NUMERICS", "MULTIBODY", "CONTACT_ENERGY", "ANATOMY",
    "NEUROMUSCULAR", "MOVEMENT", "MEASUREMENT", "IDENTIFIABILITY",
    "BIOLOGY", "RELIABILITY_UNCERTAINTY", "INTEGRITY",
)
DISPOSITIONS = frozenset(("PASS", "FAIL", "REFUSED", "NOT_IMPLEMENTED",
                          "NOT_EVALUABLE", "EVIDENCE_PENDING"))
MISSING = {
    "DROP_JUMP": "No integrated compliant impact, braking and reactive rebound model",
    "POWER_CLEAN": "Pull only; wrist/spatial shoulder, rack catch and recovery absent",
    "POWER_SNATCH": "Spatial wide-grip overhead receiving and recovery absent",
    "CLEAN": "Full-depth front-rack receiving and standing recovery absent",
    "SNATCH": "Full-depth overhead receiving and standing recovery absent",
}


def qualification_matrix():
    """Return all 120 cells plus unchanged retained variant coverage.

    These are evidence requirements, not historical test-result promotion.
    """
    cells = []
    for movement in PRIMARY_MOVEMENTS:
        for domain in DOMAINS:
            status = "EVIDENCE_PENDING"
            reason = "Fresh claim-specific final-revision evidence required"
            if domain == "BIOLOGY":
                status = "NOT_EVALUABLE"
                reason = "No licensed matched independent human/reference cohort admitted"
            elif movement in MISSING and domain in ("MULTIBODY", "CONTACT_ENERGY", "ANATOMY", "MOVEMENT"):
                status = "NOT_IMPLEMENTED"
                reason = MISSING[movement]
            cells.append(dict(movement=movement, domain=domain, disposition=status,
                              reason=reason, evidence=None))
    return dict(primary_cells=cells, retained_variants=coverage_matrix(),
                biological_validation="NOT_ESTABLISHED",
                real_paired_force_lpt_validation="NOT_ESTABLISHED")
