"""Protocol coverage matrix over the RES-490 inventory (RES-498).

Every exercise row and deferred candidate in the pinned inventory receives an
explicit status. A row that is not listed here makes ``coverage_matrix`` raise:
silent omission is impossible. Generator entry points are recorded as dotted
names only; this module imports nothing from the generator.

Status vocabulary:

* MODELED_NUMERICAL_VERIFICATION: forward-dynamics generator plus independent
  numerical tests; not empirically validated.
* SCOPED_SUBSET_NOT_IMPLEMENTED_REMAINDER: part of the protocol is modeled;
  the remainder is refused with a computed reason.
* NOT_IMPLEMENTED: explicit refusal with a reason.

Every implemented row also carries EXTERNAL_REFERENCE_NOT_EVALUABLE: no
qualified paired human/device reference is admitted.
"""

from __future__ import annotations

from .protocol_inventory import load_inventory

IMPLEMENTED = "MODELED_NUMERICAL_VERIFICATION"
SCOPED = "SCOPED_SUBSET_NOT_IMPLEMENTED_REMAINDER"
NOT_IMPLEMENTED = "NOT_IMPLEMENTED"
EXTERNAL = "EXTERNAL_REFERENCE_NOT_EVALUABLE"

ROWS = {
    "BACK_SQUAT": (IMPLEMENTED, "biosyngen.i2.squat.build_back_squat", "RES-493", "tests.test_i2e_back_squat"),
    "COUNTERMOVEMENT_JUMP": (IMPLEMENTED, "biosyngen.i2.jumps.build_jump('CMJ')", "RES-492", "tests.test_i2d_jumps"),
    "SQUAT_JUMP": (IMPLEMENTED, "biosyngen.i2.jumps.build_jump('SJ')", "RES-492", "tests.test_i2d_jumps"),
    "LOADED_CMJ": (IMPLEMENTED, "biosyngen.i2.jumps.build_jump('LOADED_CMJ')", "RES-492", "tests.test_i2d_jumps"),
    "LOADED_JUMP_SQUAT": (IMPLEMENTED, "biosyngen.i2.jumps.build_jump('LOADED_JUMP_SQUAT')", "RES-492", "tests.test_i2d_jumps"),
    "DEADLIFT": (IMPLEMENTED, "biosyngen.i2.deadlift.build_deadlift", "RES-494", "tests.test_i2f_deadlift"),
    "BENCH_PRESS": (IMPLEMENTED, "biosyngen.i2.bench.build_bench_press", "RES-495", "tests.test_i2g_bench_press"),
    "OLYMPIC_PULL_DERIVATIVE": (IMPLEMENTED, "biosyngen.i2.olympic.build_pull (CLEAN_PULL, SNATCH_PULL)", "RES-496",
                                "tests.test_i2h_olympic_pulls"),
    "CLEAN_PULL": (IMPLEMENTED, "biosyngen.i2.olympic.build_pull('CLEAN_PULL')", "RES-496", "tests.test_i2h_olympic_pulls"),
    "SNATCH_PULL": (IMPLEMENTED, "biosyngen.i2.olympic.build_pull('SNATCH_PULL')", "RES-496", "tests.test_i2h_olympic_pulls"),
    "POWER_CLEAN": (SCOPED, "biosyngen.i2.power_clean.build_power_clean('PULL_ONLY')", "RES-497",
                    "tests.test_i2i_power_clean"),
}
DEFERRED = {
    "FULL_CLEAN": "front-rack catch infeasible in the planar lumped arm (RES-497 computation)",
    "FULL_SNATCH": "overhead receipt not modeled; no wrist or 3-D shoulder DOFs",
    "HANG_CLEAN": "hang start deferred in RES-490",
    "HANG_SNATCH": "hang start deferred in RES-490",
    "HANG_POWER_CLEAN": "hang start deferred in RES-490",
    "HANG_POWER_SNATCH": "hang start deferred in RES-490",
}
VARIANT_NOTES = {
    "CMJ_UNLOADED": "CMJ (no arm swing; matches AKIMBO topology)",
    "CMJ_UNLOADED_AKIMBO": "CMJ (planar model has no arm swing)",
    "CMJ_LOADED": "LOADED_CMJ (20 kg vest welded to the trunk)",
    "CMJ_LOADED_AKIMBO": "LOADED_CMJ",
    "CMJ_UNLOADED_FORCE_ONLY": "CMJ with FORCE_PIPE_01 export only",
    "CMJ_UNLOADED_LPT_ONLY": "CMJ with LPT_PIPE_01 export only",
    "CLEAN_PULL_DUAL_V1": "CLEAN_PULL",
    "SNATCH_PULL_DUAL_V1": "SNATCH_PULL",
}
REFUSED_JUMP_SUBTYPES = ("DROP_JUMP", "REPEATED_REBOUND_JUMP")


class CoverageGap(ValueError):
    """An inventory row has no explicit status."""


def coverage_matrix(inventory: dict | None = None) -> list[dict]:
    inv = inventory or load_inventory()
    rows = []
    for row in inv["exercise_matrix"]:
        exercise = row["exercise_id"]
        if exercise not in ROWS:
            raise CoverageGap(f"inventory exercise {exercise} has no explicit coverage status")
        status, entry, issue, tests = ROWS[exercise]
        variants = list(row.get("variant_ids", []))
        missing = [v for v in variants if v not in VARIANT_NOTES]
        if missing:
            raise CoverageGap(f"{exercise} variants without explicit mapping: {missing}")
        rows.append({
            "exercise_id": exercise,
            "protocol_ids": list(row.get("protocol_ids", [])),
            "variants": {v: VARIANT_NOTES[v] for v in variants},
            "status": status,
            "external_reference": EXTERNAL,
            "generator": entry,
            "issue": issue,
            "tests": tests,
        })
    for candidate in inv["deferred_protocol_candidates"]:
        exercise = candidate["exercise_id"]
        if exercise not in DEFERRED:
            raise CoverageGap(f"deferred candidate {exercise} has no explicit refusal")
        rows.append({"exercise_id": exercise, "protocol_ids": [], "variants": {}, "status": NOT_IMPLEMENTED,
                     "external_reference": EXTERNAL, "generator": None, "issue": "RES-498", "tests": None,
                     "reason": DEFERRED[exercise]})
    for subtype in REFUSED_JUMP_SUBTYPES:
        rows.append({"exercise_id": subtype, "protocol_ids": [], "variants": {}, "status": NOT_IMPLEMENTED,
                     "external_reference": EXTERNAL, "generator": "biosyngen.i2.jumps.build_jump (refuses)",
                     "issue": "RES-492", "tests": "tests.test_i2d_jumps",
                     "reason": "rigid point-contact feet lack qualified compliance"})
    return rows
