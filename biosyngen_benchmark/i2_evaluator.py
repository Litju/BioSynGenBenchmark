"""Independent evaluator for I2 multi-support exercise submissions (RES-498).

The evaluator imports nothing from the generator. It reads the plain-dict
schema BIOSYNGEN_I2_SUBMISSION_V1 strictly: unknown fields are refused and
listed, so a submission cannot supply its own tolerance, latent labels or
solver internals. Tolerances are fixed here, not by the candidate.

Claims (outcomes PASS, REJECTED, NOT_EVALUABLE, INVALID):

* LINEAR_MOMENTUM_CLOSURE: over the whole record,
  sum_i m_i v_i(end) - sum_i m_i v_i(start) equals the integral of the
  declared external contact forces (interval average x length), plus the
  external impulses, plus M g T. This uses only contacts in the declared
  external support groups, so an omitted support (bench, bar-floor) shows
  up as an imbalance. Tolerance: 1e-6 of M g T.
* CONTACT_ADMISSIBILITY: no external contact pulls (fz >= -1e-6 W), and
  each support group satisfies |sum fx| <= mu sum fz within the same tolerance.
* BILATERAL_FORCE: NOT_EVALUABLE. Per-plate channel attribution needs an
  independently admitted bilateral calibration reference, and none is admitted.
* ANATOMICAL_PHASES: NOT_EVALUABLE. Joint-phase labels need motion capture;
  a submission that carries such labels is refused.
"""

from __future__ import annotations

import math

SCHEMA_ID = "BIOSYNGEN_I2_SUBMISSION_V1"
TOP_KEYS = {"schema", "exercise_id", "gravity_m_s2", "bodies", "initial_body_states", "frames", "impulses",
            "external_support_groups", "friction_by_group", "claims"}
FRAME_KEYS = {"t0", "t1", "body_states", "contacts"}
CONTACT_KEYS = {"name", "group", "x", "z", "fx", "fz"}
IMPULSE_KEYS = {"t", "name", "group", "x", "z", "jx", "jz"}
BODY_KEYS = {"name", "mass_kg", "inertia_kg_m2"}
KNOWN_GROUPS = {"FOOT_LEFT", "FOOT_RIGHT", "BENCH", "BAR_FLOOR", "RACK", "INTERNAL", "OTHER"}
MOMENTUM_TOLERANCE = 1e-6
FORCE_TOLERANCE = 1e-6


def _finite(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _schema_problems(sub) -> list[str]:
    if not isinstance(sub, dict):
        return ["submission is not an object"]
    problems = [f"unknown field {k}" for k in sub if k not in TOP_KEYS]
    problems += [f"missing field {k}" for k in TOP_KEYS if k not in sub]
    if problems:
        return problems
    if sub["schema"] != SCHEMA_ID:
        problems.append("unsupported schema")
    if not _finite(sub["gravity_m_s2"]) or not 9.7 < sub["gravity_m_s2"] < 9.9:
        problems.append("gravity out of context")
    for body in sub["bodies"]:
        if set(body) != BODY_KEYS or not _finite(body["mass_kg"]) or body["mass_kg"] <= 0 or not _finite(body["inertia_kg_m2"]):
            problems.append("invalid body record")
    n = len(sub["bodies"])
    for state in sub["initial_body_states"]:
        if len(state) != 6 or not all(_finite(v) for v in state):
            problems.append("invalid initial state")
    if len(sub["initial_body_states"]) != n:
        problems.append("initial state count mismatch")
    previous = None
    for frame in sub["frames"]:
        if set(frame) != FRAME_KEYS:
            problems.append("unknown or missing frame field")
            break
        if not (_finite(frame["t0"]) and _finite(frame["t1"]) and frame["t1"] >= frame["t0"]):
            problems.append("invalid frame time")
            break
        if previous is not None and abs(frame["t0"] - previous) > 1e-12:
            problems.append("frames are not contiguous")
            break
        previous = frame["t1"]
        if len(frame["body_states"]) != n or not all(len(s) == 6 and all(_finite(v) for v in s) for s in frame["body_states"]):
            problems.append("nonfinite or malformed body state")
            break
        for c in frame["contacts"]:
            if set(c) != CONTACT_KEYS or c["group"] not in KNOWN_GROUPS or not all(_finite(c[k]) for k in ("x", "z", "fx", "fz")):
                problems.append("nonfinite or malformed contact force")
                break
    for imp in sub["impulses"]:
        if set(imp) != IMPULSE_KEYS or imp["group"] not in KNOWN_GROUPS or not all(_finite(imp[k]) for k in ("t", "x", "z", "jx", "jz")):
            problems.append("nonfinite or malformed impulse")
            break
    return problems


def _momentum(sub, states) -> tuple[float, float]:
    px = sum(b["mass_kg"] * s[3] for b, s in zip(sub["bodies"], states))
    pz = sum(b["mass_kg"] * s[4] for b, s in zip(sub["bodies"], states))
    return px, pz


def evaluate_i2(sub) -> dict:
    problems = _schema_problems(sub)
    if problems:
        return {"status": "INVALID", "problems": problems, "findings": {}}
    findings: dict[str, dict] = {}
    mass = sum(b["mass_kg"] for b in sub["bodies"])
    g = sub["gravity_m_s2"]
    weight = mass * g
    external = set(sub["external_support_groups"]) - {"INTERNAL"}
    t0 = sub["frames"][0]["t0"] if sub["frames"] else 0.0
    t1 = sub["frames"][-1]["t1"] if sub["frames"] else 0.0
    impulse = [0.0, -weight * (t1 - t0)]
    for frame in sub["frames"]:
        h = frame["t1"] - frame["t0"]
        for c in frame["contacts"]:
            if c["group"] in external:
                impulse[0] += c["fx"] * h
                impulse[1] += c["fz"] * h
    for imp in sub["impulses"]:
        if imp["group"] in external:
            impulse[0] += imp["jx"]
            impulse[1] += imp["jz"]
    if sub["frames"]:
        p0 = _momentum(sub, sub["initial_body_states"])
        p1 = _momentum(sub, sub["frames"][-1]["body_states"])
        scale = weight * max(t1 - t0, 1e-12)
        residual = max(abs(p1[0] - p0[0] - impulse[0]), abs(p1[1] - p0[1] - impulse[1]))
        ok = residual <= MOMENTUM_TOLERANCE * scale
        findings["LINEAR_MOMENTUM_CLOSURE"] = {
            "outcome": "PASS" if ok else "REJECTED",
            "code": "MOMENTUM_CLOSED" if ok else "MOMENTUM_IMBALANCE_OR_INCOMPLETE_SUPPORT",
            "residual_n_s": residual, "tolerance_n_s": MOMENTUM_TOLERANCE * scale,
        }
    else:
        findings["LINEAR_MOMENTUM_CLOSURE"] = {"outcome": "NOT_EVALUABLE", "code": "NO_FRAMES"}
    tension = cone = 0
    mu = sub["friction_by_group"]
    for frame in sub["frames"]:
        sums: dict[str, list[float]] = {}
        for c in frame["contacts"]:
            if c["group"] not in external:
                continue
            if c["fz"] < -FORCE_TOLERANCE * weight:
                tension += 1
            acc = sums.setdefault(c["group"], [0.0, 0.0])
            acc[0] += c["fx"]
            acc[1] += c["fz"]
        for group, (fx, fz) in sums.items():
            if group in mu and abs(fx) > mu[group] * max(fz, 0.0) + FORCE_TOLERANCE * weight:
                cone += 1
    admissible = tension == 0 and cone == 0
    findings["CONTACT_ADMISSIBILITY"] = {
        "outcome": "PASS" if admissible else "REJECTED",
        "code": "ADMISSIBLE" if admissible else ("TENSILE_CONTACT" if tension else "FRICTION_CONE_VIOLATED"),
        "tensile_intervals": tension, "cone_violations": cone,
    }
    findings["BILATERAL_FORCE"] = {"outcome": "NOT_EVALUABLE", "code": "NO_ADMITTED_BILATERAL_CALIBRATION_REFERENCE"}
    findings["ANATOMICAL_PHASES"] = {"outcome": "NOT_EVALUABLE", "code": "MOTION_CAPTURE_REQUIRED"}
    return {"status": "EVALUATED", "problems": [], "findings": findings,
            "claim_ceilings": ["NUMERICAL_VERIFICATION_ONLY", "SIMULATED_MEASUREMENT_ONLY",
                               "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED", "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED"]}


# Fault injections (operate on plain dicts; generator-independent) ------------------
def _copy(sub):
    import copy

    return copy.deepcopy(sub)


def drop_group(sub, group: str):
    out = _copy(sub)
    for frame in out["frames"]:
        frame["contacts"] = [c for c in frame["contacts"] if c["group"] != group]
    out["impulses"] = [i for i in out["impulses"] if i["group"] != group]
    out["external_support_groups"] = [g for g in out["external_support_groups"] if g != group]
    return out


def scale_forces(sub, factor: float):
    out = _copy(sub)
    for frame in out["frames"]:
        for c in frame["contacts"]:
            c["fx"] *= factor
            c["fz"] *= factor
    return out


def inject_nan(sub):
    out = _copy(sub)
    for frame in out["frames"]:
        if frame["contacts"]:
            frame["contacts"][0]["fz"] = float("nan")
            break
    return out


def inject_tension(sub):
    out = _copy(sub)
    k = len(out["frames"]) // 2
    for c in out["frames"][k]["contacts"]:
        if c["group"].startswith("FOOT"):
            c["fz"] = -50.0
            break
    return out


def add_candidate_tolerance(sub):
    out = _copy(sub)
    out["tolerance"] = 1.0
    return out


def add_anatomical_labels(sub):
    out = _copy(sub)
    out["frames"][0]["phase"] = "DOUBLE_KNEE_BEND"
    return out


FAULTS = {
    "SCALED_FORCE_1.02": (lambda s: scale_forces(s, 1.02), ("REJECTED",), "LINEAR_MOMENTUM_CLOSURE"),
    "NAN_FORCE": (inject_nan, ("INVALID",), None),
    "TENSILE_FOOT_FORCE": (inject_tension, ("REJECTED",), "CONTACT_ADMISSIBILITY"),
    "CANDIDATE_TOLERANCE_FIELD": (add_candidate_tolerance, ("INVALID",), None),
    "ANATOMICAL_LABEL_FIELD": (add_anatomical_labels, ("INVALID",), None),
}
SUPPORT_OMISSIONS = ("BENCH", "BAR_FLOOR", "RACK", "FOOT_LEFT")


def fault_outcome(result: dict, claim: str | None) -> str:
    if result["status"] == "INVALID":
        return "INVALID"
    return result["findings"][claim]["outcome"]
