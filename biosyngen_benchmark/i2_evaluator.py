"""Independent evaluator for I2 multi-support exercise submissions (RES-498).

The evaluator imports nothing from the generator. It reads the plain-dict
schema BIOSYNGEN_I2_SUBMISSION_V1 strictly: unknown fields are refused and
listed, so a submission cannot supply its own tolerance, latent labels or
solver internals. Tolerances are fixed here, not by the candidate.

Claims (outcomes PASS, REJECTED, NOT_EVALUABLE, INVALID):

* LINEAR_MOMENTUM_CLOSURE: at every recorded interval end,
  sum_i m_i v_i(end) - sum_i m_i v_i(start) equals the integral of the
  declared external contact forces (interval average x length), plus the
  external impulses up to that end, plus M g T. This uses only contacts in the declared
  external support groups, so an omitted support (bench, bar-floor) shows
  up as an imbalance. Tolerance: 1e-6 of M g T.
* CONTACT_ADMISSIBILITY: no external contact pulls (fz >= -1e-6 W), and
  each support group satisfies |sum fx| <= mu sum fz within the same tolerance.
  Impulses are checked separately at each event time with N s tolerances.
  The schema represents vertical world support normals only; declarations
  are hypotheses, not an independently calibrated friction authority.
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
KNOWN_CLAIMS = {"LINEAR_MOMENTUM_CLOSURE", "CONTACT_ADMISSIBILITY", "BILATERAL_FORCE", "ANATOMICAL_PHASES"}


def _finite(value) -> bool:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _identity(value) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _states_valid(states, count) -> bool:
    return (isinstance(states, list) and len(states) == count
            and all(isinstance(s, list) and len(s) == 6 and all(_finite(v) for v in s) for s in states))


def _schema_problems(sub) -> list[str]:
    if not isinstance(sub, dict):
        return ["submission is not an object"]
    problems = [f"unknown field {k}" for k in sub if k not in TOP_KEYS]
    problems += [f"missing field {k}" for k in TOP_KEYS if k not in sub]
    if problems:
        return problems
    for key in ("bodies", "initial_body_states", "frames", "impulses", "external_support_groups", "claims"):
        if not isinstance(sub[key], list):
            problems.append(f"{key} must be an array")
    if not isinstance(sub["friction_by_group"], dict):
        problems.append("friction_by_group must be an object")
    if problems:
        return problems
    if sub["schema"] != SCHEMA_ID:
        problems.append("unsupported schema")
    if not _identity(sub["exercise_id"]):
        problems.append("exercise identity must be nonempty text")
    if not _finite(sub["gravity_m_s2"]) or not 9.7 < sub["gravity_m_s2"] < 9.9:
        problems.append("gravity out of context")
    if not sub["bodies"]:
        problems.append("at least one body is required")
    names = set()
    for body in sub["bodies"]:
        if (not isinstance(body, dict) or set(body) != BODY_KEYS or not _identity(body["name"])
                or not _finite(body["mass_kg"]) or body["mass_kg"] <= 0
                or not _finite(body["inertia_kg_m2"]) or body["inertia_kg_m2"] <= 0):
            problems.append("invalid body record")
            continue
        if body["name"] in names:
            problems.append("duplicate body identity")
        names.add(body["name"])
    n = len(sub["bodies"])
    if not _states_valid(sub["initial_body_states"], n):
        problems.append("invalid initial states or count")
    groups = sub["external_support_groups"]
    if not all(isinstance(g, str) and g in KNOWN_GROUPS - {"INTERNAL"} for g in groups):
        problems.append("invalid external support identity")
    elif len(set(groups)) != len(groups):
        problems.append("duplicate external support identity")
    claims = sub["claims"]
    if not claims or not all(isinstance(c, str) and c in KNOWN_CLAIMS for c in claims):
        problems.append("unknown or missing claim identity")
    elif len(set(claims)) != len(claims):
        problems.append("duplicate claim identity")
    for group, value in sub["friction_by_group"].items():
        if not isinstance(group, str) or group not in KNOWN_GROUPS - {"INTERNAL"} or not _finite(value) or value < 0:
            problems.append("invalid friction declaration")
    if any(isinstance(g, str) and g not in sub["friction_by_group"] for g in groups):
        problems.append("missing friction declaration for external support")
    if problems:
        return problems
    contact_groups = {}
    previous = None
    for frame in sub["frames"]:
        if not isinstance(frame, dict) or set(frame) != FRAME_KEYS:
            problems.append("unknown or missing frame field")
            break
        if not (_finite(frame["t0"]) and _finite(frame["t1"]) and frame["t1"] >= frame["t0"]):
            problems.append("invalid frame time")
            break
        if previous is not None and abs(frame["t0"] - previous) > 1e-12:
            problems.append("frames are not contiguous")
            break
        previous = frame["t1"]
        if not _states_valid(frame["body_states"], n):
            problems.append("nonfinite or malformed body state")
            break
        if not isinstance(frame["contacts"], list):
            problems.append("contacts must be an array")
            break
        frame_names = set()
        for c in frame["contacts"]:
            if (not isinstance(c, dict) or set(c) != CONTACT_KEYS or not _identity(c["name"])
                    or not isinstance(c["group"], str) or c["group"] not in KNOWN_GROUPS
                    or not all(_finite(c[k]) for k in ("x", "z", "fx", "fz"))):
                problems.append("nonfinite or malformed contact force")
                break
            if c["name"] in frame_names or contact_groups.get(c["name"], c["group"]) != c["group"]:
                problems.append("duplicate or inconsistent contact identity")
            frame_names.add(c["name"])
            contact_groups[c["name"]] = c["group"]
            if c["group"] != "INTERNAL" and c["group"] not in groups:
                problems.append("undeclared external contact group")
    if problems:
        return problems
    previous_impulse = None
    impulse_ids = set()
    for imp in sub["impulses"]:
        if (not isinstance(imp, dict) or set(imp) != IMPULSE_KEYS or not _identity(imp["name"])
                or not isinstance(imp["group"], str) or imp["group"] not in KNOWN_GROUPS
                or not all(_finite(imp[k]) for k in ("t", "x", "z", "jx", "jz"))):
            problems.append("nonfinite or malformed impulse")
            break
        if not sub["frames"] or not sub["frames"][0]["t0"] <= imp["t"] <= sub["frames"][-1]["t1"]:
            problems.append("impulse is outside observed record")
        if previous_impulse is not None and imp["t"] < previous_impulse:
            problems.append("impulses must be chronological")
        previous_impulse = imp["t"]
        identity = (imp["t"], imp["name"])
        if identity in impulse_ids or contact_groups.get(imp["name"], imp["group"]) != imp["group"]:
            problems.append("duplicate or inconsistent impulse identity")
        impulse_ids.add(identity)
        if imp["group"] != "INTERNAL" and imp["group"] not in groups:
            problems.append("undeclared external impulse group")
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
    # Instantaneous impact/event frames remain legal inside an observed
    # trajectory. An entirely empty or zero-duration record, however, cannot
    # establish the momentum or contact-admissibility claim by vacuity.
    if not any(frame["t1"] > frame["t0"] for frame in sub["frames"]):
        no_evidence = {"outcome": "NOT_EVALUABLE",
                       "code": "NO_POSITIVE_DURATION_OBSERVATION"}
        findings["LINEAR_MOMENTUM_CLOSURE"] = dict(no_evidence)
        findings["CONTACT_ADMISSIBILITY"] = dict(no_evidence)
        findings["BILATERAL_FORCE"] = {
            "outcome": "NOT_EVALUABLE", "code": "NO_ADMITTED_BILATERAL_CALIBRATION_REFERENCE"}
        findings["ANATOMICAL_PHASES"] = {
            "outcome": "NOT_EVALUABLE", "code": "MOTION_CAPTURE_REQUIRED"}
        return {"status": "EVALUATED", "problems": [], "findings": findings,
                "claim_ceilings": ["NUMERICAL_VERIFICATION_ONLY", "SIMULATED_MEASUREMENT_ONLY",
                                   "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED",
                                   "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED"]}
    mass = sum(b["mass_kg"] for b in sub["bodies"])
    g = sub["gravity_m_s2"]
    weight = mass * g
    external = set(sub["external_support_groups"])
    t0 = sub["frames"][0]["t0"] if sub["frames"] else 0.0
    t1 = sub["frames"][-1]["t1"] if sub["frames"] else 0.0
    scale = weight * max(t1 - t0, 1e-12)
    p0 = _momentum(sub, sub["initial_body_states"])
    if not all(_finite(v) for v in (mass, weight, scale, *p0)):
        return {"status": "INVALID", "problems": ["derived numeric overflow"], "findings": {}}
    impulse = [0.0, 0.0]
    impulse_index = 0
    residual = max_prefix_residual = 0.0
    for frame_index, frame in enumerate(sub["frames"]):
        h = frame["t1"] - frame["t0"]
        impulse[1] -= weight * h
        for c in frame["contacts"]:
            if c["group"] in external:
                impulse[0] += c["fx"] * h
                impulse[1] += c["fz"] * h
        # Co-timed frames represent instantaneous impacts, not quadrature.
        # Without frame-specific impulse tags, only the last state at a shared
        # timestamp can be checked against all impulses recorded at that time.
        while impulse_index < len(sub["impulses"]) and sub["impulses"][impulse_index]["t"] <= frame["t1"]:
            imp = sub["impulses"][impulse_index]
            if imp["group"] in external:
                impulse[0] += imp["jx"]
                impulse[1] += imp["jz"]
            impulse_index += 1
        p1 = _momentum(sub, frame["body_states"])
        residual = max(abs(p1[0] - p0[0] - impulse[0]), abs(p1[1] - p0[1] - impulse[1]))
        if not all(_finite(v) for v in (*p1, *impulse, residual)):
            return {"status": "INVALID", "problems": ["derived numeric overflow"], "findings": {}}
        co_timed_next = (frame_index + 1 < len(sub["frames"])
                        and sub["frames"][frame_index + 1]["t1"] == frame["t1"])
        if not co_timed_next:
            max_prefix_residual = max(max_prefix_residual, residual)
    if sub["frames"]:
        ok = max_prefix_residual <= MOMENTUM_TOLERANCE * scale
        findings["LINEAR_MOMENTUM_CLOSURE"] = {
            "outcome": "PASS" if ok else "REJECTED",
            "code": "MOMENTUM_CLOSED" if ok else "MOMENTUM_IMBALANCE_OR_INCOMPLETE_SUPPORT",
            "residual_n_s": residual, "max_prefix_residual_n_s": max_prefix_residual,
            "tolerance_n_s": MOMENTUM_TOLERANCE * scale,
            "scope": "recorded interval-end prefix balances; not between-sample or biological validity",
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
            if not all(_finite(v) for v in (fx, fz, mu[group] * max(fz, 0.0))):
                return {"status": "INVALID", "problems": ["derived contact overflow"], "findings": {}}
            if abs(fx) > mu[group] * max(fz, 0.0) + FORCE_TOLERANCE * weight:
                cone += 1
    impulse_tension = impulse_cone = 0
    event_sums = {}
    for imp in sub["impulses"]:
        if imp["group"] not in external:
            continue
        if imp["jz"] < -MOMENTUM_TOLERANCE * scale:
            impulse_tension += 1
        total = event_sums.setdefault((imp["t"], imp["group"]), [0.0, 0.0])
        total[0] += imp["jx"]
        total[1] += imp["jz"]
    for (_, group), (jx, jz) in event_sums.items():
        if not all(_finite(v) for v in (jx, jz, mu[group] * max(jz, 0.0))):
            return {"status": "INVALID", "problems": ["derived impulse overflow"], "findings": {}}
        if abs(jx) > mu[group] * max(jz, 0.0) + MOMENTUM_TOLERANCE * scale:
            impulse_cone += 1
    admissible = tension == cone == impulse_tension == impulse_cone == 0
    findings["CONTACT_ADMISSIBILITY"] = {
        "outcome": "PASS" if admissible else "REJECTED",
        "code": "ADMISSIBLE" if admissible else ("TENSILE_CONTACT" if tension or impulse_tension else "FRICTION_CONE_VIOLATED"),
        "tensile_intervals": tension, "cone_violations": cone,
        "tensile_impulses": impulse_tension, "impulse_cone_violations": impulse_cone,
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
