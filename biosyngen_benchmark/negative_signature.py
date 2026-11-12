"""RES-624 Gate D: pure-stdlib comparison of predeclared negative-case signatures.

The signature *producer* lives in :mod:`biosyngen_benchmark.failure_signature`
and needs the frame-level evaluator (NumPy). The cross-platform comparison job
installs no third-party packages, so the comparison itself is kept here with no
imports beyond the standard library.
"""
from __future__ import annotations


def reason_code(reason):
    """Stable category of an evaluator reason (strings or problem tuples)."""
    if isinstance(reason, str):
        return reason.split(":")[0][:120]
    if isinstance(reason, (list, tuple)):
        if not reason:
            return ""
        first = reason[0]
        if isinstance(first, (list, tuple)) and first:
            return str(first[0])[:120]
        return str(first).split(":")[0][:120]
    return str(reason)[:120]


def compare_case_signatures(a, b, *, onset_tolerance_s=.02, support_tolerance=.01, displacement_tolerance_m=1e-3):
    """Differences between two dynamic negative signatures (empty means equivalent)."""
    problems = []
    if not a or not b:
        return ["negative failure signature missing on one platform"]
    for key in ("expected", "observed", "reason"):
        if a.get(key) != b.get(key):
            problems.append(f"failure {key} differs: {a.get(key)} vs {b.get(key)}")
    fa, fb = a.get("first_event") or {}, b.get("first_event") or {}
    for key in ("code", "reason"):
        if fa.get(key) != fb.get(key):
            problems.append(f"first failure {key} differs: {fa.get(key)} vs {fb.get(key)}")
    if fa.get("t_s") is None or fb.get("t_s") is None:
        if fa.get("t_s") != fb.get("t_s"):
            problems.append("first failure time missing on one platform")
    elif abs(float(fa["t_s"])-float(fb["t_s"])) > onset_tolerance_s:
        problems.append(f"first failure time differs by {abs(float(fa['t_s'])-float(fb['t_s'])):.4f} s")
    # Only the onset that belongs to the required mechanism is gated; the
    # other one is a secondary consequence (e.g. actuator saturation after the
    # bar has already been released) and is reported, not compared.
    first_code = str(fa.get("code", ""))
    required_onset = ("contact_loss_s" if "SLIP" in first_code
                      else "actuator_saturation_s" if "ACTUATOR" in first_code else None)
    if required_onset:
        ta, tb = a.get(required_onset), b.get(required_onset)
        if (ta is None) != (tb is None):
            problems.append(f"{required_onset} present on one platform only: {ta} vs {tb}")
        elif ta is not None and abs(float(ta)-float(tb)) > onset_tolerance_s:
            problems.append(f"{required_onset} differs by {abs(float(ta)-float(tb)):.4f} s")
    pa, pb = a.get("qualified_prefix") or {}, b.get("qualified_prefix") or {}
    if (pa.get("outcome") == "RECEIVING_HOLD_QUALIFIED") != (pb.get("outcome") == "RECEIVING_HOLD_QUALIFIED"):
        problems.append(f"pre-failure qualification differs: {pa.get('outcome')} vs {pb.get('outcome')}")
    for key, tol in (("support_ratio", support_tolerance), ("bar_displacement_m", displacement_tolerance_m)):
        va, vb = pa.get(key), pb.get(key)
        if va is None or vb is None:
            if va != vb:
                problems.append(f"pre-failure {key} missing on one platform")
        elif abs(float(va)-float(vb)) > tol:
            problems.append(f"pre-failure {key} differs by {abs(float(va)-float(vb)):.4e} > {tol}")
    sa, sb = pa.get("support_share") or {}, pb.get("support_share") or {}
    for side in ("l", "r"):
        if side in sa and side in sb and abs(float(sa[side])-float(sb[side])) > support_tolerance:
            problems.append(f"pre-failure {side} support share differs by {abs(float(sa[side])-float(sb[side])):.4f}")
    ia, ib = a.get("identity") or {}, b.get("identity") or {}
    for key in ("head", "mujoco", "scipy", "numpy", "evaluator", "model_parameter_set_sha256", "controller"):
        if ia.get(key) != ib.get(key):
            problems.append(f"negative-case identity {key} differs")
    return problems
