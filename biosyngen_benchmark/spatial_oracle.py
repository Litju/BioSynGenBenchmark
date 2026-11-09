"""Independent 3-D Newton-Euler, contact and support oracle (RES-610/612).

Operates only on plain exported frames: bodies (mass, COM, linear and
angular velocity, world inertia), contacts (point, unit normal, force on
body2, bodies, friction), grip constraints (points, force on body1, bodies),
external applied forces and gravity. It never imports the generator or a
physics engine; every balance below is recomputed with NumPy from those
numbers, so a sign, frame or omission error in the export is detectable.
World body name is "world". NUMERICAL_VERIFICATION_ONLY.
"""
from __future__ import annotations

import math

import numpy as np

WORLD = "world"


def _finite(*values):
    return all(np.all(np.isfinite(np.asarray(v, dtype=float))) for v in values)


def _forces_on(frame, body):
    """All (point, force) acting on `body` from contacts, grips and external loads."""
    out = []
    for c in frame["contacts"]:
        f = np.asarray(c["force_on_body2"], float)
        if c["body2"] == body:
            out.append((np.asarray(c["point"], float), f))
        if c["body1"] == body:
            out.append((np.asarray(c["point"], float), -f))
    for g in frame["grips"]:
        f = np.asarray(g["force_on_body1"], float)
        if g["body1"] == body:
            out.append((np.asarray(g["point1"], float), f))
        if g["body2"] == body:
            out.append((np.asarray(g["point2"], float), -f))
    for e in frame.get("external", ()):
        if e["body"] == body:
            out.append((np.asarray(e["point"], float), np.asarray(e["force"], float)))
    return out


def _external_on_system(frame, members):
    """Loads crossing the boundary of the body set `members` (world reactions, applied loads)."""
    out = []
    for c in frame["contacts"]:
        f = np.asarray(c["force_on_body2"], float)
        p = np.asarray(c["point"], float)
        if (c["body2"] in members) and (c["body1"] not in members):
            out.append((p, f))
        elif (c["body1"] in members) and (c["body2"] not in members):
            out.append((p, -f))
    for g in frame["grips"]:
        f = np.asarray(g["force_on_body1"], float)
        if (g["body1"] in members) and (g["body2"] not in members):
            out.append((np.asarray(g["point1"], float), f))
        elif (g["body2"] in members) and (g["body1"] not in members):
            out.append((np.asarray(g["point2"], float), -f))
    for e in frame.get("external", ()):
        if e["body"] in members:
            out.append((np.asarray(e["point"], float), np.asarray(e["force"], float)))
    return out


def static_balance(frame, *, members=None, tolerance_n=1e-6, tolerance_n_m=1e-6):
    """Sum of loads and moments on a body set (default: every exported body) under gravity."""
    bodies = frame["bodies"] if members is None else [b for b in frame["bodies"] if b["name"] in members]
    names = {b["name"] for b in bodies}
    if not bodies or not _finite([b["com"] for b in bodies], [b["mass"] for b in bodies]):
        return dict(outcome="INVALID", code="NONFINITE_OR_EMPTY")
    g = np.asarray(frame["gravity"], float)
    force = sum((b["mass"] * g for b in bodies), np.zeros(3))
    moment = sum((np.cross(np.asarray(b["com"], float), b["mass"] * g) for b in bodies), np.zeros(3))
    for p, f in _external_on_system(frame, names):
        force = force + f
        moment = moment + np.cross(p, f)
    weight = sum(b["mass"] for b in bodies) * float(np.linalg.norm(g))
    ok = np.linalg.norm(force) <= tolerance_n * max(1.0, weight) and np.linalg.norm(moment) <= tolerance_n_m * max(1.0, weight)
    return dict(outcome="PASS" if ok else "REJECTED", code="STATIC_NEWTON_EULER",
                residual_force_n=float(np.linalg.norm(force)), residual_moment_n_m=float(np.linalg.norm(moment)),
                weight_n=weight)


def contact_admissibility(frame, *, floor_normal=(0, 0, 1), cone_slack=1e-9):
    """Nonadhesive, inside the declared Coulomb cone, floor normals vertical."""
    problems = []
    for c in frame["contacts"]:
        n = np.asarray(c["normal"], float)
        f = np.asarray(c["force_on_body2"], float)
        if not _finite(n, f) or abs(np.linalg.norm(n) - 1) > 1e-9:
            problems.append(("NONFINITE_OR_NONUNIT_NORMAL", c["geom1"], c["geom2"]))
            continue
        fn = float(f @ n)
        ft = float(np.linalg.norm(f - fn * n))
        if fn < -cone_slack * max(1.0, np.linalg.norm(f)):
            problems.append(("ADHESIVE_NORMAL_FORCE", c["geom1"], c["geom2"], fn))
        if ft > c["friction"] * max(fn, 0.0) * (1 + 1e-6) + cone_slack:
            problems.append(("OUTSIDE_FRICTION_CONE", c["geom1"], c["geom2"], ft, fn))
        if WORLD in (c["body1"], c["body2"]) and np.linalg.norm(np.abs(n) - np.abs(np.asarray(floor_normal))) > 1e-9:
            problems.append(("FLOOR_NORMAL_NOT_VERTICAL", c["geom1"], c["geom2"], n.tolist()))
    return dict(outcome="PASS" if not problems else "REJECTED", code="CONTACT_ADMISSIBILITY", problems=problems)


def _hull(points):
    pts = sorted(set(map(tuple, points)))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def support_polygon(frame, *, margin_m=0.0):
    """Centre of pressure of world reactions inside the convex hull of loaded floor points."""
    floor = [c for c in frame["contacts"] if WORLD in (c["body1"], c["body2"])]
    loaded = [(np.asarray(c["point"], float), np.asarray(c["force_on_body2"], float) * (1 if c["body2"] != WORLD else -1))
              for c in floor]
    fz = sum(f[2] for _, f in loaded)
    if not loaded or fz <= 0:
        return dict(outcome="REJECTED", code="NO_FLOOR_SUPPORT")
    cop = sum((p[:2] * f[2] for p, f in loaded), np.zeros(2)) / fz
    hull = _hull([tuple(p[:2]) for p, f in loaded if f[2] > 1e-6])
    if len(hull) < 3:
        return dict(outcome="REJECTED", code="DEGENERATE_SUPPORT", cop=cop.tolist())
    inside = min(((b[0] - a[0]) * (cop[1] - a[1]) - (b[1] - a[1]) * (cop[0] - a[0])) / math.dist(a, b)
                 for a, b in zip(hull, hull[1:] + hull[:1]))
    return dict(outcome="PASS" if inside >= margin_m else "REJECTED", code="COP_IN_SUPPORT",
                cop=cop.tolist(), signed_margin_m=float(inside), floor_vertical_n=float(fz))


def momentum(frame, about=(0.0, 0.0, 0.0), members=None):
    bodies = frame["bodies"] if members is None else [b for b in frame["bodies"] if b["name"] in members]
    o = np.asarray(about, float)
    p = sum((b["mass"] * np.asarray(b["v"], float) for b in bodies), np.zeros(3))
    h = sum((np.asarray(b["inertia_world"], float) @ np.asarray(b["w"], float)
             + b["mass"] * np.cross(np.asarray(b["com"], float) - o, np.asarray(b["v"], float)) for b in bodies), np.zeros(3))
    return p, h


def dynamic_balance(frames, dt_s, *, members=None, about=(0.0, 0.0, 0.0)):
    """Per-step dP = h (sum F_ext + M g) and dH = h (sum r x F_ext + r x m g).

    Frame k holds the state and the loads applied over step k; frame k+1
    the state after it. Members default to every exported body.
    """
    if len(frames) < 2:
        return dict(outcome="NOT_EVALUABLE", code="NO_INTERVAL")
    names = {b["name"] for b in frames[0]["bodies"]} if members is None else set(members)
    o = np.asarray(about, float)
    worst_p = worst_h = 0.0
    total_p = np.zeros(3)
    total_h = np.zeros(3)
    g = np.asarray(frames[0]["gravity"], float)
    for a, b in zip(frames, frames[1:]):
        bodies = [x for x in a["bodies"] if x["name"] in names]
        f = sum((x["mass"] * g for x in bodies), np.zeros(3))
        mo = sum((np.cross(np.asarray(x["com"], float) - o, x["mass"] * g) for x in bodies), np.zeros(3))
        for p, force in _external_on_system(a, names):
            f = f + force
            mo = mo + np.cross(p - o, force)
        p0, h0 = momentum(a, o, names)
        p1, h1 = momentum(b, o, names)
        rp = p1 - p0 - dt_s * f
        rh = h1 - h0 - dt_s * mo
        total_p, total_h = total_p + rp, total_h + rh
        worst_p, worst_h = max(worst_p, float(np.linalg.norm(rp))), max(worst_h, float(np.linalg.norm(rh)))
    return dict(code="DISCRETE_NEWTON_EULER", max_step_linear_residual_n_s=worst_p,
                max_step_angular_residual_n_m_s=worst_h, cumulative_linear_residual_n_s=float(np.linalg.norm(total_p)),
                cumulative_angular_residual_n_m_s=float(np.linalg.norm(total_h)), steps=len(frames) - 1)


def mass_properties(frame, declared_masses, *, bar_inertia=None, tolerance=1e-9):
    """Exported masses (and the bar's principal inertia) against declared values."""
    exported = {b["name"]: b["mass"] for b in frame["bodies"]}
    problems = [(k, exported.get(k), v) for k, v in declared_masses.items()
                if k not in exported or abs(exported[k] - v) > tolerance * max(1.0, v)]
    if bar_inertia is not None:
        bar = next(b for b in frame["bodies"] if b["name"] == "barbell")
        eig = sorted(np.linalg.eigvalsh(np.asarray(bar["inertia_world"], float)))
        if any(abs(a - b) > 1e-6 * max(1.0, b) for a, b in zip(eig, sorted(bar_inertia))):
            problems.append(("barbell_inertia", eig, sorted(bar_inertia)))
    return dict(outcome="PASS" if not problems else "REJECTED", code="MASS_INERTIA", problems=problems)
