"""RES-624 independent, outcome-derived overhead-receiving acceptance (frames only).

Consumes only plain exported frames (bodies, contacts, declared external
loads, hand frames, actuator force/capacity) plus the generator's declared
static verdict. It imports neither the generator nor a physics engine; every
balance is recomputed by :mod:`biosyngen_benchmark.spatial_oracle`.

Outcome codes, checked in this order (the first failure is the verdict; all
failing categories are listed under ``failures``):
  NUMERICAL_NOT_QUALIFIED   nonfinite data, or the bar / whole-system discrete
                            Newton-Euler balance does not close with the
                            exported loads (=> an undeclared force acts)
  CONTACT_INADMISSIBLE      adhesive or outside-cone contact force, non-vertical
                            floor normal, penetration beyond the declared bound
  ACTUATOR_INFEASIBLE       actuator force beyond declared capacity, or posture
                            (non-grip) actuators saturated after settling
  GRASP_GEOMETRY_INFEASIBLE no hand-bar contact after settling begins
  GRASP_SLIP                hand-bar contact lost, or the bar moves by more than
                            one shaft radius relative to a hand frame
  RECEIVING_UNSTABLE        loss of bilateral foot support, non-foot floor
                            contact, COP or COM projection outside the support hull
  STATIC_NOT_CERTIFIED      generator's static certificate was not
                            STATIC_EQUILIBRIUM_CERTIFIED
  RECEIVING_HOLD_QUALIFIED  all of the above pass (GRASP_SUPPORTED holds)

Declared tolerances and their basis:
  * Bar / system linear balance: |dP - dt*F| <= 1e-6 * W * dt per step, i.e. any
    undeclared force above one millionth of the weight is detected; Gate C/E
    exported-load residuals are ~1e-14 N s.
  * Whole-system linear balance: cumulative |sum(dP - dt*F)| <= 1e-6 * W_total * T
    (any sustained undeclared force above 1 ppm of total weight) and per step
    <= 1e-4 * W_total * dt (a transient undeclared force above 0.01% of weight).
    The per-step allowance exists because MuJoCo's implicit integrators apply
    the velocity-implicit correction after the constraint solve, so exported
    contact forces and realized internal joint forces differ by an internal,
    zero-mean term (measured 6.6e-6 N s implicitfast, 3.5e-6 implicit, present
    with zero armature); the bar alone has no joints and closes to 1e-16.
  * Bar angular balance: the one-step residual about the moving bar COM may not
    exceed the engine step's rigid-body discretization terms dt|w x Iw| +
    |(I_k+1 - I_k) w| by more than 1e-6*W*dt*0.25 m (rigid_body_check).
  * Penetration <= 0.1 shaft radius (1.4 mm): beyond it the contact geometry no
    longer represents the declared rigid shaft/pad surfaces.
  * Slip bound = shaft radius (14 mm), as in Gate C.
NUMERICAL_VERIFICATION_ONLY.
"""
from __future__ import annotations

import math

import numpy as np

from .rigid_body_check import angular_discretization_check
from .spatial_oracle import WORLD, _hull, contact_admissibility, dynamic_balance, support_polygon

SHAFT_RADIUS_M = .014
GRIP_ACTUATORS = ("_finger_flex", "_finger_distal", "_thumb_opposition")


def _finite_frames(frames):
    for f in frames:
        for b in f["bodies"]:
            if not all(math.isfinite(v) for key in ("com", "v", "w") for v in b[key]):
                return False
        for c in f["contacts"]:
            if not all(math.isfinite(v) for v in c["force_on_body2"]):
                return False
    return True


def _hand_contacts(frame, bar):
    return [c for c in frame["contacts"] if bar in (c["body1"], c["body2"])
            and (c["body1"] if c["body2"] == bar else c["body2"])[:2] in ("l_", "r_")]


def _hull_margin(points, xy):
    hull = _hull(points)
    if len(hull) < 3:
        return -math.inf
    return min(((b[0]-a[0])*(xy[1]-a[1])-(b[1]-a[1])*(xy[0]-a[0]))/math.dist(a, b)
               for a, b in zip(hull, hull[1:]+hull[:1]))


def evaluate_receiving(frames, dt_s, static_verdict, *, bar="barbell", settle_s=.1, end_s=None,
                       shaft_radius_m=SHAFT_RADIUS_M, grip_half_width_m=.25):
    end_s = frames[-1]["t"] if end_s is None else end_s
    evidence = dict(frames=len(frames), dt_s=dt_s, window=[settle_s, end_s])
    window = [f for f in frames if settle_s <= f["t"] <= end_s]
    if len(frames) < 2 or not window or not _finite_frames(frames):
        return dict(outcome="NUMERICAL_NOT_QUALIFIED", reason="NONFINITE_OR_EMPTY", evidence=evidence)
    bodies = {b["name"]: b for b in frames[0]["bodies"]}
    weight = bodies[bar]["mass"]*float(np.linalg.norm(frames[0]["gravity"]))
    total_weight = sum(b["mass"] for b in bodies.values())*float(np.linalg.norm(frames[0]["gravity"]))
    failures = []

    def fail(code, reason):
        failures.append(dict(outcome=code, reason=reason))
    # 1. numerical / undeclared-force closure (bar alone and the whole system).
    bar_nE = dynamic_balance(frames, dt_s, members={bar}, about=frames[0]["bodies"][0]["com"])
    sys_nE = dynamic_balance(frames, dt_s, about=frames[0]["bodies"][0]["com"])
    lin_tol = 1e-6*weight*dt_s
    bar_ang = angular_discretization_check(frames, dt_s, bar, grip_half_width_m)
    sys_step_tol = 1e-4*total_weight*dt_s
    sys_cum_tol = 1e-6*total_weight*dt_s*(len(frames)-1)
    evidence.update(bar_newton_euler=bar_nE, bar_angular_discretization=bar_ang, system_newton_euler=sys_nE,
                    tolerances=dict(bar_linear_n_s=lin_tol, system_step_linear_n_s=sys_step_tol,
                                    system_cumulative_linear_n_s=sys_cum_tol))
    if bar_nE["max_step_linear_residual_n_s"] > lin_tol or bar_ang["outcome"] != "PASS":
        fail("NUMERICAL_NOT_QUALIFIED", "BAR_BALANCE_OPEN_UNDECLARED_LOAD")
    if sys_nE["max_step_linear_residual_n_s"] > sys_step_tol or sys_nE["cumulative_linear_residual_n_s"] > sys_cum_tol:
        fail("NUMERICAL_NOT_QUALIFIED", "SYSTEM_BALANCE_OPEN_UNDECLARED_LOAD")
    # 2. contact admissibility.
    problems, max_pen = [], 0.0
    for f in frames:
        problems += contact_admissibility(f)["problems"]
        max_pen = max([max_pen] + [-c["dist"] for c in f["contacts"]])
    evidence.update(contact_problems=len(problems), max_penetration_m=max_pen)
    if problems or max_pen > .1*shaft_radius_m:
        fail("CONTACT_INADMISSIBLE", (problems[:3] or "PENETRATION"))
    # 3. actuator capacity.
    over, posture_sat = 0, 0
    for f in window:
        a = f.get("actuators")
        if a is None:
            fail("ACTUATOR_INFEASIBLE", "ACTUATOR_EXPORT_MISSING")
            break
        for name, force, cap in zip(a["names"], a["force"], a["capacity"]):
            if abs(force) > cap*(1+1e-9):
                over += 1
            elif abs(force) >= cap*(1-1e-9) and not name.endswith(GRIP_ACTUATORS):
                posture_sat += 1
    evidence.update(over_capacity=over, posture_saturation_samples=posture_sat)
    if over or posture_sat:
        fail("ACTUATOR_INFEASIBLE", "CAPACITY")
    # 4. grasp: persistent hand-bar contact and bounded bar motion in each hand frame.
    counts = [len(_hand_contacts(f, bar)) for f in window]
    if counts[0] == 0 and max(counts) == 0:
        fail("GRASP_GEOMETRY_INFEASIBLE", "NO_HAND_BAR_CONTACT")
    ref = window[0]
    disp = 0.0
    for f in window:
        bar_now = next(b for b in f["bodies"] if b["name"] == bar)["com"]
        bar_ref = next(b for b in ref["bodies"] if b["name"] == bar)["com"]
        for side in ("l", "r"):
            r0 = np.asarray(ref["hand_frames"][side]["rotation"])
            p0 = np.asarray(ref["hand_frames"][side]["position"])
            r1 = np.asarray(f["hand_frames"][side]["rotation"])
            p1 = np.asarray(f["hand_frames"][side]["position"])
            disp = max(disp, float(np.linalg.norm(r1.T @ (np.asarray(bar_now)-p1) - r0.T @ (np.asarray(bar_ref)-p0))))
    support = []
    for f in window:
        fz = 0.0
        for c in _hand_contacts(f, bar):
            fz += c["force_on_body2"][2]*(1 if c["body2"] == bar else -1)
        support.append(fz/weight)
    evidence.update(min_hand_contacts=min(counts), bar_displacement_in_hand_m=disp,
                    hand_support_ratio_mean=float(np.mean(support)), hand_support_ratio_min=float(np.min(support)))
    if min(counts) == 0 or disp >= shaft_radius_m:
        fail("GRASP_SLIP", "CONTACT_LOST" if min(counts) == 0 else "BAR_MOVED_IN_HAND")
    # 5. whole-body support and stability.
    worst_cop, worst_com, floor_ratio, unstable = math.inf, math.inf, [], set()
    for f in window:
        floor = [c for c in f["contacts"] if WORLD in (c["body1"], c["body2"])]
        other = {c["body1"] if c["body2"] == WORLD else c["body2"] for c in floor}
        if any(not (o.endswith("foot") or o.endswith("forefoot") or o.endswith("toes")) for o in other):
            unstable.add("NON_FOOT_FLOOR_CONTACT")
        if not any(o.startswith("l_") for o in other) or not any(o.startswith("r_") for o in other):
            unstable.add("LOST_BILATERAL_FOOT_SUPPORT")
        sp = support_polygon(f)
        if sp["outcome"] != "PASS":
            unstable.add(sp["code"])
            continue
        worst_cop = min(worst_cop, sp["signed_margin_m"])
        pts = [tuple(c["point"][:2]) for c in floor]
        m_tot = sum(b["mass"] for b in f["bodies"])
        com = sum(np.asarray(b["com"])*b["mass"] for b in f["bodies"])/m_tot
        worst_com = min(worst_com, _hull_margin(pts, com[:2]))
        floor_ratio.append(sp["floor_vertical_n"]/total_weight)
    evidence.update(min_cop_margin_m=worst_cop, min_com_projection_margin_m=worst_com,
                    floor_vertical_over_weight_mean=float(np.mean(floor_ratio)) if floor_ratio else None)
    if worst_com < 0:
        unstable.add("COM_OUTSIDE_SUPPORT")
    for reason in sorted(unstable):
        fail("RECEIVING_UNSTABLE", reason)
    if static_verdict != "STATIC_EQUILIBRIUM_CERTIFIED":
        fail("STATIC_NOT_CERTIFIED", static_verdict)
    if failures:
        # Declared order decides the verdict; every failing category is still reported.
        return dict(outcome=failures[0]["outcome"], reason=failures[0]["reason"], failures=failures, evidence=evidence)
    return dict(outcome="RECEIVING_HOLD_QUALIFIED", grasp="GRASP_SUPPORTED", failures=[], evidence=evidence)
