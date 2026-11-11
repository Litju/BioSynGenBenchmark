"""RES-624 independent, outcome-derived overhead-receiving acceptance (frames only).

Consumes only plain exported frames (bodies, contacts, declared external
loads, hand frames, actuator force/capacity) plus the generator's
declared static verdict. It imports neither the generator nor a physics engine;
every balance is recomputed by :mod:`biosyngen_benchmark.spatial_oracle` and
:mod:`biosyngen_benchmark.grasp_contract`.

Outcome codes, checked in this order (the first failure is the verdict; all
failing categories are listed under ``failures``):
  NUMERICAL_NOT_QUALIFIED      nonfinite data, or the bar / whole-system
                               discrete Newton-Euler balance does not close
                               with the exported loads (=> an undeclared force)
  CONTACT_INADMISSIBLE         adhesive, outside-cone, undeclared-friction or
                               non-vertical-floor contact, penetration beyond
                               the declared bound, or an inadmissible
                               grasp-interface contact
  ACTUATOR_INFEASIBLE          actuator force beyond declared capacity, or
                               posture (non-grip) actuators saturated
  GRASP_IDENTITY_UNADMISSIBLE  a contact claims a declared grasping body/geom
                               but the barbell, bar geom, body, geom or contact
                               point does not match the versioned contract
  GRASP_GEOMETRY_INFEASIBLE    no permitted palm/finger/thumb contact with the
                               bare shaft after settling begins
  GRASP_SLIP                   permitted contact lost, or bar motion relative
                               to the hand frames beyond the declared grip model
  GRASP_UNSUPPORTED            permitted contacts exist, but the hand wrench
                               does not carry the bar's gravito-inertial load
  GRASP_NOT_BILATERAL          both hands are not load-bearing (one hand below
                               the declared minimum support share)
  RECEIVING_UNSTABLE           loss of bilateral foot support, non-foot floor
                               contact, COP or COM projection outside the hull
  STATIC_NOT_CERTIFIED         generator's static certificate was not
                               STATIC_EQUILIBRIUM_CERTIFIED
  RECEIVING_HOLD_QUALIFIED     all of the above pass (GRASP_SUPPORTED holds)

Support is not a contact count. The evaluator distinguishes (1) geometric
palm/finger/thumb contact, (2) active load-bearing contact, (3) bilateral
support, (4) stable relative bar pose and (5) complete receiving qualification.
The load-bearing test integrates the hand-on-bar wrench over the declared
window and compares it with the bar's exact discrete momentum balance::

    m dv/dt = m g + F_hand + F_other
    dH/dt   = M_hand + M_other + (c - o) x m g

Only contacts inside the versioned grasp contract may contribute to F_hand;
every other bar contact (forearm, thigh, torso, ...) and every declared
external load is part of F_other and therefore cannot masquerade as grasp
support. The hand impulse must carry at least (1 - SUPPORT_SHORTFALL) of the
bar weight impulse, with no restriction to a 50/50 split; asymmetric bilateral
sharing is permitted and each hand only has to exceed BILATERAL_MIN_SHARE.

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
  * Grasp support: the integrated hand wrench must reproduce the bar's exact
    weight / momentum impulse to 0.1 % of W*T (force) and 0.1 % of W*L*T
    (moment). The nominal hold closes to ~1e-15 and ~5e-9 respectively.
  * Support shortfall: the hand impulse must carry >= 98 % of the bar weight
    impulse. The remaining 2 % allows the bar's own momentum change and
    bounded declared perturbations, but not a second support path.
  * Bilateral share: each hand >= 2 % of the weight impulse (3.9 N for the
    20 kg bar); the measured nominal split is 49.96/50.08 %.
  * Relative bar pose: the shaft axis may tilt at most 2 deg in a hand frame
    (the axis moves r*sin(2 deg) = 0.49 mm, 3.5 % of the shaft radius; nominal
    0.47 deg), the perpendicular relative angular rate is bounded at 0.2 rad/s
    (2.8 mm/s at the shaft surface; nominal 0.042 rad/s), axial slip at 2 mm
    (0.14 shaft radius; nominal 0.15 mm) and load-bearing contact-site
    migration at 5 mm (0.36 shaft radius; nominal 2.3 mm over 2 s). The shaft
    axis must be within 5 deg of the hand frame's declared local y grip axis
    (nominal 0.05 deg). Rotation about the shaft axis is reported and gated
    (15 deg) only when the exported bar inertia is not axisymmetric about it;
    the declared unplated shaft/sleeve body is axisymmetric, so a shaft roll is
    permitted by the declared grip rather than treated as slip. Angles use
    SO(3) geodesics with acos-on-trace wrapping, not Euler differences.
  * Bar angular balance: the one-step residual about the moving bar COM may
    not exceed the engine step's rigid-body discretization terms
    dt|w x Iw| + |(I_k+1 - I_k) w| by more than 1e-6*W*dt*0.25 m
    (rigid_body_check); it is applied after settling, where the measured
    residual-to-bound ratio is 0.018 rather than ~1 during the impact
    transient.
  * Penetration <= 0.1 shaft radius (1.4 mm): beyond it the contact geometry no
    longer represents the declared rigid shaft/pad surfaces.
  * Slip bound = shaft radius (14 mm), as in Gate C.
NUMERICAL_VERIFICATION_ONLY.
"""
from __future__ import annotations

import math

import numpy as np

from .grasp_contract import BAR_BODY, BILATERAL_MIN_SHARE, CONTRACT_VERSION, classify_frame
from .rigid_body_check import angular_discretization_check
from .spatial_oracle import WORLD, _hull, contact_admissibility, dynamic_balance, support_polygon

SHAFT_RADIUS_M = .014
GRIP_ACTUATORS = ("_finger_flex", "_finger_distal", "_thumb_opposition")
SUPPORT_SHORTFALL = .02
WRENCH_TOLERANCE = 1e-3
ACTIVE_FRAME_FRACTION = .9
# Gate B relative-pose bounds. The shaft axis may tilt by at most 2 deg in a
# hand frame (the axis moves r*sin(2 deg) = 0.49 mm, 3.5 % of the shaft radius,
# inside the pad contact); the measured nominal maximum is 0.47 deg. The
# perpendicular relative angular rate is bounded at 0.2 rad/s (2.8 mm/s at the
# shaft surface; nominal 0.042 rad/s). Axial slip is bounded at 2 mm (0.14
# shaft radius; nominal 0.15 mm) and the load-bearing contact site may migrate
# at most 5 mm in the hand frame (0.36 shaft radius; nominal 2.3 mm over 2 s).
TILT_MAX_DEG = 2.0
OMEGA_REL_PERP_MAX_RAD_S = .2
AXIAL_SLIP_MAX_M = .002
CONTACT_SITE_DRIFT_MAX_M = .005
# The declared grip seats the shaft along the hand frame's local y axis. A
# reported hand frame whose shaft axis is more than 5 deg away from that axis
# does not represent the declared hand (nominal misalignment 0.05 deg).
INITIAL_GRIP_ALIGNMENT_MAX_DEG = 5.0
# Rotation about the shaft axis is only gated when the exported bar inertia is
# not axisymmetric about that axis; the unplated shaft/sleeve body is
# axisymmetric, so the declared grip permits the shaft to roll in the hand.
TWIST_MAX_DEG_NON_AXISYMMETRIC = 15.0


def _finite_frames(frames):
    for f in frames:
        for b in f["bodies"]:
            if not all(math.isfinite(v) for key in ("com", "v", "w") for v in b[key]):
                return False
        for c in f["contacts"]:
            if not all(math.isfinite(v) for v in c["force_on_body2"]):
                return False
    return True


def _hull_margin(points, xy):
    hull = _hull(points)
    if len(hull) < 3:
        return -math.inf
    return min(((b[0]-a[0])*(xy[1]-a[1])-(b[1]-a[1])*(xy[0]-a[0]))/math.dist(a, b)
               for a, b in zip(hull, hull[1:]+hull[:1]))


def _rotation_angle(R):
    """SO(3) geodesic angle in radians, wrapped through acos on the trace."""
    return math.acos(float(np.clip((np.trace(R)-1.)/2., -1., 1.)))


def _rotation_axis(R):
    """Unit axis of a rotation matrix (the antisymmetric part, small-angle safe)."""
    v = np.array([R[2, 1]-R[1, 2], R[0, 2]-R[2, 0], R[1, 0]-R[0, 1]])
    n = float(np.linalg.norm(v))
    if n <= 1e-12:
        return np.zeros(3)
    return v/n


def _bar_axisymmetry(frame, bar=BAR_BODY):
    """True when the exported bar inertia is axisymmetric about its local y shaft axis."""
    I = np.asarray(next(b for b in frame["bodies"] if b["name"] == bar)["inertia_world"], float)
    R = np.asarray(frame["bar_frame"]["rotation"], float)
    eigvals, eigvecs = np.linalg.eigh(I)
    order = np.argsort(eigvals)
    eigvals, eigvecs = eigvals[order], eigvecs[:, order]
    axis = R[:, 1]
    aligned = abs(float(eigvecs[:, 0] @ axis)) > 1.-1e-6
    equal_transverse = abs(float(eigvals[2]-eigvals[1])) <= 1e-6*max(1., float(eigvals[2]))
    return bool(aligned and equal_transverse)


def _relative_pose(window, bar, ledgers):
    """Per-side relative bar pose, shaft-axis slip and load-site migration.

    Uses SO(3) relative rotations (no Euler angles): the shaft tilt is the angle
    between the bar's local y axis and the reference shaft axis expressed in the
    hand frame; the full relative rotation is the geodesic of
    ``R_rel(t) R_rel(0)^T``; its projection on the reference shaft axis is the
    permitted shaft-axis twist. Axial slip is the hand-frame shaft-axis
    component of the bar's position change; contact-site drift is the
    normal-force-weighted mean active contact point expressed in the hand
    frame. Angle wrapping is handled by acos on the trace.
    """
    ref = window[0]
    axis0_world = np.asarray(ref["bar_frame"]["rotation"], float)[:, 1]
    sides = ("l", "r")
    R0 = {s: np.asarray(ref["hand_frames"][s]["rotation"], float) for s in sides}
    p0 = {s: np.asarray(ref["hand_frames"][s]["position"], float) for s in sides}
    axis0_hand = {s: R0[s].T@axis0_world for s in sides}
    Rrel0 = {s: R0[s].T@np.asarray(ref["bar_frame"]["rotation"], float) for s in sides}
    q0 = {s: R0[s].T@(np.asarray(next(b for b in ref["bodies"] if b["name"] == bar)["com"], float)-p0[s]) for s in sides}
    site0 = {s: None for s in sides}
    out = dict(axisymmetric=_bar_axisymmetry(ref, bar), twist_gated=False,
               initial_grip_axis_misalignment_deg={s: 0. for s in sides},
               max_tilt_deg={s: 0. for s in sides}, max_twist_deg={s: 0. for s in sides},
               max_geodesic_deg={s: 0. for s in sides}, max_omega_rel_rad_s={s: 0. for s in sides},
               max_omega_rel_perp_rad_s={s: 0. for s in sides}, max_axial_slip_m={s: 0. for s in sides},
               max_com_displacement_m={s: 0. for s in sides}, max_contact_site_drift_m={s: 0. for s in sides},
               first={})
    if not out["axisymmetric"]:
        out["twist_gated"] = True
    for side in sides:
        axis_now = R0[side].T@axis0_world
        misalignment = math.degrees(math.acos(float(np.clip(axis_now@np.array([0., 1., 0.]), -1., 1.))))
        out["initial_grip_axis_misalignment_deg"][side] = misalignment
        if misalignment > INITIAL_GRIP_ALIGNMENT_MAX_DEG and "HAND_FRAME_GRIP_AXIS_MISALIGNED" not in out["first"]:
            out["first"]["HAND_FRAME_GRIP_AXIS_MISALIGNED"] = dict(
                t_s=float(ref["t"]), value=float(misalignment), limit=INITIAL_GRIP_ALIGNMENT_MAX_DEG, side=side)
    for f, ledger in zip(window, ledgers):
        Rbar = np.asarray(f["bar_frame"]["rotation"], float)
        axis_world = Rbar[:, 1]
        c = np.asarray(next(b for b in f["bodies"] if b["name"] == bar)["com"], float)
        wbar = np.asarray(next(b for b in f["bodies"] if b["name"] == bar)["w"], float)
        for side in sides:
            Rh = np.asarray(f["hand_frames"][side]["rotation"], float)
            ph = np.asarray(f["hand_frames"][side]["position"], float)
            wh = np.asarray(next(b for b in f["bodies"] if b["name"] == side+"_hand")["w"], float)
            axis_now = Rh.T@axis_world
            tilt = math.degrees(math.acos(float(np.clip(axis_now@axis0_hand[side], -1., 1.))))
            Rd = (Rh.T@Rbar)@Rrel0[side].T
            geodesic = math.degrees(_rotation_angle(Rd))
            twist = geodesic*float(_rotation_axis(Rd)@axis0_hand[side])
            q = Rh.T@(c-ph)
            axial = float((q-q0[side])@axis0_hand[side])
            disp = float(np.linalg.norm(q-q0[side]))
            wrel = wbar-wh
            wperp = wrel-float(wrel@axis_world)*axis_world
            rows = [r for r in ledger["contacts"] if r["side"] == side and r["active"]]
            drift = 0.
            if rows:
                weights = np.array([max(r["normal_force"], 0.) for r in rows])
                pts = np.array([np.asarray(r["point"], float) for r in rows])
                site = Rh.T@((pts*weights[:, None]).sum(0)/weights.sum()-ph)
                if site0[side] is None:
                    site0[side] = site
                drift = float(np.linalg.norm(site-site0[side]))
            out["max_tilt_deg"][side] = max(out["max_tilt_deg"][side], tilt)
            out["max_twist_deg"][side] = max(out["max_twist_deg"][side], abs(twist))
            out["max_geodesic_deg"][side] = max(out["max_geodesic_deg"][side], geodesic)
            out["max_omega_rel_rad_s"][side] = max(out["max_omega_rel_rad_s"][side], float(np.linalg.norm(wrel)))
            out["max_omega_rel_perp_rad_s"][side] = max(out["max_omega_rel_perp_rad_s"][side], float(np.linalg.norm(wperp)))
            out["max_axial_slip_m"][side] = max(out["max_axial_slip_m"][side], abs(axial))
            out["max_com_displacement_m"][side] = max(out["max_com_displacement_m"][side], disp)
            out["max_contact_site_drift_m"][side] = max(out["max_contact_site_drift_m"][side], drift)
            for key, value, limit in (("RELATIVE_ROTATION_EXCEEDS_GRIP", tilt, TILT_MAX_DEG),
                                      ("RELATIVE_ANGULAR_VELOCITY_EXCEEDS_GRIP", float(np.linalg.norm(wperp)),
                                       OMEGA_REL_PERP_MAX_RAD_S),
                                      ("AXIAL_SLIP_EXCEEDS_GRIP", abs(axial), AXIAL_SLIP_MAX_M),
                                      ("CONTACT_SITE_SLIP_EXCEEDS_GRIP", drift, CONTACT_SITE_DRIFT_MAX_M)):
                if value > limit and key not in out["first"]:
                    out["first"][key] = dict(t_s=float(f["t"]), value=float(value), limit=float(limit), side=side)
            if out["twist_gated"] and abs(twist) > TWIST_MAX_DEG_NON_AXISYMMETRIC and "SHAFT_TWIST_EXCEEDS_GRIP" not in out["first"]:
                out["first"]["SHAFT_TWIST_EXCEEDS_GRIP"] = dict(t_s=float(f["t"]), value=float(abs(twist)),
                                                                limit=TWIST_MAX_DEG_NON_AXISYMMETRIC, side=side)
    out["limits"] = dict(tilt_deg=TILT_MAX_DEG, omega_rel_perp_rad_s=OMEGA_REL_PERP_MAX_RAD_S,
                         axial_slip_m=AXIAL_SLIP_MAX_M, contact_site_drift_m=CONTACT_SITE_DRIFT_MAX_M,
                         twist_deg=TWIST_MAX_DEG_NON_AXISYMMETRIC if out["twist_gated"] else None)
    return out


def _support_balance(window, dt_s, bar, weight, ledgers, grip_half_width_m):
    """Integrate the hand-on-bar wrench against the bar's exact discrete balance.

    ``window`` frames are consecutive, and ``ledgers[k]`` classifies frame k.
    Force balance is exact about any origin; the angular balance is taken about
    the bar COM, where dH/dt = M exactly (the moving-point correction
    v x m v vanishes). The *support* ratio is the separate physical statement
    that the permitted hand contacts carried the bar's weight impulse.
    """
    g = np.asarray(window[0]["gravity"], float)
    body0 = next(b for b in window[0]["bodies"] if b["name"] == bar)
    mass = float(body0["mass"])
    total_t = float(window[-1]["t"]-window[0]["t"])
    j_hand = np.zeros(3)
    j_other = np.zeros(3)
    k_hand = np.zeros(3)
    k_other = np.zeros(3)
    sides = {s: 0.0 for s in ("l", "r")}
    active_frames = {s: 0 for s in ("l", "r")}
    per_frame_ratio = []
    worst_force_ratio = worst_moment_ratio = 0.0
    first_wrench_failure = None
    for k, (a, b) in enumerate(zip(window, window[1:])):
        ledger = ledgers[k]
        valid = {r["index"] for r in ledger["contacts"] if r["admissible"]}
        hand_rows = [r for r in ledger["contacts"] if r["admissible"]]
        A = next(x for x in a["bodies"] if x["name"] == bar)
        B = next(x for x in b["bodies"] if x["name"] == bar)
        c_a = np.asarray(A["com"], float)
        f_hand = sum((np.asarray(r["force_on_bar"], float) for r in hand_rows), np.zeros(3))
        m_hand = sum((np.cross(np.asarray(r["point"], float)-c_a, np.asarray(r["force_on_bar"], float))
                      for r in hand_rows), np.zeros(3))
        f_other = np.zeros(3)
        m_other = np.zeros(3)
        for index, c in enumerate(a["contacts"]):
            if bar not in (c["body1"], c["body2"]) or index in valid:
                continue
            force = np.asarray(c["force_on_body2"], float)
            if c["body2"] != bar:
                force = -force
            point = np.asarray(c["point"], float)
            f_other = f_other + force
            m_other = m_other + np.cross(point-c_a, force)
        for e in a.get("external", ()):
            if e["body"] == bar:
                force = np.asarray(e["force"], float)
                f_other = f_other + force
                m_other = m_other + np.cross(np.asarray(e["point"], float)-c_a, force)
        accel = (np.asarray(B["v"], float)-np.asarray(A["v"], float))/dt_s
        f_required = mass*accel - mass*g - f_other
        h_a = np.asarray(A["inertia_world"], float)@np.asarray(A["w"], float)
        h_b = np.asarray(B["inertia_world"], float)@np.asarray(B["w"], float)
        m_required = (h_b-h_a)/dt_s - m_other
        force_ratio = float(np.linalg.norm(f_hand-f_required))/max(weight, 1e-9)
        moment_ratio = float(np.linalg.norm(m_hand-m_required))/max(weight*grip_half_width_m, 1e-9)
        worst_force_ratio = max(worst_force_ratio, force_ratio)
        worst_moment_ratio = max(worst_moment_ratio, moment_ratio)
        if first_wrench_failure is None and force_ratio > WRENCH_TOLERANCE:
            first_wrench_failure = dict(code="GRASP_UNSUPPORTED", reason="SUPPORT_FORCE_WRENCH_MISMATCH",
                                        t_s=float(window[k]["t"]))
        if first_wrench_failure is None and moment_ratio > WRENCH_TOLERANCE:
            first_wrench_failure = dict(code="GRASP_UNSUPPORTED", reason="SUPPORT_MOMENT_WRENCH_MISMATCH",
                                        t_s=float(window[k]["t"]))
        j_hand = j_hand + dt_s*f_hand
        j_other = j_other + dt_s*f_other
        k_hand = k_hand + dt_s*m_hand
        k_other = k_other + dt_s*m_other
        per_frame_ratio.append(f_hand[2]/weight)
        for side in sides:
            rows = [r for r in hand_rows if r["side"] == side]
            f_side = sum((np.asarray(r["force_on_bar"], float) for r in rows), np.zeros(3))
            sides[side] += dt_s*f_side[2]
            if any(r["active"] for r in rows):
                active_frames[side] += 1
    weight_impulse = weight*total_t
    j_required = mass*(np.asarray(next(x for x in window[-1]["bodies"] if x["name"] == bar)["v"], float)
                       - np.asarray(next(x for x in window[0]["bodies"] if x["name"] == bar)["v"], float)) \
        - mass*g*total_t - j_other
    impulse_res = float(np.linalg.norm(j_hand-j_required))
    A0 = next(x for x in window[0]["bodies"] if x["name"] == bar)
    A1 = next(x for x in window[-1]["bodies"] if x["name"] == bar)
    h0 = np.asarray(A0["inertia_world"], float)@np.asarray(A0["w"], float)
    h1 = np.asarray(A1["inertia_world"], float)@np.asarray(A1["w"], float)
    k_required = (h1-h0) - k_other
    moment_impulse_res = float(np.linalg.norm(k_hand-k_required))
    support_ratio = float(j_hand[2]/weight_impulse) if weight_impulse else 0.0
    shares = {s: float(sides[s]/weight_impulse) if weight_impulse else 0.0 for s in sides}
    return dict(hand_impulse_n_s=j_hand.tolist(), other_impulse_n_s=j_other.tolist(),
                required_impulse_n_s=j_required.tolist(), weight_impulse_n_s=float(weight_impulse),
                support_ratio=support_ratio, support_share=shares,
                support_force_residual_ratio=impulse_res/weight_impulse if weight_impulse else 0.0,
                support_moment_residual_ratio=moment_impulse_res/(weight*grip_half_width_m*total_t) if weight_impulse else 0.0,
                worst_step_force_residual_ratio=worst_force_ratio, worst_step_moment_residual_ratio=worst_moment_ratio,
                hand_support_ratio_min=float(min(per_frame_ratio)) if per_frame_ratio else None,
                hand_support_ratio_mean=float(np.mean(per_frame_ratio)) if per_frame_ratio else None,
                active_frame_fraction={s: active_frames[s]/max(1, len(window)) for s in sides},
                first_wrench_failure=first_wrench_failure)


def evaluate_receiving(frames, dt_s, static_verdict, *, bar="barbell", settle_s=.1, end_s=None,
                       shaft_radius_m=SHAFT_RADIUS_M, grip_half_width_m=.25):
    end_s = frames[-1]["t"] if end_s is None else end_s
    evidence = dict(frames=len(frames), dt_s=dt_s, window=[settle_s, end_s],
                    grasp_contract=CONTRACT_VERSION)
    window = [f for f in frames if settle_s <= f["t"] <= end_s]
    if len(frames) < 2 or not window or not _finite_frames(frames):
        return dict(outcome="NUMERICAL_NOT_QUALIFIED", reason="NONFINITE_OR_EMPTY", evidence=evidence)
    missing = sorted({k for f in window for k in ("hand_frames", "bar_frame") if k not in f})
    if missing:
        return dict(outcome="NUMERICAL_NOT_QUALIFIED", reason="FRAME_SCHEMA_MISSING:" + ",".join(missing),
                    evidence=evidence)
    bodies = {b["name"]: b for b in frames[0]["bodies"]}
    weight = bodies[bar]["mass"]*float(np.linalg.norm(frames[0]["gravity"]))
    total_weight = sum(b["mass"] for b in bodies.values())*float(np.linalg.norm(frames[0]["gravity"]))
    failures = []
    first_events = []

    def fail(code, reason, t_s=None):
        failures.append(dict(outcome=code, reason=reason))
        if t_s is not None:
            first_events.append(dict(code=code, reason=str(reason), t_s=float(t_s)))
    # 1. numerical / undeclared-force closure (bar alone and the whole system).
    bar_nE = dynamic_balance(frames, dt_s, members={bar}, about=frames[0]["bodies"][0]["com"])
    sys_nE = dynamic_balance(frames, dt_s, about=frames[0]["bodies"][0]["com"])
    lin_tol = 1e-6*weight*dt_s
    bar_ang = angular_discretization_check(window, dt_s, bar)
    bar_ang["tolerance_n_m_s"] = bar_ang["excess_tolerance_n_m_s"]
    sys_step_tol = 1e-4*total_weight*dt_s
    sys_cum_tol = 1e-6*total_weight*dt_s*(len(frames)-1)
    evidence.update(bar_newton_euler=bar_nE, bar_angular_discretization=bar_ang, system_newton_euler=sys_nE,
                    tolerances=dict(bar_linear_n_s=lin_tol, system_step_linear_n_s=sys_step_tol,
                                    system_cumulative_linear_n_s=sys_cum_tol,
                                    bar_angular_n_m_s=bar_ang["tolerance_n_m_s"]))
    if bar_nE["max_step_linear_residual_n_s"] > lin_tol:
        fail("NUMERICAL_NOT_QUALIFIED", "BAR_BALANCE_OPEN_UNDECLARED_LOAD")
    if bar_ang["outcome"] != "PASS":
        fail("NUMERICAL_NOT_QUALIFIED", bar_ang.get("reason", "BAR_ANGULAR_DISCRETIZATION_MISMATCH"))
    if sys_nE["max_step_linear_residual_n_s"] > sys_step_tol or sys_nE["cumulative_linear_residual_n_s"] > sys_cum_tol:
        fail("NUMERICAL_NOT_QUALIFIED", "SYSTEM_BALANCE_OPEN_UNDECLARED_LOAD")
    # 2. contact admissibility.
    problems, max_pen = [], 0.0
    first_contact_t = None
    for f in frames:
        frame_problems = contact_admissibility(f)["problems"]
        frame_pen = max([-c["dist"] for c in f["contacts"]], default=0.)
        if (frame_problems or frame_pen > .1*shaft_radius_m) and first_contact_t is None:
            first_contact_t = float(f["t"])
        problems += frame_problems
        max_pen = max(max_pen, frame_pen)
    evidence.update(contact_problems=len(problems), max_penetration_m=max_pen)
    if problems or max_pen > .1*shaft_radius_m:
        fail("CONTACT_INADMISSIBLE", (problems[:3] or "PENETRATION"), first_contact_t)
    # 3. actuator capacity.
    over, posture_sat = 0, 0
    first_capacity_t = None
    for f in window:
        a = f.get("actuators")
        if a is None:
            fail("ACTUATOR_INFEASIBLE", "ACTUATOR_EXPORT_MISSING", f["t"])
            break
        frame_capacity = False
        for name, force, cap in zip(a["names"], a["force"], a["capacity"]):
            if abs(force) > cap*(1+1e-9):
                over += 1
                frame_capacity = True
            elif abs(force) >= cap*(1-1e-9) and not name.endswith(GRIP_ACTUATORS):
                posture_sat += 1
                frame_capacity = True
        if frame_capacity and first_capacity_t is None:
            first_capacity_t = float(f["t"])
    evidence.update(over_capacity=over, posture_saturation_samples=posture_sat)
    if over or posture_sat:
        fail("ACTUATOR_INFEASIBLE", "CAPACITY", first_capacity_t)
    # 4. grasp identity, geometry, relative slip, load-bearing support and bilateral share.
    ledgers = [classify_frame(f, bar) for f in window]
    identity_problems = sorted({str(p) for led in ledgers for p in led["identity_problems"]})
    contract_problems = sorted({str((r["geom"], p)) for led in ledgers for r in led["contacts"] for p in r["problems"]})
    nongrasp_max = max((len(led["nongrasp"]) for led in ledgers), default=0)
    geometric = [len(led["contacts"]) for led in ledgers]
    active = [sum(1 for r in led["contacts"] if r["active"]) for led in ledgers]
    active_side = {s: [sum(1 for r in led["contacts"] if r["active"] and r["side"] == s) for led in ledgers] for s in ("l", "r")}
    evidence.update(grasp_contract=CONTRACT_VERSION, grasp_identity_problems=identity_problems,
                    grasp_contract_problems=contract_problems, nongrasp_bar_contacts_max=nongrasp_max,
                    min_grasp_contacts=min(geometric) if geometric else 0,
                    min_grasp_active_contacts=min(active) if active else 0,
                    min_grasp_active_contacts_left=min(active_side["l"]) if active_side["l"] else 0,
                    min_grasp_active_contacts_right=min(active_side["r"]) if active_side["r"] else 0,
                    min_hand_contacts=min(geometric) if geometric else 0)
    if contract_problems:
        fail("CONTACT_INADMISSIBLE", contract_problems[:3])
    if identity_problems:
        fail("GRASP_IDENTITY_UNADMISSIBLE", identity_problems[:3])
        # A forged grasp identity can only ever be refused; do not let a later
        # support test turn the verdict into something weaker.
    if max(geometric) == 0:
        fail("GRASP_GEOMETRY_INFEASIBLE", "NO_PERMITTED_GRASP_CONTACT")
    # Relative bar pose in each hand frame (SO(3) orientation, axial slip and
    # load-bearing contact-site migration, not COM translation alone).
    pose = _relative_pose(window, bar, ledgers)
    evidence["relative_bar_pose"] = pose
    disp = max(pose["max_com_displacement_m"].values())
    support = _support_balance(window, dt_s, bar, weight, ledgers, grip_half_width_m)
    evidence.update(min_hand_contacts=min(geometric) if geometric else 0, bar_displacement_in_hand_m=disp,
                    hand_support_ratio_mean=support["hand_support_ratio_mean"],
                    hand_support_ratio_min=support["hand_support_ratio_min"],
                    hand_support_impulse_ratio=support["support_ratio"],
                    hand_support_share=support["support_share"],
                    hand_support_force_residual_ratio=support["support_force_residual_ratio"],
                    hand_support_moment_residual_ratio=support["support_moment_residual_ratio"],
                    hand_active_frame_fraction=support["active_frame_fraction"],
                    grasp_impulse=support)
    if min(geometric) == 0:
        t = next(f["t"] for f, n in zip(window, geometric) if n == 0)
        fail("GRASP_SLIP", "CONTACT_LOST", t)
    elif disp >= shaft_radius_m:
        fail("GRASP_SLIP", "BAR_MOVED_IN_HAND")
    elif pose["first"]:
        reason = next(iter(pose["first"]))
        ev = pose["first"][reason]
        fail("GRASP_SLIP", reason, ev["t_s"])
    elif support["first_wrench_failure"]:
        ev = support["first_wrench_failure"]
        fail("GRASP_UNSUPPORTED", ev["reason"], ev["t_s"])
    elif (support["support_ratio"] < 1-SUPPORT_SHORTFALL
          or support["support_force_residual_ratio"] > WRENCH_TOLERANCE
          or support["support_moment_residual_ratio"] > WRENCH_TOLERANCE):
        fail("GRASP_UNSUPPORTED", "INSUFFICIENT_HAND_SUPPORT", window[0]["t"])
    elif any(support["support_share"][s] < BILATERAL_MIN_SHARE for s in ("l", "r")):
        fail("GRASP_NOT_BILATERAL", "UNILATERAL_OR_NON_LOAD_BEARING_HAND")
    elif any(support["active_frame_fraction"][s] < ACTIVE_FRAME_FRACTION for s in ("l", "r")):
        fail("GRASP_NOT_BILATERAL", "HAND_NOT_CONTINUOUSLY_LOAD_BEARING")
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
    evidence["first_events"] = first_events
    if failures:
        # Declared order decides the verdict; every failing category is still reported.
        first = failures[0]
        first_event = next((e for e in first_events if e["code"] == first["outcome"]), None)
        return dict(outcome=first["outcome"], reason=first["reason"], failures=failures, evidence=evidence,
                    first_failure=first_event)
    return dict(outcome="RECEIVING_HOLD_QUALIFIED", grasp="GRASP_SUPPORTED", failures=[], evidence=evidence,
                first_failure=None)
