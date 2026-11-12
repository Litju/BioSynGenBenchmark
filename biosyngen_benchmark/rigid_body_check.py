"""RES-624 Gate E: independent angular-momentum reference for a free rigid body.

``angular_reference_check`` integrates the rigid-body attitude dynamics of an
exported body through one engine step with an independent fourth-order
Runge-Kutta reference, using only the exported state, the exported body
inertia and the exported loads::

    I_b w_b' = R^T tau_world - w_b x I_b w_b
    q'       = 1/2 q (x) (0, w_b)

and compares the predicted world angular momentum and attitude with the
exported ones. This is a *different* numerical method from the engine (RK4 on
the attitude ODE vs MuJoCo's implicit integrator), so it detects an omitted
torque, a displaced application point, a corrupted inertia, a reversed contact
moment, an undeclared angular impulse or an altered angular velocity rather
than merely re-checking an algebraic envelope. The per-step momentum
tolerance is 1e-7 N m s: 60x the measured nominal-window residual (1.6e-9)
and equivalent to detecting an unexported 1e-4 N m torque over one millisecond
step. The attitude residual (engine first-order quaternion update; measured
2.4e-6 to 6.0e-6 on the valid perturbed/10 s/subject holds) is a *gross*
kinematic-consistency gate at 5e-5 rad, not the fault-sensitive criterion.

The older ``angular_discretization_check`` derived algebraic bound is retained
as a supplementary receipt (it is what the pre-repair evidence recorded).

NUMERICAL_VERIFICATION_ONLY.
"""
from __future__ import annotations

import math

import numpy as np

from .spatial_oracle import _forces_on

ANGULAR_REFERENCE_TOLERANCE_N_M_S = 1e-7
ATTITUDE_REFERENCE_TOLERANCE_RAD = 5e-5
REFERENCE_SUBSTEPS = 16


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])


def _quat_to_R(q):
    w, x, y, z = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def _R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t+1)*2
        w, x, y, z = .25*s, (R[2, 1]-R[1, 2])/s, (R[0, 2]-R[2, 0])/s, (R[1, 0]-R[0, 1])/s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1+R[0, 0]-R[1, 1]-R[2, 2])*2
        w, x, y, z = (R[2, 1]-R[1, 2])/s, .25*s, (R[0, 1]+R[1, 0])/s, (R[0, 2]+R[2, 0])/s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1+R[1, 1]-R[0, 0]-R[2, 2])*2
        w, x, y, z = (R[0, 2]-R[2, 0])/s, (R[0, 1]+R[1, 0])/s, .25*s, (R[1, 2]+R[2, 1])/s
    else:
        s = np.sqrt(1+R[2, 2]-R[0, 0]-R[1, 1])*2
        w, x, y, z = (R[1, 0]-R[0, 1])/s, (R[0, 2]+R[2, 0])/s, (R[1, 2]+R[2, 1])/s, .25*s
    q = np.array([w, x, y, z])
    return q/np.linalg.norm(q)


def integrate_attitude(R0, w0, inertia_body, torque_world, dt_s, substeps=REFERENCE_SUBSTEPS):
    """RK4 integration of the torque-driven rigid-body attitude over one step.

    ``inertia_body`` is the constant body-frame inertia about the COM and
    ``torque_world`` the world-frame torque applied over the step (the frame
    export's contacts, grips and external loads are fixed in world over the
    step). Returns (R, w_world).
    """
    q = _R_to_quat(np.asarray(R0, float))
    w = np.asarray(w0, float).copy()
    D = np.asarray(inertia_body, float)
    tau_world = np.asarray(torque_world, float)
    h = dt_s/substeps

    def deriv(qq, ww):
        R = _quat_to_R(qq)
        wb = R.T@ww
        dwb = np.linalg.solve(D, R.T@tau_world - np.cross(wb, D@wb))
        return .5*_quat_mul(qq, np.array([0., *wb])), R@dwb

    for _ in range(substeps):
        k1q, k1w = deriv(q, w)
        k2q, k2w = deriv(q+h/2*k1q, w+h/2*k1w)
        k3q, k3w = deriv(q+h/2*k2q, w+h/2*k2w)
        k4q, k4w = deriv(q+h*k3q, w+h*k3w)
        q = q+h/6*(k1q+2*k2q+2*k3q+k4q)
        q /= np.linalg.norm(q)
        w = w+h/6*(k1w+2*k2w+2*k3w+k4w)
    return _quat_to_R(q), w


def angular_reference_check(frames, dt_s, body="barbell", *, tolerance_n_m_s=ANGULAR_REFERENCE_TOLERANCE_N_M_S,
                            attitude_tolerance_rad=ATTITUDE_REFERENCE_TOLERANCE_RAD,
                            substeps=REFERENCE_SUBSTEPS, frame_key="bar_frame"):
    """Per-step independent attitude reference for the exported body.

    Requires the exporter's ``bar_frame`` (body position and rotation); an
    export without it is refused, not silently reduced to the algebraic bound.
    """
    if len(frames) < 2:
        return dict(code="RIGID_BODY_ANGULAR_REFERENCE", outcome="REJECTED", reason="NO_INTERVAL",
                    tolerance_n_m_s=tolerance_n_m_s, max_residual_n_m_s=float("nan"), steps=0)
    if any(frame_key not in f for f in frames):
        return dict(code="RIGID_BODY_ANGULAR_REFERENCE", outcome="REJECTED", reason="BAR_ORIENTATION_NOT_EXPORTED",
                    tolerance_n_m_s=tolerance_n_m_s, max_residual_n_m_s=float("nan"), steps=len(frames)-1)
    worst, worst_t, total, worst_attitude = 0.0, None, 0, 0.0
    for a, b in zip(frames, frames[1:]):
        A = next(x for x in a["bodies"] if x["name"] == body)
        B = next(x for x in b["bodies"] if x["name"] == body)
        R0 = np.asarray(a[frame_key]["rotation"], float)
        I0 = np.asarray(A["inertia_world"], float)
        inertia_body = R0.T@I0@R0
        c = np.asarray(A["com"], float)
        torque_world = sum((np.cross(np.asarray(p, float)-c, np.asarray(f, float))
                            for p, f in _forces_on(a, body)), np.zeros(3))
        R1, w1 = integrate_attitude(R0, np.asarray(A["w"], float), inertia_body, torque_world, dt_s, substeps)
        predicted = R1@(inertia_body@(R1.T@w1))
        exported = np.asarray(B["inertia_world"], float)@np.asarray(B["w"], float)
        residual = float(np.linalg.norm(predicted-exported))
        R_exported = np.asarray(b[frame_key]["rotation"], float)
        attitude = math.acos(float(np.clip((np.trace(R1.T@R_exported)-1.)/2., -1., 1.)))
        worst_attitude = max(worst_attitude, attitude)
        total += 1
        if residual > worst:
            worst, worst_t = residual, float(a["t"])
    momentum_ok = worst <= tolerance_n_m_s
    attitude_ok = worst_attitude <= attitude_tolerance_rad
    reason = None
    if not momentum_ok:
        reason = "ANGULAR_MOMENTUM_REFERENCE_MISMATCH"
    elif not attitude_ok:
        reason = "ANGULAR_ATTITUDE_REFERENCE_MISMATCH"
    return dict(code="RIGID_BODY_ANGULAR_REFERENCE", max_residual_n_m_s=worst, tolerance_n_m_s=tolerance_n_m_s,
                max_attitude_residual_rad=worst_attitude, attitude_tolerance_rad=attitude_tolerance_rad,
                worst_step_t_s=worst_t, steps=total, substeps=substeps,
                outcome="PASS" if momentum_ok and attitude_ok else "REJECTED", reason=reason)


def angular_discretization_check(frames, dt_s, body="barbell", lever_m=.25):
    worst_excess, worst_ratio, worst_residual = 0.0, 0.0, 0.0
    g = float(np.linalg.norm(frames[0]["gravity"]))
    weight = next(b for b in frames[0]["bodies"] if b["name"] == body)["mass"]*g
    for a, b in zip(frames, frames[1:]):
        A = next(x for x in a["bodies"] if x["name"] == body)
        B = next(x for x in b["bodies"] if x["name"] == body)
        c = np.asarray(A["com"], float)
        tau = sum((np.cross(p-c, f) for p, f in _forces_on(a, body)), np.zeros(3))
        ia, ib = np.asarray(A["inertia_world"], float), np.asarray(B["inertia_world"], float)
        wa, wb = np.asarray(A["w"], float), np.asarray(B["w"], float)
        r = float(np.linalg.norm(ib @ wb - ia @ wa - dt_s*tau))
        bound = dt_s*float(np.linalg.norm(np.cross(wa, ia @ wa))) + float(np.linalg.norm((ib-ia) @ wb))
        worst_residual = max(worst_residual, r)
        worst_excess = max(worst_excess, r-bound)
        if bound > 0:
            worst_ratio = max(worst_ratio, r/bound)
    tol = 1e-6*weight*dt_s*lever_m
    return dict(code="RIGID_BODY_ANGULAR_DISCRETE", max_residual_n_m_s=worst_residual,
                max_excess_over_discretization_n_m_s=worst_excess, max_residual_to_bound_ratio=worst_ratio,
                excess_tolerance_n_m_s=tol, outcome="PASS" if worst_excess <= tol else "REJECTED")
