"""RES-624 derived discrete angular-momentum check for a free rigid body (frames only).

For one body about its moving COM, dH/dt = sum (r_i - c) x F_i exactly. A
one-step discrete comparison H_{k+1} - H_k - dt*tau_k, with H = I_world(q) w,
leaves two known discretization terms of the engine's step:
  * the gyroscopic term evaluated at the step start, |dt * w_k x I_k w_k|, and
  * the change of world inertia over the step, |(I_{k+1} - I_k) w_{k+1}|.
The residual must not exceed their sum (measured ratio 0.9999998 in Gate C);
any excess is an unexported torque. The excess tolerance 1e-6*W*dt*L_ref
detects an undeclared force above one millionth of the weight at the grip
lever. Contact torques are zero for condim-3 contacts; declared external
loads are included. NUMERICAL_VERIFICATION_ONLY.
"""
from __future__ import annotations

import numpy as np

from .spatial_oracle import _forces_on


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
