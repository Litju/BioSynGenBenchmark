"""RES-624 contact-force feasibility mechanics for hand-bar grasp and support.

Two linear programmes over the *inscribed* Coulomb pyramid (every pyramid
force lies inside the true elliptic cone, so a feasible LP is a conservative
witness and never accepts a force the engine's cone would reject):

``object_wrench_feasibility``
    Can nonnegative, frictionally admissible forces at the given contact
    points balance a rigid object's gravity (plus declared external loads)?
    Used to diagnose whether a contact *topology* can ever support the bar.

``generalized_equilibrium``
    Static equilibrium of a complete MuJoCo model in generalized coordinates,
    qfrc_bias - qfrc_passive = J_c^T f + B tau + J_eq^T eta, with contact
    forces in the pyramid, actuator forces inside declared force ranges,
    bilateral equality forces free and *no* other generalized force (free
    joints receive only contact and gravity terms). Returns
    STATIC_GRASP_INFEASIBLE when no admissible forces exist, ACTUATOR_INFEASIBLE
    when contact forces exist only with torque beyond capacity.

Neither programme reads a pose optimizer's residual: kinematic closure is
not an argument here. NUMERICAL_VERIFICATION_ONLY.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np
from scipy.optimize import linprog

PYRAMID_EDGES = 8
# Equality residual of the LP is reported; feasibility uses a tolerance that is
# a multiple of the floating-point scale of the generalized force vector, not
# a tuned physical allowance (HiGHS primal feasibility tolerance is 1e-7).
LP_RESIDUAL_REL = 1e-6


def pyramid_edges(normal, tangent1, tangent2, mu, k=PYRAMID_EDGES):
    """Unit-normal-component edges n + mu (cos a t1 + sin a t2), inscribed in the cone."""
    n, t1, t2 = (np.asarray(v, float) for v in (normal, tangent1, tangent2))
    return np.array([n + mu*(math.cos(2*math.pi*i/k)*t1 + math.sin(2*math.pi*i/k)*t2) for i in range(k)])


def _tangents(normal):
    n = np.asarray(normal, float)/np.linalg.norm(normal)
    a = np.array([1., 0, 0]) if abs(n[0]) < .9 else np.array([0, 1., 0])
    t1 = np.cross(n, a)
    t1 /= np.linalg.norm(t1)
    return n, t1, np.cross(n, t1)


def object_wrench_feasibility(points, normals, mus, com, mass, gravity=(0, 0, -9.81), external=(),
                              max_normal_n=None):
    """Minimum-L1 wrench residual of a rigid object supported only at contacts.

    normals point *into* the object (direction of the force the contact can
    push with). ``external`` = [(point, force)] declared non-contact loads.
    ``max_normal_n`` optionally bounds each contact's normal force.
    """
    com, g = np.asarray(com, float), np.asarray(gravity, float)
    target = -mass*g
    moment_target = np.zeros(3)
    for p, f in external:
        target = target - np.asarray(f, float)
        moment_target = moment_target - np.cross(np.asarray(p, float)-com, f)
    columns, normal_rows = [], []
    for i, (p, n, mu) in enumerate(zip(points, normals, mus)):
        n, t1, t2 = _tangents(n)
        for e in pyramid_edges(n, t1, t2, mu):
            columns.append(np.concatenate([e, np.cross(np.asarray(p, float)-com, e)]))
            normal_rows.append(i)
    weight = max(1., mass*float(np.linalg.norm(g)))
    rhs = np.concatenate([target, moment_target])
    nc = len(columns)
    if nc == 0:
        return dict(feasible=False, code="NO_CONTACTS", residual_force_n=float(np.linalg.norm(target)),
                    residual_moment_n_m=float(np.linalg.norm(moment_target)), contacts=0)
    a = np.array(columns).T
    # variables: lambda (nc) >= 0, slack+ (6), slack- (6)
    a_eq = np.hstack([a, np.eye(6), -np.eye(6)])
    cost = np.concatenate([np.zeros(nc), np.ones(12)])
    a_ub = b_ub = None
    if max_normal_n is not None:
        a_ub = np.zeros((len(points), nc+12))
        for j, i in enumerate(normal_rows):
            a_ub[i, j] = 1.
        b_ub = np.full(len(points), float(max_normal_n))
    res = linprog(cost, A_ub=a_ub, b_ub=b_ub, A_eq=a_eq, b_eq=rhs, bounds=[(0, None)]*(nc+12), method="highs")
    if res.status != 0:
        return dict(feasible=False, code="LP_FAILED", message=res.message, contacts=len(points))
    lam = res.x[:nc]
    resid = a @ lam - rhs
    normal = np.zeros(len(points))
    for j, i in enumerate(normal_rows):
        normal[i] += lam[j]
    feasible = float(np.linalg.norm(resid)) <= LP_RESIDUAL_REL*weight
    return dict(feasible=bool(feasible), code="SUPPORTABLE" if feasible else "UNSUPPORTABLE_BY_CONTACTS",
                residual_force_n=float(np.linalg.norm(resid[:3])), residual_moment_n_m=float(np.linalg.norm(resid[3:])),
                normal_forces_n=normal.tolist(), contacts=len(points))


def candidate_contacts(model, data, geoms, margin_m):
    """Contacts within ``margin_m`` for the given geoms, from the engine's own collision.

    Only detection margin is changed (and restored); geometry, eligibility
    masks and friction are the compiled ones. Returns plain rows.
    """
    ids = sorted({mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, g) if isinstance(g, str) else int(g) for g in geoms})
    old = model.geom_margin.copy()
    old_gap = model.geom_gap.copy()
    try:
        model.geom_margin[ids] = margin_m
        model.geom_gap[ids] = 0
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        mujoco.mj_collision(model, data)
        rows = []
        for i in range(data.ncon):
            c = data.contact[i]
            if c.geom1 in ids or c.geom2 in ids:
                if c.dist <= margin_m:
                    frame = c.frame.reshape(3, 3)
                    rows.append(dict(geom1=int(c.geom1), geom2=int(c.geom2),
                                     body1=int(model.geom_bodyid[c.geom1]), body2=int(model.geom_bodyid[c.geom2]),
                                     point=c.pos.copy(), normal=frame[0].copy(), t1=frame[1].copy(), t2=frame[2].copy(),
                                     mu=float(c.friction[0]), dist=float(c.dist)))
        return rows
    finally:
        model.geom_margin[:] = old
        model.geom_gap[:] = old_gap
        mujoco.mj_forward(model, data)


def _point_jacobian(model, data, body, point):
    jp = np.zeros((3, model.nv))
    jr = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jp, jr, np.asarray(point, float), int(body))
    return jp


def generalized_equilibrium(model, data, contacts, *, actuator_scale=1.0, min_normal_n=0.0):
    """Static generalized equilibrium certificate at the current qpos (qvel taken as 0).

    contacts: rows from :func:`candidate_contacts`. The contact force acts on
    body2 along +normal (MuJoCo convention) and equal-opposite on body1.
    actuator_scale scales declared force ranges (used to test reduced capacity).
    Returns a dict with code in {STATIC_EQUILIBRIUM_FEASIBLE, STATIC_GRASP_INFEASIBLE,
    ACTUATOR_INFEASIBLE} and the force/torque solution.
    """
    saved_qvel = data.qvel.copy()
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    rhs = data.qfrc_bias - data.qfrc_passive
    nv = model.nv
    cols, owner = [], []
    for i, c in enumerate(contacts):
        jac = _point_jacobian(model, data, c["body2"], c["point"]) - _point_jacobian(model, data, c["body1"], c["point"])
        for e in pyramid_edges(c["normal"], c["t1"], c["t2"], c["mu"]):
            cols.append(jac.T @ e)
            owner.append(i)
    contact_a = np.array(cols).T if cols else np.zeros((nv, 0))
    # Actuators: generalized force = moment^T * force.
    act = np.zeros((nv, model.nu))
    for a in range(model.nu):
        if model.actuator_trntype[a] != mujoco.mjtTrn.mjTRN_JOINT:
            raise ValueError("only joint actuators are certified")
        j = model.actuator_trnid[a, 0]
        act[model.jnt_dofadr[j], a] = model.actuator_gear[a, 0]
    cap = np.maximum(np.abs(model.actuator_forcerange[:, 0]), np.abs(model.actuator_forcerange[:, 1]))
    # Bilateral equality rows from the engine's constraint Jacobian.
    eq_rows = [k for k in range(data.nefc) if data.efc_type[k] == mujoco.mjtConstraint.mjCNSTR_EQUALITY]
    if eq_rows:
        jac_eq = np.zeros((data.nefc, nv))
        if mujoco.mj_isSparse(model):
            mujoco.mju_sparse2dense(jac_eq, data.efc_J, data.efc_J_rownnz, data.efc_J_rowadr, data.efc_J_colind)
        else:
            jac_eq[:] = data.efc_J[:data.nefc*nv].reshape(data.nefc, nv)
        eq_a = jac_eq[eq_rows].T
    else:
        eq_a = np.zeros((nv, 0))
    data.qvel[:] = saved_qvel
    mujoco.mj_forward(model, data)
    nc, nu, ne = contact_a.shape[1], model.nu, eq_a.shape[1]
    scale = max(1., float(np.abs(rhs).max()))

    def solve(bound_actuators, minimize_utilization):
        # variables: lambda(nc)>=0, tau(nu), eta(ne) free, slack+(nv), slack-(nv), u (utilization)
        a_eq = np.hstack([contact_a, act, eq_a, np.eye(nv), -np.eye(nv), np.zeros((nv, 1))])
        n = nc+nu+ne+2*nv+1
        bounds = [(0, None)]*nc + [(None, None)]*nu + [(None, None)]*ne + [(0, None)]*(2*nv) + [(0, None)]
        a_ub, b_ub = [], []
        for a in range(nu):  # |tau_a| <= u cap_a ; and u <= actuator_scale when bounded
            row = np.zeros(n); row[nc+a] = 1; row[-1] = -cap[a]; a_ub.append(row); b_ub.append(0)
            row = np.zeros(n); row[nc+a] = -1; row[-1] = -cap[a]; a_ub.append(row); b_ub.append(0)
        if min_normal_n > 0:
            for i in range(len(contacts)):
                row = np.zeros(n)
                for j, o in enumerate(owner):
                    if o == i:
                        row[j] = -1
                a_ub.append(row); b_ub.append(-min_normal_n)
        if bound_actuators:
            bounds[-1] = (0, actuator_scale)
        cost = np.zeros(n)
        cost[nc+nu+ne:nc+nu+ne+2*nv] = 1/scale
        if minimize_utilization:
            cost[-1] = 1e-3
        return linprog(cost, A_ub=np.array(a_ub) if a_ub else None, b_ub=np.array(b_ub) if b_ub else None, A_eq=a_eq, b_eq=rhs, bounds=bounds,
                       method="highs")

    def unpack(res):
        x = res.x
        lam, tau, eta = x[:nc], x[nc:nc+nu], x[nc+nu:nc+nu+ne]
        resid = contact_a @ lam + act @ tau + eq_a @ eta - rhs
        forces = [np.zeros(3) for _ in contacts]
        for j, o in enumerate(owner):
            c = contacts[o]
            forces[o] = forces[o] + lam[j]*pyramid_edges(c["normal"], c["t1"], c["t2"], c["mu"])[j % PYRAMID_EDGES]
        return lam, tau, eta, resid, forces

    tol = LP_RESIDUAL_REL*scale
    bounded = solve(True, True)
    if bounded.status == 0:
        lam, tau, eta, resid, forces = unpack(bounded)
        feasible = float(np.abs(resid).max()) <= tol
    else:
        feasible = False
    if not feasible:
        free = solve(False, True)
        if free.status == 0:
            lam, tau, eta, resid, forces = unpack(free)
            free_ok = float(np.abs(resid).max()) <= tol
        else:
            free_ok = False
        code = "ACTUATOR_INFEASIBLE" if free_ok else "STATIC_GRASP_INFEASIBLE"
        if free.status != 0:
            return dict(code=code, feasible=False, lp_message=free.message, contacts=len(contacts))
    else:
        code = "STATIC_EQUILIBRIUM_FEASIBLE"
    util = np.abs(tau)/cap
    return dict(code=code, feasible=code == "STATIC_EQUILIBRIUM_FEASIBLE", contacts=len(contacts),
                max_generalized_residual=float(np.abs(resid).max()), residual_tolerance=float(tol),
                actuator_force=tau.tolist(), actuator_utilization=util.tolist(),
                max_utilization=float(util.max()) if nu else 0.0, capacity_margin=float(1-util.max()) if nu else 1.0,
                equality_force=eta.tolist(), contact_forces=[f.tolist() for f in forces],
                contact_normal_n=[float(f @ c["normal"]) for f, c in zip(forces, contacts)])
