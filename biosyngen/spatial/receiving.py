"""Static receiving configurations and short perturbed holds (RES-610).

FRONT_RACK: bar on both anterior-deltoid shelves with both hands on the bar.
OVERHEAD: bar carried by both hands with arms overhead; no shoulder contact.
Depth: POWER if the hip joint centre is above the knee (thigh inclination
> +0.05 rad), FULL if below (< -0.05 rad); the band between is AMBIGUOUS and
refused as either label. This is an explicit model criterion with a stated
band, not a measured athlete classification.

A configuration is FEASIBLE only if (i) the pose solve closes feet, grips and
(front rack) shelf contact within tolerance, inside declared joint ranges
with margin; (ii) the static quasi-equilibrium QP finds nonnegative contact
forces inside the friction pyramid, free grip forces and actuator torques
within declared capacity that balance gravity on *both* free bodies (athlete
root and bar) with no residual; (iii) a forward simulation holding the pose
with bounded torques stays upright under a perturbation. A static pose is
not a clean or a snatch; no catch is simulated here.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import time

import mujoco
import numpy as np
from scipy.optimize import least_squares, minimize

from .model import ARM, GIRDLE, LEG, MIRROR, SIDES, AthleteBarbellSpec, grip_tags, joint_specs
from .plant import SpatialPlant

RECEIVING = ("FRONT_RACK", "OVERHEAD")
DEPTHS = ("POWER", "FULL")
RANGE_MARGIN_RAD = 0.03
POSE_TOLERANCE_M = 1e-4
DEPTH_BAND_RAD = 0.05

# Symmetric pose variables (left value copied to the right side).
SIDE_JOINTS = tuple(j.name for j in LEG + GIRDLE + ARM)
VARIABLES = ("pelvis_x", "pelvis_z", "pelvis_pitch", "lumbar_flex", *SIDE_JOINTS, "bar_x", "bar_z")

NOMINAL = {
    ("FRONT_RACK", "FULL"): dict(pelvis_x=-.12, pelvis_z=.46, pelvis_pitch=.45, lumbar_flex=-.1, hip_flex=1.75,
                                 knee_flex=2.2, ankle_dorsi=.6, sh_flex=1.6, sh_abd=.35, sh_rot=.6, elbow_flex=2.3,
                                 forearm_sup=.4, wrist_flex=-1.1, bar_x=.1, bar_z=1.0),
    ("FRONT_RACK", "POWER"): dict(pelvis_x=-.18, pelvis_z=.66, pelvis_pitch=.55, lumbar_flex=-.1, hip_flex=1.3,
                                  knee_flex=1.5, ankle_dorsi=.45, sh_flex=1.6, sh_abd=.35, sh_rot=.6, elbow_flex=2.3,
                                  forearm_sup=.4, wrist_flex=-1.1, bar_x=.05, bar_z=1.2),
    ("OVERHEAD", "FULL"): dict(pelvis_x=-.10, pelvis_z=.47, pelvis_pitch=.35, lumbar_flex=-.2, hip_flex=1.75,
                               knee_flex=2.2, ankle_dorsi=.6, sh_flex=2.7, sh_abd=.5, sh_rot=0.0, elbow_flex=.05,
                               wrist_flex=-.6, bar_x=.06, bar_z=1.65),
    ("OVERHEAD", "POWER"): dict(pelvis_x=-.15, pelvis_z=.67, pelvis_pitch=.5, lumbar_flex=-.2, hip_flex=1.3,
                                knee_flex=1.5, ankle_dorsi=.45, sh_flex=2.7, sh_abd=.5, sh_rot=0.0, elbow_flex=.05,
                                wrist_flex=-.6, bar_x=.06, bar_z=1.85),
}
# Declared soft targets: thigh inclination (rad, + = hip above knee) and trunk
# inclination from vertical (rad). Hypotheses for a receiving posture, not data.
THIGH_TARGET = {"FULL": -.15, "POWER": .35}
TRUNK_TARGET = {"FRONT_RACK": .30, "OVERHEAD": .25}
TRUNK_LIMIT_RAD = .60          # refused beyond: not an upright receiving posture
CLOSURE_WEIGHT = 20000.0
# Declared hand capacities (ASSUMED_UNCALIBRATED): per-site connect force
# component bound and the moment a hand may transmit through its two sites.
GRIP_SITE_SPACING_M = .07
GRIP_MOMENT_CAPACITY_N_M = 15.0
GRIP_FORCE_CAPACITY_N = 900.0
BAR_CLEARANCE_M = .005          # required bar-to-body clearance off declared supports


def declared_support(receiving, geom1, geom2):
    """Contacts a receiving configuration may use as load paths."""
    pair = {geom1, geom2}
    if "floor" in pair and any(g and "_foot_" in g for g in pair):
        return True
    return receiving == "FRONT_RACK" and "bar_shaft" in pair and any(g and g.endswith("_deltoid") for g in pair)


def _clearance_geoms(plant):
    m = plant.model
    bar = plant.body_id("barbell")
    out = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] in (0, bar) or not (m.geom_contype[g] & 4):
            continue
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[g]))
        if name.endswith("_deltoid") or body.endswith(("_hand", "_forearm")):
            continue
        out.append(g)
    return out


def bar_clearances(plant):
    m, d = plant.model, plant.data
    shaft = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "bar_shaft")
    out = {}
    for g in _clearance_geoms(plant):
        out[mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or f"geom{g}"] = float(
            mujoco.mj_geomDistance(m, d, shaft, g, 1.0, None))
    return out


# Declared grip variants: (grip model, site distance from wrist, half-width).
GRIP_VARIANTS = {
    "CLOSED_PALM_V1": ("BILATERAL_TWO_SITE_CONNECT_V1", .085),
    "CLOSED_FINGER_V1": ("BILATERAL_TWO_SITE_CONNECT_V1", .13),
    "OPEN_FINGER_RACK_V1": ("BILATERAL_SINGLE_SITE_CONNECT_V1", .13),
}


def default_spec(receiving, grip=None, half_width=None):
    grip = grip or ("OPEN_FINGER_RACK_V1" if receiving == "FRONT_RACK" else "CLOSED_PALM_V1")
    model, distal = GRIP_VARIANTS[grip]
    width = half_width or (.25 if receiving == "FRONT_RACK" else .45)
    return AthleteBarbellSpec(grip_half_width_m=width, grip_site_distal_m=distal, grip_model=model)


def _limits():
    spec = {j.name: j for j in LEG + GIRDLE + ARM}
    lo, hi = [], []
    for name in VARIABLES:
        if name in spec:
            lo.append(spec[name].lower + RANGE_MARGIN_RAD)
            hi.append(spec[name].upper - RANGE_MARGIN_RAD)
        elif name == "lumbar_flex":
            lo.append(-.50 + RANGE_MARGIN_RAD)
            hi.append(.90 - RANGE_MARGIN_RAD)
        else:
            lo.append(-np.inf)
            hi.append(np.inf)
    return np.array(lo), np.array(hi)


def apply_pose(plant: SpatialPlant, x):
    """Write a symmetric pose vector into qpos (qvel zero) and run kinematics."""
    v = dict(zip(VARIABLES, x))
    d, m = plant.data, plant.model
    d.qpos[:] = m.qpos0
    d.qvel[:] = 0
    p = plant.joint_qpos("pelvis_free")
    d.qpos[p:p + 3] = (v["pelvis_x"], 0.0, v["pelvis_z"])
    half = .5 * v["pelvis_pitch"]
    d.qpos[p + 3:p + 7] = (math.cos(half), 0.0, math.sin(half), 0.0)   # rotation about +y
    d.qpos[plant.joint_qpos("lumbar_flex")] = v["lumbar_flex"]
    for side in SIDES:
        for name in SIDE_JOINTS:
            d.qpos[plant.joint_qpos(f"{side}_{name}")] = v[name]
    b = plant.joint_qpos("bar_free")
    d.qpos[b:b + 3] = (v["bar_x"], 0.0, v["bar_z"])
    d.qpos[b + 3:b + 7] = (1.0, 0.0, 0.0, 0.0)
    mujoco.mj_kinematics(m, d)
    mujoco.mj_comPos(m, d)


def _residuals(plant, x, receiving, depth, penetration=5e-4):
    apply_pose(plant, x)
    d, m, s = plant.data, plant.model, plant.spec.scale
    out = []
    r = .02 * s
    for side in SIDES:
        y = MIRROR[side] * plant.spec.stance_half_width_m
        for tag, fx, fy in (("heel_m", -.06, -.035), ("heel_l", -.06, .035), ("toe_m", .18, -.04), ("toe_l", .18, .04)):
            gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_foot_{tag}")
            target = (fx * s, y + MIRROR[side] * fy * s, r - penetration)
            out.extend(CLOSURE_WEIGHT * (d.geom_xpos[gid] - target))
        for tag in grip_tags(plant.spec):
            out.extend(CLOSURE_WEIGHT * (d.site_xpos[plant.site_id(f"{side}_grip_{tag}")] - d.site_xpos[plant.site_id(f"bar_{side}_grip_{tag}")]))
        if receiving == "FRONT_RACK":
            gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_deltoid")
            bar = d.xpos[plant.body_id("barbell")]
            delta = d.geom_xpos[gid][[0, 2]] - bar[[0, 2]]
            out.append(CLOSURE_WEIGHT * (np.linalg.norm(delta) - (m.geom_size[gid][0] + .014 - penetration)))
            # The shelf must be below the bar centre (bar resting on top).
            out.append(CLOSURE_WEIGHT * min(0.0, -delta[1] - .5 * m.geom_size[gid][0]))
    for clearance in bar_clearances(plant).values():
        out.append(CLOSURE_WEIGHT * min(0.0, clearance - BAR_CLEARANCE_M))
    com = plant.com()
    # Balance is certified by the centre of pressure inside the support
    # polygon (static QP + independent oracle), so the COM target is soft.
    out.append(3 * (com[0] - .06 * s))
    v = dict(zip(VARIABLES, x))
    out.append(3 * (thigh_inclination(plant) - THIGH_TARGET[depth]))
    out.append(3 * (trunk_inclination(plant) - TRUNK_TARGET[receiving]))
    if receiving == "OVERHEAD":
        out.append(2 * v["elbow_flex"])                # locked-out arms
    nominal = NOMINAL[(receiving, depth)]
    out.extend(.05 * (v[k] - nominal.get(k, 0.0)) for k in VARIABLES if k not in ("bar_x", "bar_z", "pelvis_x", "pelvis_z"))
    return np.array(out)


def thigh_inclination(plant):
    """Mean angle of the hip-to-knee line above horizontal (rad); positive = hip above knee."""
    angles = []
    for side in SIDES:
        hip = plant.data.xpos[plant.body_id(f"{side}_thigh")]
        knee = plant.data.site_xpos[plant.site_id(f"{side}_knee")]
        dz, dh = hip[2] - knee[2], math.hypot(hip[0] - knee[0], hip[1] - knee[1])
        angles.append(math.atan2(dz, dh))
    return sum(angles) / 2


def trunk_inclination(plant):
    """Angle of the lumbar-to-neck line from vertical (rad)."""
    lumbar = plant.data.xpos[plant.body_id("torso")]
    neck = plant.data.site_xpos[plant.site_id("neck")]
    up = neck - lumbar
    return math.atan2(math.hypot(up[0], up[1]), up[2])


def depth_label(inclination):
    if inclination > DEPTH_BAND_RAD:
        return "POWER"
    if inclination < -DEPTH_BAND_RAD:
        return "FULL"
    return "AMBIGUOUS"


def solve_pose(plant, receiving, depth):
    lo, hi = _limits()
    nominal = NOMINAL[(receiving, depth)]
    x0 = np.clip(np.array([nominal.get(k, 0.0) for k in VARIABLES]), lo + 1e-6, hi - 1e-6)
    result = least_squares(lambda x: _residuals(plant, x, receiving, depth), x0, bounds=(lo, hi),
                           xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=4000)
    apply_pose(plant, result.x)
    closure = _closure(plant, receiving)
    return result.x, closure, result


def _closure(plant, receiving):
    d, m = plant.data, plant.model
    grips = [float(np.linalg.norm(d.site_xpos[plant.site_id(f"{s}_grip_{t}")] - d.site_xpos[plant.site_id(f"bar_{s}_grip_{t}")]))
             for s in SIDES for t in grip_tags(plant.spec)]
    shelf = []
    if receiving == "FRONT_RACK":
        for side in SIDES:
            gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_deltoid")
            bar = d.xpos[plant.body_id("barbell")]
            shelf.append(float(np.linalg.norm(d.geom_xpos[gid][[0, 2]] - bar[[0, 2]]) - (m.geom_size[gid][0] + .014)))
    return dict(max_grip_separation_m=max(grips), shelf_gaps_m=shelf)


# --------------------------------------------------------------- static QP
@dataclass
class StaticCertificate:
    feasible: bool
    reason: str
    torques_n_m: dict
    max_torque_ratio: float
    contacts: list
    grips: list
    free_residual: float
    weight_n: float
    details: dict = field(default_factory=dict)


def static_equilibrium(plant, receiving, mu=None, depth=None):
    """Minimum-effort static balance: S'tau + Jc' f + Jeq' lam = g(q), f in pyramid.

    Uses MuJoCo only for kinematics, Jacobians and the generalized gravity
    vector; contact forces are this QP's unknowns, not MuJoCo solver output.
    """
    m, d = plant.model, plant.data
    mujoco.mj_forward(m, d)
    gravity = d.qfrc_bias.copy()            # qvel = 0: pure generalized gravity
    mu = plant.spec.foot_friction if mu is None else mu
    nv = m.nv
    columns, labels = [], []
    contact_rows = []
    undeclared = []
    for i in range(d.ncon):
        c = d.contact[i]
        g1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom1)
        g2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)
        if not declared_support(receiving, g1, g2):
            if c.dist < 0:
                undeclared.append((g1, g2, float(c.dist)))
            continue
        if c.dist > 1e-3:
            continue
        b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
        frame = c.frame.reshape(3, 3)
        jac1, jac2 = np.zeros((3, nv)), np.zeros((3, nv))
        mujoco.mj_jac(m, d, jac1, None, c.pos, b1)
        mujoco.mj_jac(m, d, jac2, None, c.pos, b2)
        jac = jac2 - jac1                    # force on body2 along +normal
        edges = [frame[0] + mu * frame[1], frame[0] - mu * frame[1], frame[0] + mu * frame[2], frame[0] - mu * frame[2]]
        for e in edges:
            columns.append(jac.T @ e)
            labels.append(("contact", i))
        contact_rows.append((i, c, frame, edges, b1, b2))
    n_beta = len(columns)
    grip_rows = []
    for e in range(m.neq):
        s1, s2 = int(m.eq_obj1id[e]), int(m.eq_obj2id[e])
        j1, j2 = np.zeros((3, nv)), np.zeros((3, nv))
        mujoco.mj_jacSite(m, d, j1, None, s1)
        mujoco.mj_jacSite(m, d, j2, None, s2)
        for k in range(3):
            columns.append((j1 - j2)[k])
            labels.append(("grip", e))
        grip_rows.append((e, s1, s2))
    A = np.array(columns).T if columns else np.zeros((nv, 0))
    free = [r for r in plant.free_dofs if np.any(np.abs(A[r]) > 1e-12)]
    unsupported = [r for r in plant.free_dofs if r not in free and abs(gravity[r]) > 1e-9]
    act = plant.actuated
    act_idx = [dof for _, dof, _ in act]
    caps = np.array([cap for *_, cap in act])
    n = A.shape[1]

    def torques(x):
        return gravity[act_idx] - A[act_idx] @ x

    def effort(x):
        tau = torques(x) / caps
        return float(tau @ tau) + 1e-8 * float(x[:n_beta] @ x[:n_beta])

    constraints = [dict(type="eq", fun=lambda x: A[free] @ x - gravity[free], jac=lambda x: A[free])]
    weight = plant.total_mass() * 9.81
    x0 = np.zeros(n)
    if n_beta:
        x0[:n_beta] = weight / n_beta
    bounds = [(0, None)] * n_beta + [(-GRIP_FORCE_CAPACITY_N, GRIP_FORCE_CAPACITY_N)] * (n - n_beta)
    # Two connect sites per hand: their force difference is a hand moment.
    couple = GRIP_MOMENT_CAPACITY_N_M / (.5 * GRIP_SITE_SPACING_M)
    hands = {}
    for k_, (kind, e) in enumerate(labels):
        if kind == "grip":
            hands.setdefault(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_EQUALITY, e)[:-2], []).append(k_)
    for idx in hands.values():
        a_, b_ = idx[:3], idx[3:]
        for i_, j_ in zip(a_, b_):
            constraints.append(dict(type="ineq", fun=lambda x, i_=i_, j_=j_: couple - (x[i_] - x[j_])))
            constraints.append(dict(type="ineq", fun=lambda x, i_=i_, j_=j_: couple + (x[i_] - x[j_])))
    started = time.perf_counter()
    res = minimize(effort, x0, method="SLSQP", bounds=bounds, constraints=constraints,
                   options=dict(maxiter=2000, ftol=1e-14))
    x = res.x
    residual = float(np.max(np.abs(A[free] @ x - gravity[free]))) if free else 0.0
    tau = torques(x)
    ratio = float(np.max(np.abs(tau) / caps))
    contacts = []
    k = 0
    for i, c, frame, edges, b1, b2 in contact_rows:
        f = sum(x[k + j] * edges[j] for j in range(4))
        k += 4
        contacts.append(dict(geom1=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom1),
                             geom2=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2),
                             body1=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b1),
                             body2=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b2),
                             point=c.pos.tolist(), normal=frame[0].tolist(), force_on_body2=f.tolist(),
                             friction=mu))
    grips = []
    for e, s1, s2 in grip_rows:
        f = x[k:k + 3]
        k += 3
        grips.append(dict(name=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_EQUALITY, e),
                          body1=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.site_bodyid[s1])),
                          body2=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.site_bodyid[s2])),
                          point1=d.site_xpos[s1].tolist(), point2=d.site_xpos[s2].tolist(), force_on_body1=f.tolist()))
    reason = "FEASIBLE"
    if undeclared:
        reason = "UNDECLARED_INTERPENETRATION"
    elif unsupported:
        reason = "UNSUPPORTED_FREE_DIRECTION"
    elif not res.success and residual > 1e-6:
        reason = f"QP_FAILED: {res.message}"
    elif residual > 1e-6 * weight:
        reason = "UNBALANCED_FREE_BODY_RESIDUAL"
    elif ratio > 1.0:
        reason = "ACTUATOR_CAPACITY_EXCEEDED"
    elif trunk_inclination(plant) > TRUNK_LIMIT_RAD:
        reason = "TRUNK_NOT_UPRIGHT"
    elif depth is not None and depth_label(thigh_inclination(plant)) != depth:
        reason = "DEPTH_NOT_ATTAINED"
    return StaticCertificate(reason == "FEASIBLE", reason,
                             {name: float(t) for (name, _, _), t in zip(act, tau)}, ratio, contacts, grips,
                             residual, weight, dict(qp_message=str(res.message), qp_iterations=int(res.nit),
                                                    qp_wall_s=time.perf_counter() - started,
                                                    active_contacts=len(contacts), undeclared=undeclared))


# --------------------------------------------------------------- dynamic hold
ARM_CHAIN = ("girdle_elev", "girdle_prot", "sh_flex", "sh_abd", "sh_rot", "elbow_flex", "forearm_sup",
             "wrist_flex", "wrist_dev")
# Declared servo-stiffness hypotheses per receiving topology. A front rack closes
# a loop (two grips plus two shelf contacts); stiff arms then squeeze the bar
# against the shoulders. RELAXED_RACK_ARMS lowers arm-chain stiffness so the
# shelf carries the bar and the fingers only stabilise it.
SERVO_PROFILES = {"UNIFORM_V1": {}, "RELAXED_RACK_ARMS_V1": {name: .05 for name in ARM_CHAIN}}


def set_servo_profile(plant, profile):
    from .model import KD_PER_CAPACITY, KP_PER_CAPACITY
    scale = SERVO_PROFILES[profile]
    m = plant.model
    for i, (name, _, cap) in enumerate(plant.actuated):
        factor = scale.get(name.split("_", 1)[1] if name[:2] in ("l_", "r_") else name, 1.0)
        kp, kd = KP_PER_CAPACITY * cap * factor, KD_PER_CAPACITY * cap * max(factor, .2)
        m.actuator_gainprm[i, 0] = kp
        m.actuator_biasprm[i, 1:3] = (-kp, -kd)


def hold_trial(plant, x_pose, certificate, *, duration_s=1.5, push_n=60.0, push_window=(.3, .4),
               record_every=1, servo_profile="UNIFORM_V1"):
    """Bounded-torque servo hold about the static pose with a feed-forward and a bar push.

    The push is an external force on the bar COM along +x over push_window,
    recorded as external work. Saturation means |actuator torque| reached its
    declared capacity on any joint in a step.
    """
    m, d = plant.model, plant.data
    set_servo_profile(plant, servo_profile)
    apply_pose(plant, x_pose)
    mujoco.mj_forward(m, d)
    act = plant.actuated
    qadr = [plant.joint_qpos(name) for name, _, _ in act]
    dadr = [dof for _, dof, _ in act]
    q_ref = d.qpos[qadr].copy()
    tau_ff = np.array([certificate.torques_n_m[name] for name, _, _ in act])
    caps = np.array([cap for *_, cap in act])
    kp = m.actuator_gainprm[:, 0].copy()
    d.ctrl[:] = q_ref + tau_ff / kp
    bar = plant.body_id("barbell")
    frames, saturated = [], 0
    steps = int(round(duration_s / m.opt.timestep))
    energy0 = float(d.energy.sum())
    work_act = work_ext = work_damp = 0.0
    h = m.opt.timestep
    for k in range(steps):
        d.xfrc_applied[:] = 0
        if push_window[0] <= d.time < push_window[1]:
            d.xfrc_applied[bar, 0] = push_n
        mujoco.mj_forward(m, d)
        saturated += int(np.any(np.abs(d.actuator_force) >= caps * (1 - 1e-9)))
        if k % record_every == 0:
            frame = plant.export_frame()
            frame["external"] = [dict(body="barbell", point=d.xipos[bar].tolist(),
                                      force=d.xfrc_applied[bar, :3].tolist())]
            frame["max_torque_ratio"] = float(np.max(np.abs(d.actuator_force) / caps))
            frame["energy_j"] = float(d.energy.sum())
            frames.append(frame)
        v_bar, _ = plant.body_velocity(bar)
        work_act += float(d.actuator_force @ d.qvel[dadr]) * h
        work_ext += float(d.xfrc_applied[bar, :3] @ v_bar) * h
        work_damp -= float(np.sum(m.dof_damping * d.qvel ** 2)) * h
        mujoco.mj_step(m, d)
    d.xfrc_applied[:] = 0
    mujoco.mj_forward(m, d)
    final = plant.export_frame()
    final["external"] = []
    final["energy_j"] = float(d.energy.sum())
    frames.append(final)
    return dict(frames=frames, saturated_steps=saturated, steps=steps, energy0_j=energy0,
                energy_final_j=float(d.energy.sum()), actuator_work_j=work_act, external_work_j=work_ext,
                damping_work_j=work_damp, final_com=plant.com().tolist(),
                final_max_speed=float(np.max(np.abs(d.qvel))),
                max_grip_separation_m=max(g["separation_m"] for g in plant.grip_ledger()),
                joint_angles=plant.joint_angles())


def joint_range_violations(plant, angles, margin=0.0):
    specs = dict(joint_specs())
    return {k: v for k, v in angles.items()
            if not specs[k].lower - margin <= v <= specs[k].upper + margin}
