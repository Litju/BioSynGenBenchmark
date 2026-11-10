"""RES-624 Gates D/E: equilibrium-certified overhead receiving for Human V2.

Unlike the frozen RES-618 ``receiving_v2.solve_candidate`` (kinematic
closure only, kept as the RED witness), a candidate here is built around the
hand-bar relation *measured* in the Gate C isolated fixture (bar seated on the
pads, fingers wrapped, palm facing up) and is accepted only if

1. a generalized static-equilibrium LP finds contact forces inside the
   inscribed friction pyramids, actuator forces inside declared capacity and
   free scapular-rhythm equality forces that balance gravity on *every*
   generalized coordinate, including both free joints (no hidden pelvis or
   bar support);
2. the centre of pressure of the certified floor forces lies inside the
   convex hull of loaded floor contacts;
3. joint values lie inside declared ROM and no non-adjacent human geoms
   interpenetrate.

Otherwise the result is STATIC_GRASP_INFEASIBLE / ACTUATOR_INFEASIBLE /
POSE_INADMISSIBLE. Kinematic convergence alone never produces a PASS.

The forward hold applies only bounded actuator commands: the certified
equilibrium torques as feed-forward plus bounded joint feedback, and
constant bounded finger flexion with a bounded thumb servo (the Gate C
grip controller). No bar or root state is written after t=0.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import mujoco
import numpy as np
from scipy.optimize import least_squares

from .grasp_diagnostics import PADS
from .grasp_fixture import FixtureSpec, run_trial
from .grasp_mechanics import candidate_contacts, generalized_equilibrium
from .receiving import NOMINAL
from .receiving_v2 import SIDE

HAND_FIXED = ("finger_flex", "finger_distal", "thumb_opposition")
POSE_SIDE = tuple(j for j in SIDE if j not in HAND_FIXED)
VARIABLES = ("pelvis_x", "pelvis_z", "pelvis_pitch", "lumbar_flex", "thoracic_flex", *POSE_SIDE, "bar_x", "bar_z")
SOLES = ("hindfoot", "forefoot", "mtp")
CONTACT_MARGIN_M = 5e-4
# A certified contact may not penetrate beyond 0.1 shaft radius: deeper overlap no longer
# represents the declared rigid shaft/pad surfaces (same bound as the acceptance evaluator).
MAX_PENETRATION_M = .1*.014


class _Null:
    def record(self, extra=None):
        return None


@dataclass(frozen=True)
class HandBarRelation:
    """Settled relation from the Gate C fixture (hand-frame shaft line and finger posture)."""
    shaft_point_hand: tuple
    finger_flex: float
    finger_distal: float
    thumb_opposition: float
    tilt_deg: float
    grip_frac: float


def qualified_relation(tilt_deg=0.0, grip_frac=.6, settle_s=1.0):
    fx, *_ = run_trial(FixtureSpec(tilt_deg=tilt_deg), grip_frac, settle_s, record_every=10**9, recorder=_Null())
    m, d = fx.model, fx.data
    h, b = fx.body_id("l_hand"), fx.body_id("barbell")
    r = d.xmat[h].reshape(3, 3)
    p = r.T @ (d.xipos[b]-d.xpos[h])
    axis = r.T @ d.xmat[b].reshape(3, 3)[:, 1]
    p_line = p - (p @ axis)*axis  # closest point of the shaft line to the palm origin, hand frame
    q = {j: float(d.qpos[fx.joint_qpos("l_"+j)]) for j in HAND_FIXED}
    return HandBarRelation(tuple(map(float, p_line)), q["finger_flex"], q["finger_distal"], q["thumb_opposition"],
                           tilt_deg, grip_frac)


def apply_pose(plant, x, relation):
    m, d = plant.model, plant.data
    v = dict(zip(VARIABLES, x))
    d.qpos[:] = m.qpos0
    d.qvel[:] = 0
    d.qpos[:3] = [v["pelvis_x"], 0, v["pelvis_z"]]
    a = v["pelvis_pitch"]
    d.qpos[3:7] = [math.cos(a/2), 0, math.sin(a/2), 0]
    for name in ("lumbar_flex", "thoracic_flex"):
        d.qpos[plant.joint_qpos(name)] = v[name]
    for side in ("l", "r"):
        for name in POSE_SIDE:
            d.qpos[plant.joint_qpos(side+"_"+name)] = v[name]
        for name in HAND_FIXED:
            d.qpos[plant.joint_qpos(side+"_"+name)] = getattr(relation, name)
        d.qpos[plant.joint_qpos(side+"_scap_up")] = .4*v["sh_abd"]
        d.qpos[plant.joint_qpos(side+"_scap_tilt")] = .1*v["sh_flex"]
    q = plant.joint_qpos("bar_free")
    d.qpos[q:q+3] = [v["bar_x"], 0, v["bar_z"]]
    d.qpos[q+3:q+7] = [1, 0, 0, 0]
    mujoco.mj_kinematics(m, d)
    mujoco.mj_comPos(m, d)


def _bounds(m):
    low, high = [], []
    for name in VARIABLES:
        if name in ("pelvis_x", "bar_x"):
            a, b = -.7, .7
        elif name in ("pelvis_z", "bar_z"):
            a, b = .25, 2.5
        elif name == "pelvis_pitch":
            a, b = -.4, .8
        else:
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "l_"+name if name in POSE_SIDE else name)
            a, b = m.jnt_range[j] + [.01, -.01]
        low.append(a)
        high.append(b)
    return np.array(low), np.array(high)


def solve_supported_candidate(plant, relation, depth="POWER", max_nfev=2000, palm_up_weight=20.0):
    """Pose with the qualified hand-bar relation, feet on the floor and total COM over the feet."""
    m, d = plant.model, plant.data
    low, high = _bounds(m)
    nominal = {**NOMINAL[("OVERHEAD", depth)], "wrist_flex": -1.2, "sh_flex": 2.9}
    initial = np.clip([nominal.get(k, 0.) for k in VARIABLES], low+1e-6, high-1e-6)
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    shaft = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "bar_shaft")
    width = plant.spec.equipment.grip_half_width_m
    scale = plant.spec.profile.stature_m/1.75
    c = np.array(relation.shaft_point_hand)
    bar = plant.body_id("barbell")
    sole_ids = {s: [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{s}_{t}_sole") for t in SOLES] for s in "lr"}

    def residual(x):
        apply_pose(plant, x, relation)
        out = []
        for side, sign in (("l", 1), ("r", -1)):
            for tag, g in zip(SOLES, sole_ids[side]):
                dist = mujoco.mj_geomDistance(m, d, floor, g, 1., None)
                out.append(2000*(min(0., dist+.0001) if tag == "mtp" else dist+.0001))
            fb = plant.body_id(side+"_foot")
            out.extend(100*(d.xpos[fb, :2]-[0, sign*.15*scale]))
            h = plant.body_id(side+"_hand")
            r = d.xmat[h].reshape(3, 3)
            target = d.xpos[bar] + [0, sign*width, 0]
            out.extend(3000*(d.xpos[h] + r @ c - target))
            out.append(2000*(1-abs(r[1, 1])))          # hand y axis along the bar axis (world y)
            out.append(palm_up_weight*(1-(-r[2, 0])))   # palm normal (-x_hand) toward +z
        for name in ("head_collision", "neck_collision", "thorax_collision", "lumbar_collision"):
            g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
            out.append(2000*min(0, mujoco.mj_geomDistance(m, d, shaft, g, 1., None)-.005))
        hip, knee = d.xpos[plant.body_id("l_thigh")], d.xpos[plant.body_id("l_shank")]
        theta = math.atan2(hip[2]-knee[2], np.linalg.norm(hip[:2]-knee[:2]))
        out.append(10*(theta-(-.15 if depth == "FULL" else .35)))
        # Total (human + bar) COM over the centroid of the six sole geoms.
        com = np.sum(m.body_mass[1:, None]*d.xipos[1:], axis=0)/m.body_mass[1:].sum()
        soles = np.mean([d.geom_xpos[g] for ids in sole_ids.values() for g in ids], axis=0)
        out.append(200*(com[0]-soles[0]))
        out.append(.5*x[VARIABLES.index("elbow_flex")])
        out.extend(.02*(x-initial))
        return out

    res = least_squares(residual, initial, bounds=(low, high), max_nfev=max_nfev, ftol=1e-12, xtol=1e-12, gtol=1e-12)
    apply_pose(plant, res.x, relation)
    mujoco.mj_forward(m, d)
    h = plant.body_id("l_hand")
    palm = -d.xmat[h].reshape(3, 3)[:, 0]
    return dict(outcome="CANDIDATE_ONLY", optimizer_success=bool(res.success), evaluations=int(res.nfev),
                residual_norm=float(np.linalg.norm(res.fun)), x=res.x.tolist(),
                variables=dict(zip(VARIABLES, map(float, res.x))), qpos=d.qpos.tolist(),
                palm_tilt_from_vertical_deg=float(math.degrees(math.acos(np.clip(palm[2], -1, 1)))))


def _hull_margin(points, cop):
    pts = sorted(set(map(tuple, points)))
    if len(pts) < 3:
        return -math.inf

    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1]+upper[:-1]
    return min(((b[0]-a[0])*(cop[1]-a[1])-(b[1]-a[1])*(cop[0]-a[0]))/math.dist(a, b) for a, b in zip(hull, hull[1:]+hull[:1]))


def self_intersections(plant, tolerance_m=1e-4):
    """Non-adjacent human geom pairs (masks ignored) that interpenetrate beyond tolerance."""
    m, d = plant.model, plant.data
    bar = plant.body_id("barbell")
    human = [g for g in range(m.ngeom) if m.geom_bodyid[g] not in (0, bar)]
    hits, worst = [], math.inf
    for i, a in enumerate(human):
        for b in human[i+1:]:
            ba, bb = int(m.geom_bodyid[a]), int(m.geom_bodyid[b])
            if ba == bb or m.body_parentid[ba] == bb or m.body_parentid[bb] == ba:
                continue
            if m.body_parentid[ba] == m.body_parentid[bb] and m.body_parentid[ba] != 0:
                continue  # siblings at one joint centre (e.g. clavicle/neck on thorax)
            dist = mujoco.mj_geomDistance(m, d, a, b, .05, None)
            worst = min(worst, dist)
            if dist < -tolerance_m:
                hits.append((mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, a) or f"geom{a}",
                             mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, b) or f"geom{b}", float(dist)))
    return dict(pairs_checked=len(human)*(len(human)-1)//2, penetrating=hits, min_distance_m=float(worst))


def certify(plant, actuator_scale=1.0, margin_m=CONTACT_MARGIN_M):
    """Static equilibrium certificate of the current Human V2 configuration."""
    m, d = plant.model, plant.data
    shaft = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "bar_shaft")
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    pads = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{s}_{p}") for s in "lr" for p in PADS]
    soles = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{s}_{t}_sole") for s in "lr" for t in SOLES]
    rows = candidate_contacts(m, d, pads+soles+[shaft, floor], margin_m)
    keep = [c for c in rows if ({c["geom1"], c["geom2"]} & {shaft} and {c["geom1"], c["geom2"]} & set(pads))
            or ({c["geom1"], c["geom2"]} & {floor} and {c["geom1"], c["geom2"]} & set(soles))]
    kept = {id(c) for c in keep}
    other = [c for c in rows if id(c) not in kept and c["dist"] < 0]
    cert = generalized_equilibrium(m, d, keep, actuator_scale=actuator_scale)
    out = dict(code=cert["code"], feasible=cert["feasible"], contacts=len(keep),
               hand_contacts=sum(1 for c in keep if shaft in (c["geom1"], c["geom2"])),
               floor_contacts=sum(1 for c in keep if floor in (c["geom1"], c["geom2"])),
               undeclared_penetrations=[(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c["geom1"]),
                                         mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c["geom2"]), c["dist"]) for c in other],
               max_penetration_m=float(-min((c["dist"] for c in keep), default=0)))
    if "actuator_force" not in cert:
        return out
    forces = [np.asarray(f) for f in cert["contact_forces"]]
    bar = plant.body_id("barbell")
    on_bar = sum((f if c["body2"] == bar else -f) for f, c in zip(forces, keep) if bar in (c["body1"], c["body2"]))
    floor_rows = [(np.asarray(c["point"]), f if c["body2"] != 0 else -f) for f, c in zip(forces, keep) if 0 in (c["body1"], c["body2"])]
    fz = sum(f[2] for _, f in floor_rows)
    cop = sum((p[:2]*f[2] for p, f in floor_rows), np.zeros(2))/fz if fz > 0 else None
    loaded = [tuple(p[:2]) for p, f in floor_rows if f[2] > 1e-6]
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in range(m.nu)]
    util = dict(zip(names, cert["actuator_utilization"]))
    joints = {}
    for j in range(m.njnt):
        if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
            q = d.qpos[m.jnt_qposadr[j]]
            joints[mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)] = float(min(q-m.jnt_range[j, 0], m.jnt_range[j, 1]-q))
    weight_total = float(m.body_mass[1:].sum()*9.81)
    out.update(max_generalized_residual=cert["max_generalized_residual"], residual_tolerance=cert["residual_tolerance"],
               max_utilization=cert["max_utilization"], capacity_margin=cert["capacity_margin"],
               most_loaded_actuators=sorted(util.items(), key=lambda kv: -kv[1])[:6],
               actuator_force={n: float(f) for n, f in zip(names, cert["actuator_force"])},
               bar_contact_force_n=np.asarray(on_bar).tolist() if not np.isscalar(on_bar) else [0, 0, 0],
               bar_weight_n=float(m.body_mass[bar]*9.81), floor_vertical_n=float(fz), total_weight_n=weight_total,
               cop=None if cop is None else cop.tolist(),
               cop_hull_margin_m=None if cop is None else float(_hull_margin(loaded, cop)),
               min_joint_limit_margin_rad=float(min(joints.values())),
               joints_at_limit=[k for k, v in joints.items() if v < 1e-9],
               contact_forces=[dict(geom1=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c["geom1"]),
                                    geom2=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c["geom2"]),
                                    point=np.asarray(c["point"]).tolist(), force_on_body2=f.tolist(), dist=c["dist"])
                               for f, c in zip(forces, keep)])
    return out


def qualify_static(plant, actuator_scale=1.0):
    cert = certify(plant, actuator_scale)
    si = self_intersections(plant)
    if not cert["feasible"]:
        verdict = cert["code"]
    elif cert["hand_contacts"] == 0:
        verdict = "STATIC_GRASP_INFEASIBLE"
    elif cert["cop_hull_margin_m"] is None or cert["cop_hull_margin_m"] < 0:
        verdict = "STATIC_GRASP_INFEASIBLE"
    elif (si["penetrating"] or cert["undeclared_penetrations"] or cert["joints_at_limit"]
          or cert["max_penetration_m"] > MAX_PENETRATION_M):
        verdict = "POSE_INADMISSIBLE"
    else:
        verdict = "STATIC_EQUILIBRIUM_CERTIFIED"
    return dict(verdict=verdict, certificate=cert, self_intersection=si)


class EquilibriumHoldController:
    """Bounded feedback around the certified pose with certified feed-forward torques."""

    # Ankle strategy: total sagittal COM feedback, N m per m and N m s per m (both ankles).
    # Gravity's destabilizing stiffness is (m g) = 932 N m/m for the nominal system; the
    # proportional gain is ~3x that, the derivative gain gives a damping ratio ~1 for a
    # 0.9 m inverted pendulum. Commands stay within declared ankle capacity.
    COM_KP = 3000.0
    COM_KD = 900.0

    def __init__(self, plant, certificate, grip_frac=.6, gain_scale=1.0, balance=True):
        self.plant = plant
        m = plant.model
        self.balance = balance
        self.com_target = self.com()[0]
        self.targets = plant.joint_angles()
        self.ff = certificate["actuator_force"]
        self.grip_frac = grip_frac
        self.gain_scale = gain_scale
        self.names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in range(m.nu)]

    def com(self):
        m, d = self.plant.model, self.plant.data
        return (m.body_mass[1:, None]*d.xipos[1:]).sum(0)/m.body_mass[1:].sum()

    def com_velocity(self):
        m, d = self.plant.model, self.plant.data
        v6 = np.zeros(6)
        total = np.zeros(3)
        for b in range(1, m.nbody):
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, b, v6, 0)
            total += m.body_mass[b]*v6[3:]
        return total/m.body_mass[1:].sum()

    def apply(self, release=False):
        m, d, p = self.plant.model, self.plant.data, self.plant
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        ankle = 0.0
        if self.balance:
            # Joint axis is -y: positive torque pitches the shank forward, so oppose COM error.
            ankle = -.5*(self.COM_KP*(self.com()[0]-self.com_target)+self.COM_KD*self.com_velocity()[0])
        for a, name in enumerate(self.names):
            j = m.actuator_trnid[a, 0]
            q, dof = d.qpos[m.jnt_qposadr[j]], m.jnt_dofadr[j]
            cap = m.actuator_forcerange[a, 1]
            ff = self.ff.get(name, 0.) + (ankle if name.endswith("_ankle_dorsi") else 0.)
            if name.endswith(("_finger_flex", "_finger_distal")):
                d.ctrl[a] = (-.3 if release else self.grip_frac)*cap
            elif name.endswith("_thumb_opposition"):
                d.ctrl[a] = np.clip(10*cap*(self.targets[name]-q)-.25*cap*d.qvel[dof], -cap, cap)
            elif m.actuator_biastype[a] == mujoco.mjtBias.mjBIAS_AFFINE:
                d.ctrl[a] = self.targets[name] + ff/m.actuator_gainprm[a, 0]
            else:
                kp = 10*cap*self.gain_scale
                d.ctrl[a] = np.clip(kp*(self.targets[name]-q)-.25*cap*d.qvel[dof]+ff, -cap, cap)


def hand_frames(plant):
    m, d = plant.model, plant.data
    out = {}
    for side in ("l", "r"):
        h = plant.body_id(side+"_hand")
        out[side] = dict(position=d.xpos[h].tolist(), rotation=d.xmat[h].reshape(3, 3).tolist())
    return out


def actuator_export(plant):
    m, d = plant.model, plant.data
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in range(m.nu)]
    return dict(names=names, force=d.actuator_force.tolist(), capacity=m.actuator_forcerange[:, 1].tolist(),
                ctrl=d.ctrl.tolist())


def forward_receiving_hold(plant, certificate, duration_s=2.0, grip_frac=.6, push=None, release_at_s=None,
                           recorder=None, frame_every=1, balance=True):
    """Bounded-actuation forward hold; returns plain frames for the independent acceptance evaluator.

    push = (body, t0, t1, force_xyz): a *declared* external perturbation, exported in the frames.
    """
    m, d = plant.model, plant.data
    ctl = EquilibriumHoldController(plant, certificate, grip_frac=grip_frac, balance=balance)
    frames = []
    for k in range(round(duration_s/m.opt.timestep)):
        d.xfrc_applied[:] = 0
        if push is not None and push[1] <= d.time < push[2]:
            d.xfrc_applied[plant.body_id(push[0]), :3] = push[3]
        ctl.apply(release=release_at_s is not None and d.time >= release_at_s)
        mujoco.mj_forward(m, d)
        if recorder is not None:
            recorder.record()
        if k % frame_every == 0:
            f = plant.export_frame()
            f["external"] = [dict(body=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b), point=d.xipos[b].tolist(),
                                  force=d.xfrc_applied[b, :3].tolist()) for b in range(1, m.nbody) if np.any(d.xfrc_applied[b])]
            f["hand_frames"] = hand_frames(plant)
            f["actuators"] = actuator_export(plant)
            f["energy"] = dict(qvel_dot_actuator=float(d.qvel @ d.qfrc_actuator),
                               qvel_dot_passive=float(d.qvel @ d.qfrc_passive),
                               qvel_dot_constraint=float(d.qvel @ d.qfrc_constraint))
            frames.append(f)
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    f = plant.export_frame()
    f["hand_frames"] = hand_frames(plant)
    f["actuators"] = actuator_export(plant)
    frames.append(f)
    return frames


def build_certified_candidate(spec=None, depth="POWER", tilt_deg=0.0, grip_frac=.6, max_nfev=2000):
    """Solve + certify; returns (plant, candidate, static qualification, relation)."""
    from .model_v2 import HumanV2Spec
    from .plant_v2 import HumanV2Plant
    plant = HumanV2Plant.build(spec or HumanV2Spec())
    relation = qualified_relation(tilt_deg, grip_frac)
    candidate = solve_supported_candidate(plant, relation, depth, max_nfev)
    static = qualify_static(plant)
    return plant, candidate, static, relation
