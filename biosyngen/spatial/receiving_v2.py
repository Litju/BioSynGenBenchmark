"""V2 pose candidates and genuine bounded forward holds, with no lift classifier.

Pose solving is kinematics only. A candidate is never a receiving PASS until
its physical support, stability and independent balances have been checked.
"""
import math
import mujoco
import numpy as np
from scipy.optimize import least_squares

from .receiving import NOMINAL

SIDE = ("hip_flex", "hip_abd", "hip_rot", "knee_flex", "ankle_dorsi", "subtalar_inv", "forefoot_flex", "mtp_flex",
        "girdle_elev", "girdle_prot", "sh_flex", "sh_abd", "sh_rot", "elbow_flex", "forearm_sup", "wrist_flex", "wrist_dev",
        "finger_flex", "finger_distal", "thumb_opposition")
VARIABLES = ("pelvis_x", "pelvis_z", "pelvis_pitch", "lumbar_flex", "thoracic_flex", *SIDE, "bar_x", "bar_z")


def apply_candidate(plant, x, receiving):
    m, d = plant.model, plant.data
    values = dict(zip(VARIABLES, x))
    d.qpos[:] = m.qpos0
    d.qvel[:] = 0
    d.qpos[:3] = [values["pelvis_x"], 0, values["pelvis_z"]]
    angle = values["pelvis_pitch"]
    d.qpos[3:7] = [math.cos(angle/2), 0, math.sin(angle/2), 0]
    for name in ("lumbar_flex", "thoracic_flex"):
        d.qpos[plant.joint_qpos(name)] = values[name]
    for side in ("l", "r"):
        for name in SIDE:
            d.qpos[plant.joint_qpos(side+"_"+name)] = values[name]
        d.qpos[plant.joint_qpos(side+"_scap_up")] = .4*values["sh_abd"]
        d.qpos[plant.joint_qpos(side+"_scap_tilt")] = .1*values["sh_flex"]
    q = plant.joint_qpos("bar_free")
    d.qpos[q:q+3] = [values["bar_x"], 0, values["bar_z"]]
    d.qpos[q+3:q+7] = [1, 0, 0, 0]
    mujoco.mj_kinematics(m, d)
    mujoco.mj_comPos(m, d)


def solve_candidate(plant, receiving, depth, max_nfev=1000):
    if receiving not in ("FRONT_RACK", "OVERHEAD") or depth not in ("POWER", "FULL"):
        raise ValueError("undeclared receiving configuration")
    m, d = plant.model, plant.data
    low, high = [], []
    for name in VARIABLES:
        if name in ("pelvis_x", "bar_x"):
            a, b = -.7, .7
        elif name in ("pelvis_z", "bar_z"):
            a, b = .25, 2.5
        elif name == "pelvis_pitch":
            a, b = -.4, .8
        else:
            jname = "l_"+name if name in SIDE else name
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname)
            a, b = m.jnt_range[j] + [.01, -.01]
        low.append(a)
        high.append(b)
    nominal = {**NOMINAL[(receiving, depth)], "finger_flex": 1.6 if receiving == "OVERHEAD" else .4,
               "finger_distal": 1.0 if receiving == "OVERHEAD" else .2, "thumb_opposition": .9}
    initial = np.clip([nominal.get(k, 0.) for k in VARIABLES], np.array(low)+1e-6, np.array(high)-1e-6)
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    shaft = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "bar_shaft")
    width = plant.spec.equipment.grip_half_width_m
    scale = plant.spec.profile.stature_m/1.75

    def residual(x):
        apply_candidate(plant, x, receiving)
        output = []
        for side, sign in (("l", 1), ("r", -1)):
            for tag in ("hindfoot", "forefoot", "mtp"):
                g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, side+"_"+tag+"_sole")
                distance = mujoco.mj_geomDistance(m, d, floor, g, 1., None)
                output.append(2000*(min(0., distance+.0001) if tag == "mtp" else distance+.0001))
            b = plant.body_id(side+"_foot")
            output.extend(100*(d.xpos[b, :2]-[0, sign*.15*scale]))
            hand = plant.body_id(side+"_hand")
            center = d.xpos[hand] + d.xmat[hand].reshape(3, 3) @ np.array([-.026*scale, 0, -.085*scale])
            bar = d.xpos[plant.body_id("barbell")] + [0, sign*width, 0]
            output.extend(2000*(center-bar))
            if receiving == "OVERHEAD":
                # Align the shaft with the palm's transverse contact direction.
                output.extend(2000*(d.xmat[hand].reshape(3, 3)[:, 1]-[0, 1, 0]))
            for geom_name in ("palm_collision", "finger_flex_pad", "finger_distal_pad", "thumb_opposition_pad"):
                g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, side+"_"+geom_name)
                output.append(2000*min(0., mujoco.mj_geomDistance(m, d, shaft, g, 1., None)+.0002))
            if receiving == "FRONT_RACK":
                g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, side+"_deltoid")
                output.append(2000*(mujoco.mj_geomDistance(m, d, shaft, g, 1., None)+.0001))
        for name in ("head_collision", "neck_collision", "thorax_collision", "lumbar_collision"):
            g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
            output.append(2000*min(0, mujoco.mj_geomDistance(m, d, shaft, g, 1., None)-.005))
        hip = d.xpos[plant.body_id("l_thigh")]
        knee = d.xpos[plant.body_id("l_shank")]
        theta = math.atan2(hip[2]-knee[2], np.linalg.norm(hip[:2]-knee[:2]))
        output.append(10*(theta-(-.15 if depth == "FULL" else .35)))
        human = np.arange(m.nbody) != plant.body_id("barbell")
        human_com = np.sum(m.body_mass[human, None]*d.xipos[human], axis=0)/sum(m.body_mass[human])
        output.append(10*(human_com[0]-.04))
        output.append(.5*x[VARIABLES.index("elbow_flex")] if receiving == "OVERHEAD" else 0)
        output.extend(.02*(x-initial))
        return output

    result = least_squares(residual, initial, bounds=(low, high), max_nfev=max_nfev,
                           ftol=1e-10, xtol=1e-10, gtol=1e-10)
    apply_candidate(plant, result.x, receiving)
    mujoco.mj_forward(m, d)
    return dict(outcome="CANDIDATE_ONLY", optimizer_success=bool(result.success), evaluations=int(result.nfev),
                residual_norm=float(np.linalg.norm(result.fun)), variables=dict(zip(VARIABLES, map(float, result.x))),
                qpos=d.qpos.tolist(), min_contact_distance_m=min((float(c.dist) for c in d.contact), default=0))


def forward_hold(plant, duration_s=.5, push_n=0):
    m, d = plant.model, plant.data
    targets = plant.joint_angles()
    frames = []
    for _ in range(round(duration_s/m.opt.timestep)):
        mujoco.mj_forward(m, d)
        ff = {name: float(d.qfrc_bias[dof]) for name, dof, _ in plant.actuated}
        plant.set_targets(targets, ff)
        d.xfrc_applied[plant.body_id("barbell"), 0] = push_n if .1 <= d.time < .15 else 0
        mujoco.mj_forward(m, d)
        frame = plant.export_frame()
        frame["external"] = [dict(body="barbell", point=d.xipos[plant.body_id("barbell")].tolist(),
                                  force=d.xfrc_applied[plant.body_id("barbell"), :3].tolist())]
        frame["energy_j"] = float(sum(d.energy))
        frames.append(frame)
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    frames.append(plant.export_frame())
    return dict(frames=frames, final_pelvis_z_m=float(d.qpos[2]), max_speed=float(max(abs(d.qvel))),
                final_bar_z_m=float(d.xpos[plant.body_id("barbell"), 2]))
