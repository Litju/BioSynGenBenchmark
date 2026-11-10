"""RES-624 Gate C: isolated two-hand / free-bar physical grasp fixture.

The two hands are the *compiled Human V2 hand subtrees* (palm, grouped
proximal and distal fingers, thumb, their joints, pads, masses and actuator
capacities) copied verbatim from ``build_mjcf_v2``; only the wrist joints are
removed and each palm is mounted rigidly to the world at a declared pose.
That mount is the single declared constraint; its reaction is reported.

The bar is the V2 barbell body (repaired masks) on a free joint. There is no
weld, connect, external force or position/velocity write on the bar after
the start: it can only be held by frictional contact with the pads, and it
lands on the floor if the grasp fails. Finger and thumb motors receive
bounded torques only.

Orientation: tilt 0 = palm facing up with the fingers pointing forward
(+x). Increasing tilt rotates the hands about the bar axis so the fingers
point upward; tilt 90 deg reproduces the RES-618 RED relation (palm normal
horizontal, fingers up, bar in front of the palm).
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .model import AthleteBarbellSpec, bar_inertia
from .model_v2 import HumanV2Spec, build_mjcf_v2

HAND_JOINTS = ("finger_flex", "finger_distal", "thumb_opposition")
PALM_HALF_THICKNESS = .012
SHAFT_RADIUS = .014


@dataclass(frozen=True)
class FixtureSpec:
    tilt_deg: float = 0.0
    friction: float | None = None      # None: compiled V2 values (pads/shaft 1.0)
    bar_mass_kg: float = 20.0
    mount_height_m: float = 1.0
    grip_along_palm_m: float = -.085   # bar centre along the palm long axis (V2 grip sites)
    initial_gap_m: float = 5e-4        # bar starts just off the palm: no initial penetration
    initial_finger_flex: float = .3
    initial_finger_distal: float = .2
    initial_thumb: float = 0.0
    timestep_s: float = .001
    # Subject whose compiled hands are mounted (None: nominal HumanV2Spec, unchanged behavior). The grip
    # offset above is the nominal-hand value (0.447 hand lengths) and scales with the subject's hand.
    human: HumanV2Spec | None = None


def _rotation(tilt_deg):
    # Base: hand x -> -z (palm normal -x_hand -> +z), hand y -> -y, hand z -> -x (fingers +x).
    base = np.array([[0, 0, -1], [0, -1, 0], [-1, 0, 0]], float).T
    a = -math.radians(tilt_deg)
    ry = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    return ry @ base


def _quat(rot):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, rot.reshape(-1))
    return q


def build_fixture_mjcf(spec=FixtureSpec()):
    human = spec.human or HumanV2Spec()
    v2 = ET.fromstring(build_mjcf_v2(human))
    bodies = {b.get("name"): b for b in v2.iter("body")}
    root = ET.Element("mujoco", model="RES624_ISOLATED_GRASP")
    ET.SubElement(root, "compiler", angle="radian", inertiafromgeom="false", autolimits="true")
    option = copy.deepcopy(v2.find("option"))
    option.set("timestep", str(spec.timestep_s))
    root.append(option)
    root.append(copy.deepcopy(v2.find("default")))
    world = ET.SubElement(root, "worldbody")
    world.append(copy.deepcopy(next(g for g in v2.find("worldbody").findall("geom") if g.get("name") == "floor")))
    actuator = ET.SubElement(root, "actuator")
    rot = _rotation(spec.tilt_deg)
    q = _quat(rot)
    half = AthleteBarbellSpec().grip_half_width_m
    v2_act = {a.get("name"): a for a in v2.find("actuator")}
    for side, y in (("l", half), ("r", -half)):
        hand = copy.deepcopy(bodies[side+"_hand"])
        for joint in list(hand.findall("joint")):
            hand.remove(joint)  # wrist removed: palm rigidly mounted (declared constraint)
        hand.set("pos", f"0 {y} {spec.mount_height_m}")
        hand.set("quat", " ".join(map(str, q)))
        world.append(hand)
        for j in HAND_JOINTS:
            world_act = copy.deepcopy(v2_act[f"{side}_{j}"])
            actuator.append(world_act)
    bar = copy.deepcopy(bodies["barbell"])
    eq = AthleteBarbellSpec(plate_mass_kg=0)
    scale = spec.bar_mass_kg/eq.total_load_kg
    inertial = bar.find("inertial")
    inertial.set("mass", str(spec.bar_mass_kg))
    inertial.set("diaginertia", " ".join(str(scale*v) for v in bar_inertia(eq)))
    hand_scale = human.profile.dimensions.hand_length_m/HumanV2Spec().profile.dimensions.hand_length_m
    local = np.array([-(PALM_HALF_THICKNESS*human.profile.stature_m/HumanV2Spec().profile.stature_m+SHAFT_RADIUS+spec.initial_gap_m), 0, spec.grip_along_palm_m*hand_scale])
    centre = np.array([0, 0, spec.mount_height_m]) + rot @ local
    bar.set("pos", " ".join(map(str, centre)))
    for site in list(bar.findall("site")):
        bar.remove(site)
    world.append(bar)
    if spec.friction is not None:
        for g in root.iter("geom"):
            if g.get("class") == "hand" or g.get("name") in ("bar_shaft", "bar_sleeve_l", "bar_sleeve_r"):
                g.set("friction", f"{spec.friction} .005 .0001")
    return ET.tostring(root, encoding="unicode")


class GraspFixture:
    """Thin plant-like adapter (model, data, ids, plain exports) for the fixture."""

    def __init__(self, spec=FixtureSpec()):
        self.spec = spec
        self.model = mujoco.MjModel.from_xml_string(build_fixture_mjcf(spec))
        self.data = mujoco.MjData(self.model)
        m, d = self.model, self.data
        for side in ("l", "r"):
            for j, v in (("finger_flex", spec.initial_finger_flex), ("finger_distal", spec.initial_finger_distal),
                         ("thumb_opposition", spec.initial_thumb)):
                d.qpos[self.joint_qpos(f"{side}_{j}")] = v
        mujoco.mj_forward(m, d)

    def joint_qpos(self, name):
        return int(self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)])

    def joint_dof(self, name):
        return int(self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)])

    def body_id(self, name):
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)

    def actuator_id(self, name):
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

    def set_grip(self, finger_frac, distal_frac=None, thumb_frac=0.0, thumb_target=None):
        """Bounded flexion torques as fractions of declared capacity (clipped by the engine)."""
        m, d = self.model, self.data
        distal_frac = finger_frac if distal_frac is None else distal_frac
        for side in ("l", "r"):
            for j, frac in (("finger_flex", finger_frac), ("finger_distal", distal_frac)):
                a = self.actuator_id(f"{side}_{j}")
                d.ctrl[a] = frac*m.actuator_forcerange[a, 1]
            a = self.actuator_id(f"{side}_thumb_opposition")
            cap = m.actuator_forcerange[a, 1]
            if thumb_target is None:
                d.ctrl[a] = thumb_frac*cap
            else:
                j = f"{side}_thumb_opposition"
                err = thumb_target-d.qpos[self.joint_qpos(j)]
                d.ctrl[a] = np.clip(10*cap*err-.25*cap*d.qvel[self.joint_dof(j)], -cap, cap)

    def mount_reaction(self):
        """World-on-palm reaction of each rigid mount (engine cfrc_int), world frame [force, torque@palm origin]."""
        m, d = self.model, self.data
        mujoco.mj_rnePostConstraint(m, d)
        out = {}
        for side in ("l", "r"):
            b = self.body_id(side+"_hand")
            c = d.cfrc_int[b]  # [torque, force] about subtree_com of the root, world orientation
            ref = d.subtree_com[m.body_rootid[b]]
            torque = c[:3] + np.cross(ref-d.xpos[b], c[3:])
            out[side] = dict(force_n=c[3:].tolist(), torque_about_palm_origin_n_m=torque.tolist())
        return out

    def export_frame(self):
        """Plain frame in the independent-oracle schema (bodies, contacts, gravity)."""
        m, d = self.model, self.data
        bodies = []
        v6 = np.zeros(6)
        for b in range(1, m.nbody):
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, b, v6, 0)
            r = d.ximat[b].reshape(3, 3)
            bodies.append(dict(name=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b), mass=float(m.body_mass[b]),
                               com=d.xipos[b].tolist(), v=v6[3:].tolist(), w=v6[:3].tolist(),
                               inertia_world=(r @ np.diag(m.body_inertia[b]) @ r.T).tolist()))
        contacts = []
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            mujoco.mj_contactForce(m, d, i, f6)
            fr = c.frame.reshape(3, 3)
            g1, g2 = int(c.geom1), int(c.geom2)
            contacts.append(dict(geom1=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g1),
                                 geom2=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g2),
                                 body1=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[g1])),
                                 body2=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[g2])),
                                 point=c.pos.tolist(), normal=fr[0].tolist(),
                                 force_on_body2=(f6[0]*fr[0]+f6[1]*fr[1]+f6[2]*fr[2]).tolist(),
                                 friction=float(c.friction[0]), dist=float(c.dist)))
        ext = [dict(body=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b), point=d.xipos[b].tolist(),
                    force=d.xfrc_applied[b, :3].tolist()) for b in range(1, m.nbody) if np.any(d.xfrc_applied[b])]
        return dict(t=float(d.time), bodies=bodies, contacts=contacts, grips=[], external=ext,
                    gravity=m.opt.gravity.tolist())


SETTLE_S = .5
# Classification bound: a hold-window bar displacement beyond one shaft radius
# means the shaft has left the contact patch that carried it; the trial
# reports the displacement and the classification at 0.1x and 10x this bound.
SLIP_BOUND_M = SHAFT_RADIUS


def run_trial(spec=FixtureSpec(), grip_frac=1.0, duration_s=1.5, release_at_s=None, release_frac=-.3,
              push_n=None, record_every=1, recorder=None):
    """Bounded-torque forward simulation of the isolated grasp; no bar writes after t=0."""
    from .grasp_diagnostics import GraspRecorder
    fx = GraspFixture(spec)
    m, d = fx.model, fx.data
    rec = recorder or GraspRecorder(fx)
    bar = fx.body_id("barbell")
    frames, positions, hand_contacts, times = [], [], [], []
    for k in range(round(duration_s/m.opt.timestep)):
        frac = grip_frac if release_at_s is None or d.time < release_at_s else release_frac
        fx.set_grip(frac, thumb_target=0.0)
        d.xfrc_applied[bar] = 0
        if push_n is not None and push_n[0] <= d.time < push_n[1]:
            d.xfrc_applied[bar, :3] = push_n[2]
        mujoco.mj_forward(m, d)
        if k % record_every == 0:
            rec.record()
            frames.append(fx.export_frame())
        positions.append(d.xipos[bar].copy())
        hand_contacts.append(sum(1 for c in d.contact[:d.ncon]
                                 if bar in (m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2])
                                 and 0 not in (m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2])))
        times.append(d.time)
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    return fx, rec, frames, np.array(positions), np.array(hand_contacts), np.array(times)


def classify_trial(positions, hand_contacts, times, *, settle_s=SETTLE_S, end_s=None, bound_m=SLIP_BOUND_M):
    end_s = times[-1] if end_s is None else end_s
    window = (times >= settle_s) & (times <= end_s)
    if not np.any(hand_contacts > 0):
        return dict(outcome="GRASP_GEOMETRY_INFEASIBLE", hold_displacement_m=None)
    ref = positions[np.argmax(window)]
    disp = float(np.max(np.linalg.norm(positions[window]-ref, axis=1)))
    persistent = bool(np.all(hand_contacts[window] > 0))
    supported = persistent and disp < bound_m
    return dict(outcome="GRASP_SUPPORTED" if supported else "GRASP_SLIP", contact_persistent=persistent,
                hold_displacement_m=disp, bound_m=bound_m,
                robust_at_tenth_and_tenfold_bound=[persistent and disp < .1*bound_m, persistent and disp < 10*bound_m])
