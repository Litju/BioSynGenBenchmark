"""RES-624 time-resolved hand-bar grasp instrumentation (generator side).

Recording never changes the state: it only calls kinematic/Jacobian queries
after the step's ``mj_forward``. A test replays the original RES-618 hold
with and without the recorder and requires bitwise-identical trajectories.

Each record holds, for the step's state and the loads applied over it:
bar pose/twist/momentum, hand and pad poses/velocities, signed pad-to-shaft
distances, compiled contact eligibility, every active hand/bar/floor contact
with normal and tangential force, cone utilization, slip velocity and
impulse, hand/wrist/shoulder actuator forces and saturation, floor support,
contact power and independently computed kinetic/potential energy.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

from .grasp_mechanics import object_wrench_feasibility, candidate_contacts, _tangents

PADS = ("palm_collision", "finger_flex_pad", "finger_distal_pad", "thumb_opposition_pad")
GRIP_JOINTS = ("finger_flex", "finger_distal", "thumb_opposition", "wrist_flex", "wrist_dev", "forearm_sup",
               "elbow_flex", "sh_flex", "sh_abd", "sh_rot")
SIDES = ("l", "r")


def _name(m, kind, i):
    return mujoco.mj_id2name(m, kind, int(i))


def _gid(m, name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)


def _bid(m, name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)


def pair_eligibility(m, g1, g2):
    """Compiled MuJoCo eligibility of a geom pair (bitmask, same body, parent filter, excludes).

    Rule (MuJoCo 3.x): (contype1 & conaffinity2) or (contype2 & conaffinity1);
    never within one body; parent/child bodies are filtered unless the parent
    is the world (filterparent enabled); explicit <exclude> body pairs drop it.
    """
    g1, g2 = (_gid(m, g) if isinstance(g, str) else int(g) for g in (g1, g2))
    b1, b2 = int(m.geom_bodyid[g1]), int(m.geom_bodyid[g2])
    bitmask = bool((m.geom_contype[g1] & m.geom_conaffinity[g2]) or (m.geom_contype[g2] & m.geom_conaffinity[g1]))
    same = b1 == b2
    filter_parent = not (m.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
    w1, w2 = int(m.body_weldid[b1]), int(m.body_weldid[b2])
    p1, p2 = int(m.body_weldid[m.body_parentid[w1]]), int(m.body_weldid[m.body_parentid[w2]])
    parent = filter_parent and w1 != 0 and w2 != 0 and (w1 == p2 or w2 == p1)
    welded = w1 == w2
    excluded = any({int(m.exclude_signature[k] >> 16), int(m.exclude_signature[k] & 0xFFFF)} == {b1, b2}
                   for k in range(m.nexclude))
    eligible = bitmask and not same and not parent and not welded and not excluded
    return dict(geom1=_name(m, mujoco.mjtObj.mjOBJ_GEOM, g1), geom2=_name(m, mujoco.mjtObj.mjOBJ_GEOM, g2),
                contype=[int(m.geom_contype[g1]), int(m.geom_contype[g2])],
                conaffinity=[int(m.geom_conaffinity[g1]), int(m.geom_conaffinity[g2])],
                bitmask=bitmask, same_body=same, parent_filtered=bool(parent), welded=welded,
                excluded=bool(excluded), eligible=bool(eligible))


def _point_velocity(m, d, body, point):
    jp = np.zeros((3, m.nv))
    mujoco.mj_jac(m, d, jp, None, np.asarray(point, float), int(body))
    return jp @ d.qvel


def contact_rows(m, d, dt):
    rows = []
    f6 = np.zeros(6)
    for i in range(d.ncon):
        c = d.contact[i]
        mujoco.mj_contactForce(m, d, i, f6)
        frame = c.frame.reshape(3, 3)
        world = f6[0]*frame[0] + f6[1]*frame[1] + f6[2]*frame[2]
        b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
        vrel = _point_velocity(m, d, b2, c.pos) - _point_velocity(m, d, b1, c.pos)
        slip = vrel - (vrel @ frame[0])*frame[0]
        tangent = math.hypot(f6[1], f6[2])
        mu = float(c.friction[0])
        rows.append(dict(geom1=_name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom1), geom2=_name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2),
                         body1=_name(m, mujoco.mjtObj.mjOBJ_BODY, b1), body2=_name(m, mujoco.mjtObj.mjOBJ_BODY, b2),
                         point=c.pos.tolist(), normal=frame[0].tolist(), force_on_body2=world.tolist(),
                         normal_force_n=float(f6[0]), tangent_force_n=float(tangent), friction=mu,
                         cone_utilization=float(tangent/(mu*f6[0])) if f6[0] > 0 and mu > 0 else (0.0 if tangent == 0 else math.inf),
                         slip_speed_m_s=float(np.linalg.norm(slip)), normal_speed_m_s=float(vrel @ frame[0]),
                         impulse_n_s=(world*dt).tolist(), power_w=float(world @ vrel), dist_m=float(c.dist)))
    return rows


def energies(m, d):
    mv = np.zeros(m.nv)
    mujoco.mj_mulM(m, d, mv, d.qvel)
    kinetic = .5*float(d.qvel @ mv)
    potential = -float(sum(m.body_mass[b]*(m.opt.gravity @ d.xipos[b]) for b in range(1, m.nbody)))
    return kinetic, potential


class GraspRecorder:
    """Collects per-step grasp evidence. Call :meth:`record` after the step's mj_forward."""

    def __init__(self, plant, bar="barbell", shaft="bar_shaft", hands=True):
        self.plant, self.m = plant, plant.model
        m = self.m
        self.bar, self.shaft = _bid(m, bar), _gid(m, shaft)
        self.weight = float(m.body_mass[self.bar]*np.linalg.norm(m.opt.gravity))
        self.pads = {s: {p: _gid(m, f"{s}_{p}") for p in PADS} for s in SIDES} if hands else {}
        self.eligibility = {s: {p: pair_eligibility(m, self.shaft, g) for p, g in pads.items()}
                            for s, pads in self.pads.items()}
        self.actuators = {}
        for a in range(m.nu):
            name = _name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
            if any(name.endswith("_"+j) for j in GRIP_JOINTS):
                self.actuators[name] = a
        self.records = []
        self.contact_work_j = 0.0
        self.actuator_work_j = 0.0

    def record(self, extra=None):
        m, d = self.m, self.plant.data
        dt = m.opt.timestep
        v6 = np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, self.bar, v6, 0)
        w, v = v6[:3].copy(), v6[3:].copy()
        rot = d.ximat[self.bar].reshape(3, 3)
        inertia = rot @ np.diag(m.body_inertia[self.bar]) @ rot.T
        mass = float(m.body_mass[self.bar])
        contacts = contact_rows(m, d, dt)
        bar_name = _name(m, mujoco.mjtObj.mjOBJ_BODY, self.bar)
        on_bar = []
        for c in contacts:
            if bar_name in (c["body1"], c["body2"]):
                sign = 1 if c["body2"] == bar_name else -1
                other = c["geom1"] if c["body2"] == bar_name else c["geom2"]
                on_bar.append(dict(c, other=other, force_on_bar=(sign*np.asarray(c["force_on_body2"])).tolist()))
        bar_force = sum((np.asarray(c["force_on_bar"]) for c in on_bar), np.zeros(3))
        com = d.xipos[self.bar]
        bar_moment = sum((np.cross(np.asarray(c["point"])-com, c["force_on_bar"]) for c in on_bar), np.zeros(3))
        hands = {}
        for s, pads in self.pads.items():
            h = _bid(m, s+"_hand")
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, h, v6, 0)
            dist = {}
            for p, g in pads.items():
                fromto = np.zeros(6)
                dist[p] = float(mujoco.mj_geomDistance(m, d, self.shaft, g, .2, fromto))
            hands[s] = dict(position=d.xpos[h].tolist(), orientation=d.xquat[h].tolist(),
                            palm_normal=(-d.xmat[h].reshape(3, 3)[:, 0]).tolist(),
                            linear_velocity=v6[3:].tolist(), angular_velocity=v6[:3].tolist(),
                            pad_shaft_distance_m=dist,
                            pad_positions={p: d.geom_xpos[g].tolist() for p, g in pads.items()})
        floor = [c for c in contacts if "world" in (c["body1"], c["body2"])]
        floor_force = sum((np.asarray(c["force_on_body2"])*(1 if c["body1"] == "world" else -1) for c in floor), np.zeros(3))
        actuators = {}
        for name, a in self.actuators.items():
            f = float(d.actuator_force[a])
            actuators[name] = dict(force=f, capacity=float(m.actuator_forcerange[a, 1]),
                                   saturated=bool(abs(f) >= m.actuator_forcerange[a, 1]*(1-1e-9)))
        kinetic, potential = energies(m, d)
        contact_power = float(sum(c["power_w"] for c in contacts))
        actuator_power = float(d.actuator_force @ d.actuator_velocity)
        self.contact_work_j += contact_power*dt
        self.actuator_work_j += actuator_power*dt
        ext = d.xfrc_applied[self.bar].copy()
        support = float(bar_force[2]/self.weight)
        rec = dict(step=len(self.records), t=float(d.time),
                   bar=dict(position=com.tolist(), orientation=d.xquat[self.bar].tolist(), linear_velocity=v.tolist(),
                            angular_velocity=w.tolist(), linear_momentum=(mass*v).tolist(),
                            angular_momentum_about_com=(inertia @ w).tolist(),
                            axial_spin_rad_s=float(w @ rot[:, 1]),
                            contact_force_n=bar_force.tolist(), contact_moment_about_com_n_m=bar_moment.tolist(),
                            applied_external_n=ext[:3].tolist(), support_ratio=support,
                            active_contacts=len(on_bar),
                            active_hand_contacts=sum(1 for c in on_bar if c["other"] and c["other"][:2] in ("l_", "r_"))),
                   bar_contacts=on_bar, hands=hands,
                   floor=dict(count=len(floor), force_on_human_n=floor_force.tolist()),
                   actuators=actuators, saturation_count=int(np.count_nonzero(
                       np.abs(d.actuator_force) >= m.actuator_forcerange[:, 1]*(1-1e-9))),
                   energy=dict(kinetic_j=kinetic, potential_j=potential, contact_power_w=contact_power,
                               contact_work_j=self.contact_work_j, actuator_power_w=actuator_power,
                               actuator_work_j=self.actuator_work_j))
        if extra:
            rec.update(extra)
        self.records.append(rec)
        return rec


def support_chronology(records, threshold=.5):
    """First established / first lost support and contact events.

    ``threshold`` is a *diagnostic marker* of the vertical hand-contact force
    as a fraction of bar weight (default one half); it is not an acceptance
    criterion and is reported with the full support-ratio series.
    """
    ratio = [r["bar"]["support_ratio"] for r in records]
    hand = [r["bar"]["active_hand_contacts"] for r in records]
    established = next((i for i, v in enumerate(ratio) if v >= threshold), None)
    lost = None
    if established is not None:
        lost = next((i for i in range(established, len(ratio)) if ratio[i] < threshold), None)
    contact_lost = next((i for i, n in enumerate(hand) if n == 0), None)
    regained = next((i for i in range(contact_lost, len(hand)) if hand[i] > 0), None) if contact_lost is not None else None
    t = [r["t"] for r in records]
    return dict(threshold_fraction_of_weight=threshold,
                support_established_step=established, support_established_t=None if established is None else t[established],
                support_lost_step=lost, support_lost_t=None if lost is None else t[lost],
                state="NEVER_ESTABLISHED" if established is None else ("LOST" if lost is not None else "SUSTAINED"),
                first_zero_hand_contact_step=contact_lost,
                first_zero_hand_contact_t=None if contact_lost is None else t[contact_lost],
                first_hand_recontact_step=regained, first_hand_recontact_t=None if regained is None else t[regained],
                max_support_ratio=max(ratio), min_support_ratio=min(ratio),
                initial_support_ratio=ratio[0], final_hand_contacts=hand[-1])


def diagnose_initial_support(plant, margin_m=.001, bar="barbell", shaft="bar_shaft"):
    """Evidence-ordered failure class for the initial grasp (before any step).

    1. compiled eligibility of every pad/shaft pair;
    2. geometric closure: pads within ``margin_m`` of the shaft;
    3. topology: can *any* admissible pyramid forces at the active hand
       contacts balance the bar (unbounded normal force)?  and at the
       contacts within margin (palm included if it nearly touches)?
    4. actual engine forces at t=0 against the required wrench;
    5. actuator saturation at t=0.
    Each test is independent of the pose optimizer residual.
    """
    m, d = plant.model, plant.data
    mujoco.mj_forward(m, d)
    rec = GraspRecorder(plant, bar, shaft)
    elig = rec.eligibility
    any_eligible = any(e["eligible"] for pads in elig.values() for e in pads.values())
    b = rec.bar
    mass = float(m.body_mass[b])
    com = d.xipos[b].copy()
    pad_ids = [g for pads in rec.pads.values() for g in pads.values()]
    near = [c for c in candidate_contacts(m, d, pad_ids+[rec.shaft], margin_m) if rec.shaft in (c["geom1"], c["geom2"])
            and (c["geom1"] in pad_ids or c["geom2"] in pad_ids)]
    active = [c for c in near if c["dist"] <= 0]

    def into_bar(c):
        # MuJoCo normal points from geom1 to geom2; force on body2 is +normal.
        return np.asarray(c["normal"]) if c["body2"] == b else -np.asarray(c["normal"])

    topo_active = object_wrench_feasibility([c["point"] for c in active], [into_bar(c) for c in active],
                                            [c["mu"] for c in active], com, mass, m.opt.gravity)
    topo_near = object_wrench_feasibility([c["point"] for c in near], [into_bar(c) for c in near],
                                          [c["mu"] for c in near], com, mass, m.opt.gravity)
    upward = [float(into_bar(c)[2]) for c in near]
    actual = rec.record()
    required = -mass*np.asarray(m.opt.gravity)
    if not any_eligible:
        failure = "NO_ELIGIBLE_CONTACT_PAIR"
    elif not near:
        failure = "CONTACT_GEOMETRY_CANNOT_CLOSE"
    elif not topo_near["feasible"]:
        failure = "INCORRECT_RELATIVE_ORIENTATION_NO_SUPPORTING_CONTACT_TOPOLOGY"
    elif not topo_active["feasible"]:
        failure = "INSUFFICIENT_OPPOSED_NORMAL_FORCES_AT_ACTIVE_CONTACTS"
    elif actual["bar"]["support_ratio"] < .5:
        failure = "INCONSISTENT_CONTROLLER_OR_INITIAL_EQUILIBRIUM"
    else:
        failure = "INITIAL_SUPPORT_PRESENT"
    hand_names = {s: _bid(m, s+"_hand") for s in rec.pads}
    return dict(failure_class=failure, eligibility=elig, any_eligible_pair=any_eligible,
                closure_margin_m=margin_m,
                pads_within_margin=sorted({_name(m, mujoco.mjtObj.mjOBJ_GEOM, c["geom2"] if c["geom1"] == rec.shaft else c["geom1"]) for c in near}),
                pads_in_contact=sorted({_name(m, mujoco.mjtObj.mjOBJ_GEOM, c["geom2"] if c["geom1"] == rec.shaft else c["geom1"]) for c in active}),
                contact_normal_vertical_components_into_bar=upward,
                topology_active=topo_active, topology_within_margin=topo_near,
                palm_normal_elevation_deg={s: float(math.degrees(math.asin(np.clip(-d.xmat[h].reshape(3, 3)[2, 0], -1, 1))))
                                           for s, h in hand_names.items()},
                required_bar_support_force_n=required.tolist(),
                actual_initial_contact_force_n=actual["bar"]["contact_force_n"],
                actual_initial_support_ratio=actual["bar"]["support_ratio"],
                initial_saturation_count=actual["saturation_count"])
