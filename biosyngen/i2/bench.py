"""Bench press: supine upper-body pressing with bench, foot, chest and rack supports (RES-495).

Topology, separate from the standing jumps and squats:

* The trunk (HAT) lies supine on a bench plane through two posterior contact
  points (upper back and pelvis). They form one friction patch.
* Both legs stay articulated. Each foot is flat on its own force plate, so
  each leg is a closed, zero-mobility chain between bench and floor. Leg
  joint torques ("leg drive") redistribute load between bench and feet.
* A lumped bilateral arm chain (shoulder, elbow) holds the bar through a
  revolute grip.
* The bar can touch the chest: a unilateral body-to-body contact whose
  normal is the trunk's anterior axis.
* The bar can rest on rack hooks: a unilateral world contact.

Each repetition is a declared control hypothesis:

1. setup with the bar on the hooks;
2. pre-tension (load share ramp);
3. unrack;
4. lowering to the chest, with an optional pause;
5. press to lockout;
6. lockout hold;
7. after the last repetition, rerack.

Latent events come from contact modes and bar mechanics: unrack (hook
release), lowering onset, chest touch (chest contact closes), bottom
reversal, press, sticking region, lockout and rerack (hook contact closes).

Force plates see the feet only. The whole-body support reaction needs the
bench reaction, which is reported separately; plate-only system kinetics are
refused. Internal wrist and shoulder joint forces and frontal-plane bar tilt
are NOT_EVALUABLE in this planar lumped-arm model.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math

from ..multibody import (
    ContactKind,
    ContactMode,
    ContactNormalFrame,
    ContactSpec,
    ContactState,
    MultibodyState,
    PlanarMultibodySystem,
)
from .human import (
    Anthropometry,
    HumanModel,
    JointStrength,
    LoadKind,
    LoadSpec,
    Posture,
    StrengthHypothesis,
    build_human,
)
from .instruments import CableLptModel, LptAttachment
from .phases import foot_force_z
from .simulate import ControlStage, ExerciseTrajectory, StagedJointController, simulate, static_activations

BENCH_CONTACTS = ("bench_upper_back", "bench_pelvis")
MIN_REP_TRAVEL_M = 0.05
NOT_EVALUABLE_CLAIMS = {
    "WRIST_INTERNAL_FORCE": "NOT_EVALUABLE: lumped planar arm chain has no wrist joint",
    "SHOULDER_INTERNAL_FORCE": "NOT_EVALUABLE: no scapulothoracic or glenohumeral contact model",
    "BAR_TILT": "NOT_EVALUABLE: planar model cannot represent frontal-plane bar tilt",
}


class BenchRefused(ValueError):
    """The requested bench-press configuration is not physically supported."""


@dataclass(frozen=True, slots=True)
class BenchProtocol:
    load_mass_kg: float = 60.0
    repetitions: int = 2
    bench_height_m: float = 0.45
    legs: tuple[float, float] = (0.15, -1.40)      # declared ankle and knee angles on the floor
    lockout_shoulder_elbow: tuple[float, float] = (math.pi / 2.0, 0.02)
    rack_drop_m: float = 0.03                      # hooks this far below the lockout bar height
    rack_headward_m: float = 0.15                  # hooks this far toward the head (over the forehead)
    hook_half_depth_m: float = 0.05                # finite J-hook ledge half-length
    chest_target_m: float = -0.003                 # bottom reference: bar this far "into" the chest plane (touch)
    setup_hold_s: float = 0.4
    pretension_s: float = 0.4
    unrack_s: float = 1.0
    lower_s: float = 1.2
    pause_s: float = 0.0
    press_s: float = 1.5
    lockout_hold_s: float = 0.5
    rerack_s: float = 0.6
    with_bench: bool = True
    leg_drive_n_m: float = 0.0                     # hip-extension torque per leg (declared; lifts the pelvis if large)
    press_shoulder_kp: float = 800.0
    press_elbow_kp: float = 800.0

    def __post_init__(self) -> None:
        if self.repetitions < 1 or not self.load_mass_kg > 0.0:
            raise BenchRefused("a bench set needs at least one repetition and a positive load")


def _gains(a, k, h, sh, el):
    return {"ankle": a, "knee": k, "hip": h, "shoulder": sh, "elbow": el}


@dataclass
class BenchScenario:
    model: HumanModel
    protocol: BenchProtocol
    controller: StagedJointController
    initial_state: MultibodyState
    duration_s: float
    postures: dict[str, Posture]
    metadata: dict = field(default_factory=dict)

    def run(self, dt_s: float = 0.001) -> ExerciseTrajectory:
        return simulate("BENCH_PRESS", self.model, self.controller, self.initial_state,
                        duration_s=self.duration_s, dt_s=dt_s, scenario=self.metadata)


def _supine_posture(model: HumanModel, ankle: float, knee: float, shoulder: float, elbow: float) -> Posture:
    """Hip angle that makes the trunk horizontal and supine (HAT angle pi/2)."""
    theta_thigh = -ankle - knee
    return Posture(ankle, knee, theta_thigh - math.pi / 2.0, shoulder, elbow)


def _arm_for_bar_height(model: HumanModel, legs, shoulder_ref, target_z, ankle_x, target_x=None) -> tuple[float, float]:
    """Shoulder/elbow angles that put the hand at (target_x, target_z).

    The default target x is the lockout hand x (vertical path); the bottom of
    the press uses the chest point x (a J-shaped bar path toward the hips).
    """
    ankle, knee = legs
    lockout = _supine_posture(model, ankle, knee, *shoulder_ref)
    x_target = model.hand_world(model.posture_state(lockout, ankle_x_m=ankle_x, check_bar_floor=False))[0]
    if target_x is not None:
        x_target = target_x
    shoulder, elbow = shoulder_ref
    for _ in range(60):
        state = model.posture_state(_supine_posture(model, ankle, knee, shoulder, elbow), ankle_x_m=ankle_x,
                                    check_bar_floor=False)
        x, z = model.hand_world(state)
        r = (x - x_target, z - target_z)
        if math.hypot(*r) < 1e-12:
            return shoulder, elbow
        eps = 1e-7
        xs, zs = model.hand_world(model.posture_state(_supine_posture(model, ankle, knee, shoulder + eps, elbow),
                                                      ankle_x_m=ankle_x, check_bar_floor=False))
        xe, ze = model.hand_world(model.posture_state(_supine_posture(model, ankle, knee, shoulder, elbow + eps),
                                                      ankle_x_m=ankle_x, check_bar_floor=False))
        j = ((xs - x) / eps, (xe - x) / eps, (zs - z) / eps, (ze - z) / eps)
        det = j[0] * j[3] - j[1] * j[2]
        if abs(det) < 1e-12:
            raise BenchRefused("arm posture for the requested bar height is singular")
        ds = (j[3] * r[0] - j[1] * r[1]) / det
        de = (-j[2] * r[0] + j[0] * r[1]) / det
        step = min(1.0, 0.2 / max(abs(ds), abs(de), 1e-12))
        shoulder -= step * ds
        elbow -= step * de
    raise BenchRefused("no arm posture reaches the requested bar height")


def build_bench_press(
    protocol: BenchProtocol | None = None,
    *,
    anthropometry: Anthropometry | None = None,
    strength: StrengthHypothesis | None = None,
) -> BenchScenario:
    p = protocol or BenchProtocol()
    if strength is None:
        # Declared pressing capacities for the lumped arms (both arms): the
        # hanging-arm defaults (shoulder 160, elbow-extension 120 N m) cannot
        # hold a 60 kg bar once the forearm tilts on the bar path.
        strength = replace(StrengthHypothesis(), shoulder=JointStrength(200.0, 240.0, 20.0, 20.0),
                           elbow=JointStrength(180.0, 150.0, 20.0, 20.0))
    model = build_human(anthropometry, strength, LoadSpec(LoadKind.GRIP_BAR, p.load_mass_kg, 0.08, floor_supported=False),
                        arms=True)
    hat = model.segments["hat"]
    # Posterior trunk points: local -x is posterior; along the trunk axis.
    back_depth = 0.10
    bench_points = {
        # Scapular support at shoulder level, so the bar load lies between the
        # bench contacts (a hip-side scapular point let the pelvis lift).
        "bench_upper_back": (-back_depth, hat.proximal_local_m[1] + 0.02),
        "bench_pelvis": (-back_depth, hat.distal_local_m[1] + 0.10),
    }
    chest_point = (0.12, hat.proximal_local_m[1] - 0.20)
    ankle, knee = p.legs
    probe = model.posture_state(_supine_posture(model, ankle, knee, *p.lockout_shoulder_elbow), check_bar_floor=False)
    # Translate vertically: lift the whole lying body so the back touches the bench
    # with the feet on the floor is not possible with fixed legs; instead the
    # declared bench height is matched by solving the knee angle.
    knee = _solve_knee_for_bench(model, ankle, knee, bench_points["bench_pelvis"], p.bench_height_m)
    posture_lock = _supine_posture(model, ankle, knee, *p.lockout_shoulder_elbow)
    state_lock = model.posture_state(posture_lock, check_bar_floor=False)
    bar_lock_z = model.hand_world(state_lock)[1]
    rack_z = bar_lock_z - p.rack_drop_m
    bar_lock_x = model.hand_world(state_lock)[0]
    rack_x = bar_lock_x - p.rack_headward_m   # head lies toward -x when supine
    shoulder_rack, elbow_rack = _arm_for_bar_height(model, (ankle, knee), p.lockout_shoulder_elbow, rack_z, 0.0, rack_x)
    chest_world = _point_on(state_lock, hat.body_id, chest_point)
    shoulder_bottom, elbow_bottom = _arm_for_bar_height(
        model, (ankle, knee), p.lockout_shoulder_elbow, chest_world[1] + p.chest_target_m, 0.0, chest_world[0]
    )
    contacts = list(model.system.contacts)
    if p.with_bench:
        for contact_id, local in bench_points.items():
            contacts.append(ContactSpec(contact_id, hat.body_id, local, (0.0, 1.0), 0.6, 0.0, ContactKind.BENCH,
                                        surface_point_m=(0.0, p.bench_height_m), friction_group="bench_back"))
    contacts.append(ContactSpec("bar_chest", model.load_body_id, (0.0, 0.0), (1.0, 0.0), 0.3, 0.0, ContactKind.CHEST,
                                body_b_id=hat.body_id, local_point_b_m=chest_point,
                                normal_frame=ContactNormalFrame.BODY_B))
    contacts.append(ContactSpec("rack_hooks", model.load_body_id, (0.0, 0.0), (0.0, 1.0), 0.4, 0.0, ContactKind.RACK,
                                surface_point_m=(rack_x, rack_z),
                                surface_extent_m=(-p.hook_half_depth_m, p.hook_half_depth_m)))
    system = model.system
    model.system = PlanarMultibodySystem(system.bodies, system.joints, tuple(contacts),
                                         gravity_m_s2=system.gravity_m_s2, clock_id=system.clock_id,
                                         actuators=system.actuators)
    setup = _supine_posture(model, ankle, knee, shoulder_rack, elbow_rack)
    base = model.posture_state(setup, check_bar_floor=False)
    active = dict(base.contacts)
    if p.with_bench:
        for contact_id in BENCH_CONTACTS:
            gap = model.system.contact_kinematics(base, contact_id).gap_m
            if abs(gap) > 1e-6:
                raise BenchRefused(f"{contact_id} does not meet the bench (gap {gap:.4g} m)")
            active[contact_id] = ContactState(ContactMode.STICK)
    active["rack_hooks"] = ContactState(ContactMode.STICK)
    activations = _arm_static_activations(model, base)
    initial = model.system.make_state(base.bodies, active_contacts=active, activations=activations)
    static = model.system.solve_accelerations(initial, excitations=dict(initial.activations))
    worst = max(abs(v) for acc in static.accelerations.values() for v in (*acc.linear_m_s2, acc.angular_rad_s2))
    if worst > 1e-6:
        raise BenchRefused(f"SUPPORT_INCOMPLETE: the declared supports cannot hold the lifter statically ({worst:.3g} m/s^2)")
    postures = {
        "setup": setup,
        "lockout": posture_lock,
        "bottom": _supine_posture(model, ankle, knee, shoulder_bottom, elbow_bottom),
    }
    arms_only = ("shoulder", "elbow")
    hold = _gains(400, 400, 400, 800, 800)
    # Leg drive: the legs form a zero-mobility loop, so a hip reference offset
    # yields a sustained hip-extension torque of kp * offset (declared).
    drive_offset = p.leg_drive_n_m / hold["hip"]
    setup = Posture(setup.ankle, setup.knee, setup.hip - drive_offset, setup.shoulder, setup.elbow)
    posture_lock = Posture(posture_lock.ankle, posture_lock.knee, posture_lock.hip - drive_offset,
                           posture_lock.shoulder, posture_lock.elbow)
    postures["bottom"] = Posture(postures["bottom"].ankle, postures["bottom"].knee,
                                 postures["bottom"].hip - drive_offset, postures["bottom"].shoulder, postures["bottom"].elbow)
    damp = _gains(40, 40, 40, 80, 80)
    press = _gains(400, 400, 400, p.press_shoulder_kp, p.press_elbow_kp)
    rack_hand = (rack_x, rack_z)
    lock_hand = (bar_lock_x, bar_lock_z)
    bottom_hand = (chest_world[0], chest_world[1] + p.chest_target_m)
    rerack_hand = (rack_x, rack_z - 0.005)   # aim slightly into the hooks so the bar settles on them
    hold_rack = _gains(400, 400, 400, 800, 800)
    stages = [
        ControlStage("SETUP_HOLD", setup, hold, damp, True, lambda m, s, t: t >= p.setup_hold_s,
                     "declared setup time", load_share=0.0, feedforward_joints=arms_only),
        ControlStage("PRETENSION", setup, hold, damp, True, lambda m, s, t: t >= p.pretension_s,
                     "declared pre-tension ramp", load_share=lambda t: t / p.pretension_s, feedforward_joints=arms_only),
        ControlStage("UNRACK", posture_lock, hold, damp, True, lambda m, s, t: t >= p.unrack_s,
                     "declared unrack time", feedforward_joints=arms_only,
                     reference_fn=bar_path(rack_hand, lock_hand, p.unrack_s, posture_lock)),
    ]
    for rep in range(1, p.repetitions + 1):
        stages.append(ControlStage(f"REP{rep}_LOWER", postures["bottom"], hold, damp, True,
                                   lambda m, s, t: t >= p.lower_s, "declared lowering tempo",
                                   feedforward_joints=arms_only,
                                   reference_fn=bar_path(lock_hand, bottom_hand, p.lower_s, posture_lock)))
        if p.pause_s > 0.0:
            stages.append(ControlStage(f"REP{rep}_PAUSE", postures["bottom"], hold, damp, True,
                                       lambda m, s, t: t >= p.pause_s, "declared pause",
                                       feedforward_joints=arms_only,
                                       reference_fn=bar_path(bottom_hand, bottom_hand, p.pause_s, posture_lock)))
        stages.append(ControlStage(f"REP{rep}_PRESS", posture_lock, press, damp, True,
                                   lambda m, s, t: t >= p.press_s, "declared press tempo",
                                   feedforward_joints=arms_only,
                                   reference_fn=bar_path(bottom_hand, lock_hand, p.press_s, posture_lock)))
        stages.append(ControlStage(f"REP{rep}_LOCKOUT", posture_lock, hold, damp, True,
                                   lambda m, s, t: t >= p.lockout_hold_s, "declared lockout hold",
                                   feedforward_joints=arms_only,
                                   reference_fn=bar_path(lock_hand, lock_hand, p.lockout_hold_s, posture_lock)))
    stages.append(ControlStage("RERACK", setup, hold_rack, damp, True, lambda m, s, t: t >= p.rerack_s,
                               "declared rerack time", feedforward_joints=arms_only,
                               reference_fn=bar_path(lock_hand, rerack_hand, p.rerack_s, posture_lock)))
    stages.append(ControlStage("RELEASE", setup, hold_rack, damp, True, None, "", feedforward_joints=arms_only,
                               load_share=lambda t: max(0.0, 1.0 - t / 0.3),
                               reference_fn=bar_path(rerack_hand, rerack_hand, 1.0, posture_lock)))
    controller = StagedJointController(stages, "I2_BENCH_STAGED_PD_V1")
    duration = (p.setup_hold_s + p.pretension_s + p.unrack_s
                + p.repetitions * (p.lower_s + p.pause_s + p.press_s + p.lockout_hold_s) + p.rerack_s + 0.4)
    metadata = {"exercise_id": "BENCH_PRESS", "protocol": p, "controller_id": controller.controller_id,
                "control_status": "LATENT_CONTROL_HYPOTHESIS", "parameter_status": "ASSUMED_UNCALIBRATED",
                "not_evaluable_claims": NOT_EVALUABLE_CLAIMS}
    return BenchScenario(model, p, controller, initial, duration, postures, metadata)


def arm_ik(model: HumanModel, state: MultibodyState, hand: tuple[float, float]) -> tuple[float, float]:
    """Closed-form two-link IK: (shoulder, elbow) joint angles placing the grip at ``hand``.

    Arm directions are d(theta) = (sin theta, -cos theta), so the grip offset
    from the shoulder is -i e^{i theta_u} (L1 + L2 e^{i e}). The elbow is taken
    in flexion (e >= 0), matching a supine press with the elbows toward the floor.
    """
    upper, fore = model.segments["upper_arm"], model.segments["forearm"]
    l1 = math.dist(upper.proximal_local_m, upper.distal_local_m)
    l2 = math.dist(fore.proximal_local_m, model.hand_local_m)
    shoulder = model.point_world(state, "upper_arm", upper.proximal_local_m)
    rx, rz = hand[0] - shoulder[0], hand[1] - shoulder[1]
    reach = math.hypot(rx, rz)
    cos_e = (reach * reach - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    if cos_e < -1.0 - 1e-6:
        raise BenchRefused(f"bar target inside the minimum arm reach ({reach:.3f} m)")
    # Beyond full reach the reference saturates at a straight arm aimed at the target.
    elbow = math.acos(max(-1.0, min(1.0, cos_e)))
    theta_r = math.atan2(rx, -rz)
    theta_upper = theta_r - math.atan2(l2 * math.sin(elbow), l1 + l2 * math.cos(elbow))
    shoulder_angle = theta_upper - state.bodies["hat"].angle_rad
    # Same branch as the current shoulder angle (angles are 2 pi periodic).
    current = model.system.joint_state(state, "shoulder")[0]
    shoulder_angle += 2.0 * math.pi * round((current - shoulder_angle) / (2.0 * math.pi))
    return shoulder_angle, elbow


def bar_path(start: tuple[float, float], end: tuple[float, float], duration_s: float, base: Posture):
    """Stage reference: minimum-jerk straight hand path resolved by arm IK."""

    def reference(model: HumanModel, state: MultibodyState, elapsed: float):
        def at(t: float) -> tuple[float, float]:
            u = max(0.0, min(1.0, t / duration_s))
            b = u ** 3 * (10.0 - 15.0 * u + 6.0 * u * u)
            return arm_ik(model, state, (start[0] + b * (end[0] - start[0]), start[1] + b * (end[1] - start[1])))

        h = 1e-4
        shoulder, elbow = at(elapsed)
        shoulder_n, elbow_n = at(elapsed + h)
        posture = Posture(base.ankle, base.knee, base.hip, shoulder, elbow)
        return posture, {"shoulder": (shoulder_n - shoulder) / h, "elbow": (elbow_n - elbow) / h}

    return reference


def _arm_static_activations(model: HumanModel, state: MultibodyState) -> dict[str, float]:
    """Arm-only static activations (the legs form a zero-mobility loop when supine)."""
    from .human import EXTENSION_SIGN

    torques = model.gravity_compensation(state, 0.0)
    out: dict[str, float] = {}
    for joint in ("shoulder", "elbow"):
        strength = model.strength.joint(joint)
        torque = torques[joint]
        if torque * EXTENSION_SIGN[joint] >= 0.0:
            out[f"{joint}_extensor"], out[f"{joint}_flexor"] = torque / strength.extensor_n_m, 0.0
        else:
            out[f"{joint}_flexor"], out[f"{joint}_extensor"] = torque / strength.flexor_n_m, 0.0
        if max(abs(out[f"{joint}_extensor"]), abs(out[f"{joint}_flexor"])) > 1.0:
            raise BenchRefused(f"UNSAFE_OR_INFEASIBLE_SETUP: {joint} cannot hold the arms statically")
    return out


def _point_on(state, body_id, local):
    b = state.bodies[body_id]
    c, s = math.cos(b.angle_rad), math.sin(b.angle_rad)
    return b.position_m[0] + c * local[0] - s * local[1], b.position_m[1] + s * local[0] + c * local[1]


def _solve_knee_for_bench(model, ankle, knee, pelvis_local, bench_height):
    """Knee angle that brings the posterior pelvis point onto the bench plane."""

    def gap(k):
        state = model.posture_state(_supine_posture(model, ankle, k, math.pi / 2.0, 0.0), check_bar_floor=False)
        return _point_on(state, "hat", pelvis_local)[1] - bench_height

    lo, hi = knee - 0.8, knee + 0.8
    g_lo, g_hi = gap(lo), gap(hi)
    if g_lo * g_hi > 0.0:
        raise BenchRefused("no knee angle places the trunk on the bench with feet on the floor")
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        g_mid = gap(mid)
        if (g_mid > 0.0) == (g_lo > 0.0):
            lo, g_lo = mid, g_mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


@dataclass(frozen=True, slots=True)
class BenchRep:
    index: int
    lowering_onset_s: float
    chest_touch_s: float | None
    bottom_reversal_s: float | None
    lockout_s: float | None
    bar_travel_m: float
    chest_impulse_n_s: float
    sticking_region_s: tuple[float, float] | None
    rack_contact_during_rep: bool
    bench_contact_lost: tuple[str, ...]
    refusals: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class BenchAnalysis:
    unrack_s: float | None
    rerack_s: float | None
    reps: tuple[BenchRep, ...]
    system_weight_n: float
    support_events: tuple[tuple[float, str, tuple[str, ...]], ...]
    refusals: tuple[str, ...]


def analyze_bench(trajectory: ExerciseTrajectory, *, bounce_impulse_n_s: float = 5.0) -> BenchAnalysis:
    model = trajectory.model
    frames = trajectory.frames
    refusals: list[str] = []
    if not trajectory.termination.startswith("DURATION_REACHED"):
        refusals.append(f"RUN_TERMINATED: {trajectory.termination}")
    times = [f.t_end_s for f in frames]
    bar_z = [f.state.bodies["load"].position_m[1] for f in frames]
    bar_v = [f.state.bodies["load"].velocity_m_s[1] for f in frames]
    racked = [f.state.contacts["rack_hooks"].mode is not ContactMode.OPEN for f in frames]
    unrack_i = next((i for i, r in enumerate(racked) if not r), None)
    unrack = times[unrack_i] if unrack_i is not None else None
    if unrack is None:
        refusals.append("BAR_NOT_UNRACKED")
    rerack = None
    if unrack_i is not None:
        rerack_i = next((i for i in range(unrack_i, len(frames)) if racked[i]), None)
        rerack = times[rerack_i] if rerack_i is not None else None
    reps: list[BenchRep] = []
    i = unrack_i or 0
    end_i = len(frames) if rerack is None else times.index(rerack)
    while i < end_i:
        onset = next((k for k in range(i, end_i) if bar_v[k] < -0.02), None)
        if onset is None:
            break
        bottom = next((k for k in range(onset, end_i - 1) if bar_v[k] < 0.0 <= bar_v[k + 1]), None)
        if bottom is None:
            break
        if bar_z[onset] - bar_z[bottom] < MIN_REP_TRAVEL_M:
            i = bottom + 1   # a lockout wobble is not a repetition
            continue
        top = next((k for k in range(bottom + 1, end_i) if bar_v[k] <= 0.01 and bar_z[k] > bar_z[bottom] + 0.5 * (bar_z[onset] - bar_z[bottom])), None)
        last = top if top is not None else end_i - 1
        touch = next((times[k] for k in range(onset, last + 1) if frames[k].state.contacts["bar_chest"].mode is not ContactMode.OPEN), None)
        impulse = sum(imp.impulse_world_n_s[0] ** 2 for imp in trajectory.impacts
                      if imp.contact_id == "bar_chest" and times[onset] <= imp.time_s <= times[last]) ** 0.5
        rack_hit = any(racked[k] for k in range(onset, last + 1))
        lost = tuple(
            c for c in BENCH_CONTACTS
            if c in frames[onset].state.contacts
            and any(frames[k].state.contacts[c].mode is ContactMode.OPEN for k in range(onset, last + 1))
        )
        peak = max(bar_v[bottom:last + 1])
        sticking = None
        for k in range(bottom + 2, last - 1):
            if bar_v[k - 1] < bar_v[k] >= bar_v[k + 1] and bar_v[k] > 0.1:
                m = next((j for j in range(k + 1, last) if bar_v[j] <= bar_v[j - 1] and bar_v[j] < bar_v[j + 1]), None)
                if m is not None and bar_v[k] - bar_v[m] >= 0.05 * peak and max(bar_v[m:last + 1]) >= bar_v[m] + 0.05 * peak:
                    sticking = (times[k], times[m])
                break
        rep_refusals = []
        if touch is None:
            rep_refusals.append("NO_CHEST_TOUCH: bottom reversal without chest contact (partial range)")
        if impulse > bounce_impulse_n_s:
            rep_refusals.append(f"BAR_BOUNCE: chest impact impulse {impulse:.2f} N s exceeds declared limit")
        if rack_hit:
            rep_refusals.append("SUPPORT_INCOMPLETE: bar loaded the rack hooks during the repetition")
        if lost:
            rep_refusals.append(f"BENCH_CONTACT_LOST: {', '.join(lost)} left the bench during the repetition")
        if top is None:
            rep_refusals.append("REP_NOT_COMPLETED")
        reps.append(BenchRep(len(reps) + 1, times[onset], touch, times[bottom], times[top] if top is not None else None,
                             bar_z[onset] - bar_z[bottom], impulse, sticking, rack_hit, lost, tuple(rep_refusals)))
        if top is None:
            break
        i = top + 1
    weight = model.total_mass_kg() * model.gravity_m_s2
    support_events = tuple(
        (e.time_s, e.event_id, tuple(c for c in e.contact_ids if c in BENCH_CONTACTS))
        for e in trajectory.events
        if e.event_id in ("CONTACT_RELEASE", "CONTACT_IMPACT") and any(c in BENCH_CONTACTS for c in e.contact_ids)
    )
    return BenchAnalysis(unrack, rerack, tuple(reps), weight, support_events, tuple(refusals))


def support_reaction(trajectory: ExerciseTrajectory, frame) -> dict[str, float]:
    """Vertical reaction per declared support group (feet, bench, rack) for one interval."""
    model = trajectory.model
    feet = foot_force_z(trajectory, frame)
    bench = sum(frame.contact_forces[c].force_world_n[1] for c in BENCH_CONTACTS if c in frame.contact_forces)
    rack = frame.contact_forces["rack_hooks"].force_world_n[1] if "rack_hooks" in frame.contact_forces else 0.0
    return {"feet": feet, "bench": bench, "rack": rack}


def system_kinetics_from_plates(plate_force_n: float, bench_reaction_n: float | None) -> float:
    if bench_reaction_n is None:
        raise BenchRefused("NOT_EVALUABLE: feet-only plates are incomplete; the bench reaction is required")
    return plate_force_n + bench_reaction_n


def claim(name: str) -> None:
    """Explicit refusal for claims the planar lumped-arm model cannot support."""
    raise BenchRefused(NOT_EVALUABLE_CLAIMS.get(name, f"NOT_EVALUABLE: unknown claim {name}"))


def bench_lpt(trajectory: ExerciseTrajectory) -> CableLptModel:
    model = trajectory.model
    bar_x = trajectory.initial_state.bodies[model.load_body_id].position_m[0]
    return CableLptModel(LptAttachment(model.load_body_id, (0.0, 0.0), 0.60, "BAR_COLLAR"), anchor_m=(bar_x, 0.60, 0.0))
