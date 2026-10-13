"""Deadlift: pre-tension, bar-floor support transfer, lift, lockout and return (RES-494).

Topology: the bilateral human with a lumped arm chain holds the bar through a
revolute GRIP joint at the hands. The bar starts on the floor, supported by a
unilateral frictional bar-floor contact at the plate rim, outside the force
plates. The athlete's feet stay on separate plates.

Mechanics, not triggers:

* Pre-tension. A declared feedforward share of the bar weight ramps from 0
  to 1 while the posture reference is unchanged. The arms then pull, so the
  bar-floor reaction falls and the plate force rises while the bar has not
  moved. Plate force rise therefore precedes bar motion physically.
* Lift-off is the instant the bar-floor normal reaction reaches zero (the
  contact opens). It is never inferred from bar position or a force threshold.
* The return lowers the bar to the floor, where it touches down (impact or
  grazing touchdown) and the share ramps back to 0 (reset).

Instruments: plates measure foot contact only. During setup and pre-tension
they do not carry the bar weight. A whole-system wrench (plates plus
bar-floor reaction) is evaluable only if the bar-floor reaction is
independently known; ``system_force_from_plates`` refuses otherwise.

The bar is rigid. Bar bending/whip, plate rolling and grip slack are
NOT_IMPLEMENTED, and the force rise before motion comes only from load
transfer, not from bar elasticity.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from ..multibody import ContactMode, MultibodyState
from .human import (
    Anthropometry,
    HumanModel,
    LoadKind,
    LoadSpec,
    Posture,
    StrengthHypothesis,
    build_human,
)
from .instruments import CableLptModel, LptAttachment
from .phases import foot_force_z
from .simulate import ControlStage, ExerciseTrajectory, StagedJointController, simulate, static_activations


class DeadliftRefused(ValueError):
    """The requested deadlift configuration is not physically supported."""


@dataclass(frozen=True, slots=True)
class DeadliftProtocol:
    load_mass_kg: float = 100.0
    setup_ankle: float = 0.45
    setup_arm_angle_rad: float = -0.10
    bar_over_ankle_x_m: float = 0.05
    lockout: Posture = Posture(0.05, -0.05, 0.02, 0.0, 0.0)
    setup_hold_s: float = 0.5
    pretension_s: float = 0.6
    lift_s: float = 1.4
    lockout_hold_s: float = 0.6
    lower_s: float = 1.6
    reset_s: float = 0.6
    bar_floor_friction: float = 0.5

    def __post_init__(self) -> None:
        if not self.load_mass_kg > 0.0:
            raise DeadliftRefused("a deadlift requires a positive bar load")


def _gains(a, k, h, sh=600.0, el=600.0):
    return {"ankle": a, "knee": k, "hip": h, "shoulder": sh, "elbow": el}


def solve_setup(model: HumanModel, ankle: float, arm_angle_rad: float, bar_x: float) -> Posture:
    """Knee and hip angles that put straight-armed hands on the floor bar.

    The ankle angle and the absolute arm angle (0 = hanging straight down,
    negative = hands behind the shoulder) are declared. Hand x = bar x and
    hand z = plate radius are solved for (knee, hip) by Newton iteration with
    finite-difference Jacobians; the shoulder angle follows from the arm angle.
    """
    target = (bar_x, model.load.plate_radius_m)
    knee, hip = -1.6, 1.6

    def hand(k, h):
        theta_hat = (-ankle - k) - h
        state = model.posture_state(Posture(ankle, k, h, arm_angle_rad - theta_hat, 0.0), check_bar_floor=False)
        return model.hand_world(state)

    for _ in range(60):
        x, z = hand(knee, hip)
        r = (x - target[0], z - target[1])
        if math.hypot(*r) < 1e-12:
            break
        eps = 1e-7
        xk, zk = hand(knee + eps, hip)
        xh, zh = hand(knee, hip + eps)
        j11, j12 = (xk - x) / eps, (xh - x) / eps
        j21, j22 = (zk - z) / eps, (zh - z) / eps
        det = j11 * j22 - j12 * j21
        if abs(det) < 1e-12:
            raise DeadliftRefused("setup posture is kinematically singular")
        step_k = (j22 * r[0] - j12 * r[1]) / det
        step_h = (-j21 * r[0] + j11 * r[1]) / det
        scale = min(1.0, 0.3 / max(abs(step_k), abs(step_h), 1e-12))
        knee -= scale * step_k
        hip -= scale * step_h
    else:
        raise DeadliftRefused("no setup posture reaches the floor bar")
    theta_hat = (-ankle - knee) - hip
    return Posture(ankle, knee, hip, arm_angle_rad - theta_hat, 0.0)


@dataclass
class DeadliftScenario:
    model: HumanModel
    protocol: DeadliftProtocol
    controller: StagedJointController
    initial_state: MultibodyState
    duration_s: float
    setup_posture: Posture
    metadata: dict = field(default_factory=dict)

    def run(self, dt_s: float = 0.001) -> ExerciseTrajectory:
        return simulate("DEADLIFT", self.model, self.controller, self.initial_state,
                        duration_s=self.duration_s, dt_s=dt_s, scenario=self.metadata)


def build_deadlift(
    protocol: DeadliftProtocol | None = None,
    *,
    anthropometry: Anthropometry | None = None,
    strength: StrengthHypothesis | None = None,
) -> DeadliftScenario:
    p = protocol or DeadliftProtocol()
    load = LoadSpec(LoadKind.GRIP_BAR, p.load_mass_kg, 0.08, floor_friction=p.bar_floor_friction)
    model = build_human(anthropometry, strength, load, arms=True)
    setup = solve_setup(model, p.setup_ankle, p.setup_arm_angle_rad, p.bar_over_ankle_x_m)
    probe = model.posture_state(setup)
    try:
        activations = static_activations(model, probe, load_share=0.0)
    except Exception as exc:
        raise DeadliftRefused(f"UNSAFE_OR_INFEASIBLE_SETUP: {exc}") from exc
    initial = model.posture_state(setup, activations=activations)
    if initial.contacts[model.bar_floor_contact_id].mode is ContactMode.OPEN:
        raise DeadliftRefused("setup does not place the bar on its floor support")
    kb, db = 3000.0, 1200.0
    hold = _gains(300, 1500, 1500)
    damp = _gains(60, 150, 150, 60, 60)
    stages = [
        ControlStage("SETUP_HOLD", setup, hold, damp, True,
                     lambda m, s, t: t >= p.setup_hold_s, "declared setup time", 0.0, kb, db, load_share=0.0),
        ControlStage("PRETENSION", setup, hold, damp, True,
                     lambda m, s, t: t >= p.pretension_s, "declared pre-tension ramp elapsed", 0.0, kb, db,
                     load_share=lambda t: t / p.pretension_s),
        ControlStage("LIFT", p.lockout, _gains(300, 2500, 2500), damp, True,
                     lambda m, s, t: t >= p.lift_s, "declared lift tempo elapsed", p.lift_s, kb, db),
        ControlStage("LOCKOUT_HOLD", p.lockout, hold, damp, True,
                     lambda m, s, t: t >= p.lockout_hold_s, "declared lockout hold", 0.0, kb, db),
        ControlStage("LOWER", setup, _gains(300, 2500, 2500), damp, True,
                     lambda m, s, t: t >= p.lower_s, "declared lowering tempo elapsed", p.lower_s, kb, db),
        ControlStage("RESET", setup, hold, damp, True, None, "", 0.0, kb, db,
                     load_share=lambda t: max(0.0, 1.0 - t / p.reset_s)),
    ]
    ankle_x = model.point_world(initial, "left_foot", model.foot_points["ankle"])[0]
    from .simulate import supported_com
    controller = StagedJointController(
        stages, "I2_DEADLIFT_STAGED_PD_V1",
        balance_reference_x_m=supported_com(model, initial, 0.0)[0] - ankle_x,
    )
    duration = p.setup_hold_s + p.pretension_s + p.lift_s + p.lockout_hold_s + p.lower_s + p.reset_s
    metadata = {
        "exercise_id": "DEADLIFT", "protocol": p, "controller_id": controller.controller_id,
        "control_status": "LATENT_CONTROL_HYPOTHESIS", "parameter_status": "ASSUMED_UNCALIBRATED",
        "bar_model": "RIGID_BAR_NO_WHIP", "grip": "REVOLUTE_GRIP_NO_SLACK",
    }
    return DeadliftScenario(model, p, controller, initial, duration, setup, metadata)


@dataclass(frozen=True, slots=True)
class DeadliftAnalysis:
    pretension_onset_s: float | None
    lift_off_s: float | None
    bar_passes_knee_s: float | None
    lockout_s: float | None
    descent_onset_s: float | None
    bar_floor_return_s: float | None
    peak_bar_height_m: float
    plate_rise_before_lift_n: float | None
    body_weight_n: float
    system_weight_n: float
    refusals: tuple[str, ...]


def analyze_deadlift(trajectory: ExerciseTrajectory) -> DeadliftAnalysis:
    model = trajectory.model
    system = model.system
    g = model.gravity_m_s2
    total = model.total_mass_kg()
    bar_mass = model.load.mass_kg
    body_weight = (total - bar_mass) * g
    frames = trajectory.frames
    contact = model.bar_floor_contact_id
    refusals: list[str] = []
    if not trajectory.termination.startswith("DURATION_REACHED"):
        refusals.append(f"RUN_TERMINATED: {trajectory.termination}")
    fz = [foot_force_z(trajectory, f) for f in frames]
    onset = next((f.t_start_s for f, v in zip(frames, fz) if v > body_weight * (1 + 1e-6)), None)
    lift_i = next((i for i, f in enumerate(frames) if f.state.contacts[contact].mode is ContactMode.OPEN), None)
    lift = frames[lift_i].t_end_s if lift_i is not None else None
    if lift is None:
        refusals.append("LOAD_NOT_LIFTED: the bar-floor reaction never reached zero")
    bar_z = [f.state.bodies[model.load_body_id].position_m[1] for f in frames]
    bar_v = [f.state.bodies[model.load_body_id].velocity_m_s[1] for f in frames]
    knee_pass = lockout = descent = None
    if lift_i is not None:
        for i in range(lift_i, len(frames)):
            knee_z = model.point_world(frames[i].state, "left_shank", model.segments["left_shank"].proximal_local_m)[1]
            if knee_pass is None and bar_z[i] >= knee_z:
                knee_pass = frames[i].t_end_s
            if lockout is None and i > lift_i + 5 and abs(bar_v[i]) <= 0.01 and bar_z[i] > 0.5 * max(bar_z):
                lockout = frames[i].t_end_s
            if lockout is not None and descent is None and bar_v[i] < -0.02:
                descent = frames[i].t_end_s
                break
    ret = None
    if descent is not None:
        for event in trajectory.events:
            if event.time_s > descent and contact in event.contact_ids and event.event_id in ("CONTACT_IMPACT", "CONTACT_TOUCHDOWN"):
                ret = event.time_s
                break
        if ret is None:
            ret = next((f.t_end_s for f in frames if f.t_end_s > descent and f.state.contacts[contact].mode is not ContactMode.OPEN), None)
    rise = None
    if lift_i is not None:
        rise = fz[max(0, lift_i - 1)] - body_weight
    return DeadliftAnalysis(onset, lift, knee_pass, lockout, descent, ret, max(bar_z) - model.load.plate_radius_m,
                            rise, body_weight, total * g, tuple(refusals))


def system_force_from_plates(plate_force_n: float, bar_floor_reaction_n: float | None) -> float:
    """Whole-system vertical external force, if the bar-floor reaction is known.

    The plates measure foot contact only. While the bar may touch the floor,
    the system force needs an independently measured bar-floor reaction, so
    this refuses rather than equating the plate force with the system force.
    """
    if bar_floor_reaction_n is None:
        raise DeadliftRefused(
            "NOT_EVALUABLE: bar-floor reaction is not independently measured; plates alone do not give the system wrench"
        )
    return plate_force_n + bar_floor_reaction_n


def deadlift_lpt(trajectory: ExerciseTrajectory, *, routed_to: str = "BAR_COLLAR",
                 max_retraction_speed_m_s: float = 6.0) -> CableLptModel:
    """Bar-collar LPT with the reel under the collar; HIP_BELT is a routing error control."""
    model = trajectory.model
    bar_x = trajectory.initial_state.bodies[model.load_body_id].position_m[0]
    if routed_to == "BAR_COLLAR":
        attachment = LptAttachment(model.load_body_id, (0.0, 0.0), 0.60, "BAR_COLLAR")
    elif routed_to == "HIP_BELT":
        attachment = LptAttachment("hat", model.hat_points["hip_belt"], 0.0, "HIP_BELT_MISROUTED")
    else:
        raise DeadliftRefused(f"unknown LPT routing {routed_to}")
    return CableLptModel(attachment, anchor_m=(bar_x, 0.60, 0.0), max_retraction_speed_m_s=max_retraction_speed_m_s)
