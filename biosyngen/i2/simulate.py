"""Latent trajectory recording and declared joint control for I2 exercises.

The recorder stores what the engine actually applied in each step:

* contact impulses (dynamic plus velocity projection), converted to
  interval-average forces;
* impact impulses, kept as Dirac impulses at the impact time;
* the end-of-step state, contact modes, activations, actuator and passive
  joint torques.

Nothing here is a measurement. The instruments module maps these latent
records to simulated device exports.

Control is a declared hypothesis (LATENT_CONTROL_HYPOTHESIS). A stage holds a
reference posture, PD gains and a gravity-compensation flag. Stages switch
on proprioceptive predicates (joint angles and rates), touch (contact
modes) or declared command times. Movement events and phases are never
read from the controller; they are derived from forces and states in
``phases.py``.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
import math

from ..multibody import (
    AppliedWrench,
    ConstraintSolveError,
    ContactEvent,
    ContactMode,
    MultibodyState,
)
from .human import EXTENSION_SIGN, JOINTS, SIDES, HumanModel, Posture


@dataclass(frozen=True, slots=True)
class JointLimit:
    """Passive end-range torque: k * excess^2 + c * excess rate (declared)."""

    lower_rad: float
    upper_rad: float
    stiffness_n_m_per_rad2: float = 5000.0
    damping_n_m_s_per_rad: float = 20.0

    def torque(self, angle: float, rate: float) -> float:
        if angle < self.lower_rad:
            excess = self.lower_rad - angle
            return self.stiffness_n_m_per_rad2 * excess * excess + self.damping_n_m_s_per_rad * max(0.0, -rate)
        if angle > self.upper_rad:
            excess = angle - self.upper_rad
            return -(self.stiffness_n_m_per_rad2 * excess * excess + self.damping_n_m_s_per_rad * max(0.0, rate))
        return 0.0


DEFAULT_LIMITS = {
    "ankle": JointLimit(-0.90, 0.75),
    "knee": JointLimit(-2.50, -0.01),
    "hip": JointLimit(-0.35, 2.30),
}


@dataclass(frozen=True, slots=True)
class ControlStage:
    name: str
    reference: Posture
    kp_n_m_per_rad: Mapping[str, float]
    kd_n_m_s_per_rad: Mapping[str, float]
    gravity_compensation: bool = True
    # Predicate on (model, state, time in stage) that ends this stage.
    exit_when: Callable[[HumanModel, MultibodyState, float], bool] | None = None
    exit_reason: str = ""
    # Minimum-jerk blend from the posture at stage entry to the reference.
    transition_s: float = 0.0
    # Ankle-strategy balance: extension torque k (x_com - x_ref) + d v_com_x
    # per ankle while both feet are supported (declared hypothesis).
    balance_gain_n_m_per_m: float = 0.0
    balance_damping_n_m_s_per_m: float = 0.0
    # Maximum excitation magnitude per joint (declared effort/coordination).
    effort_cap: Mapping[str, float] | None = None
    # Hip strategy: the same COM error drives hip extension torque (declared).
    hip_balance_gain_n_m_per_m: float = 0.0
    hip_balance_damping_n_m_s_per_m: float = 0.0


@dataclass
class StagedJointController:
    """Joint-space PD plus static feedforward, with declared stage switching."""

    stages: Sequence[ControlStage]
    controller_id: str
    # COM target relative to the mean ankle x of the supporting feet.
    balance_reference_x_m: float = 0.0
    stage_index: int = 0
    stage_start_s: float = 0.0
    commands: list[tuple[float, str, str]] = field(default_factory=list)
    _entry_posture: dict[str, dict[str, float]] = field(default_factory=dict)

    @property
    def stage(self) -> ControlStage:
        return self.stages[self.stage_index]

    def update_stage(self, model: HumanModel, state: MultibodyState) -> None:
        while self.stage_index < len(self.stages) - 1:
            stage = self.stage
            if stage.exit_when is None or not stage.exit_when(model, state, state.time_s - self.stage_start_s):
                return
            self.stage_index += 1
            self.stage_start_s = state.time_s
            self._entry_posture = {
                side: {joint: angle for joint, (angle, _) in model.joint_angles(state, side).items()}
                for side in SIDES
            }
            self.commands.append((state.time_s, self.stage.name, stage.exit_reason))

    def excitations(self, model: HumanModel, state: MultibodyState) -> dict[str, float]:
        self.update_stage(model, state)
        stage = self.stage
        feet_supported = all(
            any(state.contacts[c].mode is not ContactMode.OPEN for c in model.contact_ids[side])
            for side in SIDES
        )
        feedforward = model.gravity_compensation(state) if (stage.gravity_compensation and feet_supported) else {}
        balance = hip_balance = 0.0
        if feet_supported and (stage.balance_gain_n_m_per_m or stage.hip_balance_gain_n_m_per_m):
            mass = model.total_mass_kg()
            com_x = model.system.center_of_mass(state)[0]
            com_vx = model.system.linear_momentum(state)[0] / mass
            # The reference is relative to the current support (mean ankle x),
            # not a fixed world point: after a jump the feet land elsewhere.
            ankle_x = 0.5 * sum(
                model.point_world(state, f"{side}_foot", model.foot_points["ankle"])[0] for side in SIDES
            )
            error = com_x - (ankle_x + self.balance_reference_x_m)
            balance = stage.balance_gain_n_m_per_m * error + stage.balance_damping_n_m_s_per_m * com_vx
            hip_balance = stage.hip_balance_gain_n_m_per_m * error + stage.hip_balance_damping_n_m_s_per_m * com_vx
        elapsed = state.time_s - self.stage_start_s
        if stage.transition_s > 0.0 and elapsed < stage.transition_s:
            s_ = elapsed / stage.transition_s
            blend = s_ ** 3 * (10.0 - 15.0 * s_ + 6.0 * s_ * s_)
            blend_rate = 30.0 * s_ ** 2 * (1.0 - s_) ** 2 / stage.transition_s
        else:
            blend, blend_rate = 1.0, 0.0
        out: dict[str, float] = {}
        for side in SIDES:
            angles = model.joint_angles(state, side)
            entry = self._entry_posture.get(side)
            for joint in JOINTS:
                angle, rate = angles[joint]
                target = getattr(stage.reference, joint)
                start = entry[joint] if entry is not None else target
                reference = start + blend * (target - start)
                reference_rate = blend_rate * (target - start)
                torque = (
                    feedforward.get(f"{side}_{joint}", 0.0)
                    + stage.kp_n_m_per_rad[joint] * (reference - angle)
                    + stage.kd_n_m_s_per_rad[joint] * (reference_rate - rate)
                )
                if joint == "ankle":
                    torque += EXTENSION_SIGN["ankle"] * balance
                elif joint == "hip":
                    torque += EXTENSION_SIGN["hip"] * hip_balance
                strength = model.strength.joint(joint)
                scale = model.strength.left_scale if side == "left" else model.strength.right_scale
                cap = 1.0 if stage.effort_cap is None else stage.effort_cap.get(joint, 1.0)
                if torque * EXTENSION_SIGN[joint] >= 0.0:
                    out[f"{side}_{joint}_extensor"] = max(-cap, min(cap, torque / (strength.extensor_n_m * scale)))
                    out[f"{side}_{joint}_flexor"] = 0.0
                else:
                    out[f"{side}_{joint}_flexor"] = max(-cap, min(cap, torque / (strength.flexor_n_m * scale)))
                    out[f"{side}_{joint}_extensor"] = 0.0
        return out


def static_activations(model: HumanModel, state: MultibodyState) -> dict[str, float]:
    """Activations that hold the posture exactly under the static feedforward."""
    torques = model.gravity_compensation(state)
    out: dict[str, float] = {}
    for side in SIDES:
        scale = model.strength.left_scale if side == "left" else model.strength.right_scale
        for joint in JOINTS:
            torque = torques[f"{side}_{joint}"]
            strength = model.strength.joint(joint)
            if torque * EXTENSION_SIGN[joint] >= 0.0:
                out[f"{side}_{joint}_extensor"] = torque / (strength.extensor_n_m * scale)
                out[f"{side}_{joint}_flexor"] = 0.0
            else:
                out[f"{side}_{joint}_flexor"] = torque / (strength.flexor_n_m * scale)
                out[f"{side}_{joint}_extensor"] = 0.0
            if max(abs(out[f"{side}_{joint}_extensor"]), abs(out[f"{side}_{joint}_flexor"])) > 1.0:
                raise ConstraintSolveError(f"{side} {joint} cannot hold the declared posture statically")
    return out


@dataclass(frozen=True, slots=True)
class ContactForceSample:
    contact_id: str
    point_world_m: tuple[float, float]
    force_world_n: tuple[float, float]
    normal_force_n: float
    tangential_force_n: float


@dataclass(frozen=True, slots=True)
class LatentFrame:
    """One integration interval [t_start, t_end] and its end state."""

    t_start_s: float
    t_end_s: float
    state: MultibodyState
    contact_forces: Mapping[str, ContactForceSample]
    activations: Mapping[str, float]
    actuator_torques_n_m: Mapping[str, float]
    passive_torques_n_m: Mapping[str, float]
    joint_power_w: float
    controller_stage: str


@dataclass(frozen=True, slots=True)
class ImpactImpulse:
    time_s: float
    contact_id: str
    point_world_m: tuple[float, float]
    impulse_world_n_s: tuple[float, float]


@dataclass
class ExerciseTrajectory:
    exercise_id: str
    model: HumanModel
    initial_state: MultibodyState
    frames: list[LatentFrame]
    impacts: list[ImpactImpulse]
    events: list[ContactEvent]
    control_commands: list[tuple[float, str, str]]
    termination: str
    scenario: Mapping[str, object]

    def times(self) -> list[float]:
        return [self.initial_state.time_s] + [f.t_end_s for f in self.frames]

    def states(self) -> list[MultibodyState]:
        return [self.initial_state] + [f.state for f in self.frames]


def _passive_wrenches(model: HumanModel, state: MultibodyState, limits: Mapping[str, JointLimit]):
    wrenches: list[AppliedWrench] = []
    torques: dict[str, float] = {}
    power = 0.0
    for side in SIDES:
        angles = model.joint_angles(state, side)
        for joint in JOINTS:
            angle, rate = angles[joint]
            torque = limits[joint].torque(angle, rate)
            torques[f"{side}_{joint}"] = torque
            if torque != 0.0:
                spec = model.system._joint_by_id[f"{side}_{joint}"]
                wrenches.append(AppliedWrench(spec.body_b_id, f"passive:{side}_{joint}", moment_n_m=torque))
                wrenches.append(AppliedWrench(spec.body_a_id, f"passive:{side}_{joint}", moment_n_m=-torque))
                power += torque * rate
    return wrenches, torques, power


def _joint_rates(model: HumanModel, state: MultibodyState) -> dict[str, float]:
    return {
        f"{side}_{joint}": model.system.joint_state(state, f"{side}_{joint}")[1]
        for side in SIDES
        for joint in JOINTS
    }


def body_points_below_floor(model: HumanModel, state: MultibodyState) -> list[str]:
    """Non-foot points that a rigid-floor model does not allow below z = 0."""
    hat = model.segments["hat"]
    checks = {
        "hip": (hat.body_id, model.hat_points["hip"]),
        "head_top": (hat.body_id, model.hat_points["head_top"]),
    }
    for side in SIDES:
        shank = model.segments[f"{side}_shank"]
        checks[f"{side}_knee"] = (shank.body_id, shank.proximal_local_m)
    return [name for name, (b, p) in checks.items() if model.point_world(state, b, p)[1] < 0.0]


def simulate(
    exercise_id: str,
    model: HumanModel,
    controller: StagedJointController,
    initial_state: MultibodyState,
    *,
    duration_s: float,
    dt_s: float = 0.001,
    limits: Mapping[str, JointLimit] | None = None,
    scenario: Mapping[str, object] | None = None,
    max_steps: int = 200_000,
) -> ExerciseTrajectory:
    system = model.system
    limits = limits or DEFAULT_LIMITS
    state = initial_state
    frames: list[LatentFrame] = []
    impacts: list[ImpactImpulse] = []
    events: list[ContactEvent] = []
    termination = "DURATION_REACHED"
    for _ in range(max_steps):
        if state.time_s >= duration_s - 1e-12:
            break
        h = min(dt_s, duration_s - state.time_s)
        excitations = controller.excitations(model, state)
        wrenches, passive, passive_power = _passive_wrenches(model, state, limits)
        _, torques, _ = system._assemble_generalized_forces(state, excitations, wrenches)
        rates = _joint_rates(model, state)
        actuator_power = sum(
            torque * rates[system.actuator_joint_id(actuator_id)] for actuator_id, torque in torques.items()
        )
        try:
            step = system.advance(state, h, excitations=excitations, external_wrenches=wrenches)
        except ConstraintSolveError as exc:
            termination = f"CONSTRAINT_SOLVE_REFUSED: {exc}"
            break
        elapsed = step.elapsed_s
        forces: dict[str, ContactForceSample] = {}
        if elapsed > 0.0:
            for ledger, frame_state in (
                (step.contact_impulses_n_s, state),
                (step.projection_impulses_n_s, step.state),
            ):
                for contact_id, (normal, tangential) in ledger.items():
                    kin = system.contact_kinematics(frame_state, contact_id)
                    fx = (normal * kin.normal_world[0] + tangential * kin.tangent_world[0]) / elapsed
                    fz = (normal * kin.normal_world[1] + tangential * kin.tangent_world[1]) / elapsed
                    previous = forces.get(contact_id)
                    point = system.contact_kinematics(state, contact_id).point_a_world_m
                    if previous is None:
                        forces[contact_id] = ContactForceSample(contact_id, point, (fx, fz), normal / elapsed, tangential / elapsed)
                    else:
                        forces[contact_id] = ContactForceSample(
                            contact_id, point,
                            (previous.force_world_n[0] + fx, previous.force_world_n[1] + fz),
                            previous.normal_force_n + normal / elapsed,
                            previous.tangential_force_n + tangential / elapsed,
                        )
        for event in step.events:
            events.append(event)
            if event.event_id == "CONTACT_IMPACT":
                impact_state = step.state
                for contact_id in event.normal_impulses_n_s:
                    kin = system.contact_kinematics(impact_state, contact_id)
                    normal = event.normal_impulses_n_s.get(contact_id, 0.0)
                    tangential = event.tangential_impulses_n_s.get(contact_id, 0.0)
                    impacts.append(ImpactImpulse(
                        event.time_s, contact_id, kin.point_a_world_m,
                        (normal * kin.normal_world[0] + tangential * kin.tangent_world[0],
                         normal * kin.normal_world[1] + tangential * kin.tangent_world[1]),
                    ))
        frames.append(LatentFrame(
            state.time_s, step.state.time_s, step.state, forces, dict(state.activations), torques,
            passive, actuator_power + passive_power, controller.stage.name,
        ))
        state = step.state
        below = body_points_below_floor(model, state)
        if below:
            termination = f"BODY_GROUND_CONTACT_NOT_MODELED: {', '.join(below)}"
            break
    else:
        termination = "MAX_STEPS_REACHED"
    return ExerciseTrajectory(
        exercise_id, model, initial_state, frames, impacts, events,
        list(controller.commands), termination, dict(scenario or {}),
    )
