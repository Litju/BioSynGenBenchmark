"""Back squat: continuous bilateral support, multi-repetition forward dynamics (RES-493).

Topology: the bilateral human of ``human.py`` with a barbell resting on the
posterior shoulder shelf. The bar-on-back contact is unilateral and
frictional, so slippage is possible and is detected, not prevented. Both feet
stay on their plates, and flight is a refused state. Safety pins may be
declared as a world contact under the bar. Any bar-on-pin load makes that
repetition SUPPORT_INCOMPLETE: its metrics are refused, not reinterpreted.

Each repetition is a declared control hypothesis:

* descent: minimum-jerk reference over the declared descent time;
* optional bottom pause;
* ascent: minimum-jerk reference over the declared ascent time, with
  saturating joint PD (a capacity or force-velocity shortfall shows up as
  lag, i.e. a sticking region);
* lockout hold.

Latent repetition events come from the bar and system-COM mechanics only:

* descent onset: bar vertical velocity first below -v_on;
* bottom reversal: bar vertical velocity crosses 0 upward;
* ascent onset: bar vertical velocity first above +0.05 m/s after the bottom
  (declared; above the end-of-descent rebound). A pause lies between bottom
  reversal and ascent onset;
* sticking region: a local bar-velocity maximum above 0.1 m/s, then a dip
  of at least 5 % of the peak ascent velocity, then re-acceleration by the
  same amount (declared thresholds); ordinary lockout deceleration is not one;
* a candidate repetition needs at least 0.05 m of bar descent;
* rep end: bar vertical velocity back inside ±v_on with the bar above 98 % of
  its start height.

``v_on`` = 0.02 m/s is a declared numerical threshold, not a device threshold.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
import warnings

from ..multibody import ContactKind, ContactMode, ContactSpec, MultibodyState
from .human import (
    Anthropometry,
    HumanModel,
    LoadKind,
    LoadSpec,
    Posture,
    StrengthHypothesis,
    balanced_posture,
    build_human,
)
from .instruments import CableLptModel, LptAttachment
from .phases import foot_force_z
from .simulate import ControlStage, ExerciseTrajectory, StagedJointController, simulate, static_activations

REP_VELOCITY_THRESHOLD_M_S = 0.02
ASCENT_ONSET_THRESHOLD_M_S = 0.05
MIN_REP_DESCENT_M = 0.05


class SquatRefused(ValueError):
    """The requested squat configuration is not physically supported."""


@dataclass(frozen=True, slots=True)
class SquatProtocol:
    load_mass_kg: float = 60.0
    repetitions: int = 3
    stance_ankle_knee: tuple[float, float] = (0.08, -0.12)
    bottom_ankle_knee: tuple[float, float] = (0.60, -1.75)
    descent_time_s: float = 1.2
    bottom_pause_s: float = 0.0
    ascent_time_s: float = 1.0
    lockout_hold_s: float = 0.8
    start_hold_s: float = 0.5
    bar_friction: float = 0.8
    safety_pin_height_m: float | None = None
    balance_com_x_m: float = 0.04
    ascent_effort: float = 1.0

    def __post_init__(self) -> None:
        if self.repetitions < 1:
            raise SquatRefused("a squat set needs at least one repetition")
        if not self.load_mass_kg > 0.0:
            raise SquatRefused("a back squat requires a positive bar load")
        for name in ("descent_time_s", "ascent_time_s", "lockout_hold_s", "start_hold_s"):
            if not getattr(self, name) > 0.0:
                raise SquatRefused(f"{name} must be positive")


def _gains(a: float, k: float, h: float) -> dict[str, float]:
    return {"ankle": a, "knee": k, "hip": h}


@dataclass
class SquatScenario:
    model: HumanModel
    protocol: SquatProtocol
    controller: StagedJointController
    initial_state: MultibodyState
    duration_s: float
    metadata: dict = field(default_factory=dict)

    def run(self, dt_s: float = 0.001) -> ExerciseTrajectory:
        return simulate("BACK_SQUAT", self.model, self.controller, self.initial_state,
                        duration_s=self.duration_s, dt_s=dt_s, scenario=self.metadata)


def build_back_squat(
    protocol: SquatProtocol | None = None,
    *,
    anthropometry: Anthropometry | None = None,
    strength: StrengthHypothesis | None = None,
) -> SquatScenario:
    p = protocol or SquatProtocol()
    model = build_human(anthropometry, strength, LoadSpec(LoadKind.BACK_BAR, p.load_mass_kg, 0.08, p.bar_friction))
    if p.safety_pin_height_m is not None:
        model = _with_safety_pins(model, p)
    x = p.balance_com_x_m
    stance = balanced_posture(model, *p.stance_ankle_knee, x)
    bottom = balanced_posture(model, *p.bottom_ankle_knee, x)
    probe = model.posture_state(stance)
    try:
        activations = static_activations(model, probe)
        static_activations(model, model.posture_state(bottom))
    except Exception as exc:  # the declared posture cannot be held statically
        raise SquatRefused(f"UNSAFE_OR_INFEASIBLE_TORQUE_DEMAND: {exc}") from exc
    initial = model.posture_state(stance, activations=activations)
    static = model.system.solve_accelerations(initial, excitations=dict(initial.activations))
    if static.contact_states[model.load_contact_id].mode is not ContactMode.STICK:
        raise SquatRefused(
            "BAR_SLIPPAGE_AT_SETUP: the declared shoulder-shelf friction cannot hold the bar statically"
        )
    kb, db = 3000.0, 1200.0
    stages = [ControlStage("SET_START_HOLD", stance, _gains(300, 1500, 1500), _gains(60, 150, 150), True,
                           lambda m, s, t: t >= p.start_hold_s, "declared set start", 0.0, kb, db)]
    for rep in range(1, p.repetitions + 1):
        stages.append(ControlStage(
            f"REP{rep}_DESCENT", bottom, _gains(300, 1500, 1500), _gains(60, 150, 150), True,
            lambda m, s, t: t >= p.descent_time_s, "declared descent tempo elapsed", p.descent_time_s, kb, db,
        ))
        if p.bottom_pause_s > 0.0:
            stages.append(ControlStage(
                f"REP{rep}_PAUSE", bottom, _gains(300, 1500, 1500), _gains(60, 150, 150), True,
                lambda m, s, t: t >= p.bottom_pause_s, "declared pause elapsed", 0.0, kb, db,
            ))
        stages.append(ControlStage(
            f"REP{rep}_ASCENT", stance, _gains(300, 2500, 2500), _gains(60, 150, 150), True,
            lambda m, s, t: t >= p.ascent_time_s, "declared ascent tempo elapsed", p.ascent_time_s, kb, db,
            {"knee": p.ascent_effort, "hip": p.ascent_effort, "ankle": p.ascent_effort},
        ))
        stages.append(ControlStage(
            f"REP{rep}_LOCKOUT", stance, _gains(300, 1500, 1500), _gains(60, 150, 150), True,
            lambda m, s, t: t >= p.lockout_hold_s, "declared lockout hold elapsed", 0.2, kb, db,
        ))
    ankle_x = model.point_world(initial, "left_foot", model.foot_points["ankle"])[0]
    controller = StagedJointController(
        stages, "I2_BACK_SQUAT_STAGED_PD_V1",
        balance_reference_x_m=model.system.center_of_mass(initial)[0] - ankle_x,
    )
    duration = p.start_hold_s + p.repetitions * (
        p.descent_time_s + p.bottom_pause_s + p.ascent_time_s + p.lockout_hold_s
    )
    metadata = {
        "exercise_id": "BACK_SQUAT", "protocol": p, "load_kind": "BACK_BAR",
        "controller_id": controller.controller_id, "control_status": "LATENT_CONTROL_HYPOTHESIS",
        "parameter_status": "ASSUMED_UNCALIBRATED",
    }
    return SquatScenario(model, p, controller, initial, duration, metadata)


def _with_safety_pins(model: HumanModel, p: SquatProtocol) -> HumanModel:
    """Rebuild the system with a frictionless safety-pin contact under the bar."""
    from ..multibody import PlanarMultibodySystem

    system = model.system
    pins = ContactSpec(
        "safety_pins", model.load_body_id, (0.0, 0.0), (0.0, 1.0), 0.0, 0.0, ContactKind.RACK,
        surface_point_m=(0.0, p.safety_pin_height_m),
    )
    model.system = PlanarMultibodySystem(
        system.bodies, system.joints, system.contacts + (pins,),
        gravity_m_s2=system.gravity_m_s2, clock_id=system.clock_id, actuators=system.actuators,
    )
    return model


@dataclass(frozen=True, slots=True)
class SquatRep:
    index: int
    descent_onset_s: float
    bottom_reversal_s: float | None
    ascent_onset_s: float | None
    rep_end_s: float | None
    sticking_region_s: tuple[float, float] | None
    bar_displacement_m: float
    peak_descent_velocity_m_s: float
    peak_ascent_velocity_m_s: float | None
    mean_ascent_velocity_m_s: float | None
    bar_slip_m: float
    support_incomplete: bool
    refusals: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class SquatAnalysis:
    reps: tuple[SquatRep, ...]
    system_weight_n: float
    body_weight_n: float
    bar_weight_n: float
    flight_detected: bool
    joint_work_j: float
    bar_contact_vertical_work_estimate_j: float
    system_com_pseudo_work_j: float
    refusals: tuple[str, ...]

    @property
    def work_definitions(self) -> dict[str, str]:
        return {
            "joint_work_j": "sum of modeled active/passive joint torque-times-rate, left-step quadrature; not metabolic work",
            "bar_contact_vertical_work_estimate_j": "vertical shoulder-force times bar-COM vertical velocity, end-step quadrature; excludes horizontal/rotational and impact work",
            "system_com_pseudo_work_j": "foot force times system-COM vertical velocity, end-step quadrature; pseudo-work, not work of stationary foot contact",
        }

    @property
    def system_com_work_j(self) -> float:
        warnings.warn("system_com_work_j is COM pseudo-work; use system_com_pseudo_work_j", DeprecationWarning, stacklevel=2)
        return self.system_com_pseudo_work_j

    @property
    def bar_work_on_bar_j(self) -> float:
        warnings.warn("bar_work_on_bar_j is only a vertical quadrature estimate; use bar_contact_vertical_work_estimate_j",
                      DeprecationWarning, stacklevel=2)
        return self.bar_contact_vertical_work_estimate_j


def bar_state(trajectory: ExerciseTrajectory, state: MultibodyState) -> tuple[float, float]:
    bar = state.bodies[trajectory.model.load_body_id]
    return bar.position_m[1], bar.velocity_m_s[1]


def analyze_squat(trajectory: ExerciseTrajectory) -> SquatAnalysis:
    model = trajectory.model
    system = model.system
    g = model.gravity_m_s2
    total_mass = model.total_mass_kg()
    bar_mass = model.load.mass_kg
    frames = trajectory.frames
    states = [f.state for f in frames]
    times = [f.t_end_s for f in frames]
    bar_z = [bar_state(trajectory, s)[0] for s in states]
    bar_v = [bar_state(trajectory, s)[1] for s in states]
    z_start = bar_state(trajectory, trajectory.initial_state)[0]
    refusals: list[str] = []
    if not trajectory.termination.startswith("DURATION_REACHED"):
        refusals.append(f"RUN_TERMINATED: {trajectory.termination}")
    flight = any(
        all(f.state.contacts[c].mode is ContactMode.OPEN for c in model.contact_ids[side])
        for f in frames for side in model.contact_ids
    )
    if flight:
        refusals.append("FLIGHT_FORBIDDEN: a foot left its plate during a back squat")
    shelf_tangent = []
    for s in states:
        if model.load_contact_id in s.contacts and s.contacts[model.load_contact_id].mode is not ContactMode.OPEN:
            shelf_tangent.append(system.contact_kinematics(s, model.load_contact_id).relative_position_m)
    reps: list[SquatRep] = []
    v_on = REP_VELOCITY_THRESHOLD_M_S
    i = 0
    n = len(frames)
    while i < n:
        onset = next((k for k in range(i, n) if bar_v[k] < -v_on), None)
        if onset is None:
            break
        bottom = next((k for k in range(onset, n - 1) if bar_v[k] < 0.0 <= bar_v[k + 1]), None)
        if bottom is not None and z_start - bar_z[bottom] < MIN_REP_DESCENT_M:
            # A sub-threshold wobble (e.g. at lockout) is not a repetition.
            i = bottom + 1
            continue
        end = None
        if bottom is not None:
            end = next((k for k in range(bottom + 1, n) if abs(bar_v[k]) <= v_on and bar_z[k] >= z_start - 0.02 * z_start), None)
        last = end if end is not None else n - 1
        sticking = None
        peak_ascent = mean_ascent = None
        ascent_onset = None
        if bottom is not None:
            ascent_onset = next((k for k in range(bottom, last + 1) if bar_v[k] > ASCENT_ONSET_THRESHOLD_M_S), None)
            ascent = range(bottom + 1, last + 1)
            peak_ascent = max(bar_v[k] for k in ascent) if last > bottom else None
            if end is not None and ascent_onset is not None and end > ascent_onset:
                # Mean concentric velocity from ascent onset (excludes any pause).
                mean_ascent = (bar_z[end] - bar_z[ascent_onset]) / (times[end] - times[ascent_onset])
            # Sticking region: a local velocity maximum above 0.1 m/s followed by
            # a dip of at least 5 % of the peak ascent velocity (declared).
            for k in range(bottom + 2, last - 1):
                if bar_v[k - 1] < bar_v[k] >= bar_v[k + 1] and bar_v[k] > 0.1:
                    m = next((j for j in range(k + 1, last) if bar_v[j] <= bar_v[j - 1] and bar_v[j] < bar_v[j + 1]), None)
                    if m is not None and bar_v[k] - bar_v[m] >= 0.05 * (peak_ascent or 0.0) and bar_v[m] > v_on:
                        # A genuine sticking region is followed by re-acceleration,
                        # not by the ordinary deceleration into lockout.
                        if max(bar_v[m:last + 1]) >= bar_v[m] + 0.05 * (peak_ascent or 0.0):
                            sticking = (times[k], times[m])
                        break
        slip = 0.0
        if model.load_contact_id is not None:
            rel = [system.contact_kinematics(states[k], model.load_contact_id).tangential_velocity_m_s
                   for k in range(onset, last + 1) if states[k].contacts[model.load_contact_id].mode is not ContactMode.OPEN]
            slip = sum(abs(v) for v in rel) * (times[1] - times[0] if n > 1 else 0.0)
        pinned = any(
            "safety_pins" in states[k].contacts and states[k].contacts["safety_pins"].mode is not ContactMode.OPEN
            for k in range(onset, last + 1)
        )
        rep_refusals = []
        if pinned:
            rep_refusals.append("SUPPORT_INCOMPLETE: bar loaded the safety pins; rep metrics refused")
        if end is None:
            rep_refusals.append("REP_NOT_COMPLETED: bar did not return to lockout within the record")
        if slip > 0.01:
            rep_refusals.append(f"BAR_SLIPPAGE: {slip:.3f} m tangential slip on the shoulder shelf")
        reps.append(SquatRep(
            len(reps) + 1, frames[onset].t_start_s,
            times[bottom] if bottom is not None else None,
            times[ascent_onset] if bottom is not None and ascent_onset is not None else None,
            times[end] if end is not None else None,
            sticking, z_start - min(bar_z[onset:last + 1]),
            min(bar_v[onset:last + 1]), peak_ascent, mean_ascent, slip, pinned, tuple(rep_refusals),
        ))
        if end is None:
            break
        i = end + 1
    joint_work = sum(f.joint_power_w * (f.t_end_s - f.t_start_s) for f in frames)
    bar_work = 0.0
    com_work = 0.0
    for f in frames:
        h = f.t_end_s - f.t_start_s
        sample = f.contact_forces.get(model.load_contact_id)
        bar_vz = f.state.bodies[model.load_body_id].velocity_m_s[1]
        if sample is not None:
            # Recorded force acts on body A = bar. This vertical component
            # estimate excludes horizontal/rotational and instantaneous work.
            bar_work += sample.force_world_n[1] * bar_vz * h
        com_vz = system.linear_momentum(f.state)[1] / total_mass
        com_work += foot_force_z(trajectory, f) * com_vz * h
    return SquatAnalysis(
        tuple(reps), total_mass * g, (total_mass - bar_mass) * g, bar_mass * g, flight,
        joint_work, bar_work, com_work, tuple(refusals),
    )


def squat_lpt(trajectory: ExerciseTrajectory) -> CableLptModel:
    """Bar-collar LPT with the reel on the floor directly under the collar at set start.

    This aligned setup makes the cable initially vertical. A misplaced reel is
    a separate, declared negative control.
    """
    model = trajectory.model
    bar_x = trajectory.initial_state.bodies[model.load_body_id].position_m[0]
    return CableLptModel(LptAttachment(model.load_body_id, (0.0, 0.0), 0.60, "BAR_COLLAR"), anchor_m=(bar_x, 0.60, 0.0))
