"""Olympic pulling derivatives: clean pull and snatch pull from the floor (RES-496).

Topology is the RES-494 deadlift (gripped bar, floor-supported at the plate
rim, feet on separate plates), with three differences:

* Grip width. The planar model sees the sagittal projection of the arms. A
  wide snatch grip abducts the arms (declared 30 deg), which shortens the
  projected reach by cos(30 deg). The clean grip uses 10 deg. A 40 deg
  declaration put the solved setup hip beyond its passive range and is
  refused by the setup range check.
* Explosive pull. The pull is a fast minimum-jerk drive to a triple-extension
  posture (ankles plantarflexed, knees and hips extended), not a deadlift
  lockout. Heels may rise and the feet may leave the plates (a jump finish).
* Finish and lowering. The bar returns to the floor; catch and turnover are
  out of scope here (RES-497).

Variants follow the RES-490 inventory (OLYMPIC_PULL_DERIVATIVE_DUAL_V1 with
CLEAN_PULL_DUAL_V1 and SNATCH_PULL_DUAL_V1). Hang variants are listed there as
deferred (NOT_IMPLEMENTED) and block variants are absent, so both are refused.

Latent events come from bar, contact and joint kinematics:

* lift-off: the bar-floor contact opens;
* first-pull end: the bar passes knee height;
* transition: the knee re-flexes after the bar passes the knee, reported
  only if the knee angle actually re-flexes;
* second pull: from the end of the transition (or the first-pull end) to the
  peak bar vertical velocity;
* heel release and foot separation (flight), when they occur;
* bar peak height;
* bar-floor return.

FORCE/LPT cannot identify joint phases (double knee bend, triple extension).
``claim_from_sensors`` refuses them without motion capture.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math

from ..multibody import ContactMode, MultibodyState
from .deadlift import DeadliftRefused, solve_setup
from .human import Anthropometry, HumanModel, LoadKind, LoadSpec, Posture, StrengthHypothesis, build_human
from .instruments import CableLptModel, LptAttachment
from .phases import foot_force_z
from .simulate import ControlStage, ExerciseTrajectory, StagedJointController, simulate, static_activations, supported_com

PULL_VARIANTS = {
    "CLEAN_PULL": {"protocol_id": "OLYMPIC_PULL_DERIVATIVE_DUAL_V1", "variant_id": "CLEAN_PULL_DUAL_V1",
                   "grip_abduction_rad": math.radians(10.0)},
    "SNATCH_PULL": {"protocol_id": "OLYMPIC_PULL_DERIVATIVE_DUAL_V1", "variant_id": "SNATCH_PULL_DUAL_V1",
                    "grip_abduction_rad": math.radians(30.0)},
}
REFUSED_VARIANTS = {
    "HANG_CLEAN_PULL": "NOT_IMPLEMENTED: hang start state/height deferred in the RES-490 inventory",
    "HANG_SNATCH_PULL": "NOT_IMPLEMENTED: hang start state/height deferred in the RES-490 inventory",
    "BLOCK_CLEAN_PULL": "NOT_IMPLEMENTED: block starts are not present in the RES-490 inventory",
    "BLOCK_SNATCH_PULL": "NOT_IMPLEMENTED: block starts are not present in the RES-490 inventory",
}
SENSOR_REFUSALS = {
    "DOUBLE_KNEE_BEND": "NOT_EVALUABLE: knee re-flexion is a joint-kinematic phase; plates and bar LPT cannot identify it",
    "TRIPLE_EXTENSION": "NOT_EVALUABLE: joint extension state requires motion capture, not FORCE/LPT",
    "TECHNIQUE_PROFICIENCY": "NOT_EVALUABLE: bar-only LPT does not establish technique proficiency",
}


class PullRefused(ValueError):
    """The requested pull variant or claim is not supported."""


@dataclass(frozen=True, slots=True)
class PullProtocol:
    variant: str = "CLEAN_PULL"
    load_mass_kg: float = 70.0
    setup_ankle: float | None = None          # None: variant default (clean 0.45, snatch 0.65)
    setup_arm_angle_rad: float = -0.10
    bar_over_ankle_x_m: float = 0.05
    # Declared search result (runs/i2/res496/strategy_search.md): a flat-foot
    # extension target; heel rise and any brief foot separation then come from
    # the dynamics. Plantarflexed or hip-hyperextended targets fell.
    extension: Posture = Posture(0.05, -0.05, 0.02, 0.0, 0.0)
    ankle_effort: float = 1.0
    leg_effort: float = 1.0
    setup_hold_s: float = 0.4
    pretension_s: float = 0.4
    pull_s: float = 1.0
    finish_hold_s: float = 0.4
    lower_s: float = 1.6
    reset_s: float = 0.5


def _gains(a, k, h, sh=600.0, el=600.0):
    return {"ankle": a, "knee": k, "hip": h, "shoulder": sh, "elbow": el}


@dataclass
class PullScenario:
    model: HumanModel
    protocol: PullProtocol
    controller: StagedJointController
    initial_state: MultibodyState
    duration_s: float
    setup_posture: Posture
    metadata: dict = field(default_factory=dict)

    def run(self, dt_s: float = 0.001) -> ExerciseTrajectory:
        return simulate(self.protocol.variant, self.model, self.controller, self.initial_state,
                        duration_s=self.duration_s, dt_s=dt_s, scenario=self.metadata)


def build_pull(
    protocol: PullProtocol | None = None,
    *,
    anthropometry: Anthropometry | None = None,
    strength: StrengthHypothesis | None = None,
) -> PullScenario:
    p = protocol or PullProtocol()
    if p.variant in REFUSED_VARIANTS:
        raise PullRefused(f"{p.variant}: {REFUSED_VARIANTS[p.variant]}")
    if p.variant not in PULL_VARIANTS:
        raise PullRefused(f"{p.variant}: not a registered pull variant")
    info = PULL_VARIANTS[p.variant]
    a = anthropometry or Anthropometry()
    projection = math.cos(info["grip_abduction_rad"])
    a = replace(a, upper_arm_length_fraction=a.upper_arm_length_fraction * projection,
                forearm_grip_length_fraction=a.forearm_grip_length_fraction * projection)
    model = build_human(a, strength, LoadSpec(LoadKind.GRIP_BAR, p.load_mass_kg, 0.08), arms=True)
    setup_ankle = p.setup_ankle if p.setup_ankle is not None else (0.65 if p.variant == "SNATCH_PULL" else 0.45)
    try:
        setup = solve_setup(model, setup_ankle, p.setup_arm_angle_rad, p.bar_over_ankle_x_m)
    except DeadliftRefused as exc:
        raise PullRefused(str(exc)) from exc
    from .simulate import DEFAULT_LIMITS

    for joint in ("ankle", "knee", "hip", "shoulder", "elbow"):
        value = getattr(setup, joint)
        limit = DEFAULT_LIMITS[joint]
        if not limit.lower_rad <= value <= limit.upper_rad:
            raise PullRefused(f"SETUP_OUT_OF_RANGE: {joint} {value:.3f} rad outside [{limit.lower_rad}, {limit.upper_rad}]")
    probe = model.posture_state(setup)
    try:
        activations = static_activations(model, probe, load_share=0.0)
    except Exception as exc:
        raise PullRefused(f"UNSAFE_OR_INFEASIBLE_SETUP: {exc}") from exc
    initial = model.posture_state(setup, activations=activations)
    kb, db = 3000.0, 1200.0
    hold = _gains(300, 1500, 1500)
    damp = _gains(60, 150, 150, 60, 60)
    explosive = _gains(3000, 4000, 4000, 800, 800)
    stages = [
        ControlStage("SETUP_HOLD", setup, hold, damp, True, lambda m, s, t: t >= p.setup_hold_s,
                     "declared setup time", 0.0, kb, db, load_share=0.0),
        ControlStage("PRETENSION", setup, hold, damp, True, lambda m, s, t: t >= p.pretension_s,
                     "declared pre-tension ramp", 0.0, kb, db, load_share=lambda t: t / p.pretension_s),
        ControlStage("PULL", p.extension, explosive, damp, True, lambda m, s, t: t >= p.pull_s,
                     "declared pull time", p.pull_s, kb, db,
                     {"ankle": p.ankle_effort, "knee": p.leg_effort, "hip": p.leg_effort}),
        ControlStage("FINISH", Posture(0.05, -0.08, 0.05, 0.0, 0.0), hold, damp, True,
                     lambda m, s, t: t >= p.finish_hold_s, "declared finish", 0.25, kb, db),
        ControlStage("LOWER", setup, _gains(300, 2500, 2500), damp, True, lambda m, s, t: t >= p.lower_s,
                     "declared lowering tempo", p.lower_s, kb, db),
        ControlStage("RESET", setup, hold, damp, True, None, "", 0.0, kb, db,
                     load_share=lambda t: max(0.0, 1.0 - t / p.reset_s)),
    ]
    ankle_x = model.point_world(initial, "left_foot", model.foot_points["ankle"])[0]
    controller = StagedJointController(
        stages, f"I2_{p.variant}_STAGED_PD_V1",
        balance_reference_x_m=supported_com(model, initial, 0.0)[0] - ankle_x,
    )
    duration = p.setup_hold_s + p.pretension_s + p.pull_s + p.finish_hold_s + p.lower_s + p.reset_s
    metadata = {"exercise_id": p.variant, **info, "protocol": p, "controller_id": controller.controller_id,
                "control_status": "LATENT_CONTROL_HYPOTHESIS", "parameter_status": "ASSUMED_UNCALIBRATED",
                "grip_projection_factor": projection}
    return PullScenario(model, p, controller, initial, duration, setup, metadata)


@dataclass(frozen=True, slots=True)
class PullAnalysis:
    lift_off_s: float | None
    first_pull_end_s: float | None
    transition_s: tuple[float, float] | None
    second_pull_end_s: float | None
    peak_bar_velocity_m_s: float | None
    heel_release_s: float | None
    flight_segments_s: tuple[tuple[float, float], ...]
    bar_peak_height_m: float
    bar_return_s: float | None
    refusals: tuple[str, ...]


def analyze_pull(trajectory: ExerciseTrajectory) -> PullAnalysis:
    model = trajectory.model
    frames = trajectory.frames
    refusals: list[str] = []
    if not trajectory.termination.startswith("DURATION_REACHED"):
        refusals.append(f"RUN_TERMINATED: {trajectory.termination}")
    times = [f.t_end_s for f in frames]
    contact = model.bar_floor_contact_id
    bar_z = [f.state.bodies["load"].position_m[1] for f in frames]
    bar_v = [f.state.bodies["load"].velocity_m_s[1] for f in frames]
    lift_i = next((i for i, f in enumerate(frames) if f.state.contacts[contact].mode is ContactMode.OPEN), None)
    if lift_i is None:
        refusals.append("LOAD_NOT_LIFTED")
        return PullAnalysis(None, None, None, None, None, None, (), 0.0, None, tuple(refusals))
    knee_pass_i = None
    for i in range(lift_i, len(frames)):
        knee_z = model.point_world(frames[i].state, "left_shank", model.segments["left_shank"].proximal_local_m)[1]
        if bar_z[i] >= knee_z:
            knee_pass_i = i
            break
    # Peak bar velocity during the upward drive (end of the second pull).
    up_end = next((i for i in range(lift_i, len(frames)) if bar_v[i] < 0.0), len(frames) - 1)
    peak_i = max(range(lift_i, up_end + 1), key=lambda i: bar_v[i])
    # Transition: knee flexion velocity becomes negative (re-flexion) after the bar passes the knee.
    transition = None
    if knee_pass_i is not None:
        knee = [model.system.joint_state(frames[i].state, "left_knee") for i in range(len(frames))]
        start = next((i for i in range(knee_pass_i, peak_i) if knee[i][1] < -0.05), None)
        if start is not None:
            end = next((i for i in range(start, peak_i + 1) if knee[i][1] >= 0.0), None)
            if end is not None:
                transition = (times[start], times[end])
    heel_i = next((i for i in range(lift_i, len(frames))
                   if all(frames[i].state.contacts[f"{s}_heel"].mode is ContactMode.OPEN for s in ("left", "right"))), None)
    segments: list[tuple[float, float]] = []
    start = None
    for i in range(lift_i, len(frames)):
        airborne = all(frames[i].state.contacts[c].mode is ContactMode.OPEN for v in model.contact_ids.values() for c in v)
        if airborne and start is None:
            start = times[i]
        elif not airborne and start is not None:
            segments.append((start, times[i]))
            start = None
    if start is not None:
        segments.append((start, times[-1]))
    ret = None
    for event in trajectory.events:
        if event.time_s > times[peak_i] and contact in event.contact_ids and event.event_id in ("CONTACT_IMPACT", "CONTACT_TOUCHDOWN"):
            ret = event.time_s
            break
    return PullAnalysis(
        times[lift_i], times[knee_pass_i] if knee_pass_i is not None else None, transition,
        times[peak_i], bar_v[peak_i], times[heel_i] if heel_i is not None else None, tuple(segments),
        max(bar_z) - model.load.plate_radius_m, ret, tuple(refusals),
    )


def grip_force_z(trajectory: ExerciseTrajectory, index: int) -> float:
    """Vertical grip force on the bar from bar Newton balance (independent of the solver).

    m (v1 - v0) / h = F_grip + F_floor - m g over one recorded interval.
    """
    frames = trajectory.frames
    frame = frames[index]
    previous = trajectory.initial_state if index == 0 else frames[index - 1].state
    h = frame.t_end_s - frame.t_start_s
    m = trajectory.model.load.mass_kg
    dv = frame.state.bodies["load"].velocity_m_s[1] - previous.bodies["load"].velocity_m_s[1]
    floor = frame.contact_forces.get(trajectory.model.bar_floor_contact_id)
    floor_z = 0.0 if floor is None else floor.force_world_n[1]
    return m * dv / h + m * trajectory.model.gravity_m_s2 - floor_z


def claim_from_sensors(name: str) -> None:
    raise PullRefused(SENSOR_REFUSALS.get(name, f"NOT_EVALUABLE: unknown sensor claim {name}"))


def pull_lpt(trajectory: ExerciseTrajectory) -> CableLptModel:
    model = trajectory.model
    bar_x = trajectory.initial_state.bodies["load"].position_m[0]
    return CableLptModel(LptAttachment("load", (0.0, 0.0), 0.60, "BAR_COLLAR"), anchor_m=(bar_x, 0.60, 0.0))
