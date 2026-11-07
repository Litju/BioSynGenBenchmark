"""Vertical jump family scenarios on the bilateral multibody human (RES-492).

Each registered subtype has its own topology, initial state and declared
control strategy:

| Subtype | Topology | Start | Strategy |
| --- | --- | --- | --- |
| CMJ | unloaded, two feet on two plates | quiet stance | countermovement, propulsion, flight, landing |
| SJ | unloaded | static squat hold | concentric-only propulsion; no countermovement command |
| LOADED_CMJ | weighted vest welded to the trunk | quiet stance | as CMJ, with load-balanced postures |
| LOADED_JUMP_SQUAT | barbell on the posterior shoulder shelf (unilateral frictional contact) | quiet stance | shallower countermovement; bar may separate in flight |

The strategies were chosen by small declared parameter searches
(runs/i2/res492/strategy_search.md) for a near-vertical takeoff and a
recoverable landing. They are control hypotheses, not human strategies.
Drop jumps and repeated rebound jumps are refused: the rigid point-contact
foot has no heel-pad or tissue compliance, so their impact loading would be
an unqualified impulse.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field, replace
import math

from ..multibody import ContactMode, MultibodyState
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
from .simulate import ControlStage, ExerciseTrajectory, StagedJointController, simulate, static_activations


SUPPORTED_SUBTYPES = ("CMJ", "SJ", "LOADED_CMJ", "LOADED_JUMP_SQUAT")
REFUSED_SUBTYPES = {
    "DROP_JUMP": "NOT_IMPLEMENTED: rigid point-contact feet lack heel-pad/tissue compliance; drop-landing loading would be an unqualified impulse",
    "REPEATED_REBOUND_JUMP": "NOT_IMPLEMENTED: rebound contact phases need qualified compliant foot contact; rigid impacts are not a valid stretch-shortening model",
}


class JumpRefused(ValueError):
    """The requested jump subtype is not physically supported by this model."""


@dataclass(frozen=True, slots=True)
class JumpStrategy:
    """Declared control hypothesis for one jump trial (all angles in rad)."""

    balance_com_x_m: float = 0.04
    stance_ankle_knee: tuple[float, float] = (0.08, -0.12)
    squat_ankle_knee: tuple[float, float] = (0.55, -1.40)
    extension: Posture = Posture(-0.60, -0.02, 0.0)
    flight: Posture = Posture(-0.20, -0.30, 0.30)
    absorb_ankle_knee: tuple[float, float] = (0.35, -0.90)
    command_time_s: float = 0.30
    descent_time_s: float = 0.40
    ankle_effort: float = 0.45
    landing_time_s: float = 0.35
    balance_gain: float = 3000.0
    balance_damping: float = 1200.0
    # Hip strategy after landing: a COM error behind the reference drives hip
    # extension (trunk back), whose reaction shears the COM forward.
    hip_balance_gain: float = 0.0
    hip_balance_damping: float = 0.0
    landing_ankle_kp: float = 150.0
    landing_ankle_kd: float = 30.0

    def scaled_depth(self, knee: float, ankle: float) -> "JumpStrategy":
        return replace(self, squat_ankle_knee=(ankle, knee))


def _gains(ankle: float, knee: float, hip: float) -> dict[str, float]:
    return {"ankle": ankle, "knee": knee, "hip": hip}


def feet_released(model: HumanModel, state: MultibodyState, _t: float = 0.0) -> bool:
    return all(state.contacts[c].mode is ContactMode.OPEN for side in model.contact_ids for c in model.contact_ids[side])


def foot_touched(model: HumanModel, state: MultibodyState, _t: float = 0.0) -> bool:
    return any(state.contacts[c].mode is not ContactMode.OPEN for side in model.contact_ids for c in model.contact_ids[side])


def _knee_rate(model: HumanModel, state: MultibodyState) -> float:
    return 0.5 * sum(model.system.joint_state(state, f"{side}_knee")[1] for side in ("left", "right"))


@dataclass
class JumpScenario:
    subtype: str
    model: HumanModel
    controller: StagedJointController
    initial_state: MultibodyState
    duration_s: float
    strategy: JumpStrategy
    metadata: dict = field(default_factory=dict)

    def run(self, dt_s: float = 0.001) -> ExerciseTrajectory:
        return simulate(
            self.subtype, self.model, self.controller, self.initial_state,
            duration_s=self.duration_s, dt_s=dt_s, scenario=self.metadata,
        )


def build_jump(
    subtype: str,
    *,
    anthropometry: Anthropometry | None = None,
    strength: StrengthHypothesis | None = None,
    load_mass_kg: float | None = None,
    strategy: JumpStrategy | None = None,
    duration_s: float = 2.6,
) -> JumpScenario:
    if subtype in REFUSED_SUBTYPES:
        raise JumpRefused(f"{subtype}: {REFUSED_SUBTYPES[subtype]}")
    if subtype not in SUPPORTED_SUBTYPES:
        raise JumpRefused(f"{subtype}: not a registered jump subtype")
    st = strategy or JumpStrategy()
    if subtype == "LOADED_CMJ":
        load = LoadSpec(LoadKind.VEST, 20.0 if load_mass_kg is None else load_mass_kg, 0.3)
        if strategy is None:
            # Declared search result: less flight hip flexion so the heavier,
            # vest-shifted body lands with the COM over the feet.
            st = replace(st, flight=Posture(-0.20, -0.30, 0.10))
    elif subtype == "LOADED_JUMP_SQUAT":
        load = LoadSpec(LoadKind.BACK_BAR, 30.0 if load_mass_kg is None else load_mass_kg, 0.05)
        if strategy is None:
            st = replace(st, squat_ankle_knee=(0.45, -1.10))
    else:
        if load_mass_kg not in (None, 0.0):
            raise JumpRefused(f"{subtype} is an unloaded protocol; load {load_mass_kg} kg refused")
        load = LoadSpec()
    model = build_human(anthropometry, strength, load)
    x = st.balance_com_x_m
    stance = balanced_posture(model, *st.stance_ankle_knee, x)
    squat = balanced_posture(model, *st.squat_ankle_knee, x)
    absorb = balanced_posture(model, *st.absorb_ankle_knee, x)
    flight = st.flight
    kb, db = st.balance_gain, st.balance_damping
    landing_stages = [
        ControlStage("FLIGHT", flight, _gains(600, 600, 600), _gains(40, 40, 40), False,
                     foot_touched, "foot contact sensed"),
        ControlStage("LANDING", absorb, _gains(st.landing_ankle_kp, 1500, 1500), _gains(st.landing_ankle_kd, 250, 250), True,
                     lambda m, s, t: t > st.landing_time_s, "declared absorption time", 0.15, kb, db,
                     None, -st.hip_balance_gain, -st.hip_balance_damping),
        ControlStage("STAND", stance, _gains(st.landing_ankle_kp, 800, 800), _gains(100, 100, 100), True, None, "", 0.5, kb, db,
                     None, -st.hip_balance_gain, -st.hip_balance_damping),
    ]
    propulsion = ControlStage(
        "PROPULSION", st.extension, _gains(5000, 5000, 5000), _gains(20, 20, 20), True,
        feet_released, "both feet released (touch)", 0.0, kb, db, {"ankle": st.ankle_effort},
    )
    if subtype == "SJ":
        start_posture = squat
        stages = [
            ControlStage("STATIC_SQUAT_HOLD", squat, _gains(800, 800, 800), _gains(60, 60, 60), True,
                         lambda m, s, t: t >= st.command_time_s, "declared command time", 0.0, kb, db),
            propulsion,
            *landing_stages,
        ]
    else:
        start_posture = stance
        stages = [
            ControlStage("QUIET_STANCE", stance, _gains(800, 800, 800), _gains(60, 60, 60), True,
                         lambda m, s, t: t >= st.command_time_s, "declared command time", 0.0, kb, db),
            ControlStage(
                "COUNTERMOVEMENT", squat, _gains(900, 900, 900), _gains(80, 80, 80), True,
                lambda m, s, t: t >= st.descent_time_s and _knee_rate(m, s) >= -0.3,
                "squat reference reached and knee flexion stopped (proprioceptive)", st.descent_time_s, kb, db,
            ),
            propulsion,
            *landing_stages,
        ]
    probe = model.posture_state(start_posture)
    initial = model.posture_state(start_posture, activations=static_activations(model, probe))
    ankle_x = model.point_world(initial, "left_foot", model.foot_points["ankle"])[0]
    controller = StagedJointController(
        stages, f"I2_{subtype}_STAGED_PD_V1",
        balance_reference_x_m=model.system.center_of_mass(initial)[0] - ankle_x,
    )
    metadata = {
        "subtype": subtype,
        "load_kind": load.kind.value,
        "load_mass_kg": load.mass_kg,
        "body_mass_kg": model.anthropometry.body_mass_kg,
        "stature_m": model.anthropometry.stature_m,
        "strength_scales": (model.strength.left_scale, model.strength.right_scale),
        "strategy": st,
        "controller_id": controller.controller_id,
        "control_status": "LATENT_CONTROL_HYPOTHESIS",
        "parameter_status": "ASSUMED_UNCALIBRATED",
    }
    return JumpScenario(subtype, model, controller, initial, duration_s, st, metadata)
