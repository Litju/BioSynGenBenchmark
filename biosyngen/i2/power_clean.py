"""Power clean: verified pull phases plus an explicit catch infeasibility refusal (RES-497).

Scope decision (recorded, not hidden). A front-rack catch places the bar on
the anterior shoulders. The grip must then lie about 0.08-0.15 m from the
shoulder joint centre. The planar model's lumped arm is an upper arm plus a
rigid forearm+hand with no wrist joint, and it reaches that point only with
elbow flexion beyond its declared range (2.6 rad). Real lifters reach the rack
through wrist extension and 3-D arm abduction/external rotation, neither of
which the model has. So:

* the pull is simulated with the RES-496 clean-pull dynamics, and the
  power-clean pull events (REP_START, LIFT_OFF, FIRST_PULL_END,
  SECOND_PULL_ONSET, EXTENSION_END) come from those mechanics;
* TURNOVER_ONSET, CATCH_CONTACT, RECOVERY_END and REP_END are reported as
  NOT_IMPLEMENTED with the computed kinematic reason, never synthesized from
  a force template or a bar-position trigger;
* ``build_power_clean(scope="FULL")`` refuses.

``front_rack_feasibility`` computes the elbow flexion required at a declared
rack offset. It is an independent closed-form check (law of cosines) and the
basis of the refusal.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math

from .human import Anthropometry
from .olympic import PullAnalysis, PullProtocol, analyze_pull, build_pull
from .simulate import DEFAULT_LIMITS, ExerciseTrajectory

INVENTORY = {"protocol_id": "POWER_CLEAN_DUAL_V1", "movement_registry": "SEMANTIC_ONLY in RES-490"}
NOT_IMPLEMENTED_EVENTS = ("TURNOVER_ONSET", "CATCH_CONTACT", "RECOVERY_END", "REP_END")
MISSING_DOFS = ("wrist flexion/extension", "3-D arm abduction and external rotation", "scapular elevation",
                "foot relocation (split/jump feet)")


class PowerCleanRefused(ValueError):
    """The requested power-clean scope is not physically supported by this model."""


@dataclass(frozen=True, slots=True)
class RackFeasibility:
    rack_offset_m: float
    upper_arm_m: float
    forearm_grip_m: float
    required_elbow_flexion_rad: float | None
    elbow_limit_rad: float
    feasible: bool
    reason: str


def front_rack_feasibility(anthropometry: Anthropometry | None = None, *, rack_offset_m: float = 0.12,
                           grip_projection: float = math.cos(math.radians(10.0))) -> RackFeasibility:
    """Elbow flexion needed to put the lumped grip at ``rack_offset_m`` from the shoulder joint."""
    for name, value in (("rack_offset_m", rack_offset_m), ("grip_projection", grip_projection)):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise PowerCleanRefused(f"INVALID_RACK_GEOMETRY: {name} must be positive finite numeric data")
    if grip_projection > 1.0:
        raise PowerCleanRefused("INVALID_RACK_GEOMETRY: grip_projection must not exceed 1")
    a = anthropometry or Anthropometry()
    l1 = a.upper_arm_length_fraction * a.stature_m * grip_projection
    l2 = a.forearm_grip_length_fraction * a.stature_m * grip_projection
    limit = DEFAULT_LIMITS["elbow"].upper_rad
    if rack_offset_m < abs(l1 - l2):
        return RackFeasibility(rack_offset_m, l1, l2, None, limit, False,
                               "rack offset shorter than the minimum arm fold")
    if rack_offset_m > l1 + l2:
        return RackFeasibility(rack_offset_m, l1, l2, None, limit, False,
                               "rack offset exceeds maximum two-link reach")
    cos_e = (rack_offset_m ** 2 - l1 ** 2 - l2 ** 2) / (2.0 * l1 * l2)
    required = math.acos(max(-1.0, min(1.0, cos_e)))
    feasible = required <= limit
    reason = ("within declared elbow range" if feasible else
              f"requires elbow flexion {required:.3f} rad > declared limit {limit:.2f} rad (no wrist joint)")
    return RackFeasibility(rack_offset_m, l1, l2, required, limit, feasible, reason)


@dataclass(frozen=True, slots=True)
class PowerCleanResult:
    pull: PullAnalysis
    events_s: dict[str, float | None]
    not_implemented: dict[str, str]
    rack: RackFeasibility


def build_power_clean(scope: str = "PULL_ONLY", *, load_mass_kg: float = 60.0,
                      anthropometry: Anthropometry | None = None):
    rack = front_rack_feasibility(anthropometry)
    if scope == "FULL":
        if rack.feasible:  # pragma: no cover - would need a catch implementation
            raise PowerCleanRefused("FULL scope requires a catch model that is not implemented")
        raise PowerCleanRefused(f"CATCH_NOT_IMPLEMENTED: {rack.reason}; missing DOFs: {', '.join(MISSING_DOFS)}")
    if scope != "PULL_ONLY":
        raise PowerCleanRefused(f"unknown power-clean scope {scope}")
    scenario = build_pull(replace(PullProtocol(), variant="CLEAN_PULL", load_mass_kg=load_mass_kg),
                          anthropometry=anthropometry)
    scenario.metadata.update({"exercise_id": "POWER_CLEAN", **INVENTORY, "scope": scope,
                              "not_implemented_events": NOT_IMPLEMENTED_EVENTS})
    return scenario


def analyze_power_clean(trajectory: ExerciseTrajectory) -> PowerCleanResult:
    pull = analyze_pull(trajectory)
    # The pull builder already projects arm lengths for its declared grip.
    rack = front_rack_feasibility(trajectory.model.anthropometry, grip_projection=1.0)
    events = {
        "REP_START": trajectory.initial_state.time_s,
        "LIFT_OFF": pull.lift_off_s,
        "FIRST_PULL_END": pull.first_pull_end_s,
        "SECOND_PULL_ONSET": pull.transition_s[1] if pull.transition_s else pull.first_pull_end_s,
        "EXTENSION_END": pull.second_pull_end_s,
    }
    reason = f"NOT_IMPLEMENTED: {rack.reason}; catch contact and impulse are not simulated"
    return PowerCleanResult(pull, events, {name: reason for name in NOT_IMPLEMENTED_EVENTS}, rack)


def catch_impulse_from_force_template(*_args, **_kwargs):
    """Refuse the shortcut the issue forbids: no catch from a force template."""
    raise PowerCleanRefused("FORBIDDEN: catch or floor events may not be synthesized from force templates")
