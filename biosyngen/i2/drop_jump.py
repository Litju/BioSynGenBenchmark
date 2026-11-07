"""Experimental articulated drop/rebound candidate; not a qualified protocol.

Initial state is immediately after elevated support release, at rest. The
platform support interval is not simulated. All foot contacts are compliant.
No RSI or biological qualification is supplied by this builder.
"""
from dataclasses import dataclass, replace

from ..compliant_contact import HuntCrossleyContact, finite
from ..multibody import ContactState, DynamicsError
from .human import build_human, balanced_posture
from .jumps import feet_released, foot_touched, _gains
from .simulate import ControlStage, StagedJointController, simulate


@dataclass
class DropJumpCandidate:
    model: object
    controller: object
    initial_state: object
    contact_law: HuntCrossleyContact
    duration_s: float
    drop_height_m: float

    def run(self, dt_s=.0002):
        return simulate("DROP_JUMP_CANDIDATE", self.model, self.controller, self.initial_state,
                        duration_s=self.duration_s, dt_s=dt_s,
                        compliant_contacts={cid: self.contact_law for ids in self.model.contact_ids.values() for cid in ids},
                        scenario={"status": "EXPERIMENTAL_NOT_QUALIFIED", "drop_height_m": self.drop_height_m,
                                  "platform_support_interval": "NOT_IMPLEMENTED",
                                  "parameter_status": "ASSUMED_UNCALIBRATED"})


def build_drop_jump_candidate(*, drop_height_m=.15, contact_law=None, duration_s=1.5):
    finite(drop_height_m, "drop height")
    finite(duration_s, "duration")
    if drop_height_m <= 0 or duration_s <= 0:
        raise DynamicsError("drop height and duration must be positive")
    model = build_human()
    squat = balanced_posture(model, .35, -.9, .05)
    extension = balanced_posture(model, .05, -.1, .05)
    base = model.posture_state(squat)
    initial = replace(base,
        bodies={key: replace(b, position_m=(b.position_m[0], b.position_m[1]+drop_height_m))
                for key, b in base.bodies.items()},
        contacts={key: ContactState() for key in base.contacts})
    def reversal(m, s, t):
        return m.system.linear_momentum(s)[1] >= 0
    stages = [
        ControlStage("DESCENT", squat, _gains(600, 600, 600), _gains(40, 40, 40), False, foot_touched),
        ControlStage("BRAKING", squat, _gains(1000, 1500, 1500), _gains(60, 100, 100), True, reversal),
        ControlStage("REACTIVE_PROPULSION", extension, _gains(5000, 5000, 5000), _gains(20, 20, 20), True, feet_released),
        ControlStage("FLIGHT", squat, _gains(600, 600, 600), _gains(40, 40, 40), False, foot_touched),
        ControlStage("FINAL_LANDING", squat, _gains(1000, 1500, 1500), _gains(60, 100, 100), True),
    ]
    return DropJumpCandidate(model, StagedJointController(stages, "EXPERIMENTAL_DROP_REBOUND_V1"),
                             initial, contact_law or HuntCrossleyContact(), duration_s, drop_height_m)
