"""Experimental articulated drop/rebound candidate; not a qualified protocol.

The default preserves the historical post-release candidate. An explicit
platform_release_s adds rigid elevated support followed by withdrawal of that
stationary support, without resetting body position or momentum. Ground-foot
contacts are compliant; platform reactions are separate from plate forces.
No RSI or biological qualification is supplied by this builder.
"""
from dataclasses import dataclass, replace

from ..compliant_contact import HuntCrossleyContact, finite
from ..multibody import ContactState, ContactKind, ContactMode, DynamicsError, PlanarMultibodySystem
from .human import build_human, balanced_posture
from .jumps import feet_released, foot_touched, _gains
from .simulate import ControlStage, StagedJointController, simulate, static_activations


@dataclass
class DropJumpCandidate:
    model: object
    controller: object
    initial_state: object
    contact_law: HuntCrossleyContact
    duration_s: float
    drop_height_m: float
    platform_release_s: float | None = None
    platform_ids: tuple[str, ...] = ()

    def run(self, dt_s=.0002, *, compliant_scheme=None):
        return simulate("DROP_JUMP_CANDIDATE", self.model, self.controller, self.initial_state,
                        duration_s=self.duration_s, dt_s=dt_s, compliant_scheme=compliant_scheme,
                        compliant_contacts={cid: self.contact_law for ids in self.model.contact_ids.values() for cid in ids},
                        contact_release_times={cid: self.platform_release_s for cid in self.platform_ids},
                        scenario={"status": "EXPERIMENTAL_NOT_QUALIFIED", "drop_height_m": self.drop_height_m,
                                  "platform_support_interval": ("NOT_IMPLEMENTED" if self.platform_release_s is None
                                                                else [0, self.platform_release_s]),
                                  "platform_release": "stationary support withdrawal; no impulse or body reset",
                                  "parameter_status": "ASSUMED_UNCALIBRATED"})


def build_drop_jump_candidate(*, drop_height_m=.15, contact_law=None, duration_s=1.5,
                              platform_release_s=None):
    finite(drop_height_m, "drop height")
    finite(duration_s, "duration")
    if drop_height_m <= 0 or duration_s <= 0:
        raise DynamicsError("drop height and duration must be positive")
    model = build_human()
    squat = balanced_posture(model, .35, -.9, .05)
    extension = balanced_posture(model, .05, -.1, .05)
    base = model.posture_state(squat)
    platform_ids = ()
    if platform_release_s is not None:
        if finite(platform_release_s, "platform release") <= 0 or platform_release_s >= duration_s:
            raise DynamicsError("platform release must be inside the trial duration")
        platforms = [replace(c, contact_id="platform_"+c.contact_id,
                             surface_point_m=(c.surface_point_m[0], drop_height_m),
                             kind=ContactKind.PLATFORM, plate_frame=None,
                             measurement_channel_id=None, friction_group="platform_"+c.friction_group)
                     for c in model.system.contacts]
        platform_ids = tuple(c.contact_id for c in platforms)
        original = model.system
        model.system = PlanarMultibodySystem(original.bodies, original.joints,
            [*original.contacts, *platforms], gravity_m_s2=original.gravity_m_s2,
            clock_id=original.clock_id, actuators=original.actuators,
            linear_solver=original._linear_solver, linear_solver_id=original.linear_solver_id,
            projection_policy=original.projection_policy)
    initial = replace(base,
        bodies={key: replace(b, position_m=(b.position_m[0], b.position_m[1]+drop_height_m))
                for key, b in base.bodies.items()},
        contacts={**{key: ContactState() for key in base.contacts},
                  **{key: ContactState(ContactMode.STICK) for key in platform_ids}})
    if platform_ids:
        initial = model.system.make_state(initial.bodies, active_contacts=initial.contacts,
                                          activations=static_activations(model, initial))
    def reversal(m, s, t):
        return m.system.linear_momentum(s)[1] >= 0
    stages = [
        ControlStage("DESCENT", squat, _gains(600, 600, 600), _gains(40, 40, 40), False, foot_touched),
        ControlStage("BRAKING", squat, _gains(1000, 1500, 1500), _gains(60, 100, 100), True, reversal),
        ControlStage("REACTIVE_PROPULSION", extension, _gains(5000, 5000, 5000), _gains(20, 20, 20), True, feet_released),
        ControlStage("FLIGHT", squat, _gains(600, 600, 600), _gains(40, 40, 40), False, foot_touched),
        ControlStage("FINAL_LANDING", squat, _gains(1000, 1500, 1500), _gains(60, 100, 100), True),
    ]
    if platform_ids:
        stages.insert(0, ControlStage("PLATFORM_SUPPORT", squat, _gains(800, 800, 800),
            _gains(60, 60, 60), True, lambda m, s, t: s.time_s >= platform_release_s))
    controller = StagedJointController(stages, "EXPERIMENTAL_DROP_REBOUND_V1",
        additional_support_ids={side: tuple("platform_"+cid for cid in ids)
                                for side, ids in model.contact_ids.items()} if platform_ids else {})
    return DropJumpCandidate(model, controller,
                             initial, contact_law or HuntCrossleyContact(), duration_s, drop_height_m,
                             platform_release_s, platform_ids)
