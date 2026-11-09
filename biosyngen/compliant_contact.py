"""Finite Hunt–Crossley point contact, SI units, uncalibrated material law.

F = k d**n max(0, 1 + a d_dot); U = k d**(n+1)/(n+1).
The nonadhesive clamp preserves nonnegative dissipation even during forced
separation. No rigid impulse is converted into a force peak.
Reference: https://simbody.github.io/simbody-3.6-doxygen/api/classSimTK_1_1HuntCrossleyForce.html
Here a includes the reference's 3/2 coefficient. Friction is a regularized
Coulomb law; it permits creep and does not assert exact static sticking.
"""
from dataclasses import dataclass, replace
import math

from .multibody import (AppliedWrench, ContactMode, ContactState, DynamicsError,
                        PlanarMultibodySystem)


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise DynamicsError(f"{name} must be finite numeric data")
    return value


@dataclass(frozen=True, slots=True)
class CompliantResponse:
    normal_n: float
    tangent_n: float
    stored_energy_j: float
    dissipated_power_w: float


@dataclass(frozen=True, slots=True)
class HuntCrossleyContact:
    stiffness: float = 500_000.0
    exponent: float = 1.5
    damping_s_m: float = 0.5
    friction: float = 0.8
    slip_transition_m_s: float = 0.01

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            finite(getattr(self, name), name)
        if self.stiffness <= 0 or self.exponent < 1 or self.damping_s_m < 0 or self.friction < 0 or self.slip_transition_m_s <= 0:
            raise DynamicsError("inadmissible compliant material parameters")

    def evaluate(self, gap_m, normal_velocity_m_s, tangent_velocity_m_s=0.0):
        for value, name in ((gap_m, "gap"), (normal_velocity_m_s, "normal velocity"),
                            (tangent_velocity_m_s, "tangential velocity")):
            finite(value, name)
        depth = max(0.0, -gap_m)
        elastic = self.stiffness * depth ** self.exponent
        normal = elastic * max(0.0, 1.0 - self.damping_s_m * normal_velocity_m_s)
        tangent = -self.friction * normal * math.tanh(tangent_velocity_m_s / self.slip_transition_m_s)
        stored = self.stiffness * depth ** (self.exponent + 1) / (self.exponent + 1)
        dissipated = -(normal - elastic) * normal_velocity_m_s - tangent * tangent_velocity_m_s
        for value in (normal, tangent, stored, dissipated):
            finite(value, "computed contact response")
        return CompliantResponse(normal, tangent, stored, dissipated)


class CompliantPlanarStepper:
    """Replace selected world contacts with finite forces, preserving rigid others.

    Responses are evaluated at interval start (explicit first-order stepping).
    Finite compliance permits material deformation below the nominal surface.
    Refinement is mandatory; this adapter supplies no automatic stability claim.
    """
    def __init__(self, system, laws, *, release_times=None):
        self.geometry = system
        self.laws = dict(laws)
        self.release_times = dict(release_times or {})
        rigid_ids = {s.contact_id for s in system.contacts if s.contact_id not in laws}
        if not set(self.release_times) <= rigid_ids:
            raise DynamicsError("release times must identify rigid contacts")
        for time in self.release_times.values():
            if finite(time, "support release time") < 0:
                raise DynamicsError("support release time must be nonnegative")
        self._removed = frozenset()
        self.specs = {s.contact_id: s for s in system.contacts if s.contact_id in laws}
        if not self.laws or set(self.specs) != set(self.laws):
            raise DynamicsError("compliant contacts must identify existing surfaces")
        if any(s.body_b_id is not None for s in self.specs.values()):
            raise DynamicsError("compliant body-body contact is not implemented")
        if any(not isinstance(law, HuntCrossleyContact) for law in self.laws.values()):
            raise DynamicsError("explicit compliant constitutive law required")
        self.dynamics = PlanarMultibodySystem(
            system.bodies, system.joints,
            [s for s in system.contacts if s.contact_id not in laws],
            gravity_m_s2=system.gravity_m_s2, clock_id=system.clock_id,
            actuators=system.actuators, linear_solver=system._linear_solver,
            linear_solver_id=system.linear_solver_id, projection_policy=system.projection_policy)

    def advance(self, state, dt_s, *, excitations=None, external_wrenches=()):
        removed = frozenset(cid for cid, time in self.release_times.items() if state.time_s >= time)
        if any(state.time_s < time < state.time_s + dt_s for time in self.release_times.values()):
            raise DynamicsError("integration interval must end at support release")
        if removed != self._removed:
            self.dynamics = PlanarMultibodySystem(
                self.geometry.bodies, self.geometry.joints,
                [s for s in self.geometry.contacts if s.contact_id not in self.laws and s.contact_id not in removed],
                gravity_m_s2=self.geometry.gravity_m_s2, clock_id=self.geometry.clock_id,
                actuators=self.geometry.actuators, linear_solver=self.geometry._linear_solver,
                linear_solver_id=self.geometry.linear_solver_id, projection_policy=self.geometry.projection_policy)
            self._removed = removed
        wrenches = list(external_wrenches)
        responses = {}
        for cid, spec in self.specs.items():
            kin = self.geometry.contact_kinematics(state, cid)
            response = self.laws[cid].evaluate(kin.gap_m, kin.normal_velocity_m_s,
                                                kin.tangential_velocity_m_s)
            responses[cid] = response
            force = tuple(response.normal_n*n + response.tangent_n*t
                          for n, t in zip(kin.normal_world, kin.tangent_world))
            wrenches.append(AppliedWrench(spec.body_a_id, cid, force, spec.local_point_a_m))
        rigid_state = replace(state, contacts={k: v for k, v in state.contacts.items()
                                             if k not in self.laws and k not in removed})
        step = self.dynamics.advance(rigid_state, dt_s, excitations=excitations,
                                     external_wrenches=wrenches)
        contacts = dict(step.state.contacts)
        contacts.update({cid: ContactState() for cid in removed})
        contacts.update({cid: ContactState() for cid in self.laws})
        geometric_state = replace(step.state, contacts=contacts)
        for cid in self.laws:
            kin = self.geometry.contact_kinematics(geometric_state, cid)
            contacts[cid] = (ContactState(ContactMode.SLIDE, slip_direction=1 if kin.tangential_velocity_m_s >= 0 else -1)
                             if kin.gap_m < 0 else ContactState())
        return replace(step, state=replace(step.state, contacts=contacts)), responses
