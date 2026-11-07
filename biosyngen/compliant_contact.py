"""Finite Hunt–Crossley point contact, SI units, uncalibrated material law.

F = k d**n max(0, 1 + a d_dot); U = k d**(n+1)/(n+1).
The nonadhesive clamp preserves nonnegative dissipation even during forced
separation. No rigid impulse is converted into a force peak.
Reference: https://simbody.github.io/simbody-3.6-doxygen/api/classSimTK_1_1HuntCrossleyForce.html
Here a includes the reference's 3/2 coefficient. Friction is a regularized
Coulomb law; it permits creep and does not assert exact static sticking.
"""
from dataclasses import dataclass
import math

from .multibody import DynamicsError


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
