"""Reusable planar multibody dynamics with explicit contacts and parameter authority.

Coordinates use an inertial x-z lab frame, +x anterior/right, +z up, and
positive rotation from +x toward +z (CCW in the x-z plane). Every body has
COM coordinates [x_m, z_m, angle_rad] and velocities [vx_m_s, vz_m_s,
omega_rad_s]. The module returns latent mechanical contact wrenches only; it
does not generate force-plate/LPT observations or synchronize device clocks.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from enum import Enum
import math
from .i1.linalg import solve


Vec2 = tuple[float, float]
_ZERO_2: Vec2 = (0.0, 0.0)
_CONTACT_GAP_TOLERANCE_M = 1e-9
_CONTACT_VELOCITY_TOLERANCE_M_S = 1e-9
_CONTACT_FORCE_TOLERANCE_N = 1e-8
_CONTACT_IMPULSE_TOLERANCE_N_S = 1e-8
_CONSTRAINT_TOLERANCE = 1e-9
_IMPACT_ENERGY_TOLERANCE_J = 1e-8


class DynamicsError(ValueError):
    """Invalid physical state or topology."""


class ConstraintSolveError(DynamicsError):
    """The requested equality/contact topology is singular or infeasible."""


class SupportIncompleteError(DynamicsError):
    """A protocol's explicit support contract is not satisfied."""


class ParameterDomainError(DynamicsError):
    """A physical parameter lies outside its admissible data domain.

    ``field`` names the rejected parameter and ``reason`` is a stable code:
    NONNUMERIC, NONFINITE or NONPOSITIVE. Messages may change; codes may not.
    """

    def __init__(self, field: str, reason: str, message: str) -> None:
        super().__init__(message)
        self.field = field
        self.reason = reason


class ParameterStatus(str, Enum):
    MANUFACTURED_TEST_FIXTURE = "MANUFACTURED_TEST_FIXTURE"
    ASSUMED_UNCALIBRATED = "ASSUMED_UNCALIBRATED"
    SOURCE_VERIFIED = "SOURCE_VERIFIED"


@dataclass(frozen=True, slots=True)
class ParameterProvenance:
    parameter_id: str
    status: ParameterStatus
    source_reference: str
    uncertainty_fraction: float
    note: str = ""

    def __post_init__(self) -> None:
        if not self.parameter_id.strip() or not self.source_reference.strip():
            raise DynamicsError("parameter provenance requires an identity and source reference")
        uncertainty = _finite(self.uncertainty_fraction, "parameter uncertainty")
        if uncertainty < 0.0:
            raise DynamicsError("parameter uncertainty must be finite and nonnegative")
        if not isinstance(self.status, ParameterStatus):
            raise DynamicsError("parameter status must be explicit")


@dataclass(frozen=True, slots=True)
class RigidBodyDefinition:
    body_id: str
    mass_kg: float
    inertia_kg_m2: float
    mass_provenance: ParameterProvenance
    inertia_provenance: ParameterProvenance
    role: str = "UNCLASSIFIED_RIGID_BODY"

    def __post_init__(self) -> None:
        if not self.body_id.strip():
            raise DynamicsError("body_id must be nonempty")
        _positive_finite_parameter(self.mass_kg, self.body_id, "mass")
        _positive_finite_parameter(self.inertia_kg_m2, self.body_id, "inertia")
        if self.mass_provenance.parameter_id == self.inertia_provenance.parameter_id:
            raise DynamicsError("mass and inertia require distinct provenance identities")


@dataclass(frozen=True, slots=True)
class RigidBodyState:
    position_m: Vec2
    angle_rad: float
    velocity_m_s: Vec2 = _ZERO_2
    angular_velocity_rad_s: float = 0.0

    def __post_init__(self) -> None:
        _vector2(self.position_m, "position_m")
        _vector2(self.velocity_m_s, "velocity_m_s")
        _finite(self.angle_rad, "angle_rad")
        _finite(self.angular_velocity_rad_s, "angular_velocity_rad_s")


class JointKind(str, Enum):
    REVOLUTE = "REVOLUTE"
    WELD = "WELD"


class ConnectionKind(str, Enum):
    ANATOMICAL = "ANATOMICAL"
    GRIP = "GRIP"
    BACK_RACK = "BACK_RACK"
    LOAD_LINK = "LOAD_LINK"
    CATCH_LINK = "CATCH_LINK"


@dataclass(frozen=True, slots=True)
class PlanarJoint:
    joint_id: str
    body_a_id: str
    body_b_id: str
    local_anchor_a_m: Vec2
    local_anchor_b_m: Vec2
    kind: JointKind = JointKind.REVOLUTE
    connection_kind: ConnectionKind = ConnectionKind.ANATOMICAL
    relative_angle_rad: float = 0.0

    def __post_init__(self) -> None:
        if not all((self.joint_id.strip(), self.body_a_id.strip(), self.body_b_id.strip())):
            raise DynamicsError("joint identity and body IDs must be nonempty")
        if self.body_a_id == self.body_b_id:
            raise DynamicsError(f"{self.joint_id} cannot connect a body to itself")
        _vector2(self.local_anchor_a_m, "local_anchor_a_m")
        _vector2(self.local_anchor_b_m, "local_anchor_b_m")
        _finite(self.relative_angle_rad, "relative_angle_rad")
        if not isinstance(self.kind, JointKind) or not isinstance(self.connection_kind, ConnectionKind):
            raise DynamicsError("joint and load-connection kinds must be explicit")


class ContactKind(str, Enum):
    FOOT = "FOOT"
    HEEL = "HEEL"
    TOE = "TOE"
    BAR_FLOOR = "BAR_FLOOR"
    BENCH = "BENCH"
    CHEST = "CHEST"
    RACK = "RACK"
    CATCH = "CATCH"


class ContactMode(str, Enum):
    OPEN = "OPEN"
    STICK = "STICK"
    SLIDE = "SLIDE"


class ContactNormalFrame(str, Enum):
    WORLD = "WORLD"
    BODY_A = "BODY_A"
    BODY_B = "BODY_B"


@dataclass(frozen=True, slots=True)
class ForcePlateFrame:
    """Explicit planar plate coordinate frame for a foot-contact wrench.

    This is a coordinate binding only. It does not add sampling, calibration,
    noise, or a simulated device claim; each contact remains separately named.
    """

    plate_id: str
    side: str
    vertical_channel_id: str
    origin_world_m: Vec2
    x_axis_world: Vec2
    z_axis_world: Vec2

    def __post_init__(self) -> None:
        if not self.plate_id.strip() or self.side not in {"LEFT", "RIGHT"}:
            raise DynamicsError("plate frame requires a unique plate ID and LEFT/RIGHT side")
        expected_channel = "FZ_LEFT" if self.side == "LEFT" else "FZ_RIGHT"
        if self.vertical_channel_id != expected_channel:
            raise DynamicsError("plate side and vertical channel identity disagree")
        _vector2(self.origin_world_m, "plate origin")
        x_axis = _vector2(self.x_axis_world, "plate x axis")
        z_axis = _vector2(self.z_axis_world, "plate z axis")
        if not math.isclose(math.hypot(*x_axis), 1.0, abs_tol=1e-10):
            raise DynamicsError("plate x axis must be unit length")
        if not math.isclose(math.hypot(*z_axis), 1.0, abs_tol=1e-10):
            raise DynamicsError("plate z axis must be unit length")
        if abs(_dot(x_axis, z_axis)) > 1e-10 or _cross(x_axis, z_axis) <= 0.0:
            raise DynamicsError("plate axes must be orthogonal and right-handed in x-z")


@dataclass(frozen=True, slots=True)
class ContactSpec:
    contact_id: str
    body_a_id: str
    local_point_a_m: Vec2
    normal_direction: Vec2
    friction_coefficient: float
    restitution: float
    kind: ContactKind
    surface_point_m: Vec2 | None = None
    body_b_id: str | None = None
    local_point_b_m: Vec2 | None = None
    side: str | None = None
    measurement_channel_id: str | None = None
    plate_frame: ForcePlateFrame | None = None
    normal_frame: ContactNormalFrame = ContactNormalFrame.WORLD
    friction_group: str | None = None

    def __post_init__(self) -> None:
        if self.friction_group is not None and not self.friction_group.strip():
            raise DynamicsError("a declared friction group identity must be nonempty")
        if not self.contact_id.strip() or not self.body_a_id.strip():
            raise DynamicsError("contact identity and primary body ID must be nonempty")
        _vector2(self.local_point_a_m, "local_point_a_m")
        normal = _vector2(self.normal_direction, "normal_direction")
        length = math.hypot(*normal)
        if not math.isclose(length, 1.0, rel_tol=0.0, abs_tol=1e-10):
            raise DynamicsError("contact normal_direction must be a unit vector")
        if not isinstance(self.normal_frame, ContactNormalFrame):
            raise DynamicsError("contact normal frame must be explicit")
        friction = _finite(self.friction_coefficient, "friction_coefficient")
        restitution = _finite(self.restitution, "restitution")
        if friction < 0.0:
            raise DynamicsError("friction coefficient must be finite and nonnegative")
        if not 0.0 <= restitution <= 1.0:
            raise DynamicsError("restitution must lie in [0, 1]")
        if not isinstance(self.kind, ContactKind):
            raise DynamicsError("contact kind must be explicit")
        if self.body_b_id is None:
            if self.surface_point_m is None:
                raise DynamicsError("world-surface contacts require surface_point_m")
            if self.normal_frame is not ContactNormalFrame.WORLD:
                raise DynamicsError("world-surface normal must be expressed in WORLD")
            _vector2(self.surface_point_m, "surface_point_m")
            if self.local_point_b_m is not None:
                raise DynamicsError("world-surface contacts cannot declare local_point_b_m")
        else:
            if self.body_b_id == self.body_a_id or self.local_point_b_m is None:
                raise DynamicsError("body-to-body contacts require distinct bodies and a second local point")
            _vector2(self.local_point_b_m, "local_point_b_m")
            if self.surface_point_m is not None:
                raise DynamicsError("body-to-body contact uses body_b_id, not a world surface point")
        if self.kind in {ContactKind.FOOT, ContactKind.HEEL, ContactKind.TOE}:
            if self.side not in {"LEFT", "RIGHT"} or not self.measurement_channel_id:
                raise DynamicsError("foot contacts require explicit side and independent channel identity")
            if self.plate_frame is None:
                raise DynamicsError("foot contacts require explicit force-plate geometry")
            if (
                self.plate_frame.side != self.side
                or self.plate_frame.vertical_channel_id != self.measurement_channel_id
            ):
                raise DynamicsError("contact side/channel must match its force-plate frame")
        elif self.plate_frame is not None:
            raise DynamicsError("only foot/heel/toe contacts may bind force-plate geometry")


@dataclass(frozen=True, slots=True)
class ContactState:
    mode: ContactMode = ContactMode.OPEN
    slip_direction: int = 0
    tangent_anchor_m: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.mode, ContactMode):
            raise DynamicsError("contact state must be explicit")
        if self.mode is ContactMode.SLIDE and self.slip_direction not in {-1, 1}:
            raise DynamicsError("sliding contact requires a signed tangential direction")
        if self.mode is not ContactMode.SLIDE and self.slip_direction != 0:
            raise DynamicsError("only sliding contact has a slip direction")
        if self.tangent_anchor_m is not None:
            _finite(self.tangent_anchor_m, "tangent_anchor_m")


@dataclass(frozen=True, slots=True)
class SupportContract:
    topology_id: str
    allowed_contact_ids: tuple[str, ...]
    required_active_contact_ids: tuple[str, ...]

    def __post_init__(self) -> None:
        if not self.topology_id.strip():
            raise DynamicsError("support topology requires an identity")
        if any(
            not isinstance(contact_id, str) or not contact_id
            for contact_id in (*self.allowed_contact_ids, *self.required_active_contact_ids)
        ):
            raise DynamicsError("support contact IDs must be nonempty strings")
        if len(set(self.allowed_contact_ids)) != len(self.allowed_contact_ids):
            raise DynamicsError("duplicate allowed contact in support topology")
        if len(set(self.required_active_contact_ids)) != len(self.required_active_contact_ids):
            raise DynamicsError("duplicate required contact in support topology")
        if not set(self.required_active_contact_ids).issubset(self.allowed_contact_ids):
            raise DynamicsError("required contacts must be members of allowed contacts")


@dataclass(frozen=True, slots=True)
class MultibodyState:
    time_s: float
    bodies: Mapping[str, RigidBodyState]
    contacts: Mapping[str, ContactState]
    activations: Mapping[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        _finite(self.time_s, "time_s")
        for body_id, state in self.bodies.items():
            if not body_id.strip() or not isinstance(state, RigidBodyState):
                raise DynamicsError("body state mapping is malformed")
        for contact_id, status in self.contacts.items():
            if not contact_id.strip() or not isinstance(status, ContactState):
                raise DynamicsError("contact state mapping is malformed")
        for actuator_id, activation in self.activations.items():
            activation_value = _finite(activation, f"activation {actuator_id}")
            if not actuator_id.strip() or not -1.0 <= activation_value <= 1.0:
                raise DynamicsError("signed torque-activation states must be finite fractions in [-1, 1]")


@dataclass(frozen=True, slots=True)
class AppliedWrench:
    body_id: str
    source_id: str
    force_n: Vec2 = _ZERO_2
    local_point_m: Vec2 = _ZERO_2
    moment_n_m: float = 0.0

    def __post_init__(self) -> None:
        if not self.body_id.strip() or not self.source_id.strip():
            raise DynamicsError("applied wrench requires body and source identities")
        _vector2(self.force_n, "force_n")
        _vector2(self.local_point_m, "local_point_m")
        _finite(self.moment_n_m, "moment_n_m")


HILL_TORQUE_VELOCITY_MODEL_ID = "ACTIVATED_TORQUE_HILL_VELOCITY_V1"


@dataclass(frozen=True, slots=True)
class ActuatorHypothesis:
    actuator_id: str
    joint_id: str
    capacity_n_m: float
    activation_time_s: float
    deactivation_time_s: float
    provenance: ParameterProvenance
    model_id: str = "ACTIVATED_SYMMETRIC_TORQUE_CAPACITY_V1"
    max_joint_velocity_rad_s: float | None = None
    hill_curvature: float = 0.25
    eccentric_ratio: float = 1.5

    def __post_init__(self) -> None:
        if not self.actuator_id.strip() or not self.joint_id.strip():
            raise DynamicsError("actuator and joint identities must be nonempty")
        for name in ("capacity_n_m", "activation_time_s", "deactivation_time_s"):
            value = _finite(getattr(self, name), name)
            if value <= 0.0:
                raise DynamicsError(f"{name} must be positive and finite")
        if not 0.0 <= self.provenance.uncertainty_fraction <= 1.0:
            raise DynamicsError("actuator capacity uncertainty must lie in [0, 1]")
        if not self.model_id.strip():
            raise DynamicsError("actuator constitutive model identity is required")
        expected = HILL_TORQUE_VELOCITY_MODEL_ID if self.max_joint_velocity_rad_s is not None else "ACTIVATED_SYMMETRIC_TORQUE_CAPACITY_V1"
        if self.model_id != expected:
            raise DynamicsError(f"{self.actuator_id} constitutive model must be {expected} for its declared parameters")
        if self.max_joint_velocity_rad_s is not None:
            if _finite(self.max_joint_velocity_rad_s, "max_joint_velocity_rad_s") <= 0.0:
                raise DynamicsError("max_joint_velocity_rad_s must be positive")
            if _finite(self.hill_curvature, "hill_curvature") <= 0.0:
                raise DynamicsError("hill_curvature must be positive")
            if _finite(self.eccentric_ratio, "eccentric_ratio") < 1.0:
                raise DynamicsError("eccentric_ratio must be at least 1")

    def velocity_factor(self, activation: float, joint_velocity_rad_s: float) -> float:
        """Hill-type torque-velocity factor (1 when no velocity model is declared).

        u = joint velocity in the direction of the activated torque over the
        declared maximum. Shortening (u >= 0): (1 - u)/(1 + u/k), zero beyond
        u = 1. Lengthening (u < 0): 1 + (e - 1) x/(x + c) with x = -u, rising
        to e with an initial slope twice the shortening slope. Both branches
        equal 1 at u = 0. This is a declared constitutive hypothesis.
        """
        if self.max_joint_velocity_rad_s is None or activation == 0.0:
            return 1.0
        u = math.copysign(1.0, activation) * joint_velocity_rad_s / self.max_joint_velocity_rad_s
        k = self.hill_curvature
        if u >= 0.0:
            return max(0.0, (1.0 - u) / (1.0 + u / k))
        e = self.eccentric_ratio
        if e == 1.0:
            return 1.0
        c = (e - 1.0) / (2.0 * (1.0 + 1.0 / k))
        return 1.0 + (e - 1.0) * (-u) / (-u + c)


@dataclass(frozen=True, slots=True)
class ReferenceAngleController:
    """Declared PD reference controller; modeled drive is not neural truth."""

    controller_id: str
    actuator_id: str
    joint_id: str
    reference_angle_rad: float
    reference_velocity_rad_s: float
    proportional_gain_n_m_per_rad: float
    derivative_gain_n_m_s_per_rad: float
    provenance: ParameterProvenance

    def __post_init__(self) -> None:
        if not all((self.controller_id.strip(), self.actuator_id.strip(), self.joint_id.strip())):
            raise DynamicsError("controller, actuator and joint identities must be explicit")
        for name in (
            "reference_angle_rad",
            "reference_velocity_rad_s",
            "proportional_gain_n_m_per_rad",
            "derivative_gain_n_m_s_per_rad",
        ):
            value = _finite(getattr(self, name), name)
            if "gain" in name and value < 0.0:
                raise DynamicsError(f"{name} must be finite and gains nonnegative")

    def excitation(self, system: PlanarMultibodySystem, state: MultibodyState) -> float:
        if system.actuator_joint_id(self.actuator_id) != self.joint_id:
            raise DynamicsError(f"{self.controller_id} actuator is bound to a different joint")
        angle, velocity = system.joint_state(state, self.joint_id)
        requested_torque = (
            self.proportional_gain_n_m_per_rad * (self.reference_angle_rad - angle)
            + self.derivative_gain_n_m_s_per_rad * (self.reference_velocity_rad_s - velocity)
        )
        return max(-1.0, min(1.0, requested_torque / system.actuator_capacity(self.actuator_id)))


@dataclass(frozen=True, slots=True)
class ContactWrench:
    contact_id: str
    kind: ContactKind
    point_a_world_m: Vec2
    force_a_world_n: Vec2
    point_b_world_m: Vec2 | None
    force_b_world_n: Vec2
    normal_world: Vec2
    tangent_world: Vec2
    normal_force_n: float
    tangential_force_n: float
    mode: ContactMode
    side: str | None
    measurement_channel_id: str | None
    plate_id: str | None
    force_in_plate_frame_n: Vec2 | None
    moment_about_plate_origin_n_m: float | None
    moment_on_a_about_com_n_m: float
    moment_on_b_about_com_n_m: float


@dataclass(frozen=True, slots=True)
class ContactKinematics:
    contact_id: str
    point_a_world_m: Vec2
    point_b_world_m: Vec2 | None
    relative_position_m: Vec2
    relative_velocity_m_s: Vec2
    normal_world: Vec2
    tangent_world: Vec2
    gap_m: float
    normal_velocity_m_s: float
    tangential_velocity_m_s: float


@dataclass(frozen=True, slots=True)
class BodyAcceleration:
    linear_m_s2: Vec2
    angular_rad_s2: float


@dataclass(frozen=True, slots=True)
class ContactEvent:
    event_id: str
    contact_ids: tuple[str, ...]
    time_s: float
    normal_impulses_n_s: Mapping[str, float] = field(default_factory=dict)
    tangential_impulses_n_s: Mapping[str, float] = field(default_factory=dict)
    contact_work_j: float = 0.0
    detail: str = ""


@dataclass(frozen=True, slots=True)
class DynamicsResult:
    time_s: float
    clock_id: str
    linear_solver_id: str
    accelerations: Mapping[str, BodyAcceleration]
    contact_wrenches: Mapping[str, ContactWrench]
    contact_states: Mapping[str, ContactState]
    actuator_torques_n_m: Mapping[str, float]
    actuator_capacity_bounds_n_m: Mapping[str, tuple[float, float]]
    constraint_residuals: Mapping[str, float]
    contact_power_w: float
    parameter_statuses: tuple[str, ...]
    parameter_uncertainty_fractions: Mapping[str, float]
    uncertainty_policy: str = "INPUT_BOUNDS_REPORTED_NOT_PROPAGATED"
    claim_ceilings: tuple[str, ...] = (
        "NUMERICAL_VERIFICATION_ONLY",
        "SIMULATED_MEASUREMENT_ONLY",
        "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED",
        "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED",
    )


@dataclass(frozen=True, slots=True)
class StepResult:
    state: MultibodyState
    elapsed_s: float
    events: tuple[ContactEvent, ...]
    contact_work_j: float
    constraint_projection_work_j: float
    integrator_id: str = "VELOCITY_LEVEL_COULOMB_IMPULSE_EULER_MASS_METRIC_PROJECTION_V2"
    contact_impulses_n_s: Mapping[str, tuple[float, float]] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class _PointData:
    position: Vec2
    velocity: Vec2
    jacobian_x: tuple[float, ...]
    jacobian_z: tuple[float, ...]
    convective: Vec2


@dataclass(frozen=True, slots=True)
class _ContactData:
    spec: ContactSpec
    point_a: _PointData
    point_b: _PointData | None
    point_b_world: Vec2 | None
    relative_position: Vec2
    relative_velocity: Vec2
    normal: Vec2
    tangent: Vec2
    gap_m: float
    normal_velocity_m_s: float
    tangential_velocity_m_s: float
    normal_jacobian: tuple[float, ...]
    tangential_jacobian: tuple[float, ...]
    normal_bias: float
    tangential_bias: float
    tangent_coordinate_m: float


@dataclass(frozen=True, slots=True)
class _ConstraintRow:
    key: str
    jacobian: tuple[float, ...]
    force_column: tuple[float, ...]
    bias: float
    position_error: float
    velocity: float
    kind: str
    contact_id: str | None = None


def _finite(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise DynamicsError(f"{name} must be finite numeric data")
    return float(value)


def _positive_finite_parameter(value: object, owner_id: str, field_name: str) -> float:
    """Reject non-numeric, nonfinite or nonpositive mass/inertia with a typed code."""
    message = f"{owner_id} {field_name} must be positive and finite"
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ParameterDomainError(field_name, "NONNUMERIC", f"{message} (NONNUMERIC)")
    if not math.isfinite(value):
        raise ParameterDomainError(field_name, "NONFINITE", f"{message} (NONFINITE)")
    if value <= 0.0:
        raise ParameterDomainError(field_name, "NONPOSITIVE", f"{message} (NONPOSITIVE)")
    return float(value)


def _vector2(value: Sequence[float], name: str) -> Vec2:
    if len(value) != 2:
        raise DynamicsError(f"{name} must have two components")
    return _finite(value[0], f"{name}[0]"), _finite(value[1], f"{name}[1]")


def _cross(a: Vec2, b: Vec2) -> float:
    return a[0] * b[1] - a[1] * b[0]


def _rotate(point: Vec2, angle: float) -> Vec2:
    c, s = math.cos(angle), math.sin(angle)
    return c * point[0] - s * point[1], s * point[0] + c * point[1]


def _dot(a: Vec2, b: Vec2) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _sparse(row: Sequence[float]) -> list[tuple[int, float]]:
    return [(index, value) for index, value in enumerate(row) if value != 0.0]


def _weighted_products(
    left: Sequence[Sequence[tuple[int, float]]],
    right: Sequence[Sequence[tuple[int, float]]],
    weights: Sequence[float],
) -> list[list[float]]:
    """Return G[i][j] = sum_k left_i[k] * right_j[k] * weights[k] for sparse rows."""
    by_coordinate: dict[int, list[tuple[int, float]]] = {}
    for j, row in enumerate(right):
        for k, value in row:
            by_coordinate.setdefault(k, []).append((j, value * weights[k]))
    matrix = [[0.0] * len(right) for _ in left]
    for i, row in enumerate(left):
        target = matrix[i]
        for k, value in row:
            for j, weighted in by_coordinate.get(k, ()):
                target[j] += value * weighted
    return matrix


def _tangent_values(value_by_key: Mapping[str, float]) -> dict[str, float]:
    suffix = ":tangent"
    return {key[: -len(suffix)]: value for key, value in value_by_key.items() if key.endswith(suffix)}


class PlanarMultibodySystem:
    """COM-coordinate planar rigid bodies with revolute/weld joints and Coulomb contact.

    Equality constraints use Lagrange multipliers. ``solve_accelerations``
    resolves instantaneous unilateral contact forces with a deterministic
    active-set Signorini check and Coulomb stick/slide split. ``advance`` uses
    a velocity-level Coulomb impulse step (coupled joint and contact impulses,
    static-cone stick feasibility, slip-consistent sliding) with mass-metric
    joint/contact projection; impacts use simultaneous inelastic/restitution
    impulses. The model requires
    explicit body, contact, clock and parameter-provenance definitions.
    """

    def __init__(
        self,
        bodies: Sequence[RigidBodyDefinition],
        joints: Sequence[PlanarJoint],
        contacts: Sequence[ContactSpec],
        *,
        gravity_m_s2: float,
        clock_id: str,
        actuators: Sequence[ActuatorHypothesis] = (),
        linear_solver: Callable[[list[list[float]], list[float]], list[float]] | None = None,
        linear_solver_id: str | None = None,
    ) -> None:
        if not bodies:
            raise DynamicsError("a multibody system requires at least one body")
        self.bodies = tuple(bodies)
        self.joints = tuple(joints)
        self.contacts = tuple(contacts)
        self.actuators = tuple(actuators)
        self.gravity_m_s2 = _finite(gravity_m_s2, "gravity_m_s2")
        if self.gravity_m_s2 <= 0.0:
            raise DynamicsError("gravity magnitude must be positive")
        if not clock_id.strip():
            raise DynamicsError("model clock identity must be explicit")
        self.clock_id = clock_id
        if linear_solver is None:
            self._linear_solver = solve
            self.linear_solver_id = linear_solver_id or "BIOSYNGEN_I1_DENSE_GAUSSIAN_V1"
        else:
            if linear_solver_id is None:
                raise DynamicsError("custom linear solver requires a stable identity")
            self._linear_solver = linear_solver
            self.linear_solver_id = linear_solver_id
        if not self.linear_solver_id.strip():
            raise DynamicsError("linear solver identity must be nonempty")
        self._body_ids = tuple(body.body_id for body in self.bodies)
        self._body_index = {body_id: index for index, body_id in enumerate(self._body_ids)}
        self._joint_by_id = {joint.joint_id: joint for joint in self.joints}
        self._contact_by_id = {contact.contact_id: contact for contact in self.contacts}
        self._actuator_by_id = {actuator.actuator_id: actuator for actuator in self.actuators}
        if len(self._body_index) != len(self.bodies):
            raise DynamicsError("body IDs must be unique")
        if len(self._joint_by_id) != len(self.joints):
            raise DynamicsError("joint IDs must be unique")
        if len(self._contact_by_id) != len(self.contacts):
            raise DynamicsError("contact IDs must be unique")
        if len(self._actuator_by_id) != len(self.actuators):
            raise DynamicsError("actuator IDs must be unique")
        for joint in self.joints:
            if joint.body_a_id not in self._body_index or joint.body_b_id not in self._body_index:
                raise DynamicsError(f"{joint.joint_id} references an unknown body")
        plate_frames: dict[str, ForcePlateFrame] = {}
        for contact in self.contacts:
            frame = contact.plate_frame
            if frame is None:
                continue
            previous = plate_frames.get(frame.plate_id)
            if previous is not None and previous != frame:
                raise DynamicsError(f"contacts bound to {frame.plate_id} disagree on plate geometry")
            plate_frames[frame.plate_id] = frame
        plate_channels = [frame.vertical_channel_id for frame in plate_frames.values()]
        if len(plate_channels) != len(set(plate_channels)):
            raise DynamicsError("left and right plates require distinct vertical channels")
        for contact in self.contacts:
            if contact.body_a_id not in self._body_index:
                raise DynamicsError(f"{contact.contact_id} references an unknown primary body")
            if contact.body_b_id is not None and contact.body_b_id not in self._body_index:
                raise DynamicsError(f"{contact.contact_id} references an unknown second body")
        groups: dict[str, list[ContactSpec]] = {}
        for contact in self.contacts:
            if contact.friction_group is not None:
                groups.setdefault(contact.friction_group, []).append(contact)
        for group_id, members in groups.items():
            first = members[0]
            if len(members) < 2:
                raise DynamicsError(f"friction group {group_id} needs at least two contact points")
            for member in members[1:]:
                if (
                    member.body_a_id != first.body_a_id
                    or member.body_b_id != first.body_b_id
                    or member.friction_coefficient != first.friction_coefficient
                    or member.normal_direction != first.normal_direction
                    or member.normal_frame is not first.normal_frame
                    or member.plate_frame != first.plate_frame
                ):
                    raise DynamicsError(
                        f"friction group {group_id} members must share bodies, normal, friction and plate frame"
                    )
        self._friction_groups = {group_id: tuple(c.contact_id for c in members) for group_id, members in groups.items()}
        for actuator in self.actuators:
            if actuator.joint_id not in self._joint_by_id:
                raise DynamicsError(f"{actuator.actuator_id} references an unknown joint")
            if self._joint_by_id[actuator.joint_id].kind is JointKind.WELD:
                raise DynamicsError(f"{actuator.actuator_id} cannot actuate a welded connection")
        parameter_ids = [
            evidence.parameter_id
            for body in self.bodies
            for evidence in (body.mass_provenance, body.inertia_provenance)
        ] + [actuator.provenance.parameter_id for actuator in self.actuators]
        if len(parameter_ids) != len(set(parameter_ids)):
            raise DynamicsError("parameter provenance identities must be unique within a system")

    @property
    def dof_count(self) -> int:
        return 3 * len(self.bodies)

    def mass_matrix(self) -> list[list[float]]:
        """Return the configuration-independent COM-coordinate inertia matrix."""
        diagonal = self._mass_diagonal()
        return [
            [diagonal[i] if i == j else 0.0 for j in range(len(diagonal))]
            for i in range(len(diagonal))
        ]

    def _solve_linear(self, matrix: list[list[float]], rhs: list[float]) -> list[float]:
        return self._linear_solver(matrix, rhs)

    def actuator_capacity(self, actuator_id: str) -> float:
        try:
            return self._actuator_by_id[actuator_id].capacity_n_m
        except KeyError as exc:
            raise DynamicsError(f"unknown actuator {actuator_id}") from exc

    def actuator_joint_id(self, actuator_id: str) -> str:
        try:
            return self._actuator_by_id[actuator_id].joint_id
        except KeyError as exc:
            raise DynamicsError(f"unknown actuator {actuator_id}") from exc

    def joint_state(self, state: MultibodyState, joint_id: str) -> tuple[float, float]:
        self._validate_state(state)
        try:
            joint = self._joint_by_id[joint_id]
        except KeyError as exc:
            raise DynamicsError(f"unknown joint {joint_id}") from exc
        angle = _wrap_angle(
            state.bodies[joint.body_b_id].angle_rad
            - state.bodies[joint.body_a_id].angle_rad
            - joint.relative_angle_rad
        )
        velocity = (
            state.bodies[joint.body_b_id].angular_velocity_rad_s
            - state.bodies[joint.body_a_id].angular_velocity_rad_s
        )
        return angle, velocity

    def contact_kinematics(self, state: MultibodyState, contact_id: str) -> ContactKinematics:
        """Return the declared relative point and normal-axis event coordinates."""
        self._validate_state(state)
        if contact_id not in self._contact_by_id:
            raise DynamicsError(f"unknown contact {contact_id}")
        data = self._contact_data(contact_id, state)
        return ContactKinematics(
            contact_id=contact_id,
            point_a_world_m=data.point_a.position,
            point_b_world_m=data.point_b_world,
            relative_position_m=data.relative_position,
            relative_velocity_m_s=data.relative_velocity,
            normal_world=data.normal,
            tangent_world=data.tangent,
            gap_m=data.gap_m,
            normal_velocity_m_s=data.normal_velocity_m_s,
            tangential_velocity_m_s=data.tangential_velocity_m_s,
        )

    def make_state(
        self,
        body_states: Mapping[str, RigidBodyState],
        *,
        time_s: float = 0.0,
        active_contacts: Mapping[str, ContactState] | Sequence[str] = (),
        activations: Mapping[str, float] | None = None,
    ) -> MultibodyState:
        if set(body_states) != set(self._body_ids):
            raise DynamicsError("body state IDs must exactly match the system definition")
        statuses = {contact.contact_id: ContactState() for contact in self.contacts}
        if isinstance(active_contacts, Mapping):
            for contact_id, status in active_contacts.items():
                if contact_id not in statuses:
                    raise DynamicsError(f"unknown active contact {contact_id}")
                statuses[contact_id] = status
        else:
            for contact_id in active_contacts:
                if contact_id not in statuses:
                    raise DynamicsError(f"unknown active contact {contact_id}")
                statuses[contact_id] = ContactState(ContactMode.STICK)
        state = MultibodyState(time_s, dict(body_states), statuses, dict(activations or {}))
        self._validate_state(state)
        for contact_id, status in tuple(state.contacts.items()):
            if status.mode is ContactMode.STICK and status.tangent_anchor_m is None:
                data = self._contact_data(contact_id, state)
                statuses[contact_id] = ContactState(ContactMode.STICK, tangent_anchor_m=data.tangent_coordinate_m)
        return MultibodyState(state.time_s, state.bodies, statuses, state.activations)

    def _validate_state(self, state: MultibodyState) -> None:
        if set(state.bodies) != set(self._body_ids):
            raise DynamicsError("state body IDs do not match the model")
        if set(state.contacts) != set(self._contact_by_id):
            raise DynamicsError("state contact IDs do not match the model")
        if not set(state.activations).issubset(self._actuator_by_id):
            raise DynamicsError("state contains an unknown actuator activation")

    def _body_slice(self, body_id: str) -> tuple[int, int, int]:
        start = 3 * self._body_index[body_id]
        return start, start + 1, start + 2

    def _position_vector(self, state: MultibodyState) -> list[float]:
        vector: list[float] = []
        for body_id in self._body_ids:
            body = state.bodies[body_id]
            vector.extend((body.position_m[0], body.position_m[1], body.angle_rad))
        return vector

    def _velocity_vector(self, state: MultibodyState) -> list[float]:
        vector: list[float] = []
        for body_id in self._body_ids:
            body = state.bodies[body_id]
            vector.extend((body.velocity_m_s[0], body.velocity_m_s[1], body.angular_velocity_rad_s))
        return vector

    def _states_from_vectors(
        self,
        q: Sequence[float],
        qd: Sequence[float],
        time_s: float,
        contact_states: Mapping[str, ContactState],
        activations: Mapping[str, float],
    ) -> MultibodyState:
        body_states = {}
        for body_id in self._body_ids:
            ix, iz, ia = self._body_slice(body_id)
            body_states[body_id] = RigidBodyState(
                position_m=(q[ix], q[iz]),
                angle_rad=q[ia],
                velocity_m_s=(qd[ix], qd[iz]),
                angular_velocity_rad_s=qd[ia],
            )
        return MultibodyState(time_s, body_states, dict(contact_states), dict(activations))

    def _point_data(self, body_id: str, local_point: Vec2, state: MultibodyState) -> _PointData:
        body = state.bodies[body_id]
        rx, rz = _rotate(local_point, body.angle_rad)
        position = (body.position_m[0] + rx, body.position_m[1] + rz)
        velocity = (
            body.velocity_m_s[0] - body.angular_velocity_rad_s * rz,
            body.velocity_m_s[1] + body.angular_velocity_rad_s * rx,
        )
        jx = [0.0] * self.dof_count
        jz = [0.0] * self.dof_count
        ix, iz, ia = self._body_slice(body_id)
        jx[ix], jx[ia] = 1.0, -rz
        jz[iz], jz[ia] = 1.0, rx
        w2 = body.angular_velocity_rad_s * body.angular_velocity_rad_s
        return _PointData(
            position,
            velocity,
            tuple(jx),
            tuple(jz),
            (-w2 * rx, -w2 * rz),
        )

    def _contact_data(self, contact_id: str, state: MultibodyState) -> _ContactData:
        spec = self._contact_by_id[contact_id]
        point_a = self._point_data(spec.body_a_id, spec.local_point_a_m, state)
        if spec.body_b_id is None:
            point_b = None
            point_b_world = spec.surface_point_m
            relative_position = (
                point_a.position[0] - spec.surface_point_m[0],
                point_a.position[1] - spec.surface_point_m[1],
            )
            relative_velocity = point_a.velocity
            relative_bias = point_a.convective
            jac_x, jac_z = point_a.jacobian_x, point_a.jacobian_z
        else:
            point_b = self._point_data(spec.body_b_id, spec.local_point_b_m, state)
            point_b_world = point_b.position
            relative_position = (
                point_a.position[0] - point_b.position[0],
                point_a.position[1] - point_b.position[1],
            )
            relative_velocity = (
                point_a.velocity[0] - point_b.velocity[0],
                point_a.velocity[1] - point_b.velocity[1],
            )
            relative_bias = (
                point_a.convective[0] - point_b.convective[0],
                point_a.convective[1] - point_b.convective[1],
            )
            jac_x = tuple(a - b for a, b in zip(point_a.jacobian_x, point_b.jacobian_x))
            jac_z = tuple(a - b for a, b in zip(point_a.jacobian_z, point_b.jacobian_z))
        frame_omega = 0.0
        frame_angle_index = None
        if spec.normal_frame is ContactNormalFrame.WORLD:
            normal = spec.normal_direction
        else:
            frame_body_id = spec.body_a_id if spec.normal_frame is ContactNormalFrame.BODY_A else spec.body_b_id
            if frame_body_id is None:
                raise DynamicsError(f"{contact_id} requires a body frame for its normal")
            frame_body = state.bodies[frame_body_id]
            normal = _rotate(spec.normal_direction, frame_body.angle_rad)
            frame_omega = frame_body.angular_velocity_rad_s
            frame_angle_index = self._body_slice(frame_body_id)[2]
        tangent = (normal[1], -normal[0])
        normal_j = [normal[0] * x + normal[1] * z for x, z in zip(jac_x, jac_z)]
        tangent_j = [tangent[0] * x + tangent[1] * z for x, z in zip(jac_x, jac_z)]
        if frame_angle_index is not None:
            normal_j[frame_angle_index] -= _dot(tangent, relative_position)
            tangent_j[frame_angle_index] += _dot(normal, relative_position)
        normal_bias = (
            _dot(normal, relative_bias)
            - 2.0 * frame_omega * _dot(tangent, relative_velocity)
            - frame_omega * frame_omega * _dot(normal, relative_position)
        )
        tangent_bias = (
            _dot(tangent, relative_bias)
            + 2.0 * frame_omega * _dot(normal, relative_velocity)
            - frame_omega * frame_omega * _dot(tangent, relative_position)
        )
        normal_velocity = (
            _dot(normal, relative_velocity)
            - frame_omega * _dot(tangent, relative_position)
        )
        tangential_velocity = (
            _dot(tangent, relative_velocity)
            + frame_omega * _dot(normal, relative_position)
        )
        return _ContactData(
            spec,
            point_a,
            point_b,
            point_b_world,
            relative_position,
            relative_velocity,
            normal,
            tangent,
            _dot(normal, relative_position),
            normal_velocity,
            tangential_velocity,
            tuple(normal_j),
            tuple(tangent_j),
            normal_bias,
            tangent_bias,
            _dot(tangent, relative_position),
        )

    def _tangent_row_owners(self, modes: Mapping[str, ContactState]) -> dict[str, str]:
        """Map each STICK contact to the contact carrying its tangential row.

        Members of one declared friction patch (e.g. heel and toe of a rigid
        flat foot) share one tangential constraint whose capacity is mu times
        the summed member normal reactions; ungrouped contacts own their row.
        """
        owners: dict[str, str] = {}
        for contact_id, spec in self._contact_by_id.items():
            if modes[contact_id].mode is not ContactMode.STICK:
                continue
            if spec.friction_group is None:
                owners[contact_id] = contact_id
            else:
                owners[contact_id] = next(
                    member
                    for member in self._friction_groups[spec.friction_group]
                    if modes[member].mode is ContactMode.STICK
                )
        return owners

    def _harmonize_friction_groups(self, state: MultibodyState, modes: dict[str, ContactState]) -> None:
        """Active patch members share one tangential mode (they share slip)."""
        for members in self._friction_groups.values():
            active = [c for c in members if modes[c].mode is not ContactMode.OPEN]
            if len(active) < 2:
                continue
            moving = [
                c for c in active
                if modes[c].mode is ContactMode.SLIDE
                and abs(self._contact_data(c, state).tangential_velocity_m_s) > _CONTACT_VELOCITY_TOLERANCE_M_S
            ]
            if moving:
                direction = modes[moving[0]].slip_direction
                for c in active:
                    modes[c] = ContactState(ContactMode.SLIDE, slip_direction=direction)
            elif any(modes[c].mode is ContactMode.STICK for c in active):
                for c in active:
                    if modes[c].mode is not ContactMode.STICK:
                        modes[c] = ContactState(
                            ContactMode.STICK,
                            tangent_anchor_m=self._contact_data(c, state).tangent_coordinate_m,
                        )

    def _group_members(self, contact_id: str) -> tuple[str, ...]:
        group = self._contact_by_id[contact_id].friction_group
        return (contact_id,) if group is None else self._friction_groups[group]

    def _stick_tangential_shares(
        self,
        modes: Mapping[str, ContactState],
        normal_by_id: Mapping[str, float],
        tangential_by_owner: Mapping[str, float],
    ) -> dict[str, float]:
        """Report a patch's shared tangential reaction per member.

        The split is proportional to member normal reaction, so each member
        satisfies its own cone exactly when the patch does. For coplanar points
        on one plate the split has no mechanical or measured consequence.
        """
        shares: dict[str, float] = {}
        for contact_id, owner in self._tangent_row_owners(modes).items():
            members = [m for m in self._group_members(contact_id) if modes[m].mode is ContactMode.STICK]
            total = sum(max(0.0, normal_by_id.get(m, 0.0)) for m in members)
            tangential = tangential_by_owner.get(owner, 0.0)
            if total > 0.0:
                shares[contact_id] = tangential * max(0.0, normal_by_id.get(contact_id, 0.0)) / total
            else:
                shares[contact_id] = tangential / len(members)
        return shares

    def _joint_rows(self, state: MultibodyState) -> list[_ConstraintRow]:
        rows: list[_ConstraintRow] = []
        for joint in self.joints:
            a = self._point_data(joint.body_a_id, joint.local_anchor_a_m, state)
            b = self._point_data(joint.body_b_id, joint.local_anchor_b_m, state)
            for axis, j_a, j_b, bias_a, bias_b, pos_a, pos_b, vel_a, vel_b in (
                ("x", a.jacobian_x, b.jacobian_x, a.convective[0], b.convective[0], a.position[0], b.position[0], a.velocity[0], b.velocity[0]),
                ("z", a.jacobian_z, b.jacobian_z, a.convective[1], b.convective[1], a.position[1], b.position[1], a.velocity[1], b.velocity[1]),
            ):
                jac = tuple(x - y for x, y in zip(j_a, j_b))
                phi = pos_a - pos_b
                rate = vel_a - vel_b
                rows.append(_ConstraintRow(
                    f"{joint.joint_id}:{axis}", jac, jac, bias_a - bias_b, phi, rate, "JOINT"
                ))
            if joint.kind is JointKind.WELD:
                ia = self._body_slice(joint.body_a_id)[2]
                ib = self._body_slice(joint.body_b_id)[2]
                jac = [0.0] * self.dof_count
                jac[ia], jac[ib] = -1.0, 1.0
                theta_a = state.bodies[joint.body_a_id].angle_rad
                theta_b = state.bodies[joint.body_b_id].angle_rad
                rate = (
                    state.bodies[joint.body_b_id].angular_velocity_rad_s
                    - state.bodies[joint.body_a_id].angular_velocity_rad_s
                )
                rows.append(_ConstraintRow(
                    f"{joint.joint_id}:angle",
                    tuple(jac),
                    tuple(jac),
                    0.0,
                    _wrap_angle(theta_b - theta_a - joint.relative_angle_rad),
                    rate,
                    "JOINT",
                ))
        return rows

    def _contact_constraint_rows(
        self,
        state: MultibodyState,
        modes: Mapping[str, ContactState],
        *,
        slip_from_mode: bool = False,
    ) -> tuple[list[_ConstraintRow], dict[str, _ContactData]]:
        """Assemble active contact rows.

        With ``slip_from_mode`` the sliding friction direction is the mode's
        declared post-step slip direction (velocity-level stepping); otherwise
        it follows the instantaneous tangential velocity (force level).
        """
        rows: list[_ConstraintRow] = []
        data_by_id: dict[str, _ContactData] = {}
        owners = self._tangent_row_owners(modes)
        for contact_id in self._contact_by_id:
            status = modes[contact_id]
            if status.mode is ContactMode.OPEN:
                continue
            data = self._contact_data(contact_id, state)
            data_by_id[contact_id] = data
            if data.gap_m < -1e-6:
                raise ConstraintSolveError(
                    f"{contact_id} penetrates its declared surface by {-data.gap_m:.6g} m"
                )
            rows.append(_ConstraintRow(
                f"{contact_id}:normal",
                data.normal_jacobian,
                data.normal_jacobian,
                data.normal_bias,
                data.gap_m,
                data.normal_velocity_m_s,
                "CONTACT_NORMAL",
                contact_id,
            ))
            if status.mode is ContactMode.STICK and owners[contact_id] != contact_id:
                owner_data = data_by_id.get(owners[contact_id]) or self._contact_data(owners[contact_id], state)
                mismatch = max(
                    abs(a - b) for a, b in zip(data.tangential_jacobian, owner_data.tangential_jacobian)
                )
                if mismatch > 1e-6:
                    raise ConstraintSolveError(
                        f"{contact_id} is not coplanar with its friction patch ({mismatch:.3g})"
                    )
            elif status.mode is ContactMode.STICK:
                anchor = status.tangent_anchor_m
                if anchor is None:
                    anchor = data.tangent_coordinate_m
                tangential_error = data.tangent_coordinate_m - anchor
                rows.append(_ConstraintRow(
                    f"{contact_id}:tangent",
                    data.tangential_jacobian,
                    data.tangential_jacobian,
                    data.tangential_bias,
                    tangential_error,
                    data.tangential_velocity_m_s,
                    "CONTACT_TANGENT",
                    contact_id,
                ))
            elif status.mode is ContactMode.SLIDE:
                direction = status.slip_direction
                if not slip_from_mode and abs(data.tangential_velocity_m_s) > _CONTACT_VELOCITY_TOLERANCE_M_S:
                    direction = 1 if data.tangential_velocity_m_s > 0.0 else -1
                force_column = tuple(
                    n - direction * data.spec.friction_coefficient * t
                    for n, t in zip(data.normal_jacobian, data.tangential_jacobian)
                )
                rows[-1] = _ConstraintRow(
                    f"{contact_id}:normal",
                    data.normal_jacobian,
                    force_column,
                    data.normal_bias,
                    data.gap_m,
                    data.normal_velocity_m_s,
                    "CONTACT_NORMAL",
                    contact_id,
                )
        return rows, data_by_id

    def _mass_diagonal(self) -> list[float]:
        diagonal: list[float] = []
        for body in self.bodies:
            diagonal.extend((body.mass_kg, body.mass_kg, body.inertia_kg_m2))
        return diagonal

    def _assemble_generalized_forces(
        self,
        state: MultibodyState,
        excitations: Mapping[str, float],
        external_wrenches: Sequence[AppliedWrench],
    ) -> tuple[list[float], dict[str, float], dict[str, tuple[float, float]]]:
        q = [0.0] * self.dof_count
        unknown_excitations = set(excitations) - set(self._actuator_by_id)
        if unknown_excitations:
            raise DynamicsError(f"unknown actuator excitation IDs {sorted(unknown_excitations)}")
        for definition in self.bodies:
            ix, iz, _ = self._body_slice(definition.body_id)
            q[iz] -= definition.mass_kg * self.gravity_m_s2
        for wrench in external_wrenches:
            if wrench.body_id not in self._body_index:
                raise DynamicsError(f"wrench references unknown body {wrench.body_id}")
            point = self._point_data(wrench.body_id, wrench.local_point_m, state)
            ix, iz, ia = self._body_slice(wrench.body_id)
            fx, fz = wrench.force_n
            for i, value in enumerate(point.jacobian_x):
                q[i] += fx * value + fz * point.jacobian_z[i]
            q[ia] += wrench.moment_n_m
        torques: dict[str, float] = {}
        bounds: dict[str, tuple[float, float]] = {}
        for actuator in self.actuators:
            command = excitations.get(actuator.actuator_id, 0.0)
            if isinstance(command, bool) or not isinstance(command, (int, float)) or not math.isfinite(command) or not -1.0 <= command <= 1.0:
                raise DynamicsError(f"{actuator.actuator_id} excitation must lie in [-1, 1]")
            activation = state.activations.get(actuator.actuator_id, 0.0)
            joint = self._joint_by_id[actuator.joint_id]
            joint_velocity = (
                state.bodies[joint.body_b_id].angular_velocity_rad_s
                - state.bodies[joint.body_a_id].angular_velocity_rad_s
            )
            factor = actuator.velocity_factor(activation, joint_velocity)
            torque = activation * actuator.capacity_n_m * factor
            ia = self._body_slice(joint.body_a_id)[2]
            ib = self._body_slice(joint.body_b_id)[2]
            q[ia] -= torque
            q[ib] += torque
            torques[actuator.actuator_id] = torque
            uncertain = actuator.capacity_n_m * actuator.provenance.uncertainty_fraction * activation * factor
            bounds[actuator.actuator_id] = (torque - uncertain, torque + uncertain)
        return q, torques, bounds

    def _solve_kkt(
        self,
        generalized_force: Sequence[float],
        rows: Sequence[_ConstraintRow],
        targets: Sequence[float],
        *,
        velocity: Sequence[float] | None = None,
    ) -> tuple[list[float], list[float]]:
        """Solve M x - F^T lam = rhs, J x = targets by the Schur complement.

        The COM-coordinate mass matrix M is diagonal, so x = M^-1 (rhs + F^T lam)
        and lam solves (J M^-1 F^T) lam = targets - J M^-1 rhs. This is the same
        saddle-point system as the full KKT matrix; the injected linear solver
        receives the (generally nonsymmetric, for sliding rows) contact-space
        operator. Rank-deficient rows make it singular and are refused.
        """
        n = self.dof_count
        diagonal = self._mass_diagonal()
        inverse_mass = [1.0 / value for value in diagonal]
        rhs = [
            generalized_force[i] if velocity is None else diagonal[i] * velocity[i]
            for i in range(n)
        ]
        if not rows:
            return [rhs[i] * inverse_mass[i] for i in range(n)], []
        jacobians = [_sparse(row.jacobian) for row in rows]
        force_columns = [_sparse(row.force_column) for row in rows]
        operator = _weighted_products(jacobians, force_columns, inverse_mass)
        free = [rhs[i] * inverse_mass[i] for i in range(n)]
        reduced_rhs = [
            targets[r] - sum(value * free[k] for k, value in jacobians[r])
            for r in range(len(rows))
        ]
        try:
            multipliers = self._solve_linear(operator, reduced_rhs)
        except ArithmeticError as exc:
            raise ConstraintSolveError(f"singular or redundant constraint topology: {exc}") from exc
        solution = free[:]
        for r, column in enumerate(force_columns):
            lam = multipliers[r]
            if lam != 0.0:
                for k, value in column:
                    solution[k] += value * lam * inverse_mass[k]
        return solution, list(multipliers)

    def _prepare_contact_states(self, state: MultibodyState) -> dict[str, ContactState]:
        modes = dict(state.contacts)
        for contact_id, spec in self._contact_by_id.items():
            data = self._contact_data(contact_id, state)
            status = modes[contact_id]
            if (
                status.mode is not ContactMode.OPEN
                and data.gap_m > _CONTACT_GAP_TOLERANCE_M
            ):
                modes[contact_id] = ContactState()
            elif status.mode is ContactMode.OPEN:
                if data.gap_m < -1e-8:
                    raise ConstraintSolveError(f"{contact_id} begins in penetration")
                if data.gap_m <= _CONTACT_GAP_TOLERANCE_M and data.normal_velocity_m_s >= -_CONTACT_VELOCITY_TOLERANCE_M_S:
                    if abs(data.tangential_velocity_m_s) > _CONTACT_VELOCITY_TOLERANCE_M_S or spec.friction_coefficient == 0.0:
                        direction = 1 if data.tangential_velocity_m_s >= 0.0 else -1
                        modes[contact_id] = ContactState(ContactMode.SLIDE, slip_direction=direction)
                    else:
                        modes[contact_id] = ContactState(
                            ContactMode.STICK,
                            tangent_anchor_m=data.tangent_coordinate_m,
                        )
            elif status.mode is ContactMode.STICK:
                if abs(data.tangential_velocity_m_s) > _CONTACT_VELOCITY_TOLERANCE_M_S:
                    modes[contact_id] = ContactState(
                        ContactMode.SLIDE,
                        1 if data.tangential_velocity_m_s > 0.0 else -1,
                    )
                elif status.tangent_anchor_m is None:
                    modes[contact_id] = ContactState(
                        ContactMode.STICK,
                        tangent_anchor_m=data.tangent_coordinate_m,
                    )
            elif status.mode is ContactMode.SLIDE:
                if abs(data.tangential_velocity_m_s) > _CONTACT_VELOCITY_TOLERANCE_M_S:
                    modes[contact_id] = ContactState(
                        ContactMode.SLIDE,
                        1 if data.tangential_velocity_m_s > 0.0 else -1,
                    )
                elif self._contact_by_id[contact_id].friction_coefficient > 0.0:
                    modes[contact_id] = ContactState(
                        ContactMode.STICK,
                        tangent_anchor_m=data.tangent_coordinate_m,
                    )
        self._harmonize_friction_groups(state, modes)
        return modes

    def _stick_cone_violations(
        self,
        modes: Mapping[str, ContactState],
        value_by_key: Mapping[str, float],
        tolerance: float,
    ) -> list[tuple[tuple[str, ...], float]]:
        """Return (sticking members, required tangential reaction) outside the cone.

        A friction patch is tested as a whole: its shared tangential reaction
        must satisfy |t| <= mu * sum of member normal reactions.
        """
        violations = []
        for contact_id, owner in self._tangent_row_owners(modes).items():
            if owner != contact_id:
                continue
            members = tuple(m for m in self._group_members(contact_id) if modes[m].mode is ContactMode.STICK)
            normal_sum = sum(max(0.0, value_by_key.get(f"{m}:normal", 0.0)) for m in members)
            tangential = value_by_key.get(f"{contact_id}:tangent", 0.0)
            limit = self._contact_by_id[contact_id].friction_coefficient * normal_sum
            if abs(tangential) > limit + tolerance:
                violations.append((members, tangential))
        return violations

    def _active_set_solve(
        self,
        state: MultibodyState,
        generalized_force: Sequence[float],
        *,
        impulse_targets: Mapping[str, float] | None = None,
        velocity: Sequence[float] | None = None,
        extra_contact_ids: Sequence[str] = (),
    ) -> tuple[list[float], list[float], list[_ConstraintRow], dict[str, ContactState], dict[str, _ContactData]]:
        modes = self._prepare_contact_states(state)
        for contact_id in extra_contact_ids:
            if contact_id not in modes:
                raise DynamicsError(f"unknown impact contact {contact_id}")
            data = self._contact_data(contact_id, state)
            spec = self._contact_by_id[contact_id]
            if spec.friction_coefficient == 0.0:
                direction = 1 if data.tangential_velocity_m_s >= 0.0 else -1
                modes[contact_id] = ContactState(ContactMode.SLIDE, slip_direction=direction)
            else:
                modes[contact_id] = ContactState(
                    ContactMode.STICK,
                    tangent_anchor_m=data.tangent_coordinate_m,
                )
        if extra_contact_ids:
            for contact_id in self._contact_by_id:
                if contact_id in extra_contact_ids or modes[contact_id].mode is ContactMode.OPEN:
                    continue
                if any(m in extra_contact_ids for m in self._group_members(contact_id)):
                    data = self._contact_data(contact_id, state)
                    modes[contact_id] = ContactState(
                        ContactMode.STICK, tangent_anchor_m=data.tangent_coordinate_m
                    )
        for _ in range(2 * len(self.contacts) + 4):
            joint_rows = self._joint_rows(state)
            contact_rows, data_by_id = self._contact_constraint_rows(state, modes)
            rows = joint_rows + contact_rows
            if velocity is None:
                targets = [-row.bias for row in rows]
            else:
                targets = [0.0] * len(rows)
                for index, row in enumerate(rows):
                    if row.kind == "CONTACT_NORMAL" and row.contact_id in (impulse_targets or {}):
                        targets[index] = (impulse_targets or {})[row.contact_id]
            solution, multipliers = self._solve_kkt(
                generalized_force, rows, targets, velocity=velocity
            )
            multiplier_by_key = {row.key: multipliers[i] for i, row in enumerate(rows)}
            changed = False
            multiplier_tolerance = (
                _CONTACT_IMPULSE_TOLERANCE_N_S
                if velocity is not None
                else _CONTACT_FORCE_TOLERANCE_N
            )
            for contact_id in self._contact_by_id:
                status = modes[contact_id]
                if status.mode is ContactMode.OPEN:
                    continue
                normal = multiplier_by_key.get(f"{contact_id}:normal", 0.0)
                if normal < -multiplier_tolerance:
                    modes[contact_id] = ContactState()
                    changed = True
                    continue
            if changed:
                continue
            for members, tangential in self._stick_cone_violations(modes, multiplier_by_key, multiplier_tolerance):
                data = data_by_id[members[0]]
                if abs(data.tangential_velocity_m_s) > _CONTACT_VELOCITY_TOLERANCE_M_S:
                    direction = 1 if data.tangential_velocity_m_s > 0.0 else -1
                else:
                    direction = -1 if tangential > 0.0 else 1
                for member in members:
                    modes[member] = ContactState(ContactMode.SLIDE, slip_direction=direction)
                changed = True
            if changed:
                continue
            return solution, multipliers, rows, modes, data_by_id
        raise ConstraintSolveError("contact active set did not settle")

    def _impulse_step_solve(
        self,
        state: MultibodyState,
        generalized_force: Sequence[float],
        dt_s: float,
    ) -> tuple[list[float], dict[str, tuple[float, float]], dict[str, ContactState], dict[str, _ContactData]]:
        """Solve one velocity-level Coulomb contact step with coupled impulses.

        Unknowns are the post-step velocity v+ and the step impulses p of all
        joints and active contacts, solved together in one saddle-point system

            M v+ = M v + dt Q + C^T p,   J_joint v+ = 0,   J_n v+ = 0,

        so every active support shares the generalized effective mass
        W = J M^-1 J^T. Each contact mode is then tested for complementarity:
        a normal impulse must be nonadhesive (p_n >= 0, else OPEN); a STICK
        trial (J_t v+ = 0) is accepted only when its tangential impulse lies
        inside the static cone |p_t| <= mu p_n, else the contact slides
        opposite the required impulse; a SLIDE with p_t = -s mu p_n is
        accepted only when its post-step slip agrees with s, else STICK is
        tried. Reversal of slip within a step therefore yields STICK exactly
        when static friction is feasible. A repeated active set or a singular
        (rank-deficient) system is refused rather than regularized.
        """
        n = self.dof_count
        diagonal = self._mass_diagonal()
        velocity = self._velocity_vector(state)
        momentum = [diagonal[i] * velocity[i] + dt_s * generalized_force[i] for i in range(n)]
        modes = self._prepare_contact_states(state)
        visited: set[tuple[tuple[str, str, int], ...]] = set()
        for _ in range(4 * len(self.contacts) + 8):
            signature = tuple(
                (contact_id, modes[contact_id].mode.value, modes[contact_id].slip_direction)
                for contact_id in self._contact_by_id
            )
            if signature in visited:
                raise ConstraintSolveError(
                    "Coulomb impulse active set cycled; the contact problem is not uniquely resolved"
                )
            visited.add(signature)
            joint_rows = self._joint_rows(state)
            contact_rows, data_by_id = self._contact_constraint_rows(state, modes, slip_from_mode=True)
            rows = joint_rows + contact_rows
            v_plus, impulses = self._solve_kkt(momentum, rows, [0.0] * len(rows))
            impulse_by_key = {row.key: impulses[i] for i, row in enumerate(rows)}
            changed = False
            for contact_id, spec in self._contact_by_id.items():
                status = modes[contact_id]
                if status.mode is ContactMode.OPEN:
                    continue
                data = data_by_id[contact_id]
                normal = impulse_by_key[f"{contact_id}:normal"]
                if normal < -_CONTACT_IMPULSE_TOLERANCE_N_S:
                    modes[contact_id] = ContactState()
                    changed = True
                    continue
                if status.mode is ContactMode.SLIDE and spec.friction_coefficient > 0.0:
                    slip_after = sum(j * v for j, v in zip(data.tangential_jacobian, v_plus))
                    if status.slip_direction * slip_after < -_CONTACT_VELOCITY_TOLERANCE_M_S:
                        for member in self._group_members(contact_id):
                            if modes[member].mode is ContactMode.SLIDE:
                                modes[member] = ContactState(
                                    ContactMode.STICK,
                                    tangent_anchor_m=data_by_id[member].tangent_coordinate_m,
                                )
                        changed = True
            if changed:
                continue
            for members, tangential in self._stick_cone_violations(
                modes, impulse_by_key, _CONTACT_IMPULSE_TOLERANCE_N_S
            ):
                for member in members:
                    modes[member] = ContactState(
                        ContactMode.SLIDE, slip_direction=-1 if tangential > 0.0 else 1
                    )
                changed = True
            if changed:
                continue
            normals = {
                contact_id: max(0.0, impulse_by_key[f"{contact_id}:normal"])
                for contact_id in self._contact_by_id
                if modes[contact_id].mode is not ContactMode.OPEN
            }
            shares = self._stick_tangential_shares(modes, normals, _tangent_values(impulse_by_key))
            contact_impulses: dict[str, tuple[float, float]] = {}
            for contact_id, normal in normals.items():
                status = modes[contact_id]
                if status.mode is ContactMode.STICK:
                    tangential = shares[contact_id]
                else:
                    tangential = -status.slip_direction * self._contact_by_id[contact_id].friction_coefficient * normal
                contact_impulses[contact_id] = (normal, tangential)
            return v_plus, contact_impulses, modes, data_by_id
        raise ConstraintSolveError("Coulomb impulse active set did not settle")

    def solve_accelerations(
        self,
        state: MultibodyState,
        *,
        excitations: Mapping[str, float] | None = None,
        external_wrenches: Sequence[AppliedWrench] = (),
        support_contract: SupportContract | None = None,
    ) -> DynamicsResult:
        self._validate_state(state)
        if support_contract is not None:
            self.validate_support_contract(state, support_contract)
        commands = dict(excitations or {})
        generalized, actuator_torques, actuator_bounds = self._assemble_generalized_forces(
            state, commands, external_wrenches
        )
        accelerations, multipliers, rows, modes, data_by_id = self._active_set_solve(
            state,
            generalized,
        )
        if support_contract is not None:
            resolved_state = MultibodyState(state.time_s, state.bodies, modes, state.activations)
            self.validate_support_contract(resolved_state, support_contract)
        lambda_by_key = {row.key: multipliers[i] for i, row in enumerate(rows)}
        stick_shares = self._stick_tangential_shares(
            modes,
            {c: lambda_by_key.get(f"{c}:normal", 0.0) for c in self._contact_by_id},
            _tangent_values(lambda_by_key),
        )
        contact_wrenches: dict[str, ContactWrench] = {}
        contact_power = 0.0
        residuals: dict[str, float] = {}
        for contact_id, spec in self._contact_by_id.items():
            status = modes[contact_id]
            data = data_by_id.get(contact_id) or self._contact_data(contact_id, state)
            normal_force = max(0.0, lambda_by_key.get(f"{contact_id}:normal", 0.0))
            if status.mode is ContactMode.STICK:
                tangent_force = stick_shares[contact_id]
            elif status.mode is ContactMode.SLIDE:
                tangent_force = -status.slip_direction * spec.friction_coefficient * normal_force
            else:
                normal_force = tangent_force = 0.0
            force_a = (
                normal_force * data.normal[0] + tangent_force * data.tangent[0],
                normal_force * data.normal[1] + tangent_force * data.tangent[1],
            )
            force_b = (-force_a[0], -force_a[1]) if spec.body_b_id is not None else _ZERO_2
            body_a_com = state.bodies[spec.body_a_id].position_m
            moment_a = _cross(
                (data.point_a.position[0] - body_a_com[0], data.point_a.position[1] - body_a_com[1]),
                force_a,
            )
            if spec.body_b_id is not None and data.point_b_world is not None:
                body_b_com = state.bodies[spec.body_b_id].position_m
                moment_b = _cross(
                    (data.point_b_world[0] - body_b_com[0], data.point_b_world[1] - body_b_com[1]),
                    force_b,
                )
            else:
                moment_b = 0.0
            plate_frame = spec.plate_frame
            if plate_frame is None:
                plate_force = None
                plate_moment = None
            else:
                plate_force = (
                    _dot(plate_frame.x_axis_world, force_a),
                    _dot(plate_frame.z_axis_world, force_a),
                )
                moment_arm = (
                    data.point_a.position[0] - plate_frame.origin_world_m[0],
                    data.point_a.position[1] - plate_frame.origin_world_m[1],
                )
                plate_moment = _cross(moment_arm, force_a)
            contact_wrenches[contact_id] = ContactWrench(
                contact_id=contact_id,
                kind=spec.kind,
                point_a_world_m=data.point_a.position,
                force_a_world_n=force_a,
                point_b_world_m=data.point_b_world if spec.body_b_id is not None else None,
                force_b_world_n=force_b,
                normal_world=data.normal,
                tangent_world=data.tangent,
                normal_force_n=normal_force,
                tangential_force_n=tangent_force,
                mode=status.mode,
                side=spec.side,
                measurement_channel_id=spec.measurement_channel_id,
                plate_id=plate_frame.plate_id if plate_frame is not None else None,
                force_in_plate_frame_n=plate_force,
                moment_about_plate_origin_n_m=plate_moment,
                moment_on_a_about_com_n_m=moment_a,
                moment_on_b_about_com_n_m=moment_b,
            )
            relative_velocity = data.relative_velocity
            contact_power += _dot(force_a, relative_velocity)
            residuals[f"{contact_id}:nonpenetration_m"] = max(0.0, -data.gap_m)
            residuals[f"{contact_id}:complementarity_N_m"] = abs(data.gap_m * normal_force)
            residuals[f"{contact_id}:normal_force_N"] = max(0.0, -normal_force)
            residuals[f"{contact_id}:active_gap_m"] = (
                abs(data.gap_m) if status.mode is not ContactMode.OPEN else 0.0
            )
            residuals[f"{contact_id}:friction_cone_N"] = max(
                0.0,
                abs(tangent_force) - spec.friction_coefficient * normal_force,
            )
        for row in rows:
            if row.kind == "JOINT":
                residuals[f"{row.key}:position"] = abs(row.position_error)
                residuals[f"{row.key}:velocity"] = abs(row.velocity)
            row_acceleration = (
                sum(row.jacobian[i] * accelerations[i] for i in range(self.dof_count))
                + row.bias
            )
            residuals[f"{row.key}:acceleration_m_s2"] = abs(row_acceleration)
        accelerations_by_body = {}
        for body_id in self._body_ids:
            ix, iz, ia = self._body_slice(body_id)
            accelerations_by_body[body_id] = BodyAcceleration(
                linear_m_s2=(accelerations[ix], accelerations[iz]),
                angular_rad_s2=accelerations[ia],
            )
        parameter_statuses = tuple(
            sorted(
                {
                    evidence.status.value
                    for body in self.bodies
                    for evidence in (body.mass_provenance, body.inertia_provenance)
                }
                | {actuator.provenance.status.value for actuator in self.actuators}
            )
        )
        parameter_uncertainties = {
            evidence.parameter_id: evidence.uncertainty_fraction
            for definition in self.bodies
            for evidence in (definition.mass_provenance, definition.inertia_provenance)
        }
        parameter_uncertainties.update({
            actuator.provenance.parameter_id: actuator.provenance.uncertainty_fraction
            for actuator in self.actuators
        })
        return DynamicsResult(
            time_s=state.time_s,
            clock_id=self.clock_id,
            linear_solver_id=self.linear_solver_id,
            accelerations=accelerations_by_body,
            contact_wrenches=contact_wrenches,
            contact_states=modes,
            actuator_torques_n_m=actuator_torques,
            actuator_capacity_bounds_n_m=actuator_bounds,
            constraint_residuals=residuals,
            contact_power_w=contact_power,
            parameter_statuses=parameter_statuses,
            parameter_uncertainty_fractions=parameter_uncertainties,
        )

    def validate_support_contract(self, state: MultibodyState, contract: SupportContract) -> None:
        self._validate_state(state)
        unknown = set(contract.allowed_contact_ids) - set(self._contact_by_id)
        if unknown:
            raise DynamicsError(f"{contract.topology_id} references unknown contacts {sorted(unknown)}")
        active = {
            contact_id
            for contact_id, status in state.contacts.items()
            if status.mode is not ContactMode.OPEN
        }
        if not active.issubset(contract.allowed_contact_ids):
            raise SupportIncompleteError(
                f"{contract.topology_id} activates undeclared supports {sorted(active - set(contract.allowed_contact_ids))}"
            )
        missing = set(contract.required_active_contact_ids) - active
        if missing:
            raise SupportIncompleteError(
                f"{contract.topology_id} is missing required support contacts {sorted(missing)}"
            )

    def kinetic_energy(self, state: MultibodyState) -> float:
        return sum(
            0.5 * body.mass_kg * _dot(state.bodies[body.body_id].velocity_m_s, state.bodies[body.body_id].velocity_m_s)
            + 0.5 * body.inertia_kg_m2 * state.bodies[body.body_id].angular_velocity_rad_s**2
            for body in self.bodies
        )

    def potential_energy(self, state: MultibodyState) -> float:
        return sum(
            body.mass_kg * self.gravity_m_s2 * state.bodies[body.body_id].position_m[1]
            for body in self.bodies
        )

    def center_of_mass(self, state: MultibodyState) -> Vec2:
        total_mass = sum(body.mass_kg for body in self.bodies)
        return (
            sum(body.mass_kg * state.bodies[body.body_id].position_m[0] for body in self.bodies) / total_mass,
            sum(body.mass_kg * state.bodies[body.body_id].position_m[1] for body in self.bodies) / total_mass,
        )

    def linear_momentum(self, state: MultibodyState) -> Vec2:
        return (
            sum(body.mass_kg * state.bodies[body.body_id].velocity_m_s[0] for body in self.bodies),
            sum(body.mass_kg * state.bodies[body.body_id].velocity_m_s[1] for body in self.bodies),
        )

    def angular_momentum(self, state: MultibodyState, about_world_m: Vec2 = _ZERO_2) -> float:
        ox, oz = _vector2(about_world_m, "about_world_m")
        return sum(
            body.mass_kg
            * _cross(
                (
                    state.bodies[body.body_id].position_m[0] - ox,
                    state.bodies[body.body_id].position_m[1] - oz,
                ),
                state.bodies[body.body_id].velocity_m_s,
            )
            + body.inertia_kg_m2 * state.bodies[body.body_id].angular_velocity_rad_s
            for body in self.bodies
        )

    def resolve_impact(
        self,
        state: MultibodyState,
        contact_ids: Sequence[str],
    ) -> tuple[MultibodyState, ContactEvent]:
        """Resolve simultaneous active contacts with restitution and Coulomb impulse.

        Contact impulse work is the exact kinetic-energy change of the
        instantaneous velocity jump. A positive-energy impact is refused.
        """
        self._validate_state(state)
        ids = tuple(dict.fromkeys(contact_ids))
        if not ids:
            raise DynamicsError("impact requires at least one contact identity")
        targets: dict[str, float] = {}
        for contact_id in ids:
            if contact_id not in self._contact_by_id:
                raise DynamicsError(f"unknown impact contact {contact_id}")
            data = self._contact_data(contact_id, state)
            if abs(data.gap_m) > 1e-6 or data.normal_velocity_m_s >= 0.0:
                raise DynamicsError(f"{contact_id} is not a closing, nonpenetrating impact")
            targets[contact_id] = -self._contact_by_id[contact_id].restitution * data.normal_velocity_m_s
        before_ke = self.kinetic_energy(state)
        zero_force = [0.0] * self.dof_count
        qd_pre = self._velocity_vector(state)
        qd_post, impulses, rows, modes, data_by_id = self._active_set_solve(
            state,
            zero_force,
            impulse_targets=targets,
            velocity=qd_pre,
            extra_contact_ids=ids,
        )
        impulse_by_key = {row.key: impulses[i] for i, row in enumerate(rows)}
        after = self._states_from_vectors(
            self._position_vector(state),
            qd_post,
            state.time_s,
            modes,
            state.activations,
        )
        after_ke = self.kinetic_energy(after)
        work = after_ke - before_ke
        if work > _IMPACT_ENERGY_TOLERANCE_J:
            raise ConstraintSolveError(f"impact created kinetic energy ({work:.9g} J)")
        normal_impulses = {
            contact_id: max(0.0, impulse_by_key.get(f"{contact_id}:normal", 0.0))
            for contact_id in ids
        }
        stick_shares = self._stick_tangential_shares(
            modes,
            {c: impulse_by_key.get(f"{c}:normal", 0.0) for c in self._contact_by_id},
            _tangent_values(impulse_by_key),
        )
        tangential_impulses = {
            contact_id: stick_shares[contact_id]
            if modes[contact_id].mode is ContactMode.STICK
            else -modes[contact_id].slip_direction
            * self._contact_by_id[contact_id].friction_coefficient
            * normal_impulses[contact_id]
            for contact_id in ids
        }
        after = self._project_positions(after, modes)
        after = self._project_velocities(after, modes)
        event = ContactEvent(
            "CONTACT_IMPACT",
            ids,
            state.time_s,
            normal_impulses,
            tangential_impulses,
            work,
            "simultaneous rigid contact impulse; no impact is inferred unless explicitly activated",
        )
        return after, event

    def advance(
        self,
        state: MultibodyState,
        dt_s: float,
        *,
        excitations: Mapping[str, float] | None = None,
        external_wrenches: Sequence[AppliedWrench] = (),
        support_contract: SupportContract | None = None,
    ) -> StepResult:
        """Advance one velocity-level contact step, stopping at the first contact impact.

        Contact modes, impulses and the post-step velocity are solved jointly by
        ``_impulse_step_solve``; positions use the post-step velocity
        (symplectic Euler) followed by mass-metric constraint projection.
        """
        dt_s = _finite(dt_s, "dt_s")
        if dt_s <= 0.0:
            raise DynamicsError("dt_s must be positive and finite")
        self._validate_state(state)
        immediate_impacts = []
        for contact_id, status in state.contacts.items():
            if status.mode is not ContactMode.OPEN:
                continue
            data = self._contact_data(contact_id, state)
            if (
                data.gap_m <= _CONTACT_GAP_TOLERANCE_M
                and data.normal_velocity_m_s < -_CONTACT_VELOCITY_TOLERANCE_M_S
            ):
                immediate_impacts.append(contact_id)
        if immediate_impacts:
            impacted, event = self.resolve_impact(state, immediate_impacts)
            if support_contract is not None:
                self.validate_support_contract(impacted, support_contract)
            return StepResult(impacted, 0.0, (event,), event.contact_work_j, 0.0)
        if support_contract is not None:
            self.validate_support_contract(state, support_contract)
        generalized, _, _ = self._assemble_generalized_forces(
            state, dict(excitations or {}), external_wrenches
        )
        new_v, contact_impulses, step_modes, step_data = self._impulse_step_solve(
            state, generalized, dt_s
        )
        if support_contract is not None:
            self.validate_support_contract(
                MultibodyState(state.time_s, state.bodies, step_modes, state.activations),
                support_contract,
            )
        # Impulse work evaluated at the post-step relative velocity: zero for
        # stick and active normal rows, -mu p_n |v_t+| for slip.
        step_contact_work = sum(
            normal * sum(j * v for j, v in zip(step_data[contact_id].normal_jacobian, new_v))
            + tangential * sum(j * v for j, v in zip(step_data[contact_id].tangential_jacobian, new_v))
            for contact_id, (normal, tangential) in contact_impulses.items()
        )
        old_q = self._position_vector(state)
        old_v = self._velocity_vector(state)
        new_q = [old_q[i] + new_v[i] * dt_s for i in range(self.dof_count)]
        new_activations = self._advance_activations(state, dt_s, excitations or {})
        predicted = self._states_from_vectors(
            new_q,
            new_v,
            state.time_s + dt_s,
            step_modes,
            new_activations,
        )
        crossing_events = self._first_impact_crossings(state, predicted, step_modes)
        events: list[ContactEvent] = []
        released = [
            contact_id
            for contact_id, old_status in state.contacts.items()
            if old_status.mode is not ContactMode.OPEN
            and step_modes[contact_id].mode is ContactMode.OPEN
        ]
        sliding = [
            contact_id
            for contact_id, old_status in state.contacts.items()
            if old_status.mode is ContactMode.STICK
            and step_modes[contact_id].mode is ContactMode.SLIDE
        ]
        captured = [
            contact_id
            for contact_id, old_status in state.contacts.items()
            if old_status.mode is ContactMode.SLIDE
            and step_modes[contact_id].mode is ContactMode.STICK
        ]
        if released:
            events.append(ContactEvent("CONTACT_RELEASE", tuple(released), state.time_s, detail="unilateral normal impulse would be adhesive"))
        if sliding:
            events.append(ContactEvent("SLIP_ONSET", tuple(sliding), state.time_s, detail="required stick impulse exceeded the declared Coulomb cone"))
        if captured:
            events.append(ContactEvent("STICK_CAPTURE", tuple(captured), state.time_s, detail="slip reversed within the step and the stick impulse lies inside the Coulomb cone"))
        if crossing_events:
            event_time_fraction, impact_ids = crossing_events
            impact_time = state.time_s + event_time_fraction * dt_s
            impact_activations = self._advance_activations(
                state, event_time_fraction * dt_s, excitations or {}
            )
            q_event = [old_q[i] + (new_q[i] - old_q[i]) * event_time_fraction for i in range(self.dof_count)]
            v_event = [old_v[i] + (new_v[i] - old_v[i]) * event_time_fraction for i in range(self.dof_count)]
            impact_modes = dict(step_modes)
            for contact_id in impact_ids:
                data = self._contact_data(
                    contact_id,
                    self._states_from_vectors(q_event, v_event, impact_time, impact_modes, impact_activations),
                )
                impact_modes[contact_id] = ContactState(
                    ContactMode.STICK,
                    tangent_anchor_m=data.tangent_coordinate_m,
                )
            impact_state = self._states_from_vectors(
                q_event, v_event, impact_time, impact_modes, impact_activations
            )
            impact_state = self._project_positions(impact_state, impact_modes)
            impact_state, impact_event = self.resolve_impact(impact_state, impact_ids)
            if support_contract is not None:
                self.validate_support_contract(impact_state, support_contract)
            events.append(impact_event)
            return StepResult(
                impact_state,
                event_time_fraction * dt_s,
                tuple(events),
                step_contact_work * event_time_fraction + impact_event.contact_work_j,
                0.0,
                contact_impulses_n_s={
                    contact_id: (normal * event_time_fraction, tangential * event_time_fraction)
                    for contact_id, (normal, tangential) in contact_impulses.items()
                },
            )
        projected = self._project_positions(predicted, step_modes)
        projected = self._project_velocities(projected, step_modes)
        projection_work = (
            self.kinetic_energy(projected)
            + self.potential_energy(projected)
            - self.kinetic_energy(predicted)
            - self.potential_energy(predicted)
        )
        return StepResult(
            projected,
            dt_s,
            tuple(events),
            step_contact_work,
            projection_work,
            contact_impulses_n_s=contact_impulses,
        )

    def _first_impact_crossings(
        self,
        before: MultibodyState,
        after: MultibodyState,
        solved_modes: Mapping[str, ContactState],
    ) -> tuple[float, tuple[str, ...]] | None:
        candidates = []
        for contact_id, status in solved_modes.items():
            if status.mode is not ContactMode.OPEN:
                continue
            start = self._contact_data(contact_id, before)
            end = self._contact_data(contact_id, after)
            if start.gap_m > _CONTACT_GAP_TOLERANCE_M and end.gap_m <= 0.0:
                fraction = start.gap_m / (start.gap_m - end.gap_m)
                if start.normal_velocity_m_s < 0.0 and 0.0 <= fraction <= 1.0:
                    candidates.append((fraction, contact_id))
            elif start.gap_m <= _CONTACT_GAP_TOLERANCE_M and start.normal_velocity_m_s < -_CONTACT_VELOCITY_TOLERANCE_M_S:
                candidates.append((0.0, contact_id))
        if not candidates:
            return None
        first_fraction = min(x[0] for x in candidates)
        simultaneous = tuple(
            contact_id
            for fraction, contact_id in candidates
            if abs(fraction - first_fraction) * (after.time_s - before.time_s) <= 1e-8
        )
        return first_fraction, simultaneous

    def _advance_activations(
        self,
        state: MultibodyState,
        dt_s: float,
        excitations: Mapping[str, float],
    ) -> dict[str, float]:
        updated = dict(state.activations)
        for actuator in self.actuators:
            command = excitations.get(actuator.actuator_id, 0.0)
            if isinstance(command, bool) or not isinstance(command, (int, float)) or not math.isfinite(command) or not -1.0 <= command <= 1.0:
                raise DynamicsError(f"{actuator.actuator_id} excitation must lie in [-1, 1]")
            target = command
            old = state.activations.get(actuator.actuator_id, 0.0)
            tau = actuator.activation_time_s if abs(target) > abs(old) else actuator.deactivation_time_s
            updated[actuator.actuator_id] = target + (old - target) * math.exp(-dt_s / tau)
        return updated

    def _project_positions(
        self,
        state: MultibodyState,
        contact_states: Mapping[str, ContactState],
    ) -> MultibodyState:
        q = self._position_vector(state)
        qd = self._velocity_vector(state)
        diagonal = self._mass_diagonal()
        for _ in range(12):
            current = self._states_from_vectors(q, qd, state.time_s, contact_states, state.activations)
            rows = self._projection_rows(current, contact_states)
            if not rows or max(abs(error) for _, error in rows) <= _CONSTRAINT_TOLERANCE:
                return current
            sparse_rows = [_sparse(row) for row, _ in rows]
            gram = _weighted_products(sparse_rows, sparse_rows, [1.0 / value for value in diagonal])
            try:
                correction = self._solve_linear(gram, [-error for _, error in rows])
            except ArithmeticError as exc:
                raise ConstraintSolveError(f"position projection is singular: {exc}") from exc
            for row, amount in zip(sparse_rows, correction):
                for k, value in row:
                    q[k] += value * amount / diagonal[k]
        raise ConstraintSolveError("position projection did not converge")

    def _project_velocities(
        self,
        state: MultibodyState,
        contact_states: Mapping[str, ContactState],
    ) -> MultibodyState:
        qd = self._velocity_vector(state)
        q = self._position_vector(state)
        current = self._states_from_vectors(q, qd, state.time_s, contact_states, state.activations)
        rows = [row.jacobian for row in self._joint_rows(current)]
        owners = self._tangent_row_owners(contact_states)
        for contact_id, status in contact_states.items():
            if status.mode is ContactMode.OPEN:
                continue
            data = self._contact_data(contact_id, current)
            rows.append(data.normal_jacobian)
            if status.mode is ContactMode.STICK and owners[contact_id] == contact_id:
                rows.append(data.tangential_jacobian)
        if not rows:
            return current
        diagonal = self._mass_diagonal()
        sparse_rows = [_sparse(row) for row in rows]
        gram = _weighted_products(sparse_rows, sparse_rows, [1.0 / value for value in diagonal])
        rates = [sum(value * qd[k] for k, value in row) for row in sparse_rows]
        try:
            correction = self._solve_linear(gram, [-rate for rate in rates])
        except ArithmeticError as exc:
            raise ConstraintSolveError(f"velocity projection is singular: {exc}") from exc
        for row, amount in zip(sparse_rows, correction):
            for k, value in row:
                qd[k] += value * amount / diagonal[k]
        return self._states_from_vectors(q, qd, state.time_s, contact_states, state.activations)

    def _projection_rows(
        self,
        state: MultibodyState,
        contact_states: Mapping[str, ContactState],
    ) -> list[tuple[tuple[float, ...], float]]:
        rows: list[tuple[tuple[float, ...], float]] = []
        for row in self._joint_rows(state):
            rows.append((row.jacobian, row.position_error))
        owners = self._tangent_row_owners(contact_states)
        for contact_id, status in contact_states.items():
            if status.mode is ContactMode.OPEN:
                continue
            data = self._contact_data(contact_id, state)
            rows.append((data.normal_jacobian, data.gap_m))
            if (
                status.mode is ContactMode.STICK
                and status.tangent_anchor_m is not None
                and owners[contact_id] == contact_id
            ):
                tangent_error = data.tangent_coordinate_m - status.tangent_anchor_m
                rows.append((data.tangential_jacobian, tangent_error))
        return rows
