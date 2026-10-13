"""Declared parameters of the I1 planar neuromechanical jump model with provenance.

Every number carries a provenance status. None is SOURCE_VERIFIED: no primary
table was opened and read in this run. Values recalled from standard
biomechanics sources are labeled SOURCE_IDENTIFIED_VALUE_NOT_INSPECTED and are
treated as uncalibrated for any claim. The model therefore represents a
declared hypothetical adult, not a measured person or population.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math


ASSUMED = "ASSUMED_UNCALIBRATED"
RECALLED = "SOURCE_IDENTIFIED_VALUE_NOT_INSPECTED"
DERIVED = "DERIVED_FROM_DECLARED_PARAMETERS"
CONVENTION = "PHYSICAL_CONSTANT_OR_CONVENTION"


@dataclass(frozen=True, slots=True)
class Provenance:
    status: str
    source: str
    sensitivity_range: tuple[float, float] | None = None
    note: str = ""


WINTER = "Winter DA, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), anthropometric table (values recalled, table not inspected in this run)"
THELEN = "Thelen DG (2003) J Biomech Eng 125:70-77, doi:10.1115/1.1531112 (activation form; values recalled, not inspected)"
HILL = "Hill AV (1938) Proc R Soc B 126:136-195 (hyperbolic force-velocity form; curvature recalled, not inspected)"
KATZ = "Katz B (1939) J Physiol 96:45-64 (eccentric enhancement form; magnitude assumed)"


@dataclass(frozen=True, slots=True)
class BodyParameters:
    """Sagittal 4-segment body; both legs lumped into one leg chain.

    Mass fractions and segment-length fractions apply to the whole body mass
    and stature. Lumping the legs makes left/right forces non-identifiable.
    """

    body_mass_kg: float = 75.0
    stature_m: float = 1.78
    gravity_m_s2: float = 9.80665
    # Whole-body mass fractions (two feet, two shanks, two thighs lumped).
    foot_mass_fraction: float = 0.029
    shank_mass_fraction: float = 0.093
    thigh_mass_fraction: float = 0.200
    hat_mass_fraction: float = 0.678
    # Segment lengths as fractions of stature.
    shank_length_fraction: float = 0.246
    thigh_length_fraction: float = 0.245
    hat_length_fraction: float = 0.288  # greater trochanter to glenohumeral
    # COM location from the DISTAL joint for shank/thigh, from hip for HAT.
    shank_com_from_ankle_fraction: float = 0.567
    thigh_com_from_knee_fraction: float = 0.567
    hat_com_from_hip_fraction: float = 0.626
    # Radii of gyration about the segment COM, fraction of segment length.
    foot_gyration_fraction: float = 0.475
    shank_gyration_fraction: float = 0.302
    thigh_gyration_fraction: float = 0.323
    hat_gyration_fraction: float = 0.496
    # Foot geometry in the foot frame (x forward along sole, z up), metres.
    foot_length_m: float = 0.27
    ankle_height_m: float = 0.069
    heel_x_m: float = -0.06
    toe_x_m: float = 0.16  # effective metatarsophalangeal contact point
    foot_com_x_m: float = 0.05
    foot_com_z_m: float = -0.035
    friction_coefficient: float = 0.8
    # Bar attachment point on HAT (x along HAT axis, z toward posterior).
    shoulder_along_hat_m: float = 0.45
    shoulder_posterior_m: float = 0.05

    def __post_init__(self) -> None:
        total = (
            self.foot_mass_fraction
            + self.shank_mass_fraction
            + self.thigh_mass_fraction
            + self.hat_mass_fraction
        )
        if abs(total - 1.0) > 1e-12:
            raise ValueError(f"segment mass fractions must sum to 1, got {total!r}")
        for name in ("body_mass_kg", "stature_m", "gravity_m_s2", "friction_coefficient"):
            if not (getattr(self, name) > 0.0 and math.isfinite(getattr(self, name))):
                raise ValueError(f"{name} must be positive and finite")
        if not self.heel_x_m < 0.0 < self.toe_x_m:
            raise ValueError("heel must lie behind and toe ahead of the ankle")

    # Derived segment properties -------------------------------------------------
    @property
    def foot_mass_kg(self) -> float:
        return self.body_mass_kg * self.foot_mass_fraction

    @property
    def shank_mass_kg(self) -> float:
        return self.body_mass_kg * self.shank_mass_fraction

    @property
    def thigh_mass_kg(self) -> float:
        return self.body_mass_kg * self.thigh_mass_fraction

    @property
    def hat_mass_kg(self) -> float:
        return self.body_mass_kg * self.hat_mass_fraction

    @property
    def shank_length_m(self) -> float:
        return self.stature_m * self.shank_length_fraction

    @property
    def thigh_length_m(self) -> float:
        return self.stature_m * self.thigh_length_fraction

    @property
    def hat_length_m(self) -> float:
        return self.stature_m * self.hat_length_fraction

    def segment_inertias_kg_m2(self) -> tuple[float, float, float, float]:
        return (
            self.foot_mass_kg * (self.foot_gyration_fraction * self.foot_length_m) ** 2,
            self.shank_mass_kg * (self.shank_gyration_fraction * self.shank_length_m) ** 2,
            self.thigh_mass_kg * (self.thigh_gyration_fraction * self.thigh_length_m) ** 2,
            self.hat_mass_kg * (self.hat_gyration_fraction * self.hat_length_m) ** 2,
        )


@dataclass(frozen=True, slots=True)
class ActuatorParameters:
    """One joint-level Hill-type torque generator (extensor or flexor)."""

    max_isometric_torque_n_m: float
    optimal_angle_rad: float
    angle_width_rad: float
    max_shortening_velocity_rad_s: float
    hill_curvature: float = 0.25
    eccentric_max_ratio: float = 1.5


@dataclass(frozen=True, slots=True)
class JointLimits:
    min_angle_rad: float
    max_angle_rad: float
    stiffness_n_m_per_rad2: float = 5000.0
    damping_n_m_s_per_rad: float = 20.0


@dataclass(frozen=True, slots=True)
class NeuromuscularParameters:
    """Activation dynamics and joint actuators for ankle, knee and hip.

    Joint angles are extension-positive: ankle = shank minus foot angle
    (plantarflexion increases it), knee and hip = pi at full extension.
    Torques are summed over both legs.

    Capacity selection (declared, not fitted). With a first guess (extensors
    300/450/500 N m at 24/20/18 rad/s) the best optimized open-loop timing
    took off at only 0.72 m/s (apex 0.03 m). Before sliding contact existed,
    120+ random timings gave no sustained flight, only knee-lock artifacts. Of
    four tested capacity scalings, torque x1.3 with extensor max velocity x1.5
    gave the highest optimized takeoff velocity, 2.39 m/s
    (runs/i1/res482/capacity_search.txt). It was adopted because a takeoff
    velocity near 2.4 m/s is in the range commonly reported for adult CMJs.
    This calibrates to general plausibility, not to any dataset or person. The
    joint-level max velocity also stands in for the missing tendon (series
    elastic) decoupling of fibre and joint velocity.
    """

    activation_time_constant_s: float = 0.015
    deactivation_time_constant_s: float = 0.050
    minimum_excitation: float = 0.01
    ankle_extensor: ActuatorParameters = field(
        default_factory=lambda: ActuatorParameters(390.0, 1.40, 0.9, 36.0)
    )
    ankle_flexor: ActuatorParameters = field(
        default_factory=lambda: ActuatorParameters(130.0, 1.80, 0.9, 24.0)
    )
    knee_extensor: ActuatorParameters = field(
        default_factory=lambda: ActuatorParameters(585.0, 2.00, 1.1, 30.0)
    )
    knee_flexor: ActuatorParameters = field(
        default_factory=lambda: ActuatorParameters(260.0, 2.60, 1.1, 20.0)
    )
    hip_extensor: ActuatorParameters = field(
        default_factory=lambda: ActuatorParameters(650.0, 1.90, 1.2, 27.0)
    )
    hip_flexor: ActuatorParameters = field(
        default_factory=lambda: ActuatorParameters(325.0, 2.80, 1.2, 18.0)
    )
    ankle_limits: JointLimits = field(default_factory=lambda: JointLimits(0.85, 2.70))
    knee_limits: JointLimits = field(default_factory=lambda: JointLimits(0.60, math.pi))
    hip_limits: JointLimits = field(default_factory=lambda: JointLimits(0.60, math.pi + 0.35))

    def __post_init__(self) -> None:
        if not 0.0 < self.minimum_excitation < 1.0:
            raise ValueError("minimum_excitation must lie in (0, 1)")
        for name in ("activation_time_constant_s", "deactivation_time_constant_s"):
            if not getattr(self, name) > 0.0:
                raise ValueError(f"{name} must be positive")

    def actuators(self) -> tuple[ActuatorParameters, ...]:
        """Order: ankle ext, ankle flex, knee ext, knee flex, hip ext, hip flex."""
        return (
            self.ankle_extensor,
            self.ankle_flexor,
            self.knee_extensor,
            self.knee_flexor,
            self.hip_extensor,
            self.hip_flexor,
        )

    def scaled_strength(self, factor: float) -> "NeuromuscularParameters":
        def s(p: ActuatorParameters) -> ActuatorParameters:
            return replace(p, max_isometric_torque_n_m=p.max_isometric_torque_n_m * factor)

        def lim(j: JointLimits) -> JointLimits:
            return replace(
                j,
                stiffness_n_m_per_rad2=j.stiffness_n_m_per_rad2 * factor,
                damping_n_m_s_per_rad=j.damping_n_m_s_per_rad * factor,
            )

        # Passive end-range torques scale with the actuators so that a uniform
        # mass/strength scaling leaves the kinematics invariant.
        return replace(
            self,
            ankle_limits=lim(self.ankle_limits),
            knee_limits=lim(self.knee_limits),
            hip_limits=lim(self.hip_limits),
            ankle_extensor=s(self.ankle_extensor),
            ankle_flexor=s(self.ankle_flexor),
            knee_extensor=s(self.knee_extensor),
            knee_flexor=s(self.knee_flexor),
            hip_extensor=s(self.hip_extensor),
            hip_flexor=s(self.hip_flexor),
        )


@dataclass(frozen=True, slots=True)
class BarParameters:
    """Optional gripped bar/load as a point mass on a compliant coupling.

    The bar is held (bilateral coupling); the spring-damper lumps soft tissue
    and grip compliance. Stiffness and damping are uncalibrated assumptions.
    """

    mass_kg: float = 20.0
    coupling_stiffness_n_m: float = 2.0e5
    coupling_damping_n_s_m: float = 1.2e3
    half_length_lateral_m: float = 0.60  # attachment at the collar, out of plane

    def __post_init__(self) -> None:
        for name in ("mass_kg", "coupling_stiffness_n_m", "coupling_damping_n_s_m"):
            if not getattr(self, name) > 0.0:
                raise ValueError(f"{name} must be positive")


PROVENANCE: dict[str, Provenance] = {
    "body_mass_kg": Provenance(ASSUMED, "declared hypothetical adult", (55.0, 100.0)),
    "stature_m": Provenance(ASSUMED, "declared hypothetical adult", (1.60, 1.95)),
    "gravity_m_s2": Provenance(CONVENTION, "standard gravity (CGPM 1901)"),
    "segment_mass_fractions": Provenance(RECALLED, WINTER, None, "two legs lumped; fractions sum to 1"),
    "segment_length_fractions": Provenance(RECALLED, WINTER),
    "segment_com_fractions": Provenance(RECALLED, WINTER),
    "segment_gyration_fractions": Provenance(RECALLED, WINTER),
    "foot_geometry": Provenance(ASSUMED, "declared rigid foot with heel and MTP contact points", None, "no toe segment"),
    "friction_coefficient": Provenance(ASSUMED, "shoe-floor; only used as a slip refusal threshold", (0.5, 1.0)),
    "activation_time_constant_s": Provenance(RECALLED, THELEN, (0.010, 0.020)),
    "deactivation_time_constant_s": Provenance(RECALLED, THELEN, (0.040, 0.060)),
    "activation_dynamics_form": Provenance(RECALLED, THELEN, None, "tau scaled by (0.5+1.5a)"),
    "hill_curvature": Provenance(RECALLED, HILL, (0.15, 0.40)),
    "eccentric_max_ratio": Provenance(ASSUMED, KATZ, (1.3, 1.8)),
    "max_isometric_torques": Provenance(ASSUMED, "joint-level, both legs summed; not fitted to a person", (0.7, 1.3), "range is a multiplicative factor"),
    "torque_angle_curves": Provenance(ASSUMED, "Gaussian joint torque-angle relation; optimum and width declared"),
    "max_shortening_velocities": Provenance(ASSUMED, "joint-level", (0.75, 1.25), "range is a multiplicative factor"),
    "joint_limits": Provenance(ASSUMED, "passive quadratic end-range springs"),
    "bar_mass_kg": Provenance(ASSUMED, "declared external load", (0.0, 60.0)),
    "bar_coupling": Provenance(ASSUMED, "gripped bar spring-damper; lumps tissue and grip compliance", (5.0e4, 1.0e6)),
}
