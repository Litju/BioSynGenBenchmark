"""Bilateral planar human body and load topologies for I2 exercise simulators.

Seven rigid segments are used: one head-arms-trunk (HAT) segment and a left
and right thigh, shank and foot. The left and right legs lie in the same
sagittal plane but are separate bodies. Each foot carries a heel and a toe
contact. The two points form one declared friction patch, and each foot is
bound to its own force-plate frame (FZ_LEFT / FZ_RIGHT). Each joint is
driven by an extensor and a flexor actuator, each with a Hill torque-velocity
relation (RES-492 engine extension).

Segment local frames: at angle 0 a leg segment or the HAT is vertical, with
its proximal joint on +z. The foot sole is along local +x (anterior).
Relative joint angles are ``theta_distal - theta_proximal``:

* ankle = theta_foot - theta_shank (dorsiflexion positive);
* knee = theta_shank - theta_thigh (flexion negative, 0 = straight);
* hip = theta_thigh - theta_HAT (flexion positive).

Every numeric value is a declared hypothesis. Segment fractions come from
Winter (recalled, table not inspected). Torque capacities and gains are
plausibility hypotheses, not fitted to any person. Status is
ASSUMED_UNCALIBRATED throughout.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum
import math

from ..multibody import (
    HILL_TORQUE_VELOCITY_MODEL_ID,
    ActuatorHypothesis,
    ConnectionKind,
    ContactKind,
    ContactMode,
    ContactNormalFrame,
    ContactSpec,
    ContactState,
    ForcePlateFrame,
    JointKind,
    MultibodyState,
    ParameterProvenance,
    ParameterStatus,
    PlanarJoint,
    PlanarMultibodySystem,
    RigidBodyDefinition,
    RigidBodyState,
)


WINTER = (
    "Winter DA, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), "
    "anthropometric table (values recalled, table not inspected)"
)
THELEN = "Thelen DG (2003) J Biomech Eng 125:70-77 (activation time constants recalled, not inspected)"
HILL = "Hill AV (1938) Proc R Soc B 126:136-195 (hyperbolic form; curvature recalled, not inspected)"
DECLARED = "declared I2 simulation hypothesis (RES-492); not fitted to any person or dataset"

SIDES = ("left", "right")
JOINTS = ("ankle", "knee", "hip")
# Lumped bilateral arm chain (both arms in the sagittal plane, one chain).
ARM_JOINTS = ("shoulder", "elbow")
# Sign of joint-angle change produced by extension (plantarflexion, knee and
# hip extension, shoulder extension = arm backward, elbow extension).
EXTENSION_SIGN = {"ankle": -1.0, "knee": 1.0, "hip": -1.0, "shoulder": -1.0, "elbow": -1.0}


def _provenance(parameter_id: str, source: str, uncertainty: float, note: str = "") -> ParameterProvenance:
    return ParameterProvenance(parameter_id, ParameterStatus.ASSUMED_UNCALIBRATED, source, uncertainty, note)


@dataclass(frozen=True, slots=True)
class Anthropometry:
    """Whole-body mass/stature and per-leg segment fractions (Winter, recalled)."""

    body_mass_kg: float = 75.0
    stature_m: float = 1.78
    foot_mass_fraction: float = 0.0145      # per foot
    shank_mass_fraction: float = 0.0465     # per shank
    thigh_mass_fraction: float = 0.100      # per thigh
    hat_mass_fraction: float = 0.678
    shank_length_fraction: float = 0.246
    thigh_length_fraction: float = 0.245
    hat_length_fraction: float = 0.288      # greater trochanter to glenohumeral joint
    shank_com_from_ankle_fraction: float = 0.567
    thigh_com_from_knee_fraction: float = 0.567
    hat_com_from_hip_fraction: float = 0.626
    foot_gyration_fraction: float = 0.475
    shank_gyration_fraction: float = 0.302
    thigh_gyration_fraction: float = 0.323
    hat_gyration_fraction: float = 0.496
    foot_length_m: float = 0.27
    ankle_height_m: float = 0.069
    heel_x_m: float = -0.06                 # relative to the ankle, along the sole
    toe_x_m: float = 0.16                   # effective metatarsophalangeal contact point
    foot_com_from_ankle_m: tuple[float, float] = (0.05, -0.035)
    friction_coefficient: float = 0.8
    # Per-arm values (Winter, recalled); used only when an arm chain is built.
    # The HAT then excludes both arms: hat fraction - 2 (upper arm + forearm/hand).
    upper_arm_mass_fraction: float = 0.028
    forearm_hand_mass_fraction: float = 0.022
    upper_arm_length_fraction: float = 0.186
    forearm_grip_length_fraction: float = 0.189     # elbow to grip centre (forearm + 0.4 hand)
    upper_arm_com_from_shoulder_fraction: float = 0.436
    forearm_hand_com_from_elbow_fraction: float = 0.531  # derived from forearm + hand COMs
    upper_arm_gyration_fraction: float = 0.322
    forearm_hand_gyration_fraction: float = 0.35         # declared approximation for the combined segment

    def __post_init__(self) -> None:
        total = 2.0 * (self.foot_mass_fraction + self.shank_mass_fraction + self.thigh_mass_fraction) + self.hat_mass_fraction
        if abs(total - 1.0) > 1e-12:
            raise ValueError(f"segment mass fractions must sum to 1, got {total!r}")
        for name in ("body_mass_kg", "stature_m", "friction_coefficient"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{name} must be positive and finite")
        if not self.heel_x_m < 0.0 < self.toe_x_m:
            raise ValueError("heel must lie behind and toe ahead of the ankle")

    def scaled(self, *, body_mass_kg: float | None = None, stature_m: float | None = None) -> "Anthropometry":
        return replace(
            self,
            body_mass_kg=self.body_mass_kg if body_mass_kg is None else body_mass_kg,
            stature_m=self.stature_m if stature_m is None else stature_m,
        )


@dataclass(frozen=True, slots=True)
class JointStrength:
    """Per-leg joint torque capacities (N m) and maximum joint velocities (rad/s)."""

    extensor_n_m: float
    flexor_n_m: float
    extensor_max_velocity_rad_s: float
    flexor_max_velocity_rad_s: float


@dataclass(frozen=True, slots=True)
class StrengthHypothesis:
    """Declared torque generators for both legs.

    Defaults are half of the I1 lumped two-leg extensor capacities (390, 585
    and 650 N m for ankle, knee and hip) and the I1 joint-level maximum
    velocities. Flexors are weaker. ``left_scale``/``right_scale`` model
    declared strength asymmetry. These values are plausibility hypotheses.
    """

    ankle: JointStrength = JointStrength(195.0, 65.0, 36.0, 24.0)
    knee: JointStrength = JointStrength(292.5, 130.0, 30.0, 20.0)
    hip: JointStrength = JointStrength(325.0, 162.5, 27.0, 18.0)
    # Both arms lumped (used only with an arm chain); declared hypotheses.
    shoulder: JointStrength = JointStrength(160.0, 160.0, 20.0, 20.0)
    elbow: JointStrength = JointStrength(120.0, 140.0, 20.0, 20.0)
    left_scale: float = 1.0
    right_scale: float = 1.0
    activation_time_s: float = 0.015
    deactivation_time_s: float = 0.050
    hill_curvature: float = 0.25
    eccentric_ratio: float = 1.5

    def joint(self, name: str) -> JointStrength:
        return getattr(self, name)


class LoadKind(str, Enum):
    NONE = "NONE"
    VEST = "VEST"                     # load welded to the HAT near the chest (weighted vest)
    BACK_BAR = "BACK_BAR"             # barbell resting on the posterior shoulder shelf (unilateral contact)
    GRIP_BAR = "GRIP_BAR"             # barbell held in the hands (revolute grip), optionally floor-supported


@dataclass(frozen=True, slots=True)
class LoadSpec:
    kind: LoadKind = LoadKind.NONE
    mass_kg: float = 0.0
    inertia_kg_m2: float = 0.05
    friction_coefficient: float = 0.8
    # GRIP_BAR only: plate radius (bar axis height on the floor) and rim friction.
    plate_radius_m: float = 0.225
    floor_friction: float = 0.5
    floor_supported: bool = True

    def __post_init__(self) -> None:
        if self.kind is LoadKind.NONE and self.mass_kg != 0.0:
            raise ValueError("an unloaded topology cannot carry load mass")
        if self.kind is not LoadKind.NONE and not (math.isfinite(self.mass_kg) and self.mass_kg > 0.0):
            raise ValueError("a declared load must have positive finite mass")


@dataclass(frozen=True, slots=True)
class Posture:
    """Relative joint angles (rad) with both feet flat on the floor."""

    ankle: float
    knee: float
    hip: float
    shoulder: float = 0.0   # arm chain only: upper-arm angle minus HAT angle (flexion +)
    elbow: float = 0.0      # arm chain only: forearm angle minus upper-arm angle (flexion +)


@dataclass(frozen=True, slots=True)
class _Segment:
    body_id: str
    mass_kg: float
    inertia_kg_m2: float
    distal_local_m: tuple[float, float]
    proximal_local_m: tuple[float, float]


@dataclass
class HumanModel:
    """Built multibody system plus the geometry needed by controllers and instruments."""

    anthropometry: Anthropometry
    strength: StrengthHypothesis
    load: LoadSpec
    system: PlanarMultibodySystem
    segments: dict[str, _Segment]
    foot_points: dict[str, tuple[float, float]]
    hat_points: dict[str, tuple[float, float]]
    load_body_id: str | None
    contact_ids: dict[str, tuple[str, str]]          # side -> (heel, toe)
    load_contact_id: str | None
    gravity_m_s2: float
    provenance: dict[str, ParameterProvenance] = field(default_factory=dict)
    has_arms: bool = False
    hand_local_m: tuple[float, float] | None = None
    bar_floor_contact_id: str | None = None

    def arm_joint_angles(self, state: MultibodyState) -> dict[str, tuple[float, float]]:
        if not self.has_arms:
            return {}
        return {joint: self.system.joint_state(state, joint) for joint in ARM_JOINTS}

    def hand_world(self, state: MultibodyState) -> tuple[float, float]:
        return self.point_world(state, "forearm", self.hand_local_m)

    # Kinematics ---------------------------------------------------------------
    def joint_angles(self, state: MultibodyState, side: str) -> dict[str, tuple[float, float]]:
        return {
            joint: self.system.joint_state(state, f"{side}_{joint}")
            for joint in JOINTS
        }

    def total_mass_kg(self) -> float:
        return sum(body.mass_kg for body in self.system.bodies)

    def posture_state(
        self,
        posture: Posture,
        *,
        time_s: float = 0.0,
        ankle_x_m: float = 0.0,
        activations: dict[str, float] | None = None,
        check_bar_floor: bool = True,
    ) -> MultibodyState:
        """Static state with both feet flat (contacts sticking) at the posture."""
        a = self.anthropometry
        theta_foot = 0.0
        theta_shank = theta_foot - posture.ankle
        theta_thigh = theta_shank - posture.knee
        theta_hat = theta_thigh - posture.hip
        ankle = (ankle_x_m, a.ankle_height_m)
        bodies: dict[str, RigidBodyState] = {}
        for side in SIDES:
            foot = self.segments[f"{side}_foot"]
            bodies[foot.body_id] = RigidBodyState(_place(ankle, foot.proximal_local_m, theta_foot), theta_foot)
            shank = self.segments[f"{side}_shank"]
            bodies[shank.body_id] = RigidBodyState(_place(ankle, shank.distal_local_m, theta_shank), theta_shank)
            knee = _point(bodies[shank.body_id], shank.proximal_local_m)
            thigh = self.segments[f"{side}_thigh"]
            bodies[thigh.body_id] = RigidBodyState(_place(knee, thigh.distal_local_m, theta_thigh), theta_thigh)
            hip = _point(bodies[thigh.body_id], thigh.proximal_local_m)
        hat = self.segments["hat"]
        bodies[hat.body_id] = RigidBodyState(_place(hip, hat.distal_local_m, theta_hat), theta_hat)
        active: dict[str, ContactState] = {}
        for heel, toe in self.contact_ids.values():
            active[heel] = ContactState(ContactMode.STICK)
            active[toe] = ContactState(ContactMode.STICK)
        if self.has_arms:
            upper, fore = self.segments["upper_arm"], self.segments["forearm"]
            shoulder = _point(bodies[hat.body_id], self.hat_points["shoulder"])
            theta_upper = theta_hat + posture.shoulder
            bodies[upper.body_id] = RigidBodyState(_place(shoulder, upper.proximal_local_m, theta_upper), theta_upper)
            elbow = _point(bodies[upper.body_id], upper.distal_local_m)
            theta_fore = theta_upper + posture.elbow
            bodies[fore.body_id] = RigidBodyState(_place(elbow, fore.proximal_local_m, theta_fore), theta_fore)
        if self.load_body_id is not None and self.load.kind is LoadKind.GRIP_BAR:
            hand = _point(bodies["forearm"], self.hand_local_m)
            bodies[self.load_body_id] = RigidBodyState(hand, 0.0)
            if self.bar_floor_contact_id is not None:
                gap = hand[1] - self.load.plate_radius_m
                if check_bar_floor and gap < -1e-9:
                    raise ValueError(f"posture puts the bar {-gap:.4f} m below its floor support")
                if gap <= 1e-9:
                    active[self.bar_floor_contact_id] = ContactState(ContactMode.STICK)
        elif self.load_body_id is not None:
            hat_state = bodies[hat.body_id]
            if self.load.kind is LoadKind.VEST:
                anchor = self.hat_points["vest"]
            else:
                anchor = self.hat_points["shoulder_shelf"]
            bodies[self.load_body_id] = RigidBodyState(_point(hat_state, anchor), theta_hat)
            if self.load_contact_id is not None:
                active[self.load_contact_id] = ContactState(ContactMode.STICK)
        return self.system.make_state(
            bodies, time_s=time_s, active_contacts=active, activations=activations or {}
        )

    def point_world(self, state: MultibodyState, body_id: str, local: tuple[float, float]) -> tuple[float, float]:
        return _point(state.bodies[body_id], local)

    def gravity_compensation(self, state: MultibodyState, load_share: float = 1.0) -> dict[str, float]:
        """Static joint torques for symmetric bilateral support with vertical hip reactions.

        For each joint, the torque on the distal body (+T on body_b) balances
        the weight moment of everything proximal to it, i.e. the HAT and any
        load shared equally by the two legs, plus the proximal leg segments of
        the same side. This is one exact static equilibrium (zero horizontal
        hip reaction) when both feet are planted; it is used as a feedforward
        hypothesis, not as a neural model.
        """
        g = self.gravity_m_s2
        upper = [(self.segments["hat"].body_id, self.segments["hat"].mass_kg)]
        if self.has_arms:
            upper.append(("upper_arm", self.segments["upper_arm"].mass_kg))
            upper.append(("forearm", self.segments["forearm"].mass_kg))
        if self.load_body_id is not None:
            # load_share: fraction of the load weight carried by the athlete
            # (a floor-supported bar carries part of its own weight).
            upper.append((self.load_body_id, load_share * self.load.mass_kg))
        torques: dict[str, float] = {}
        for side in SIDES:
            thigh, shank = self.segments[f"{side}_thigh"], self.segments[f"{side}_shank"]
            hip = self.point_world(state, thigh.body_id, thigh.proximal_local_m)
            knee = self.point_world(state, shank.body_id, shank.proximal_local_m)
            ankle = self.point_world(state, shank.body_id, shank.distal_local_m)
            half_upper = [(state.bodies[b].position_m, 0.5 * m) for b, m in upper]
            above_hip = half_upper
            above_knee = above_hip + [(state.bodies[thigh.body_id].position_m, thigh.mass_kg)]
            above_ankle = above_knee + [(state.bodies[shank.body_id].position_m, shank.mass_kg)]
            for joint, pivot, masses in (("hip", hip, above_hip), ("knee", knee, above_knee), ("ankle", ankle, above_ankle)):
                # Equilibrium of the proximal subsystem about the joint, which
                # receives -T from the actuator: T = sum r x (0, -m g).
                torques[f"{side}_{joint}"] = sum(-(p[0] - pivot[0]) * m * g for p, m in masses)
        if self.has_arms:
            upper_arm, forearm = self.segments["upper_arm"], self.segments["forearm"]
            shoulder = self.point_world(state, "upper_arm", upper_arm.proximal_local_m)
            elbow = self.point_world(state, "forearm", forearm.proximal_local_m)
            hanging = [(state.bodies["forearm"].position_m, forearm.mass_kg)]
            if self.load_body_id is not None and self.load.kind is LoadKind.GRIP_BAR:
                hanging.append((state.bodies[self.load_body_id].position_m, load_share * self.load.mass_kg))
            # The distal (hanging) subsystem receives +T: T = sum (x - x_pivot) m g.
            torques["elbow"] = sum((p[0] - elbow[0]) * m * g for p, m in hanging)
            torques["shoulder"] = sum(
                (p[0] - shoulder[0]) * m * g
                for p, m in hanging + [(state.bodies["upper_arm"].position_m, upper_arm.mass_kg)]
            )
        return torques


def posture_com_x(model: HumanModel, posture: Posture, *, ankle_x_m: float = 0.0) -> float:
    state = model.posture_state(posture, ankle_x_m=ankle_x_m)
    return model.system.center_of_mass(state)[0]


def balanced_posture(model: HumanModel, ankle: float, knee: float, com_x_m: float) -> Posture:
    """Hip angle (bisection) that places the static system COM at com_x_m.

    With both feet flat, raising hip flexion moves the trunk (and any load)
    forward, so COM x is monotone in the hip angle over [-0.3, 2.2] rad.
    """
    lo, hi = -0.3, 2.2
    f_lo = posture_com_x(model, Posture(ankle, knee, lo)) - com_x_m
    f_hi = posture_com_x(model, Posture(ankle, knee, hi)) - com_x_m
    if f_lo * f_hi > 0.0:
        raise ValueError("no hip angle balances the requested ankle/knee posture")
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        f_mid = posture_com_x(model, Posture(ankle, knee, mid)) - com_x_m
        if (f_mid > 0.0) == (f_lo > 0.0):
            lo, f_lo = mid, f_mid
        else:
            hi = mid
    return Posture(ankle, knee, 0.5 * (lo + hi))


def _rotate(local: tuple[float, float], angle: float) -> tuple[float, float]:
    c, s = math.cos(angle), math.sin(angle)
    return c * local[0] - s * local[1], s * local[0] + c * local[1]


def _point(body: RigidBodyState, local: tuple[float, float]) -> tuple[float, float]:
    r = _rotate(local, body.angle_rad)
    return body.position_m[0] + r[0], body.position_m[1] + r[1]


def _place(world_point: tuple[float, float], local: tuple[float, float], angle: float) -> tuple[float, float]:
    """COM position such that the body's local point sits at world_point."""
    r = _rotate(local, angle)
    return world_point[0] - r[0], world_point[1] - r[1]


def build_human(
    anthropometry: Anthropometry | None = None,
    strength: StrengthHypothesis | None = None,
    load: LoadSpec | None = None,
    *,
    arms: bool = False,
    gravity_m_s2: float = 9.80665,
    clock_id: str = "I2_MODEL_CLOCK",
) -> HumanModel:
    a = anthropometry or Anthropometry()
    st = strength or StrengthHypothesis()
    ld = load or LoadSpec()
    if ld.kind is LoadKind.GRIP_BAR and not arms:
        raise ValueError("a gripped bar requires the arm chain")
    m, h = a.body_mass_kg, a.stature_m
    provenance = {
        "anthropometry": _provenance("anthropometry", WINTER, 0.10, "segment fractions; per-leg halves of lumped values"),
        "body_mass_stature": _provenance("body_mass_stature", DECLARED, 0.0, "declared hypothetical adult"),
        "foot_geometry": _provenance("foot_geometry", DECLARED, 0.15, "rigid foot with heel and MTP contact points"),
        "friction": _provenance("friction", DECLARED, 0.25, "shoe-floor Coulomb coefficient"),
        "torque_capacity": _provenance("torque_capacity", DECLARED, 0.30, "half of I1 lumped capacities"),
        "activation": _provenance("activation", THELEN, 0.30),
        "torque_velocity": _provenance("torque_velocity", HILL, 0.30),
        "load": _provenance("load", DECLARED, 0.0, ld.kind.value),
    }

    def segment_definition(body_id, mass, inertia, role, distal, proximal):
        definition = RigidBodyDefinition(
            body_id, mass, inertia,
            _provenance(f"{body_id}:mass", WINTER, 0.10),
            _provenance(f"{body_id}:inertia", WINTER, 0.20),
            role,
        )
        return definition, _Segment(body_id, mass, inertia, distal, proximal)

    l_s, l_t, l_h = a.shank_length_fraction * h, a.thigh_length_fraction * h, a.hat_length_fraction * h
    c_s, c_t, c_h = a.shank_com_from_ankle_fraction * l_s, a.thigh_com_from_knee_fraction * l_t, a.hat_com_from_hip_fraction * l_h
    fcx, fcz = a.foot_com_from_ankle_m
    definitions: list[RigidBodyDefinition] = []
    segments: dict[str, _Segment] = {}
    arm_fraction = 2.0 * (a.upper_arm_mass_fraction + a.forearm_hand_mass_fraction) if arms else 0.0
    hat_mass = (a.hat_mass_fraction - arm_fraction) * m
    d, seg = segment_definition(
        "hat", hat_mass, hat_mass * (a.hat_gyration_fraction * l_h) ** 2, "HEAD_ARMS_TRUNK",
        (0.0, -c_h), (0.0, l_h - c_h),
    )
    definitions.append(d)
    segments["hat"] = seg
    ankle_local = (-fcx, -fcz)
    foot_points = {
        "ankle": ankle_local,
        "heel": (ankle_local[0] + a.heel_x_m, ankle_local[1] - a.ankle_height_m),
        "toe": (ankle_local[0] + a.toe_x_m, ankle_local[1] - a.ankle_height_m),
    }
    for side in SIDES:
        for name, fraction, gyration, length, com, role in (
            ("thigh", a.thigh_mass_fraction, a.thigh_gyration_fraction, l_t, c_t, "THIGH"),
            ("shank", a.shank_mass_fraction, a.shank_gyration_fraction, l_s, c_s, "SHANK"),
        ):
            mass = fraction * m
            d, seg = segment_definition(
                f"{side}_{name}", mass, mass * (gyration * length) ** 2, f"{side.upper()}_{role}",
                (0.0, -com), (0.0, length - com),
            )
            definitions.append(d)
            segments[f"{side}_{name}"] = seg
        foot_mass = a.foot_mass_fraction * m
        d, seg = segment_definition(
            f"{side}_foot", foot_mass, foot_mass * (a.foot_gyration_fraction * a.foot_length_m) ** 2,
            f"{side.upper()}_FOOT", foot_points["toe"], ankle_local,
        )
        definitions.append(d)
        segments[f"{side}_foot"] = seg

    joints: list[PlanarJoint] = []
    for side in SIDES:
        joints.append(PlanarJoint(f"{side}_hip", "hat", f"{side}_thigh", segments["hat"].distal_local_m,
                                  segments[f"{side}_thigh"].proximal_local_m))
        joints.append(PlanarJoint(f"{side}_knee", f"{side}_thigh", f"{side}_shank",
                                  segments[f"{side}_thigh"].distal_local_m, segments[f"{side}_shank"].proximal_local_m))
        joints.append(PlanarJoint(f"{side}_ankle", f"{side}_shank", f"{side}_foot",
                                  segments[f"{side}_shank"].distal_local_m, ankle_local))

    contacts: list[ContactSpec] = []
    contact_ids: dict[str, tuple[str, str]] = {}
    for side in SIDES:
        channel = "FZ_LEFT" if side == "left" else "FZ_RIGHT"
        frame = ForcePlateFrame(f"{side.upper()}_PLATE", side.upper(), channel, (0.0, 0.0), (1.0, 0.0), (0.0, 1.0))
        heel_id, toe_id = f"{side}_heel", f"{side}_toe"
        for contact_id, kind, point, world_x in (
            (heel_id, ContactKind.HEEL, foot_points["heel"], a.heel_x_m),
            (toe_id, ContactKind.TOE, foot_points["toe"], a.toe_x_m),
        ):
            contacts.append(ContactSpec(
                contact_id, f"{side}_foot", point, (0.0, 1.0), a.friction_coefficient, 0.0, kind,
                surface_point_m=(world_x, 0.0), side=side.upper(), measurement_channel_id=channel,
                plate_frame=frame, friction_group=f"{side}_sole",
            ))
        contact_ids[side] = (heel_id, toe_id)

    hat_points = {
        "hip": segments["hat"].distal_local_m,
        "shoulder": segments["hat"].proximal_local_m,
        # Posterior shoulder shelf for a high-bar back squat position.
        "shoulder_shelf": (-0.07, l_h - c_h - 0.04),
        "vest": (0.06, 0.55 * l_h - c_h),
        "hip_belt": (-0.12, 0.10 - c_h),
        "head_top": (0.0, 0.47 * h - c_h),
    }
    hand_local = None
    if arms:
        l_ua, l_fa = a.upper_arm_length_fraction * h, a.forearm_grip_length_fraction * h
        c_ua, c_fa = a.upper_arm_com_from_shoulder_fraction * l_ua, a.forearm_hand_com_from_elbow_fraction * l_fa
        m_ua = 2.0 * a.upper_arm_mass_fraction * m
        m_fa = 2.0 * a.forearm_hand_mass_fraction * m
        # Arm local frame: proximal joint on +z (angle 0 = hanging straight down).
        d, seg = segment_definition(
            "upper_arm", m_ua, m_ua * (a.upper_arm_gyration_fraction * l_ua) ** 2, "BOTH_UPPER_ARMS_LUMPED",
            (0.0, -(l_ua - c_ua)), (0.0, c_ua),
        )
        definitions.append(d)
        segments["upper_arm"] = seg
        d, seg = segment_definition(
            "forearm", m_fa, m_fa * (a.forearm_hand_gyration_fraction * l_fa) ** 2, "BOTH_FOREARMS_HANDS_LUMPED",
            (0.0, -(l_fa - c_fa)), (0.0, c_fa),
        )
        definitions.append(d)
        segments["forearm"] = seg
        hand_local = segments["forearm"].distal_local_m
        joints.append(PlanarJoint("shoulder", "hat", "upper_arm", hat_points["shoulder"], segments["upper_arm"].proximal_local_m))
        joints.append(PlanarJoint("elbow", "upper_arm", "forearm", segments["upper_arm"].distal_local_m, segments["forearm"].proximal_local_m))
    load_body_id = None
    load_contact_id = None
    bar_floor_contact_id = None
    if ld.kind is LoadKind.GRIP_BAR:
        load_body_id = "load"
        definitions.append(RigidBodyDefinition(
            load_body_id, ld.mass_kg, ld.inertia_kg_m2,
            _provenance("load:mass", DECLARED, 0.0), _provenance("load:inertia", DECLARED, 0.5),
            "EXTERNAL_LOAD_GRIP_BAR",
        ))
        joints.append(PlanarJoint(
            "grip", "forearm", load_body_id, hand_local, (0.0, 0.0),
            connection_kind=ConnectionKind.GRIP,
        ))
        if ld.floor_supported:
            bar_floor_contact_id = "bar_floor"
            # The plate rim is represented by a frictional point under the bar
            # axis: bar rotation stays decoupled from rolling (declared).
            contacts.append(ContactSpec(
                bar_floor_contact_id, load_body_id, (0.0, 0.0), (0.0, 1.0), ld.floor_friction, 0.0,
                ContactKind.BAR_FLOOR, surface_point_m=(0.0, ld.plate_radius_m),
            ))
    elif ld.kind is not LoadKind.NONE:
        load_body_id = "load"
        definitions.append(RigidBodyDefinition(
            load_body_id, ld.mass_kg, ld.inertia_kg_m2,
            _provenance("load:mass", DECLARED, 0.0), _provenance("load:inertia", DECLARED, 0.5),
            f"EXTERNAL_LOAD_{ld.kind.value}",
        ))
        if ld.kind is LoadKind.VEST:
            joints.append(PlanarJoint(
                "vest_link", "hat", load_body_id, hat_points["vest"], (0.0, 0.0),
                kind=JointKind.WELD, connection_kind=ConnectionKind.LOAD_LINK,
            ))
        else:
            load_contact_id = "bar_on_back"
            contacts.append(ContactSpec(
                load_contact_id, load_body_id, (0.0, 0.0), (0.0, 1.0), ld.friction_coefficient, 0.0,
                ContactKind.RACK, body_b_id="hat", local_point_b_m=hat_points["shoulder_shelf"],
                normal_frame=ContactNormalFrame.BODY_B,
            ))

    actuators: list[ActuatorHypothesis] = []
    if arms:
        for joint in ARM_JOINTS:
            strength = st.joint(joint)
            for role, capacity, velocity in (
                ("extensor", strength.extensor_n_m, strength.extensor_max_velocity_rad_s),
                ("flexor", strength.flexor_n_m, strength.flexor_max_velocity_rad_s),
            ):
                actuators.append(ActuatorHypothesis(
                    f"{joint}_{role}", joint, capacity, st.activation_time_s, st.deactivation_time_s,
                    _provenance(f"{joint}_{role}:capacity", DECLARED, 0.30),
                    model_id=HILL_TORQUE_VELOCITY_MODEL_ID, max_joint_velocity_rad_s=velocity,
                    hill_curvature=st.hill_curvature, eccentric_ratio=st.eccentric_ratio,
                ))
    for side in SIDES:
        scale = st.left_scale if side == "left" else st.right_scale
        for joint in JOINTS:
            strength = st.joint(joint)
            for role, capacity, velocity in (
                ("extensor", strength.extensor_n_m, strength.extensor_max_velocity_rad_s),
                ("flexor", strength.flexor_n_m, strength.flexor_max_velocity_rad_s),
            ):
                actuators.append(ActuatorHypothesis(
                    f"{side}_{joint}_{role}", f"{side}_{joint}", capacity * scale,
                    st.activation_time_s, st.deactivation_time_s,
                    _provenance(f"{side}_{joint}_{role}:capacity", DECLARED, 0.30),
                    model_id=HILL_TORQUE_VELOCITY_MODEL_ID, max_joint_velocity_rad_s=velocity,
                    hill_curvature=st.hill_curvature, eccentric_ratio=st.eccentric_ratio,
                ))
    system = PlanarMultibodySystem(
        tuple(definitions), tuple(joints), tuple(contacts),
        gravity_m_s2=gravity_m_s2, clock_id=clock_id, actuators=tuple(actuators),
    )
    return HumanModel(
        anthropometry=a, strength=st, load=ld, system=system, segments=segments,
        foot_points=foot_points, hat_points=hat_points, load_body_id=load_body_id,
        contact_ids=contact_ids, load_contact_id=load_contact_id, gravity_m_s2=gravity_m_s2,
        provenance=provenance, has_arms=arms, hand_local_m=hand_local, bar_floor_contact_id=bar_floor_contact_id,
    )
