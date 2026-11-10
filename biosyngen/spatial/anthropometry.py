"""V2 inertial authority. SI units; manufactured priors, never measurements.

The normalized RES-610 proportions are a continuity hypothesis, not de Leva
table values. Literature context: doi:10.1016/0021-9290(95)00178-6.
"""
from dataclasses import dataclass, field, fields
import math
import xml.etree.ElementTree as ET

import numpy as np

from .model import AthleteBarbellSpec, build_mjcf, joint_specs


@dataclass(frozen=True)
class HumanDimensions:
    """Lengths in metres at the declared stature; synthetic adult domains."""
    shoulder_breadth_m: float = .38
    upper_arm_m: float = .30
    forearm_m: float = .26
    hand_length_m: float = .19
    hand_width_m: float = .08
    thigh_m: float = .43
    shank_m: float = .43
    foot_length_m: float = .32
    foot_width_m: float = .09
    trunk_length_m: float = .48
    trunk_width_m: float = .26
    trunk_depth_m: float = .20

    def __post_init__(self):
        limits = {"shoulder_breadth_m": (.25, .60), "upper_arm_m": (.20, .45),
                  "forearm_m": (.17, .38), "hand_length_m": (.13, .27), "hand_width_m": (.05, .13),
                  "thigh_m": (.28, .62), "shank_m": (.28, .62), "foot_length_m": (.20, .42),
                  "foot_width_m": (.06, .15), "trunk_length_m": (.30, .68),
                  "trunk_width_m": (.18, .45), "trunk_depth_m": (.12, .35)}
        for key, (low, high) in limits.items():
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{key} outside declared synthetic adult domain")


@dataclass(frozen=True)
class JointAuthority:
    name: str
    lower_rad: float
    upper_rad: float
    capacity_nm: float

    def __post_init__(self):
        values = (self.lower_rad, self.upper_rad, self.capacity_nm)
        if not self.name or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
            raise ValueError("invalid joint parameter")
        if not -math.pi <= self.lower_rad < self.upper_rad <= math.pi or not 0 < self.capacity_nm <= 500:
            raise ValueError("joint outside declared mechanical domain")


def default_joint_authority():
    return tuple(JointAuthority(name, j.lower, j.upper, j.capacity_n_m) for name, j in joint_specs())


@dataclass(frozen=True)
class SegmentInertia:
    name: str
    mass_kg: float
    com_m: tuple[float, float, float]
    tensor_kg_m2: tuple[tuple[float, float, float], ...]
    frame: str = "BODY_LOCAL_X_ANTERIOR_Y_LEFT_Z_UP"

    def __post_init__(self):
        c, i = np.asarray(self.com_m), np.asarray(self.tensor_kg_m2)
        if not self.name or self.frame != "BODY_LOCAL_X_ANTERIOR_Y_LEFT_Z_UP":
            raise ValueError("undeclared inertial frame")
        if isinstance(self.mass_kg, bool) or not math.isfinite(self.mass_kg) or self.mass_kg <= 0:
            raise ValueError("mass must be positive finite kg")
        if c.shape != (3,) or not np.isfinite(c).all():
            raise ValueError("invalid local COM")
        if i.shape != (3, 3) or not np.isfinite(i).all() or not np.array_equal(i, i.T):
            raise ValueError("inertia must be finite symmetric 3x3")
        eig = np.linalg.eigvalsh(i)
        if eig[0] <= 0 or eig[2] > eig[0] + eig[1] + 1e-12:
            raise ValueError("inertia must be SPD and physically realizable")


@dataclass(frozen=True)
class HumanSubjectProfile:
    """Bounded adult synthetic fixture domain; no empirical coverage implied.

    v2.0-a is an inertial migration stage, not a qualified full-body V2.
    Lengths scale with stature; densities need not stay fixed across profiles.
    """
    body_mass_kg: float = 75.0
    stature_m: float = 1.75
    version: str = "human-v2.0-a"
    population: str = "SYNTHETIC_ADULT_NOT_ELITE_CALIBRATED"
    parameter_status: str = "ASSUMED_UNCALIBRATED"
    uncertainty: str = "NOT_ESTIMATED_REQUIRES_SENSITIVITY"
    units: str = "kg,m,s,rad,N"
    dimensions: HumanDimensions | None = None
    joint_authority: tuple[JointAuthority, ...] = field(default_factory=default_joint_authority)
    segment_inertias: tuple[SegmentInertia, ...] = ()
    source_rights: str = "ORIGINAL_SYNTHETIC_PARAMETERS_NO_EXTERNAL_ASSETS"
    evidence: str = "NORMALIZED_RES610_CONTINUITY_PRIOR_NOT_DE_LEVA_TABLE"
    relative_parameter_uncertainty: float = .20

    def __post_init__(self):
        for key, lo, hi in (("body_mass_kg", 40, 180), ("stature_m", 1.4, 2.1)):
            v = getattr(self, key)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi:
                raise ValueError(f"{key} outside declared synthetic domain")
        if self.units != "kg,m,s,rad,N" or self.version != "human-v2.0-a":
            raise ValueError("unsupported profile units/version")
        if self.parameter_status != "ASSUMED_UNCALIBRATED":
            raise ValueError("measurements require separate provenance authority")
        if self.dimensions is None:
            reference = HumanDimensions()
            object.__setattr__(self, "dimensions", HumanDimensions(**{
                f.name: getattr(reference, f.name)*self.stature_m/1.75 for f in fields(reference)}))
        if not isinstance(self.dimensions, HumanDimensions):
            raise ValueError("typed SI dimensions required")
        if self.dimensions.thigh_m + self.dimensions.shank_m > .65 * self.stature_m:
            raise ValueError("inconsistent stature/leg proportions")
        if not isinstance(self.joint_authority, tuple) or any(not isinstance(j, JointAuthority) for j in self.joint_authority):
            raise ValueError("immutable typed joint authority required")
        if len({j.name for j in self.joint_authority}) != len(self.joint_authority):
            raise ValueError("duplicate joint authority")
        if not isinstance(self.segment_inertias, tuple) or any(not isinstance(s, SegmentInertia) for s in self.segment_inertias):
            raise ValueError("immutable typed segment authority required")
        if not .0 < self.relative_parameter_uncertainty <= .5:
            raise ValueError("uncertainty is a bounded sensitivity hypothesis, not an estimated confidence interval")
        if self.source_rights != "ORIGINAL_SYNTHETIC_PARAMETERS_NO_EXTERNAL_ASSETS":
            raise ValueError("external parameters require separate rights clearance")


def inertial_migration(profile: HumanSubjectProfile = HumanSubjectProfile()) -> str:
    """Executable inertial-only migration; V1 source and behavior stay frozen.

    Every segment keeps its V1 local COM and radius of gyration. Therefore
    I_new = I_old * m_new/m_old; fixed V1 inertias also scale by stature² and
    declared mass/75. No anatomical topology qualification is inherited.
    """
    spec = AthleteBarbellSpec(body_mass_kg=profile.body_mass_kg, stature_m=profile.stature_m)
    root = ET.fromstring(build_mjcf(spec))
    root.set("model", profile.version + "_INERTIAL_MIGRATION_ONLY")
    bodies = [b for b in root.iter("body") if b.get("name") != "barbell"]
    total = math.fsum(float(b.find("inertial").get("mass")) for b in bodies)
    factor = profile.body_mass_kg / total
    for body in bodies:
        node = body.find("inertial")
        name = body.get("name")
        mass = float(node.get("mass")) * factor
        scale = factor
        if name == "pelvis" or name.endswith(("_foot", "_hand", "_girdle")):
            scale *= profile.body_mass_kg / 75
            if name != "pelvis":
                scale *= spec.scale ** 2
        diag = [float(x) * scale for x in node.get("diaginertia").split()]
        SegmentInertia(name, mass, tuple(map(float, node.get("pos").split())),
                       tuple(tuple(v if a == b else 0.0 for b in range(3)) for a, v in enumerate(diag)))
        node.set("mass", repr(mass))
        node.set("diaginertia", " ".join(map(repr, diag)))
    if profile.segment_inertias:
        supplied = {v.name: v for v in profile.segment_inertias}
        if len(supplied) != len(profile.segment_inertias) or set(supplied) != {b.get("name") for b in bodies}:
            raise ValueError("segment registry must cover the complete human exactly once")
        if not math.isclose(math.fsum(v.mass_kg for v in supplied.values()), profile.body_mass_kg, rel_tol=0, abs_tol=1e-10):
            raise ValueError("segment registry does not conserve subject mass")
        for body in bodies:
            value, node = supplied[body.get("name")], body.find("inertial")
            node.attrib.clear()
            node.set("mass", repr(value.mass_kg))
            node.set("pos", " ".join(map(str, value.com_m)))
            i = value.tensor_kg_m2
            node.set("fullinertia", " ".join(map(str, (i[0][0], i[1][1], i[2][2], i[0][1], i[0][2], i[1][2]))))
    authorities = {j.name: j for j in profile.joint_authority}
    expected = {name for name, _ in joint_specs()}
    if set(authorities) != expected:
        raise ValueError("joint authority must cover the complete legacy articulation")
    for name, j in authorities.items():
        root.find(f".//joint[@name='{name}']").set("range", f"{j.lower_rad} {j.upper_rad}")
        a = root.find(f"./actuator/general[@name='{name}']")
        a.set("forcerange", f"{-j.capacity_nm} {j.capacity_nm}")
    return ET.tostring(root, encoding="unicode")
