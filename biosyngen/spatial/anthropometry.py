"""V2 inertial authority. SI units; manufactured priors, never measurements.

The normalized RES-610 proportions are a continuity hypothesis, not de Leva
table values. Literature context: doi:10.1016/0021-9290(95)00178-6.
"""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

import numpy as np

from .model import AthleteBarbellSpec, build_mjcf


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

    def __post_init__(self):
        for key, lo, hi in (("body_mass_kg", 40, 180), ("stature_m", 1.4, 2.1)):
            v = getattr(self, key)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi:
                raise ValueError(f"{key} outside declared synthetic domain")
        if self.units != "kg,m,s,rad,N" or self.version != "human-v2.0-a":
            raise ValueError("unsupported profile units/version")
        if self.parameter_status != "ASSUMED_UNCALIBRATED":
            raise ValueError("measurements require separate provenance authority")


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
    return ET.tostring(root, encoding="unicode")
