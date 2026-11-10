"""Executable three-segment foot candidate, not a qualified receiving plant.

Mass is partitioned from the normalized legacy foot. Uniform boxes replace
legacy inertias explicitly; compound COM/inertia therefore need requalification.
All dimensions, split fractions, joint ranges and strength are uncalibrated.
"""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

from .anthropometry import HumanSubjectProfile, inertial_migration


@dataclass(frozen=True)
class ShoeSpec:
    heel_elevation_m: float = .02
    sole_thickness_m: float = .008
    friction: float = .8

    def __post_init__(self):
        for key, low, high in (("heel_elevation_m", 0, .04), ("sole_thickness_m", .002, .02),
                               ("friction", .05, 1.5)):
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"invalid {key}")


def build_foot_candidate(profile=HumanSubjectProfile(), shoe=ShoeSpec()):
    root = ET.fromstring(inertial_migration(profile))
    root.set("model", "human-v2.0-b_FOOT_CANDIDATE_UNQUALIFIED")
    s = profile.stature_m / 1.75
    pelvis = root.find(".//body[@name='pelvis']")
    pos = list(map(float, pelvis.get("pos").split()))
    pos[2] += shoe.sole_thickness_m + shoe.heel_elevation_m
    pelvis.set("pos", " ".join(map(repr, pos)))
    actuator = root.find("actuator")
    for side in ("l", "r"):
        foot = root.find(f".//body[@name='{side}_foot']")
        mass = float(foot.find("inertial").get("mass"))
        foot.remove(foot.find("inertial"))
        for geom in list(foot.findall("geom")):
            foot.remove(geom)
        fore = ET.SubElement(foot, "body", name=f"{side}_forefoot", pos=f"{.06*s} 0 0")
        toe = ET.SubElement(fore, "body", name=f"{side}_toes", pos=f"{.10*s} 0 0")
        for body, tag, fraction, length, center, bound, capacity in (
            (foot, "hindfoot", .55, .14, -.01, None, None),
            (fore, "forefoot", .30, .10, .05, (-.15, .15), 25),
            (toe, "mtp", .15, .08, .04, (-.35, 1.05), 30)):
            m = mass * fraction
            size = (length*s, .09*s, .04*s)
            inertia = [m * (size[(a+1)%3]**2 + size[(a+2)%3]**2) / 12 for a in range(3)]
            ET.SubElement(body, "inertial", mass=repr(m), pos=f"{center*s} 0 {-.04*s}",
                          diaginertia=" ".join(map(repr, inertia)))
            if bound:
                name = f"{side}_{tag}_flex"
                ET.SubElement(body, "joint", name=name, axis="0 -1 0", range=f"{bound[0]} {bound[1]}",
                              damping="1", armature=".001", limited="true")
                ET.SubElement(actuator, "motor", name=name, joint=name, ctrllimited="true",
                              ctrlrange=f"{-capacity} {capacity}", forcelimited="true",
                              forcerange=f"{-capacity} {capacity}")
            # Heel equipment changes support height without editing ankle limits.
            elevation = shoe.heel_elevation_m if body is foot else 0
            z = -.08*s - shoe.sole_thickness_m - elevation
            ET.SubElement(body, "geom", name=f"{side}_{tag}_sole", type="box",
                          pos=f"{center*s} 0 {z + .005*s}", size=f"{length*s/2} {.045*s} {.005*s}",
                          contype="1", conaffinity="1", friction=f"{shoe.friction} .005 .0001")
    # Equipment is massless in this candidate; this assumption is explicit.
    return ET.tostring(root, encoding="unicode")
