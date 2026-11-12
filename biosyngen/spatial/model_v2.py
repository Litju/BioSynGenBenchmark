"""RES-618 torque-controlled anatomical plant; all parameters are hypotheses.

Reduced scapular rhythm: upward_rotation=.4*GH_abduction, tilt=.1*GH_flexion.
These manufactured couplings bound motion; they are not Seth's fitted
ellipsoidal model or measurements. All hand/bar support is physical contact.
"""
from dataclasses import dataclass, field
import xml.etree.ElementTree as ET

from .anthropometry import HumanSubjectProfile
from .feet_v2 import ShoeSpec, build_foot_candidate
from .model import AthleteBarbellSpec, bar_inertia

MODEL_ID = "HUMAN_V2_TORQUE_FULLBODY"
MODEL_VERSION = "2.1.0"
# RES-624 Gate B contact-mask revision; the preserved RES-618 interface version is unchanged.
CONTACT_POLICY = "RES624_B1_BAR_FLOOR_ELIGIBLE"


def _numbers(values):
    return " ".join(map(str, values))


def _box_inertial(body, mass, center, lengths):
    old = body.find("inertial")
    if old is not None:
        body.remove(old)
    x, y, z = lengths
    diag = (mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12)
    ET.SubElement(body, "inertial", mass=str(mass), pos=_numbers(center), diaginertia=_numbers(diag))


def _hinge(body, actuators, name, axis, lower, upper, capacity, *, active=True):
    ET.SubElement(body, "joint", name=name, axis=_numbers(axis), range=f"{lower} {upper}",
                  limited="true", damping=".2", armature=".001")
    if active:
        ET.SubElement(actuators, "motor", name=name, joint=name, ctrllimited="true",
                      ctrlrange=f"{-capacity} {capacity}", forcelimited="true", forcerange=f"{-capacity} {capacity}")


@dataclass(frozen=True)
class HumanV2Spec:
    profile: HumanSubjectProfile = field(default_factory=HumanSubjectProfile)
    shoe: ShoeSpec = field(default_factory=ShoeSpec)
    equipment: AthleteBarbellSpec = field(default_factory=lambda: AthleteBarbellSpec(plate_mass_kg=0))

    def __post_init__(self):
        if not isinstance(self.profile, HumanSubjectProfile) or not isinstance(self.shoe, ShoeSpec) or not isinstance(self.equipment, AthleteBarbellSpec):
            raise ValueError("typed profile, footwear and equipment specifications required")


def build_mjcf_v2(spec=HumanV2Spec()):
    p, dims = spec.profile, spec.profile.dimensions
    s = p.stature_m/1.75
    root = ET.fromstring(build_foot_candidate(p, spec.shoe))
    root.set("model", MODEL_ID + "_" + MODEL_VERSION)
    root.find("option").set("timestep", str(spec.equipment.timestep_s))
    actuator, equality = root.find("actuator"), root.find("equality")
    for item in list(equality):
        equality.remove(item)
    # Physical hands and forearms must collide with the shaft; no welded grip.
    for item in list(root.find("contact")):
        root.find("contact").remove(item)
    bodies = {b.get("name"): b for b in root.iter("body")}
    pelvis, lumbar = bodies["pelvis"], bodies["torso"]
    total_trunk = float(lumbar.find("inertial").get("mass"))
    lumbar.set("name", "lumbar")
    # Redistribute the existing trunk ledger: no added human mass.
    for geom in list(lumbar.findall("geom")):
        lumbar.remove(geom)
    for site in list(lumbar.findall("site")):
        lumbar.remove(site)
    for arm in list(lumbar.findall("body")):
        lumbar.remove(arm)
    h = dims.trunk_length_m
    _box_inertial(lumbar, total_trunk*.30, (0, 0, .1*h), (dims.trunk_depth_m, dims.trunk_width_m, .35*h))
    ET.SubElement(lumbar, "geom", name="lumbar_collision", type="ellipsoid",
                  pos=f"0 0 {.1*h}", size=_numbers((dims.trunk_depth_m/2, dims.trunk_width_m/2, .175*h)), **{"class":"segment"})
    thorax = ET.SubElement(lumbar, "body", name="thorax", pos=f"0 0 {.30*h}")
    _box_inertial(thorax, total_trunk*.50, (0, 0, .2*h), (dims.trunk_depth_m, dims.trunk_width_m, .5*h))
    for name, axis, lo, hi, cap in (("thoracic_flex", (0, 1, 0), -.25, .35, 140),
                                   ("thoracic_rot", (0, 0, 1), -.35, .35, 80)):
        _hinge(thorax, actuator, name, axis, lo, hi, cap)
    ET.SubElement(thorax, "geom", name="thorax_collision", type="ellipsoid", pos=f"0 0 {.2*h}",
                  size=_numbers((dims.trunk_depth_m/2, dims.trunk_width_m/2, .25*h)), **{"class":"segment"})
    neck = ET.SubElement(thorax, "body", name="cervical", pos=f"0 0 {.65*h}")
    _box_inertial(neck, total_trunk*.04, (0, 0, .025*s), (.07*s, .07*s, .05*s))
    _hinge(neck, actuator, "cervical_flex", (0, 1, 0), -.65, .65, 25)
    _hinge(neck, actuator, "cervical_rotation", (0, 0, 1), -1.1, 1.1, 15)
    ET.SubElement(neck, "geom", name="neck_collision", type="capsule", fromto=f"0 0 0 0 0 {.05*s}", size=str(.035*s), **{"class":"segment"})
    head = ET.SubElement(neck, "body", name="head", pos=f"0 0 {.05*s}")
    _box_inertial(head, total_trunk*.16, (0, 0, .09*s), (.16*s, .15*s, .20*s))
    _hinge(head, actuator, "head_lateral", (1, 0, 0), -.45, .45, 15)
    ET.SubElement(head, "geom", name="head_collision", type="ellipsoid", pos=f"0 0 {.09*s}",
                  size=_numbers((.08*s, .075*s, .10*s)), **{"class":"segment"})
    ET.SubElement(thorax, "site", name="neck", pos=f"0 0 {.65*h}")
    for side, sign in (("l", 1), ("r", -1)):
        clavicle = bodies[side+"_girdle"]
        clavicle.set("name", side+"_clavicle")
        clavicle.set("pos", f"0 {sign*.02*s} {.50*h}")
        thorax.append(clavicle)
        upper = bodies[side+"_upper_arm"]
        clavicle.remove(upper)
        mass = float(clavicle.find("inertial").get("mass"))
        clav_length = dims.shoulder_breadth_m/2-.05*s
        _box_inertial(clavicle, mass*.4, (0, sign*clav_length/2, 0), (.025*s, clav_length, .025*s))
        ET.SubElement(clavicle, "geom", name=side+"_clavicle_collision", type="capsule",
                      fromto=f"0 0 0 0 {sign*clav_length} 0", size=str(.012*s), **{"class":"segment"})
        scap = ET.SubElement(clavicle, "body", name=side+"_scapula", pos=f"0 {sign*clav_length} 0")
        _box_inertial(scap, mass*.6, (-.02*s, 0, -.02*s), (.025*s, .08*s, .10*s))
        _hinge(scap, actuator, side+"_scap_up", (sign, 0, 0), -.15, 1.25, 0, active=False)
        _hinge(scap, actuator, side+"_scap_tilt", (0, -1, 0), -.1, .31, 0, active=False)
        ET.SubElement(equality, "joint", name=side+"_scapular_rhythm_abd", joint1=side+"_scap_up",
                      joint2=side+"_sh_abd", polycoef="0 .4 0 0 0")
        ET.SubElement(equality, "joint", name=side+"_scapular_rhythm_flex", joint1=side+"_scap_tilt",
                      joint2=side+"_sh_flex", polycoef="0 .1 0 0 0")
        ET.SubElement(scap, "geom", name=side+"_scapula_collision", type="box", pos=f"{-.02*s} 0 {-.02*s}",
                      size=_numbers((.0125*s, .04*s, .05*s)), **{"class":"segment"})
        upper.set("pos", f"0 {sign*.03*s} 0")
        scap.append(upper)
        # Subject-specific limb geometry and inertial second moments.
        for segment, length, radius in (("upper_arm", dims.upper_arm_m, .04*s), ("forearm", dims.forearm_m, .032*s),
                                         ("thigh", dims.thigh_m, .07*s), ("shank", dims.shank_m, .05*s)):
            body = bodies[side+"_"+segment]
            mass = float(body.find("inertial").get("mass"))
            _box_inertial(body, mass, (0, 0, -.43*length), (2*radius, 2*radius, length))
            for geom in body.findall("geom"):
                if geom.get("type") == "capsule":
                    geom.set("fromto", f"0 0 {-radius} 0 0 {-length+radius}")
            child = body.find("body")
            if child is not None:
                child.set("pos", f"0 0 {-length}")
        palm = bodies[side+"_hand"]
        hand_mass = float(palm.find("inertial").get("mass"))
        _box_inertial(palm, hand_mass*.55, (0, 0, -.28*dims.hand_length_m),
                      (.024*s, dims.hand_width_m, .52*dims.hand_length_m))
        palm_geom = palm.find("geom")
        palm_geom.set("name", side+"_palm_collision")
        palm_geom.set("size", _numbers((.012*s, dims.hand_width_m/2, .26*dims.hand_length_m)))
        palm_geom.set("pos", f"0 0 {-.28*dims.hand_length_m}")
        finger = ET.SubElement(palm, "body", name=side+"_fingers_proximal", pos=f"{-.005*s} 0 {-.53*dims.hand_length_m}")
        distal = ET.SubElement(finger, "body", name=side+"_fingers_distal", pos=f"0 0 {-.24*dims.hand_length_m}")
        thumb = ET.SubElement(palm, "body", name=side+"_thumb", pos=f"{-.008*s} {sign*.045*s} {-.20*dims.hand_length_m}")
        for body, tag, frac, length, width, axis, lo, hi, cap in (
            (finger, "finger_flex", .20, .24*dims.hand_length_m, dims.hand_width_m*.85, (0, 1, 0), 0, 2.0, 12),
            (distal, "finger_distal", .10, .19*dims.hand_length_m, dims.hand_width_m*.80, (0, 1, 0), 0, 1.6, 8),
            (thumb, "thumb_opposition", .15, .29*dims.hand_length_m, .018*s, (-sign, 0, 0), -.2, 1.4, 8)):
            _box_inertial(body, hand_mass*frac, (0, 0, -length/2), (.016*s, width, length))
            _hinge(body, actuator, side+"_"+tag, axis, lo, hi, cap)
            ET.SubElement(body, "geom", name=side+"_"+tag+"_pad", type="box", pos=f"0 0 {-length/2}",
                          size=_numbers((.008*s, width/2, length/2)), friction="1 .005 .0001", **{"class":"hand"})
        # Readable primitives, separate from the authority of contact geometry.
        for body in (clavicle, scap, palm, finger, distal, thumb):
            for geom in body.findall("geom"):
                geom.set("rgba", ".45 .68 .82 1")
    z = dims.thigh_m + dims.shank_m + .08*s + spec.shoe.sole_thickness_m + spec.shoe.heel_elevation_m
    pelvis.set("pos", f"0 0 {z}")
    bar = bodies["barbell"]
    bar.set("pos", "1.5 0 .3")
    inertial = bar.find("inertial")
    inertial.set("mass", str(spec.equipment.total_load_kg))
    inertial.set("diaginertia", _numbers(bar_inertia(spec.equipment)))
    # Equipment plates have actual geometry only when present.
    if spec.equipment.plate_mass_kg == 0:
        for geom in list(bar.findall("geom")):
            if geom.get("name", "").startswith("plate_"):
                bar.remove(geom)
    # RES-624 Gate B: the inherited shaft (contype 2, conaffinity 4) and the
    # mask-less sleeves never collided with the floor (contype 1, conaffinity
    # 5): (2&5)|(1&4) == 0. A dropped or missed V2 bar must land. Bit 1 is
    # added to the shaft and sleeves only; hand/segment (4/2) and foot (1/1)
    # policies are unchanged, so human self-collision stays excluded and V1
    # (model.py) is untouched.
    for geom in bar.findall("geom"):
        if geom.get("name") in ("bar_shaft", "bar_sleeve_l", "bar_sleeve_r"):
            geom.set("contype", "2")
            geom.set("conaffinity", "5")
    for geom in root.findall(".//geom"):
        if geom.get("class") == "foot" or geom.get("name", "").endswith("_sole"):
            geom.set("rgba", ".2 .45 .8 1")
    return ET.tostring(root, encoding="unicode")
