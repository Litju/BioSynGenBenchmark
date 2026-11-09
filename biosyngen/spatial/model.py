"""Programmatic MJCF for the RES-610 spatial athlete-barbell plant.

Frame: world z up, x anterior, y left. In the neutral standing reference every
body frame is aligned with the world. Hinge axes are chosen so a positive
angle is the named anatomical motion on *both* sides (mirrored axes), hence a
symmetric posture has equal left/right values.

Parameter status: every segment mass, length, inertia, joint range and
actuator capacity below is ASSUMED_UNCALIBRATED, a manufactured 75 kg /
1.75 m fixture loosely proportioned after published segment tables. Joint
ranges reuse the planar model's declared limits where they overlap
(hip flexion 2.30, knee 2.50, ankle dorsiflexion 0.75 / plantarflexion 0.90,
elbow flexion 2.60, shoulder flexion 3.00 rad); none is loosened to admit a
pose. No third-party model or dataset is imported.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

SIDES = ("l", "r")
MIRROR = {"l": 1.0, "r": -1.0}


@dataclass(frozen=True)
class JointSpec:
    name: str
    axis: tuple[float, float, float]
    lower: float
    upper: float
    capacity_n_m: float        # symmetric actuator bound (ASSUMED_UNCALIBRATED)
    mirrored: bool = False     # axis sign flips on the right side


# Declared ranges (rad) and capacities (N m). Positive = named motion.
LUMBAR = (JointSpec("lumbar_flex", (0, 1, 0), -0.50, 0.90, 320),
          JointSpec("lumbar_lat", (1, 0, 0), -0.40, 0.40, 220),
          JointSpec("lumbar_rot", (0, 0, 1), -0.50, 0.50, 120))
LEG = (JointSpec("hip_flex", (0, -1, 0), -0.35, 2.30, 260),
       JointSpec("hip_abd", (1, 0, 0), -0.40, 0.80, 200, True),
       JointSpec("hip_rot", (0, 0, -1), -0.70, 0.70, 110, True),
       JointSpec("knee_flex", (0, 1, 0), 0.00, 2.50, 320),
       JointSpec("ankle_dorsi", (0, -1, 0), -0.90, 0.75, 220),
       JointSpec("subtalar_inv", (1, 0, 0), -0.35, 0.35, 70, True))
GIRDLE = (JointSpec("girdle_elev", (1, 0, 0), -0.20, 0.60, 90, True),
          JointSpec("girdle_prot", (0, 0, -1), -0.30, 0.40, 90, True))
ARM = (JointSpec("sh_flex", (0, -1, 0), -0.80, 3.00, 130),
       JointSpec("sh_abd", (1, 0, 0), -0.30, 3.00, 130, True),
       JointSpec("sh_rot", (0, 0, 1), -1.40, 1.40, 70, True),
       JointSpec("elbow_flex", (0, -1, 0), -0.05, 2.60, 100),
       JointSpec("forearm_sup", (0, 0, 1), -0.40, 2.40, 25, True),
       JointSpec("wrist_flex", (0, 1, 0), -1.30, 1.40, 45),
       JointSpec("wrist_dev", (1, 0, 0), -0.35, 0.60, 35, True))


# Declared hand-bar topologies. TWO_SITE: closed grip; two ball constraints
# 0.07 m apart along the bar leave only hand roll about the bar free and can
# transmit a hand moment. SINGLE_SITE: open-hand support; one ball constraint
# per hand transmits force but no moment (front-rack finger support).
GRIP_MODELS = {"BILATERAL_TWO_SITE_CONNECT_V1": (("a", -.035), ("b", .035)),
               "BILATERAL_SINGLE_SITE_CONNECT_V1": (("c", 0.0),)}


def grip_tags(spec):
    return tuple(tag for tag, _ in GRIP_MODELS[spec.grip_model])


@dataclass(frozen=True)
class AthleteBarbellSpec:
    """Declared manufactured context; every field is a model hypothesis."""

    body_mass_kg: float = 75.0
    stature_m: float = 1.75
    bar_mass_kg: float = 20.0
    plate_mass_kg: float = 20.0          # per side
    grip_half_width_m: float = 0.25      # hand centre to bar centre along y
    stance_half_width_m: float = 0.15
    foot_friction: float = 0.8
    timestep_s: float = 0.001
    grip_model: str = "BILATERAL_TWO_SITE_CONNECT_V1"
    # Distance from the wrist centre to the bar line in the hand (m at 1.75 m):
    # 0.085 = closed mid-palm hook grip; 0.13 = bar on the proximal phalanges
    # (open-hand front-rack support). The site is fixed in the hand: bar
    # relocation within the hand during a lift is NOT modelled.
    grip_site_distal_m: float = 0.085
    name: str = "res610_athlete_barbell"
    extra: dict = field(default_factory=dict)

    def __post_init__(self):
        for key in ("body_mass_kg", "stature_m", "bar_mass_kg", "grip_half_width_m",
                    "stance_half_width_m", "foot_friction", "timestep_s"):
            value = getattr(self, key)
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be positive and finite")
        if self.plate_mass_kg < 0 or not math.isfinite(self.plate_mass_kg):
            raise ValueError("plate mass must be nonnegative and finite")
        if self.grip_model not in GRIP_MODELS:
            raise ValueError(f"undeclared grip model {self.grip_model!r}")
        if not .06 <= self.grip_site_distal_m <= .16:
            raise ValueError("grip site must lie on the palm or proximal fingers (0.06-0.16 m)")
        if not .12 < self.grip_half_width_m < .60:
            raise ValueError("grip half-width outside the modelled 0.12-0.60 m shaft region")

    @property
    def scale(self) -> float:
        return self.stature_m / 1.75

    @property
    def total_load_kg(self) -> float:
        return self.bar_mass_kg + 2 * self.plate_mass_kg


# Fractions of body mass (sum 1.0) and lengths at 1.75 m (scaled linearly).
MASS_FRACTION = dict(pelvis=.112, torso=.380, girdle=.0067, upper_arm=.0271, forearm=.0162,
                     hand=.0061, thigh=.1416, shank=.0433, foot=.0137)
LENGTH = dict(lumbar=.10, shoulder_z=.38, girdle_y=.17, upper_arm=.30, forearm=.26, hand=.19,
              thigh=.43, shank=.43, ankle_z=.08, hip_y=.09, head_z=.66)


# Bounded joint servo (declared control hypothesis): torque =
# clip(kp (u - q) - kd qdot, +-capacity) with u = q_ref + tau_ff / kp set by the
# controller. Damping is integrated implicitly by MuJoCo's implicitfast scheme.
KP_PER_CAPACITY = 10.0      # N m/rad per N m of capacity
KD_PER_CAPACITY = 0.8       # N m s/rad per N m of capacity


def mass_table(spec: AthleteBarbellSpec) -> dict[str, float]:
    m = spec.body_mass_kg
    table = {"pelvis": MASS_FRACTION["pelvis"] * m, "torso": MASS_FRACTION["torso"] * m}
    for side in SIDES:
        for seg in ("girdle", "upper_arm", "forearm", "hand", "thigh", "shank", "foot"):
            table[f"{side}_{seg}"] = MASS_FRACTION[seg] * m
    return table


def _cyl(mass, radius, length):
    """Solid cylinder about its own COM, long axis z: (Ixx, Iyy, Izz)."""
    transverse = mass * (3 * radius ** 2 + length ** 2) / 12
    return (transverse, transverse, .5 * mass * radius ** 2)


def bar_inertia(spec: AthleteBarbellSpec):
    """Closed-form barbell inertia about its COM in the bar frame (axis y).

    Shaft: 2.2 m uniform rod of radius 0.014 m carrying bar_mass_kg.
    Plates: discs of radius 0.225 m, thickness 0.06 m, centred at y = +-0.75 m.
    """
    m, L, r = spec.bar_mass_kg, 2.2, .014
    ixx = izz = m * (3 * r ** 2 + L ** 2) / 12
    iyy = .5 * m * r ** 2
    p, R, t, d = spec.plate_mass_kg, .225, .06, .75
    if p > 0:
        ixx += 2 * (p * (3 * R ** 2 + t ** 2) / 12 + p * d ** 2)
        izz += 2 * (p * (3 * R ** 2 + t ** 2) / 12 + p * d ** 2)
        iyy += 2 * (.5 * p * R ** 2)
    return ixx, iyy, izz


def _fmt(values):
    return " ".join(f"{v:.9g}" for v in values)


def _joints(specs, side=None):
    out = []
    for j in specs:
        sign = MIRROR[side] if (side and j.mirrored) else 1.0
        axis = tuple(sign * a for a in j.axis)
        name = f"{side}_{j.name}" if side else j.name
        out.append(f'<joint name="{name}" type="hinge" axis="{_fmt(axis)}" range="{j.lower:.6g} {j.upper:.6g}" '
                   f'limited="true" damping="1.0" armature="0.01"/>')
    return "\n".join(out)


def joint_specs():
    """Ordered (name, JointSpec) for every actuated hinge."""
    out = [(j.name, j) for j in LUMBAR]
    for side in SIDES:
        out += [(f"{side}_{j.name}", j) for j in LEG + GIRDLE + ARM]
    return out


def build_mjcf(spec: AthleteBarbellSpec = AthleteBarbellSpec()) -> str:
    s = spec.scale
    m = mass_table(spec)
    L = {k: v * s for k, v in LENGTH.items()}
    hip_height = L["ankle_z"] + L["shank"] + L["thigh"]
    parts = []

    def inertial(mass, pos, diag):
        return f'<inertial pos="{_fmt(pos)}" mass="{mass:.9g}" diaginertia="{_fmt(diag)}"/>'

    def leg(side):
        y = MIRROR[side]
        lj, kj, aj = LEG[:3], LEG[3:4], LEG[4:]
        sphere = 0.02 * s
        foot_z = -(L["ankle_z"] - sphere)
        feet = "\n".join(
            f'<geom name="{side}_foot_{tag}" type="sphere" size="{sphere:.6g}" pos="{_fmt((x * s, y * w * s, foot_z))}" '
            f'contype="1" conaffinity="1" friction="{spec.foot_friction} 0.005 0.0001" class="foot"/>'
            for tag, x, w in (("heel_m", -.06, -.035), ("heel_l", -.06, .035), ("toe_m", .18, -.04), ("toe_l", .18, .04)))
        return f'''
      <body name="{side}_thigh" pos="0 {y * L['hip_y']:.6g} 0">
        {inertial(m[side + '_thigh'], (0, 0, -.43 * L['thigh']), _cyl(m[side + '_thigh'], .07 * s, L['thigh']))}
        {_joints(lj, side)}
        <geom type="capsule" fromto="0 0 -0.05 0 0 {-L['thigh'] + .05:.6g}" size="{.07 * s:.6g}" class="segment"/>
        <body name="{side}_shank" pos="0 0 {-L['thigh']:.6g}">
          {inertial(m[side + '_shank'], (0, 0, -.43 * L['shank']), _cyl(m[side + '_shank'], .05 * s, L['shank']))}
          {_joints(kj, side)}
          <site name="{side}_knee" pos="0 0 0" size="0.01"/>
          <geom type="capsule" fromto="0 0 -0.04 0 0 {-L['shank'] + .06:.6g}" size="{.05 * s:.6g}" class="segment"/>
          <body name="{side}_foot" pos="0 0 {-L['shank']:.6g}">
            {inertial(m[side + '_foot'], (.06 * s, 0, -.04 * s), (.0012, .0040, .0045))}
            {_joints(aj, side)}
            <site name="{side}_ankle" pos="0 0 0" size="0.01"/>
            {feet}
          </body>
        </body>
      </body>'''

    def arm(side):
        y = MIRROR[side]
        hand_l = L["hand"]
        grip = "\n".join(
            f'<site name="{side}_grip_{tag}" pos="{_fmt((-.025 * s, dy * s, -spec.grip_site_distal_m * s))}" size="0.006"/>'
            for tag, dy in GRIP_MODELS[spec.grip_model])
        return f'''
        <body name="{side}_girdle" pos="0 {y * .02 * s:.6g} {L['shoulder_z']:.6g}">
          {inertial(m[side + '_girdle'], (0, y * .08 * s, 0), (.0008, .0002, .0008))}
          {_joints(GIRDLE, side)}
          <body name="{side}_upper_arm" pos="0 {y * L['girdle_y']:.6g} 0.02">
            {inertial(m[side + '_upper_arm'], (0, 0, -.44 * L['upper_arm']), _cyl(m[side + '_upper_arm'], .045 * s, L['upper_arm']))}
            {_joints(ARM[:3], side)}
            <site name="{side}_shoulder" pos="0 0 0" size="0.01"/>
            <geom type="capsule" fromto="0 0 -0.05 0 0 {-L['upper_arm'] + .04:.6g}" size="{.04 * s:.6g}" class="segment"/>
            <geom name="{side}_deltoid" type="sphere" size="{.045 * s:.6g}" pos="{_fmt((.035 * s, 0, -.04 * s))}" class="shelf"/>
            <body name="{side}_forearm" pos="0 0 {-L['upper_arm']:.6g}">
              {inertial(m[side + '_forearm'], (0, 0, -.42 * L['forearm']), _cyl(m[side + '_forearm'], .035 * s, L['forearm']))}
              {_joints(ARM[3:5], side)}
              <site name="{side}_elbow" pos="0 0 0" size="0.01"/>
              <geom type="capsule" fromto="0 0 -0.04 0 0 {-L['forearm'] + .03:.6g}" size="{.032 * s:.6g}" class="segment"/>
              <body name="{side}_hand" pos="0 0 {-L['forearm']:.6g}">
                {inertial(m[side + '_hand'], (0, 0, -.36 * hand_l), (.0012, .0010, .0003))}
                {_joints(ARM[5:], side)}
                <site name="{side}_wrist" pos="0 0 0" size="0.01"/>
                <geom type="box" size="{_fmt((.012 * s, .04 * s, .05 * s))}" pos="0 0 {-.055 * s:.6g}" class="hand"/>
                {grip}
              </body>
            </body>
          </body>
        </body>'''

    half = .655
    gy = spec.grip_half_width_m
    bar_sites = "\n".join(
        f'<site name="bar_{side}_grip_{tag}" pos="0 {MIRROR[side] * gy + dy * s:.6g} 0" size="0.006"/>'
        for side in SIDES for tag, dy in GRIP_MODELS[spec.grip_model])
    plates = ""
    if spec.plate_mass_kg > 0:
        plates = "\n".join(
            f'<geom name="plate_{side}" type="cylinder" size="0.225 0.03" pos="0 {MIRROR[side] * .75:.6g} 0" '
            f'euler="1.5707963267948966 0 0" contype="2" conaffinity="5" mass="0"/>' for side in SIDES)
    ixx, iyy, izz = bar_inertia(spec)
    torso_inertia = _cyl(m["torso"], .15 * s, .55 * s)
    xml = f'''<mujoco model="{spec.name}">
  <compiler angle="radian" inertiafromgeom="false" autolimits="true"/>
  <option timestep="{spec.timestep_s}" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic"
          impratio="10" solver="Newton" iterations="200" tolerance="1e-10">
    <flag energy="enable"/>
  </option>
  <default>
    <geom contype="0" conaffinity="0" condim="3" solref="0.005 1" solimp="0.95 0.99 0.001" mass="0"/>
    <default class="segment"><geom contype="4" conaffinity="2"/></default>
    <default class="shelf"><geom contype="4" conaffinity="2"/></default>
    <default class="hand"><geom contype="4" conaffinity="2"/></default>
    <default class="foot"><geom contype="1" conaffinity="1"/></default>
    <equality solref="0.004 1" solimp="0.98 0.995 0.0005"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" contype="1" conaffinity="5" friction="{spec.foot_friction} 0.005 0.0001"/>
    <body name="pelvis" pos="0 0 {hip_height:.6g}">
      <freejoint name="pelvis_free"/>
      {inertial(m['pelvis'], (0, 0, .04 * s), (.08 * s * s, .05 * s * s, .09 * s * s))}
      <geom type="capsule" fromto="0 -0.11 0.05 0 0.11 0.05" size="{.08 * s:.6g}" class="segment"/>
      <body name="torso" pos="0 0 {L['lumbar']:.6g}">
        {inertial(m['torso'], (0, 0, .27 * s), torso_inertia)}
        {_joints(LUMBAR)}
        <geom name="trunk" type="capsule" fromto="0 0 0.05 0 0 {L['shoulder_z'] - .05:.6g}" size="{.12 * s:.6g}" class="segment"/>
        <geom name="head" type="sphere" size="{.1 * s:.6g}" pos="0 0 {L['head_z'] - .1 * s:.6g}" class="segment"/>
        <site name="neck" pos="0 0 {L['shoulder_z'] + .06 * s:.6g}" size="0.01"/>
        {arm('l')}
        {arm('r')}
      </body>
      {leg('l')}
      {leg('r')}
    </body>
    <body name="barbell" pos="0.15 0 1.0">
      <freejoint name="bar_free"/>
      <inertial pos="0 0 0" mass="{spec.total_load_kg:.9g}" diaginertia="{_fmt((ixx, iyy, izz))}"/>
      <geom name="bar_shaft" type="capsule" fromto="0 {-half} 0 0 {half} 0" size="0.014" contype="2" conaffinity="4"/>
      <geom name="bar_sleeve_l" type="cylinder" fromto="0 {half} 0 0 1.1 0" size="0.025"/>
      <geom name="bar_sleeve_r" type="cylinder" fromto="0 {-half} 0 0 -1.1 0" size="0.025"/>
      {plates}
      {bar_sites}
    </body>
  </worldbody>
  <contact>
    <exclude body1="l_hand" body2="barbell"/>
    <exclude body1="r_hand" body2="barbell"/>
    <exclude body1="l_forearm" body2="barbell"/>
    <exclude body1="r_forearm" body2="barbell"/>
  </contact>
  <equality>
{chr(10).join(f'    <connect name="grip_{side}_{tag}" site1="{side}_grip_{tag}" site2="bar_{side}_grip_{tag}"/>' for side in SIDES for tag in grip_tags(spec))}
  </equality>
  <actuator>
{chr(10).join(f'    <general name="{name}" joint="{name}" gaintype="fixed" biastype="affine" gainprm="{KP_PER_CAPACITY * j.capacity_n_m:.9g}" biasprm="0 {-KP_PER_CAPACITY * j.capacity_n_m:.9g} {-KD_PER_CAPACITY * j.capacity_n_m:.9g}" ctrllimited="false" forcelimited="true" forcerange="{-j.capacity_n_m} {j.capacity_n_m}"/>' for name, j in joint_specs())}
  </actuator>
</mujoco>'''
    return xml
