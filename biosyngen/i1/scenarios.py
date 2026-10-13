"""Declared reference scenarios of the I1 jump model (hypothetical adult)."""

from __future__ import annotations

import math

from .control import CountermovementStrategy, SquatJumpStrategy
from .params import BarParameters
from .simulate import JumpScenario, Posture


def _posture(ankle_deg: float, knee_deg: float, hip_deg: float) -> Posture:
    """Build absolute segment angles from extension-positive joint angles."""
    shank = math.radians(ankle_deg)
    thigh = shank + math.radians(180.0 - knee_deg)
    hat = thigh - math.radians(180.0 - hip_deg)
    return Posture(shank, thigh, hat)


# Quiet stance: ankle 86 deg, knee 174 deg, hip 176 deg (static COP 2.5 cm
# anterior of the ankle).
CMJ_POSTURE = Posture(math.radians(86.0), math.radians(92.0), math.radians(88.0))
# Squat-jump start: ankle 68 deg, knee 100 deg, hip 90 deg; static COP inside the foot.
SJ_POSTURE = _posture(68.0, 100.0, 90.0)


def reference_cmj() -> JumpScenario:
    return JumpScenario("reference_unloaded_cmj", CMJ_POSTURE, CountermovementStrategy())


# Best timing of the declared SJ search after the body-ground-contact check
# (runs/i1/res482/control-sj.json; the earlier solution rotated head-first
# into the floor: control-sj-before-body-contact-check.json).
SJ_STRATEGY = SquatJumpStrategy(push_time_s=0.3284, knee_delay_s=0.0362, ankle_delay_s=0.0941)


def reference_sj(strategy: SquatJumpStrategy | None = None) -> JumpScenario:
    return JumpScenario("reference_sj", SJ_POSTURE, strategy or SJ_STRATEGY)


# Best timing of the declared loaded search for a 20 kg gripped bar
# (runs/i1/res482/control-loaded_cmj-20kg-random-starts.json; release clamped to 0).
LOADED_STRATEGY = CountermovementStrategy(
    release_fraction=0.0, push_time_s=0.8066, knee_delay_s=-0.2109, ankle_delay_s=0.1184
)


def reference_loaded_cmj(strategy: CountermovementStrategy | None = None, bar_mass_kg: float = 20.0) -> JumpScenario:
    return JumpScenario(
        "reference_loaded_cmj",
        CMJ_POSTURE,
        strategy or LOADED_STRATEGY,
        bar=BarParameters(mass_kg=bar_mass_kg),
    )
