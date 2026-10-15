"""Angle-aware cable acquisition and observation-only vertical reconstruction.

Accuracy limits below are assumed bounded instrument hypotheses, not empirical
calibration. The inclinometer and encoder share the LPT acquisition instants;
the angle error sequence is independent of the encoder quantization.
"""

from bisect import bisect_right
from dataclasses import dataclass
import math
import random

from .instruments import CableLptModel, attachment_world

PROFILE_ID = "BSMB-CABLE-INCLINATION-1.0"
RECONSTRUCTION_ID = "BSMB-ENCODER-COS-INCLINATION-1.0"


@dataclass(frozen=True)
class InclinationModel:
    error_bound_rad: float = 0.0005
    resolution_rad: float = 0.00001
    seed: int = 93

    def __post_init__(self):
        if not math.isfinite(self.error_bound_rad) or self.error_bound_rad < 0:
            raise ValueError("angle error bound must be finite and nonnegative")
        if not math.isfinite(self.resolution_rad) or self.resolution_rad <= 0:
            raise ValueError("angle resolution must be finite and positive")


@dataclass(frozen=True)
class CableAngleSample:
    index: int
    native_time_s: float
    encoder_length_m: float
    displacement_m: float
    inclination_rad: float
    slack: bool


@dataclass(frozen=True)
class CableAngleObservation:
    samples: tuple[CableAngleSample, ...]
    # Protected reference only. Exporter selects source columns explicitly.
    true_times_s: tuple[float, ...]


def observe_cable_angle(trajectory, cable: CableLptModel, angle=InclinationModel()):
    if (not math.isfinite(cable.clock.sample_period_s) or cable.clock.sample_period_s <= 0
            or not math.isfinite(cable.clock.rate_scale) or cable.clock.rate_scale <= 0
            or not math.isfinite(cable.encoder_resolution_m) or cable.encoder_resolution_m <= 0):
        raise ValueError("invalid encoder or clock configuration")
    times = trajectory.times()
    points = [attachment_world(trajectory, s, cable.attachment) for s in trajectory.states()]
    rng_clock, rng_angle = random.Random(cable.seed), random.Random(angle.seed)
    samples, sampled_times = [], []
    spool, previous_time, zero = None, None, None
    index = 0
    while True:
        t = cable.clock.true_simulation_time(index, rng_clock)
        if t > times[-1]:
            break
        t = max(times[0], t)
        if previous_time is not None and t <= previous_time:
            raise ValueError("NONMONOTONIC_PHYSICAL_ACQUISITION")
        k = max(0, min(len(times) - 2, bisect_right(times, t) - 1))
        w = (t - times[k]) / (times[k + 1] - times[k])
        point = tuple(a + w * (b - a) for a, b in zip(points[k], points[k + 1]))
        x, y, z = (p - a for p, a in zip(point, cable.anchor_m))
        length = math.hypot(x, y, z)
        if length == 0:
            raise ValueError("COINCIDENT_ANCHOR_ATTACHMENT")
        inclination = math.atan2(math.hypot(x, y), z)
        observed = inclination + rng_angle.uniform(-angle.error_bound_rad, angle.error_bound_rad)
        observed = round(observed / angle.resolution_rad) * angle.resolution_rad
        spool = length if spool is None else max(
            length, spool - cable.max_retraction_speed_m_s * (t - previous_time))
        encoder = math.floor(spool / cable.encoder_resolution_m) * cable.encoder_resolution_m
        zero = encoder if zero is None else zero
        samples.append(CableAngleSample(index, cable.clock.nominal_native_time(index),
                                        encoder, encoder - zero, observed,
                                        spool > length + 0.5 * cable.encoder_resolution_m))
        sampled_times.append(t)
        previous_time = t
        index += 1
    return CableAngleObservation(tuple(samples), tuple(sampled_times))


def _height_interval(length, theta, length_resolution, angle_error):
    # Floor encoder: true length is in [reading, reading + one count].
    # Cosine extrema must include interior multiples of pi, including theta=0.
    lo, hi = theta - angle_error, theta + angle_error
    values = [math.cos(lo), math.cos(hi)]
    values.extend(math.cos(k * math.pi) for k in range(math.ceil(lo / math.pi),
                                                        math.floor(hi / math.pi) + 1))
    products = [r * c for r in (length, length + length_resolution) for c in values]
    # Binary64 guard, separate from physical measurement uncertainty.
    guard = 32 * math.ulp(max(1.0, length + length_resolution))
    return min(products) - guard, max(products) + guard


def reconstruct_vertical(samples, *, length_resolution_m, angle_error_bound_rad,
                         angle_resolution_rad):
    """Use only observed absolute encoder length and inclination, never states.

    Returns attachment-height change and conservative interval endpoints.
    This is not bar COM or system COM, and never a raw observation.
    """
    if not samples:
        raise ValueError("no observations")
    parameters = (length_resolution_m, angle_error_bound_rad, angle_resolution_rad)
    if any(not math.isfinite(x) or x < 0 for x in parameters) or length_resolution_m == 0:
        raise ValueError("invalid uncertainty parameters")
    intervals, heights = [], []
    for sample in samples:
        if sample.slack:
            raise ValueError("SLACK_CABLE_VERTICAL_RECONSTRUCTION_REFUSED")
        if (not all(math.isfinite(x) for x in (sample.encoder_length_m, sample.inclination_rad))
                or sample.encoder_length_m < 0):
            raise ValueError("invalid source observation")
        intervals.append(_height_interval(sample.encoder_length_m, sample.inclination_rad,
                                          length_resolution_m,
                                          angle_error_bound_rad + angle_resolution_rad / 2))
        heights.append(sample.encoder_length_m * math.cos(sample.inclination_rad))
    lower0, upper0 = intervals[0]
    return tuple((height - heights[0], low - upper0, high - lower0)
                 for height, (low, high) in zip(heights, intervals))
