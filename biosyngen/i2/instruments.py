"""Simulated dual force plates and cable LPT for I2 exercise trajectories (RES-492).

The instruments read only latent quantities that a device would transduce:

* FORCE_PIPE_01: the foot-to-plate forces of contacts bound to each plate
  frame (FZ_LEFT, FZ_RIGHT, plus Fx and My per plate);
* LPT_PIPE_01: the 3-D position of a declared attachment point on a body.

Force stages per plate: the latent wrench is a zero-order hold of the
interval-average contact force, plus impacts as Dirac impulses (area = impulse,
placed in one 0.1 ms analog bin). It then passes through a gain/cross-talk
matrix with zero offset, a 2nd-order Butterworth anti-alias filter, additive
noise and an ADC with saturation. Sampling uses the shared plate clock
(offset, drift, jitter); exported timestamps are nominal. FZ_TOTAL is the
source-preserving sum of the exported FZ_LEFT and FZ_RIGHT. It is not the
net force on the athlete plus load when a load has other supports.

LPT stages: cable length from a floor anchor, a reel retraction-speed limit
(slack flag), encoder quantization and its own clock. Provider velocity is
the central difference of the exported displacement.

Every stage parameter is ASSUMED_UNCALIBRATED. Outputs are
SIMULATED_DEVICE_HYPOTHESIS, not ForceDecks/AMTI/GymAware equivalents. A
rigid-impact Dirac through an assumed filter makes landing peak force
NOT_EVALUABLE; only impulses are mechanically meaningful.
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass, field
import math
import random

from ..i1.instruments import DeviceClock, butterworth2_coefficients, cable_length
from .simulate import ExerciseTrajectory

STAGE_STATUS = "ASSUMED_UNCALIBRATED"
OUTPUT_CLASS = "SIMULATED_DEVICE_HYPOTHESIS"
LANDING_PEAK_FORCE_STATUS = "NOT_EVALUABLE_RIGID_IMPACT_THROUGH_ASSUMED_FILTER"


@dataclass(frozen=True, slots=True)
class PlateChannelModel:
    gain: tuple[tuple[float, float, float], ...] = (
        (1.004, 0.003, 0.0),
        (0.002, 0.997, 0.0),
        (0.0, 0.0005, 1.002),
    )
    offset: tuple[float, float, float] = (0.4, -1.2, 0.05)
    noise_sd: tuple[float, float, float] = (0.8, 1.5, 0.15)


@dataclass(frozen=True, slots=True)
class DualPlateModel:
    clock: DeviceClock = field(default_factory=lambda: DeviceClock(0.001, 0.0021, 35.0, 2e-6))
    left: PlateChannelModel = field(default_factory=PlateChannelModel)
    right: PlateChannelModel = field(default_factory=lambda: PlateChannelModel(
        gain=((0.998, -0.002, 0.0), (0.001, 1.003, 0.0), (0.0, -0.0004, 0.999)),
        offset=(-0.3, 0.9, -0.04), noise_sd=(0.8, 1.5, 0.15),
    ))
    origin_depth_m: float = 0.040
    filter_cutoff_hz: float = 250.0
    adc_range: tuple[float, float, float] = (2500.0, 5000.0, 1500.0)
    adc_bits: int = 16
    analog_rate_hz: float = 10_000.0
    seed: int = 11


@dataclass(frozen=True, slots=True)
class LptAttachment:
    body_id: str
    local_point_m: tuple[float, float]
    lateral_y_m: float = 0.0
    label: str = "UNDECLARED"


@dataclass(frozen=True, slots=True)
class CableLptModel:
    attachment: LptAttachment
    clock: DeviceClock = field(default_factory=lambda: DeviceClock(0.002, -0.0137, -60.0, 5e-6))
    anchor_m: tuple[float, float, float] = (-0.15, 0.30, 0.0)
    encoder_resolution_m: float = 1.0e-4
    max_retraction_speed_m_s: float = 6.0
    seed: int = 12


@dataclass(frozen=True, slots=True)
class PlateSample:
    index: int
    native_time_s: float
    fz_left_n: float
    fz_right_n: float
    fz_total_n: float
    fx_left_n: float
    fx_right_n: float
    my_left_n_m: float
    my_right_n_m: float
    saturated: bool


@dataclass(frozen=True, slots=True)
class LptSample:
    index: int
    native_time_s: float
    displacement_m: float
    provider_velocity_m_s: float | None
    slack: bool


@dataclass(frozen=True, slots=True)
class DualPlateExport:
    samples: tuple[PlateSample, ...]
    metadata: dict


@dataclass(frozen=True, slots=True)
class LptExport:
    samples: tuple[LptSample, ...]
    metadata: dict
    zero_reference_m: float


def _plate_wrenches(trajectory: ExerciseTrajectory, model: DualPlateModel):
    """Latent piecewise-constant (Fx, Fz, My) per plate, and Dirac impulses per plate."""
    system = trajectory.model.system
    by_plate: dict[str, tuple[str, ...]] = {}
    origin: dict[str, tuple[float, float]] = {}
    for spec in system.contacts:
        if spec.plate_frame is not None:
            side = spec.plate_frame.side.lower()
            by_plate[side] = by_plate.get(side, ()) + (spec.contact_id,)
            origin[side] = spec.plate_frame.origin_world_m
    intervals = []
    for frame in trajectory.frames:
        per_plate = {}
        for side, ids in by_plate.items():
            fx = fz = my = 0.0
            for contact_id in ids:
                sample = frame.contact_forces.get(contact_id)
                if sample is None:
                    continue
                cfx, cfz = sample.force_world_n
                px, pz = sample.point_world_m
                fx += cfx
                fz += cfz
                # CCW moment about the plate origin, which lies origin_depth below the surface.
                my += (px - origin[side][0]) * cfz - (pz - (origin[side][1] - model.origin_depth_m)) * cfx
            per_plate[side] = (fx, fz, my)
        intervals.append((frame.t_start_s, frame.t_end_s, per_plate))
    impulses = []
    for impact in trajectory.impacts:
        for side, ids in by_plate.items():
            if impact.contact_id in ids:
                jx, jz = impact.impulse_world_n_s
                px, pz = impact.point_world_m
                impulses.append((impact.time_s, side, (
                    jx, jz, (px - origin[side][0]) * jz - (pz - (origin[side][1] - model.origin_depth_m)) * jx,
                )))
    return intervals, impulses


def observe_dual_plates(trajectory: ExerciseTrajectory, model: DualPlateModel | None = None) -> DualPlateExport:
    model = model or DualPlateModel()
    intervals, impulses = _plate_wrenches(trajectory, model)
    t_end = intervals[-1][1]
    dt = 1.0 / model.analog_rate_hz
    n = int(math.floor(t_end / dt)) + 1
    starts = [iv[0] for iv in intervals]
    analog: dict[str, list[list[float]]] = {}
    for side, channel in (("left", model.left), ("right", model.right)):
        ideal = [[0.0] * n for _ in range(3)]
        for k in range(n):
            t = k * dt
            i = max(0, min(len(intervals) - 1, bisect_right(starts, t) - 1))
            w = intervals[i][2].get(side, (0.0, 0.0, 0.0))
            for c in range(3):
                ideal[c][k] = w[c]
        for time_s, plate, j in impulses:
            if plate != side:
                continue
            k = min(n - 1, int(time_s / dt))
            for c in range(3):
                ideal[c][k] += j[c] / dt
        transduced = [
            [sum(channel.gain[r][c] * ideal[c][k] for c in range(3)) + channel.offset[r] for k in range(n)]
            for r in range(3)
        ]
        b, a = butterworth2_coefficients(model.filter_cutoff_hz, model.analog_rate_hz)
        filtered = []
        for series in transduced:
            y1 = y2 = x1 = x2 = series[0]
            out = []
            for x in series:
                y = b[0] * x + b[1] * x1 + b[2] * x2 - a[1] * y1 - a[2] * y2
                x2, x1, y2, y1 = x1, x, y1, y
                out.append(y)
            filtered.append(out)
        analog[side] = filtered
    rng_time = random.Random(model.seed)
    rng_noise = random.Random(model.seed + 1)
    steps = [r / (2 ** (model.adc_bits - 1)) for r in model.adc_range]
    samples: list[PlateSample] = []
    index = 0
    while True:
        t_true = model.clock.true_simulation_time(index, rng_time)
        if t_true > t_end:
            break
        t_true = max(0.0, t_true)
        pos = t_true / dt
        k = min(int(pos), n - 2)
        w = pos - k
        saturated = False
        values = {}
        for side, channel in (("left", model.left), ("right", model.right)):
            vals = []
            for c in range(3):
                v = analog[side][c][k] + w * (analog[side][c][k + 1] - analog[side][c][k])
                v += rng_noise.gauss(0.0, channel.noise_sd[c])
                if abs(v) > model.adc_range[c]:
                    saturated = True
                    v = max(-model.adc_range[c], min(model.adc_range[c], v))
                vals.append(round(v / steps[c]) * steps[c])
            values[side] = vals
        samples.append(PlateSample(
            index, model.clock.nominal_native_time(index),
            values["left"][1], values["right"][1], values["left"][1] + values["right"][1],
            values["left"][0], values["right"][0], values["left"][2], values["right"][2], saturated,
        ))
        index += 1
    metadata = {
        "sensor_id": "FORCE_PIPE_01",
        "output_class": OUTPUT_CLASS,
        "stage_status": STAGE_STATUS,
        "channels": ("FZ_LEFT", "FZ_RIGHT"),
        "derived": {"FZ_TOTAL": "FZ_LEFT + FZ_RIGHT (source-preserving bilateral plate-contact sum)"},
        "fz_total_scope": "foot-to-plate contact only; not net force on athlete plus load",
        "landing_peak_force": LANDING_PEAK_FORCE_STATUS,
        "sample_period_s": model.clock.sample_period_s,
        "native_time_offset_s": model.clock.offset_s,
        "timestamps": "nominal native clock; drift and jitter not exported",
        "calibration": "nominal unit gain; true gain/cross-talk/offset differ",
    }
    return DualPlateExport(tuple(samples), metadata)


def attachment_world(trajectory: ExerciseTrajectory, state, attachment: LptAttachment) -> tuple[float, float, float]:
    x, z = trajectory.model.point_world(state, attachment.body_id, attachment.local_point_m)
    return x, attachment.lateral_y_m, z


def observe_lpt(trajectory: ExerciseTrajectory, model: CableLptModel) -> LptExport:
    times = trajectory.times()
    points = [attachment_world(trajectory, s, model.attachment) for s in trajectory.states()]
    lengths = [cable_length(p, model.anchor_m) for p in points]
    t_end = times[-1]
    rng_time = random.Random(model.seed)
    raw = []
    spool = None
    last_t = 0.0
    index = 0
    while True:
        t_true = model.clock.true_simulation_time(index, rng_time)
        if t_true > t_end:
            break
        t_true = max(0.0, t_true)
        i = max(0, min(len(times) - 2, bisect_right(times, t_true) - 1))
        span = times[i + 1] - times[i]
        w = 0.0 if span <= 0.0 else (t_true - times[i]) / span
        length = lengths[i] + w * (lengths[i + 1] - lengths[i])
        if spool is None:
            spool = length
        else:
            spool = max(length, spool - model.max_retraction_speed_m_s * max(0.0, t_true - last_t))
        slack = spool > length + 0.5 * model.encoder_resolution_m
        counts = math.floor(spool / model.encoder_resolution_m)
        raw.append((index, model.clock.nominal_native_time(index), counts * model.encoder_resolution_m, slack))
        last_t = t_true
        index += 1
    zero = raw[0][2]
    period = model.clock.sample_period_s
    samples = []
    for k, (idx, native, value, slack) in enumerate(raw):
        velocity = None
        if 0 < k < len(raw) - 1:
            velocity = (raw[k + 1][2] - raw[k - 1][2]) / (2.0 * period)
        samples.append(LptSample(idx, native, value - zero, velocity, slack))
    metadata = {
        "sensor_id": "LPT_PIPE_01",
        "output_class": OUTPUT_CLASS,
        "stage_status": STAGE_STATUS,
        "attachment": model.attachment.label,
        "anchor_m": model.anchor_m,
        "quantity": "cable displacement from first sample along the cable (not vertical displacement)",
        "velocity_stage": "PROVIDER_DERIVED: central difference of exported displacement",
        "identifiability": "scalar cable motion does not identify 3-D attachment motion or system COM",
        "sample_period_s": period,
        "native_time_offset_s": model.clock.offset_s,
    }
    return LptExport(tuple(samples), metadata, zero)


def default_lpt(trajectory: ExerciseTrajectory) -> CableLptModel:
    """Hip-belt attachment for unloaded/vest jumps; bar collar for a back-rack bar."""
    model = trajectory.model
    if model.load_body_id is not None and model.load_contact_id is not None:
        return CableLptModel(LptAttachment(model.load_body_id, (0.0, 0.0), 0.60, "BAR_COLLAR"))
    return CableLptModel(LptAttachment("hat", model.hat_points["hip_belt"], 0.0, "HIP_BELT"))
