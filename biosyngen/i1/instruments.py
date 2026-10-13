"""Staged forward models of a force platform and a cable LPT (RES-484).

Both chains consume only the latent, modeled quantities they would physically
transduce:

* the ground wrench from the modeled foot contacts (plate);
* the 3-D position of a declared attachment point (LPT).

They emit device-like exports on their own native clocks. Every stage
parameter is ASSUMED_UNCALIBRATED: these are SIMULATED_DEVICE_HYPOTHESIS
outputs, not ForceDecks/AMTI/GymAware/WinLaborat equivalents.

Force plate stages
  1. latent ground reaction on the subject (lab frame, +z up) and the moment
     about the plate origin (top-surface centre minus a declared origin depth);
  2. transducer: gain and cross-talk matrix plus zero offset;
  3. 2nd-order Butterworth anti-alias filter, simulated on a 10 kHz analog
     grid (bilinear transform with pre-warping);
  4. additive electronic noise (seeded white Gaussian);
  5. 16-bit ADC with range saturation;
  6. sampling on the device clock: offset, drift (ppm) and timing jitter. The
     exported timestamps are the nominal ones, so jitter is unobservable;
  7. export via the NOMINAL calibration (unit gain, no cross-talk
     correction). COP is computed only when Fz exceeds the declared minimum
     load, and is missing otherwise.

LPT stages
  1. 3-D cable length from a fixed floor anchor to the attachment point;
  2. retracting reel with a maximum retraction speed, giving a slack flag
     when the attachment approaches faster than the reel can follow;
  3. encoder quantization of spool length;
  4. sampling on its own clock (offset, drift, jitter); export of displacement
     from the first sample;
  5. provider velocity = central difference of exported displacement. It is a
     derived quantity, not an independent measurement.
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass, field
import math
import random

from .simulate import JumpSample, JumpTrajectory


STAGE_STATUS = "ASSUMED_UNCALIBRATED"
OUTPUT_CLASS = "SIMULATED_DEVICE_HYPOTHESIS"


@dataclass(frozen=True, slots=True)
class DeviceClock:
    sample_period_s: float
    offset_s: float = 0.0
    drift_ppm: float = 0.0
    jitter_sd_s: float = 0.0

    @property
    def rate_scale(self) -> float:
        return 1.0 + self.drift_ppm * 1e-6

    def nominal_native_time(self, index: int) -> float:
        return self.offset_s + index * self.sample_period_s

    def true_simulation_time(self, index: int, rng: random.Random) -> float:
        jitter = rng.gauss(0.0, self.jitter_sd_s) if self.jitter_sd_s > 0.0 else 0.0
        return index * self.sample_period_s / self.rate_scale + jitter


@dataclass(frozen=True, slots=True)
class ForcePlateModel:
    clock: DeviceClock = field(default_factory=lambda: DeviceClock(0.001, 0.0021, 35.0, 2e-6))
    plate_center_x_m: float = 0.05
    origin_depth_m: float = 0.040
    gain: tuple[tuple[float, float, float], ...] = (
        (1.004, 0.003, 0.0),   # Fx row: gain, cross-talk from Fz, from My
        (0.002, 0.997, 0.0),   # Fz row
        (0.0, 0.0005, 1.002),  # My row (N m per N, -, -)
    )
    offset: tuple[float, float, float] = (0.4, -1.2, 0.05)
    noise_sd: tuple[float, float, float] = (0.8, 1.5, 0.15)
    filter_cutoff_hz: float = 250.0
    adc_range: tuple[float, float, float] = (2500.0, 10000.0, 1500.0)
    adc_bits: int = 16
    min_load_for_cop_n: float = 20.0
    seed: int = 1
    dropout_indices: frozenset[int] = frozenset()
    analog_rate_hz: float = 10_000.0
    support_coverage: str = "SINGLE_PLATE_BOTH_FEET_LUMPED"


@dataclass(frozen=True, slots=True)
class LptModel:
    clock: DeviceClock = field(default_factory=lambda: DeviceClock(0.002, -0.0137, -60.0, 5e-6))
    anchor_m: tuple[float, float, float] = (-0.15, 0.0, 0.0)
    attachment: str = "hip_belt"  # or "bar_collar"
    attachment_local_m: tuple[float, float] = (0.10, 0.12)  # on HAT: along axis, posterior
    attachment_lateral_y_m: float = 0.0
    encoder_resolution_m: float = 1.0e-4
    max_retraction_speed_m_s: float = 6.0
    seed: int = 2
    dropout_indices: frozenset[int] = frozenset()
    velocity_method: str = "central_difference_of_exported_displacement"


@dataclass(frozen=True, slots=True)
class ForceSample:
    index: int
    native_time_s: float
    fx_n: float | None
    fz_n: float | None
    my_n_m: float | None
    cop_x_m: float | None
    saturated: bool
    missing: bool


@dataclass(frozen=True, slots=True)
class LptSample:
    index: int
    native_time_s: float
    displacement_m: float | None
    provider_velocity_m_s: float | None
    slack: bool
    missing: bool


@dataclass(frozen=True, slots=True)
class ForceExport:
    model: ForcePlateModel
    samples: tuple[ForceSample, ...]
    stage_trace: dict
    metadata: dict


@dataclass(frozen=True, slots=True)
class LptExport:
    model: LptModel
    samples: tuple[LptSample, ...]
    stage_trace: dict
    metadata: dict


# Latent signal access ----------------------------------------------------------
class _Latent:
    """Piecewise-linear access to recorded latent channels (<= 1 ms spacing)."""

    def __init__(self, traj: JumpTrajectory, channels) -> None:
        self.times = [s.time_s for s in traj.samples]
        self.values = [channels(s) for s in traj.samples]
        self.t_end = self.times[-1]

    def at(self, t: float) -> tuple[float, ...]:
        if t <= self.times[0]:
            return self.values[0]
        if t >= self.t_end:
            return self.values[-1]
        i = bisect_right(self.times, t) - 1
        t0, t1 = self.times[i], self.times[i + 1]
        v0, v1 = self.values[i], self.values[i + 1]
        if t1 == t0:
            return v1
        w = (t - t0) / (t1 - t0)
        return tuple(a + w * (b - a) for a, b in zip(v0, v1))


def latent_plate_wrench(sample: JumpSample, model: ForcePlateModel) -> tuple[float, float, float]:
    """Ground reaction on the subject (Fx, Fz) and moment about the plate origin.

    The origin lies origin_depth_m below the plate surface centre. A contact
    force (Fx, Fz) applied at the COP (x_c, 0) gives the CCW (+y) moment
    M = (x_c - x_p) Fz - (0 - (-d)) Fx = (x_c - x_p) Fz - d Fx.
    """
    fx, fz = sample.grf_x_n, sample.grf_z_n
    if sample.cop_x_m is None:
        return fx, fz, 0.0
    return fx, fz, (sample.cop_x_m - model.plate_center_x_m) * fz - model.origin_depth_m * fx


def butterworth2_coefficients(cutoff_hz: float, rate_hz: float) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Bilinear-transform 2nd-order Butterworth low-pass with pre-warping."""
    k = math.tan(math.pi * cutoff_hz / rate_hz)
    q = 1.0 / math.sqrt(2.0)
    norm = 1.0 / (1.0 + k / q + k * k)
    b = (k * k * norm, 2.0 * k * k * norm, k * k * norm)
    a = (1.0, 2.0 * (k * k - 1.0) * norm, (1.0 - k / q + k * k) * norm)
    return b, a


def _filter_stream(xs: list[float], b, a, initial: float) -> list[float]:
    y1 = y2 = initial
    x1 = x2 = initial
    out = []
    for x in xs:
        y = b[0] * x + b[1] * x1 + b[2] * x2 - a[1] * y1 - a[2] * y2
        x2, x1 = x1, x
        y2, y1 = y1, y
        out.append(y)
    return out


def observe_force_plate(traj: JumpTrajectory, model: ForcePlateModel) -> ForceExport:
    latent = _Latent(traj, lambda s: latent_plate_wrench(s, model))
    dt = 1.0 / model.analog_rate_hz
    n_analog = int(math.floor(latent.t_end / dt)) + 1
    analog_t = [i * dt for i in range(n_analog)]
    ideal = [latent.at(t) for t in analog_t]
    transduced = [
        tuple(
            sum(model.gain[r][c] * w[c] for c in range(3)) + model.offset[r]
            for r in range(3)
        )
        for w in ideal
    ]
    b, a = butterworth2_coefficients(model.filter_cutoff_hz, model.analog_rate_hz)
    filtered_channels = [
        _filter_stream([w[c] for w in transduced], b, a, transduced[0][c]) for c in range(3)
    ]
    rng_time = random.Random(model.seed)
    rng_noise = random.Random(model.seed + 1)
    steps = [rng / (2 ** (model.adc_bits - 1)) for rng in model.adc_range]
    samples: list[ForceSample] = []
    index = 0
    while True:
        t_true = model.clock.true_simulation_time(index, rng_time)
        if t_true > latent.t_end:
            break
        t_true = max(0.0, t_true)
        pos = t_true / dt
        j = min(int(pos), n_analog - 2)
        w = pos - j
        values = []
        saturated = False
        for c in range(3):
            v = filtered_channels[c][j] + w * (filtered_channels[c][j + 1] - filtered_channels[c][j])
            v += rng_noise.gauss(0.0, model.noise_sd[c])
            limit = model.adc_range[c]
            if v > limit or v < -limit:
                saturated = True
                v = max(-limit, min(limit, v))
            counts = round(v / steps[c])
            values.append(counts * steps[c])
        native = model.clock.nominal_native_time(index)
        if index in model.dropout_indices:
            samples.append(ForceSample(index, native, None, None, None, None, False, True))
        else:
            fx, fz, my = values
            cop = None
            if fz > model.min_load_for_cop_n:
                cop = model.plate_center_x_m + (my + model.origin_depth_m * fx) / fz
            samples.append(ForceSample(index, native, fx, fz, my, cop, saturated, False))
        index += 1
    metadata = {
        "output_class": OUTPUT_CLASS,
        "stage_status": STAGE_STATUS,
        "quantities": "Fx, Fz ground reaction on subject (lab frame, +z up); My about plate origin (+CCW in x-z)",
        "units": {"fx": "N", "fz": "N", "my": "N m", "cop_x": "m"},
        "support_coverage": model.support_coverage,
        "bilateral_channels": "NOT_AVAILABLE: one plate under a lumped two-leg foot",
        "cop_rule": f"computed only when Fz > {model.min_load_for_cop_n} N; missing otherwise",
        "timestamps": "nominal native clock; jitter and drift are not exported",
        "calibration": "nominal unit gain applied; true gain/cross-talk/offset differ (calibration error retained)",
        "plate_center_x_m": model.plate_center_x_m,
        "origin_depth_m": model.origin_depth_m,
        "sample_period_s": model.clock.sample_period_s,
    }
    trace = {
        "filter": {"type": "butterworth2_bilinear_prewarped", "cutoff_hz": model.filter_cutoff_hz,
                   "analog_rate_hz": model.analog_rate_hz},
        "adc_step": steps,
        "analog_samples": n_analog,
    }
    return ForceExport(model, tuple(samples), trace, metadata)


def lpt_attachment_point(sample: JumpSample, traj: JumpTrajectory, model: LptModel) -> tuple[float, float, float]:
    """3-D attachment position (x, y, z). y is the declared lateral offset."""
    if model.attachment == "bar_collar":
        if sample.bar_m is None:
            raise ValueError("bar_collar attachment requires a modeled bar")
        x, z = sample.bar_m
    elif model.attachment == "hip_belt":
        q = sample.q
        p = traj.scenario.body
        hx = q[0] + p.shank_length_m * math.cos(q[3]) + p.thigh_length_m * math.cos(q[4])
        hz = q[1] + p.shank_length_m * math.sin(q[3]) + p.thigh_length_m * math.sin(q[4])
        a, r = model.attachment_local_m
        x = hx + a * math.cos(q[5]) - r * math.sin(q[5])
        z = hz + a * math.sin(q[5]) + r * math.cos(q[5])
    else:
        raise ValueError(f"unknown attachment {model.attachment!r}")
    return x, model.attachment_lateral_y_m, z


def cable_length(point: tuple[float, float, float], anchor: tuple[float, float, float]) -> float:
    return math.sqrt(sum((p - a) ** 2 for p, a in zip(point, anchor)))


def observe_lpt(traj: JumpTrajectory, model: LptModel) -> LptExport:
    def geometry(s: JumpSample) -> tuple[float, ...]:
        point = lpt_attachment_point(s, traj, model)
        return (cable_length(point, model.anchor_m),) + point

    latent = _Latent(traj, geometry)
    rng_time = random.Random(model.seed)
    index = 0
    spool = None
    last_t = 0.0
    raw: list[tuple[int, float, float, bool]] = []
    while True:
        t_true = model.clock.true_simulation_time(index, rng_time)
        if t_true > latent.t_end:
            break
        t_true = max(0.0, t_true)
        length = latent.at(t_true)[0]
        if spool is None:
            spool = length
        else:
            # Taut cable is pulled out freely; retraction is speed-limited.
            min_spool = spool - model.max_retraction_speed_m_s * max(0.0, t_true - last_t)
            spool = max(length, min_spool)
        slack = spool > length + 0.5 * model.encoder_resolution_m
        counts = math.floor(spool / model.encoder_resolution_m)
        raw.append((index, model.clock.nominal_native_time(index), counts * model.encoder_resolution_m, slack))
        last_t = t_true
        index += 1
    zero = raw[0][2]
    displacements = [r[2] - zero for r in raw]
    samples = []
    period = model.clock.sample_period_s
    for k, (idx, native, _, slack) in enumerate(raw):
        missing = idx in model.dropout_indices
        vel = None
        if 0 < k < len(raw) - 1 and not missing:
            if raw[k - 1][0] not in model.dropout_indices and raw[k + 1][0] not in model.dropout_indices:
                vel = (displacements[k + 1] - displacements[k - 1]) / (2.0 * period)
        samples.append(
            LptSample(idx, native, None if missing else displacements[k], vel, slack, missing)
        )
    metadata = {
        "output_class": OUTPUT_CLASS,
        "stage_status": STAGE_STATUS,
        "quantity": "cable displacement from first sample along the cable (not vertical displacement)",
        "units": {"displacement": "m", "provider_velocity": "m/s"},
        "attachment": model.attachment,
        "anchor_m": model.anchor_m,
        "velocity_stage": f"PROVIDER_DERIVED: {model.velocity_method}",
        "encoder_resolution_m": model.encoder_resolution_m,
        "slack_model": f"retraction limited to {model.max_retraction_speed_m_s} m/s",
        "identifiability": "scalar cable motion does not identify 3-D attachment motion or COM",
        "sample_period_s": period,
    }
    trace = {"zero_reference_m": zero, "slack_samples": sum(1 for r in raw if r[3])}
    return LptExport(model, tuple(samples), trace, metadata)
