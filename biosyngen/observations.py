"""Separate simulated FORCE and cable-LPT forward observation models."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass, replace
from enum import Enum
import math
import random
import sys

from .mechanics import StateSample, Trajectory


MAX_SAMPLES_PER_SERIES = 100_000
SIMULATED_STAGE = "SIMULATED_OBSERVATION"


class AttachmentTarget(str, Enum):
    BODY_COM = "body_com"
    BAR_ATTACHMENT = "bar_attachment"


@dataclass(frozen=True, slots=True)
class ClockModel:
    """A sensor-local clock with native tick period, offset, and rate drift.

    Native time is offset + (1 + drift) * simulation time. The simulation-time
    sampling interval is therefore native tick period divided by the rate scale.
    """

    sample_period_s: float
    offset_s: float = 0.0
    drift_ppm: float = 0.0

    def __post_init__(self) -> None:
        _require_positive_finite("sample_period_s", self.sample_period_s)
        _require_finite("offset_s", self.offset_s)
        _require_finite("drift_ppm", self.drift_ppm)
        if self.rate_scale <= 0.0:
            raise ValueError("clock drift makes the clock rate nonpositive")

    @property
    def rate_scale(self) -> float:
        return 1.0 + self.drift_ppm * 1e-6

    def simulation_time(self, native_time_s: float) -> float:
        _require_finite("native_time_s", native_time_s)
        return (native_time_s - self.offset_s) / self.rate_scale


@dataclass(frozen=True, slots=True)
class ObservationMetadata:
    source_stage: str
    quantity: str
    units: str
    frame: str
    positive_direction: str
    support_coverage: str
    timebase: str = "sensor_native_clock_seconds"


@dataclass(frozen=True, slots=True)
class ForcePlateConfig:
    clock: ClockModel
    offset_n: float = 0.0
    noise_std_n: float = 0.0
    quantization_n: float = 0.0
    saturation_n: float | None = None
    seed: int = 0
    missing_indices: frozenset[int] = frozenset()
    two_foot_channels_declared: bool = False
    assumed_left_fraction: float | None = None

    def __post_init__(self) -> None:
        _require_finite("offset_n", self.offset_n)
        _require_nonnegative_finite("noise_std_n", self.noise_std_n)
        _require_nonnegative_finite("quantization_n", self.quantization_n)
        if self.saturation_n is not None:
            _require_positive_finite("saturation_n", self.saturation_n)
        if not isinstance(self.seed, int):
            raise ValueError("seed must be an integer")
        object.__setattr__(
            self, "missing_indices", _validated_missing_indices(self.missing_indices)
        )
        if self.assumed_left_fraction is not None:
            _require_finite("assumed_left_fraction", self.assumed_left_fraction)
            if not 0.0 <= self.assumed_left_fraction <= 1.0:
                raise ValueError("assumed_left_fraction must be between 0 and 1")
            if not self.two_foot_channels_declared:
                raise ValueError("bilateral split requires two declared foot-contact channels")


@dataclass(frozen=True, slots=True)
class ForcePlateSample:
    sample_index: int
    native_time_s: float
    total_vertical_force_n: float | None
    left_vertical_force_n: float | None
    right_vertical_force_n: float | None
    missing: bool
    saturated: bool


@dataclass(frozen=True, slots=True)
class ForcePlateSeries:
    clock: ClockModel
    duration_s: float
    metadata: ObservationMetadata
    samples: tuple[ForcePlateSample, ...]


@dataclass(frozen=True, slots=True)
class CableLptConfig:
    clock: ClockModel
    anchor_x_m: float
    anchor_z_m: float
    attachment_x_m: float
    attachment_target: AttachmentTarget
    length_zero_reference_m: float = 0.0
    length_offset_m: float = 0.0
    length_noise_std_m: float = 0.0
    length_quantization_m: float = 0.0
    length_saturation_m: float | None = None
    rate_offset_m_s: float = 0.0
    rate_noise_std_m_s: float = 0.0
    rate_quantization_m_s: float = 0.0
    rate_saturation_m_s: float | None = None
    angle_zero_rad: float = 0.0
    angle_offset_rad: float = 0.0
    angle_noise_std_rad: float = 0.0
    angle_quantization_rad: float = 0.0
    seed: int = 0
    missing_indices: frozenset[int] = frozenset()

    def __post_init__(self) -> None:
        if not isinstance(self.attachment_target, AttachmentTarget):
            raise ValueError("attachment_target must be explicit")
        for name in (
            "anchor_x_m",
            "anchor_z_m",
            "attachment_x_m",
            "length_zero_reference_m",
            "length_offset_m",
            "rate_offset_m_s",
            "angle_zero_rad",
            "angle_offset_rad",
        ):
            _require_finite(name, getattr(self, name))
        if self.length_zero_reference_m < 0.0:
            raise ValueError("length_zero_reference_m must be nonnegative")
        for name in (
            "length_noise_std_m",
            "length_quantization_m",
            "rate_noise_std_m_s",
            "rate_quantization_m_s",
            "angle_noise_std_rad",
            "angle_quantization_rad",
        ):
            _require_nonnegative_finite(name, getattr(self, name))
        for name in ("length_saturation_m", "rate_saturation_m_s"):
            value = getattr(self, name)
            if value is not None:
                _require_positive_finite(name, value)
        if not isinstance(self.seed, int):
            raise ValueError("seed must be an integer")
        object.__setattr__(
            self, "missing_indices", _validated_missing_indices(self.missing_indices)
        )


@dataclass(frozen=True, slots=True)
class CableLptSample:
    sample_index: int
    native_time_s: float
    cable_length_m: float | None
    cable_rate_m_s: float | None
    cable_angle_rad: float | None
    missing: bool
    saturated: bool


@dataclass(frozen=True, slots=True)
class CableLptSeries:
    clock: ClockModel
    duration_s: float
    metadata: ObservationMetadata
    samples: tuple[CableLptSample, ...]


@dataclass(frozen=True, slots=True)
class CableGeometry:
    length_m: float
    rate_m_s: float
    angle_rad: float


def partition_support_force(total_force_n: float, left_fraction: float) -> tuple[float, float]:
    """Return an explicitly assumed two-channel split before observation error."""
    _require_finite("total_force_n", total_force_n)
    _require_finite("left_fraction", left_fraction)
    if total_force_n < 0.0:
        raise ValueError("total_force_n must be nonnegative")
    if not 0.0 <= left_fraction <= 1.0:
        raise ValueError("left_fraction must be between 0 and 1")
    left_force_n = total_force_n * left_fraction
    return left_force_n, total_force_n - left_force_n


def cable_kinematics(
    anchor_xz_m: tuple[float, float],
    attachment_xz_m: tuple[float, float],
    attachment_velocity_xz_m_s: tuple[float, float],
) -> CableGeometry:
    """Compute fixed-anchor cable length, directional rate, and angle.

    Coordinates use +x horizontal and +z upward. Cable rate is the projection
    of attachment velocity onto the cable direction, not vertical velocity.
    """
    for name, vector in (
        ("anchor_xz_m", anchor_xz_m),
        ("attachment_xz_m", attachment_xz_m),
        ("attachment_velocity_xz_m_s", attachment_velocity_xz_m_s),
    ):
        if len(vector) != 2:
            raise ValueError(f"{name} must have two coordinates")
        for value in vector:
            _require_finite(name, value)
    dx = attachment_xz_m[0] - anchor_xz_m[0]
    dz = attachment_xz_m[1] - anchor_xz_m[1]
    length_m = math.hypot(dx, dz)
    if length_m <= 0.0:
        raise ValueError("anchor and attachment must not coincide")
    projected_rate_m_s = (
        dx * attachment_velocity_xz_m_s[0] + dz * attachment_velocity_xz_m_s[1]
    ) / length_m
    return CableGeometry(
        length_m=length_m,
        rate_m_s=projected_rate_m_s,
        angle_rad=math.atan2(dz, dx),
    )


def observe_force_plate(
    trajectory: Trajectory,
    config: ForcePlateConfig,
) -> ForcePlateSeries:
    """Generate a simulated total FORCE channel and optional assumed split."""
    duration_s = trajectory.duration_s
    schedule = _clock_schedule(config.clock, duration_s)
    _validate_missing_indices(config.missing_indices, len(schedule))
    sampler = _TrajectorySampler(trajectory)
    total_rng = random.Random(config.seed)
    left_rng = random.Random(config.seed + 1)
    right_rng = random.Random(config.seed + 2)
    samples: list[ForcePlateSample] = []

    for sample_index, (simulation_time_s, native_time_s) in enumerate(schedule):
        if sample_index in config.missing_indices:
            samples.append(
                ForcePlateSample(
                    sample_index,
                    native_time_s,
                    None,
                    None,
                    None,
                    missing=True,
                    saturated=False,
                )
            )
            continue

        latent_force_n = sampler.at(simulation_time_s).normal_force_n
        ideal_left_n: float | None = None
        ideal_right_n: float | None = None
        if config.assumed_left_fraction is not None:
            ideal_left_n, ideal_right_n = partition_support_force(
                latent_force_n, config.assumed_left_fraction
            )
        total_value, total_clipped = _observe_scalar(
            latent_force_n,
            offset=config.offset_n,
            noise_std=config.noise_std_n,
            quantization=config.quantization_n,
            saturation=config.saturation_n,
            rng=total_rng,
        )
        left_value: float | None = None
        right_value: float | None = None
        left_clipped = False
        right_clipped = False
        if ideal_left_n is not None and ideal_right_n is not None:
            left_value, left_clipped = _observe_scalar(
                ideal_left_n,
                offset=config.offset_n,
                noise_std=config.noise_std_n,
                quantization=config.quantization_n,
                saturation=config.saturation_n,
                rng=left_rng,
            )
            right_value, right_clipped = _observe_scalar(
                ideal_right_n,
                offset=config.offset_n,
                noise_std=config.noise_std_n,
                quantization=config.quantization_n,
                saturation=config.saturation_n,
                rng=right_rng,
            )
        samples.append(
            ForcePlateSample(
                sample_index,
                native_time_s,
                total_value,
                left_value,
                right_value,
                missing=False,
                saturated=total_clipped or left_clipped or right_clipped,
            )
        )

    coverage = (
        "two_virtual_foot_channels_with_declared_force_partition"
        if config.assumed_left_fraction is not None
        else "declared_system_total_support_only"
    )
    return ForcePlateSeries(
        clock=config.clock,
        duration_s=duration_s,
        metadata=ObservationMetadata(
            source_stage=SIMULATED_STAGE,
            quantity="vertical_ground_reaction_force",
            units="N",
            frame="inertial_lab_z",
            positive_direction="upward",
            support_coverage=coverage,
        ),
        samples=tuple(samples),
    )


def observe_cable_lpt(
    trajectory: Trajectory,
    config: CableLptConfig,
) -> CableLptSeries:
    """Generate a simulated geometric cable length/rate/angle series."""
    duration_s = trajectory.duration_s
    schedule = _clock_schedule(config.clock, duration_s)
    _validate_missing_indices(config.missing_indices, len(schedule))
    sampler = _TrajectorySampler(trajectory)
    length_rng = random.Random(config.seed)
    rate_rng = random.Random(config.seed + 1)
    angle_rng = random.Random(config.seed + 2)
    samples: list[CableLptSample] = []

    for sample_index, (simulation_time_s, native_time_s) in enumerate(schedule):
        if sample_index in config.missing_indices:
            samples.append(
                CableLptSample(
                    sample_index,
                    native_time_s,
                    None,
                    None,
                    None,
                    missing=True,
                    saturated=False,
                )
            )
            continue

        state = sampler.at(simulation_time_s)
        if config.attachment_target is AttachmentTarget.BODY_COM:
            attachment_z_m = state.body_com_height_m
            attachment_vz_m_s = state.body_com_velocity_m_s
        else:
            if state.bar_attachment_height_m is None or state.bar_attachment_velocity_m_s is None:
                raise ValueError("bar attachment target requires a declared bar/load")
            attachment_z_m = state.bar_attachment_height_m
            attachment_vz_m_s = state.bar_attachment_velocity_m_s
        geometry = cable_kinematics(
            (config.anchor_x_m, config.anchor_z_m),
            (config.attachment_x_m, attachment_z_m),
            (0.0, attachment_vz_m_s),
        )
        length, length_clipped = _observe_scalar(
            geometry.length_m - config.length_zero_reference_m,
            offset=config.length_offset_m,
            noise_std=config.length_noise_std_m,
            quantization=config.length_quantization_m,
            saturation=config.length_saturation_m,
            rng=length_rng,
        )
        rate, rate_clipped = _observe_scalar(
            geometry.rate_m_s,
            offset=config.rate_offset_m_s,
            noise_std=config.rate_noise_std_m_s,
            quantization=config.rate_quantization_m_s,
            saturation=config.rate_saturation_m_s,
            rng=rate_rng,
        )
        angle, angle_clipped = _observe_scalar(
            geometry.angle_rad - config.angle_zero_rad,
            offset=config.angle_offset_rad,
            noise_std=config.angle_noise_std_rad,
            quantization=config.angle_quantization_rad,
            saturation=None,
            rng=angle_rng,
        )
        samples.append(
            CableLptSample(
                sample_index,
                native_time_s,
                length,
                rate,
                angle,
                missing=False,
                saturated=length_clipped or rate_clipped or angle_clipped,
            )
        )

    return CableLptSeries(
        clock=config.clock,
        duration_s=duration_s,
        metadata=ObservationMetadata(
            source_stage=SIMULATED_STAGE,
            quantity="zeroed_cable_length_rate_and_angle",
            units="m;m/s;rad",
            frame="fixed_anchor_xz_plane",
            positive_direction="cable_outward;increasing_length;counterclockwise_angle",
            support_coverage="declared_attachment_target_only",
        ),
        samples=tuple(samples),
    )


def validate_observation_clock(
    series: ForcePlateSeries | CableLptSeries,
    *,
    expected_clock: ClockModel,
) -> None:
    """Refuse a series whose native timestamps conflict with its declared clock."""
    if series.clock != expected_clock:
        raise ValueError("observation timebase differs from the declared clock")
    if not series.samples:
        raise ValueError("observation series has no samples")
    previous_native_time: float | None = None
    for expected_index, sample in enumerate(series.samples):
        if sample.sample_index != expected_index:
            raise ValueError("native clock sample indices must be contiguous")
        expected_native = series.clock.offset_s + expected_index * series.clock.sample_period_s
        tolerance = 16.0 * sys.float_info.epsilon * max(1.0, abs(expected_native))
        if abs(sample.native_time_s - expected_native) > tolerance:
            raise ValueError("native timestamps do not match the declared sample clock")
        simulation_time = series.clock.simulation_time(sample.native_time_s)
        if simulation_time < -tolerance or simulation_time > series.duration_s + tolerance:
            raise ValueError("clock maps a sample outside the declared simulation interval")
        if previous_native_time is not None and sample.native_time_s <= previous_native_time:
            raise ValueError("native timestamps must be strictly increasing")
        previous_native_time = sample.native_time_s
        if sample.missing:
            values = _sample_values(sample)
            if any(value is not None for value in values):
                raise ValueError("missing observation must not carry a signal value")
        elif isinstance(sample, ForcePlateSample):
            if sample.total_vertical_force_n is None:
                raise ValueError("observed FORCE sample requires its total channel")
            if (sample.left_vertical_force_n is None) != (sample.right_vertical_force_n is None):
                raise ValueError("bilateral FORCE channels must be both present or both absent")
        elif any(value is None for value in _sample_values(sample)):
            raise ValueError("observed LPT sample requires length, rate, and angle")


def state_at_time(trajectory: Trajectory, time_s: float) -> StateSample:
    """Evaluate the piecewise exact latent trajectory at an off-grid time."""
    return _TrajectorySampler(trajectory).at(time_s)


class _TrajectorySampler:
    def __init__(self, trajectory: Trajectory) -> None:
        self._trajectory = trajectory
        self._samples = trajectory.samples
        self._times = tuple(sample.time_s for sample in self._samples)

    def at(self, time_s: float) -> StateSample:
        _require_finite("time_s", time_s)
        start_time = self._times[0]
        end_time = self._times[-1]
        tolerance = 16.0 * sys.float_info.epsilon * max(1.0, abs(end_time))
        if time_s < start_time - tolerance or time_s > end_time + tolerance:
            raise ValueError("requested time is outside the latent trajectory")
        bounded_time = min(max(time_s, start_time), end_time)
        left_index = bisect_right(self._times, bounded_time) - 1
        left = self._samples[left_index]
        elapsed_s = bounded_time - left.time_s
        if elapsed_s == 0.0:
            return left
        acceleration = left.body_com_acceleration_m_s2
        body_height = (
            left.body_com_height_m
            + left.body_com_velocity_m_s * elapsed_s
            + 0.5 * acceleration * elapsed_s**2
        )
        body_velocity = left.body_com_velocity_m_s + acceleration * elapsed_s
        bar_offset = (
            None
            if left.bar_attachment_height_m is None
            else left.bar_attachment_height_m - left.body_com_height_m
        )
        system_com_offset = left.system_com_height_m - left.body_com_height_m
        return replace(
            left,
            time_s=bounded_time,
            body_com_height_m=body_height,
            body_com_velocity_m_s=body_velocity,
            bar_attachment_height_m=(
                None if bar_offset is None else body_height + bar_offset
            ),
            bar_attachment_velocity_m_s=(
                None if bar_offset is None else body_velocity
            ),
            system_com_height_m=body_height + system_com_offset,
            system_com_velocity_m_s=body_velocity,
        )


def _clock_schedule(
    clock: ClockModel,
    duration_s: float,
) -> tuple[tuple[float, float], ...]:
    _require_nonnegative_finite("duration_s", duration_s)
    sample_ratio = duration_s * clock.rate_scale / clock.sample_period_s
    if not math.isfinite(sample_ratio):
        raise ValueError("clock schedule sample count is not finite")
    count_bound = math.floor(sample_ratio) + 1
    if count_bound > MAX_SAMPLES_PER_SERIES:
        raise ValueError(f"clock schedule exceeds {MAX_SAMPLES_PER_SERIES} samples")
    schedule: list[tuple[float, float]] = []
    for sample_index in range(count_bound):
        simulation_time_s = sample_index * clock.sample_period_s / clock.rate_scale
        if simulation_time_s > duration_s:
            break
        native_time_s = clock.offset_s + sample_index * clock.sample_period_s
        schedule.append((simulation_time_s, native_time_s))
    if not schedule:
        raise ValueError("clock schedule produced no samples")
    return tuple(schedule)


def _observe_scalar(
    value: float,
    *,
    offset: float,
    noise_std: float,
    quantization: float,
    saturation: float | None,
    rng: random.Random,
) -> tuple[float, bool]:
    observed = value + offset
    if noise_std > 0.0:
        observed += rng.gauss(0.0, noise_std)
    if quantization > 0.0:
        observed = round(observed / quantization) * quantization
    saturated = False
    if saturation is not None:
        clipped = min(saturation, max(-saturation, observed))
        saturated = clipped != observed
        observed = clipped
    return observed, saturated


def _sample_values(sample: ForcePlateSample | CableLptSample) -> tuple[float | None, ...]:
    if isinstance(sample, ForcePlateSample):
        return (
            sample.total_vertical_force_n,
            sample.left_vertical_force_n,
            sample.right_vertical_force_n,
        )
    return (sample.cable_length_m, sample.cable_rate_m_s, sample.cable_angle_rad)


def _validated_missing_indices(indices: frozenset[int]) -> frozenset[int]:
    validated = frozenset(indices)
    if any(not isinstance(index, int) or index < 0 for index in validated):
        raise ValueError("missing_indices must contain nonnegative integers")
    return validated


def _validate_missing_indices(indices: frozenset[int], sample_count: int) -> None:
    if any(index >= sample_count for index in indices):
        raise ValueError("missing_indices contains an index outside the sample schedule")


def _require_finite(name: str, value: float) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")


def _require_nonnegative_finite(name: str, value: float) -> None:
    _require_finite(name, value)
    if value < 0.0:
        raise ValueError(f"{name} must be nonnegative")


def _require_positive_finite(name: str, value: float) -> None:
    _require_finite(name, value)
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")
