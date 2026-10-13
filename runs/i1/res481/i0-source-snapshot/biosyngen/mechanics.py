"""Exact piecewise-constant 1D vertical mechanics for the I0 pilot.

Ground reaction is prescribed by each stance phase. This module does not infer
muscle forces, foot kinematics, biological control, or measured signals.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite


class PhaseMode(str, Enum):
    """Declared support mode for a time interval."""

    STANCE = "STANCE"
    FLIGHT = "FLIGHT"


@dataclass(frozen=True, slots=True)
class MassProperties:
    """Body and optional rigidly coupled bar/load mass in SI units."""

    body_mass_kg: float
    bar_mass_kg: float | None = None
    bar_offset_m: float = 0.0

    def __post_init__(self) -> None:
        _require_positive_finite("body_mass_kg", self.body_mass_kg)
        if self.bar_mass_kg is not None:
            _require_positive_finite("bar_mass_kg", self.bar_mass_kg)
            _require_finite("bar_offset_m", self.bar_offset_m)
        elif self.bar_offset_m != 0.0:
            raise ValueError("bar_offset_m requires an explicitly declared bar_mass_kg")

    @property
    def total_mass_kg(self) -> float:
        return self.body_mass_kg + (self.bar_mass_kg or 0.0)

    @property
    def has_bar(self) -> bool:
        return self.bar_mass_kg is not None


@dataclass(frozen=True, slots=True)
class Phase:
    """A phase with a constant prescribed total normal force.

    The phase force is an input for the declared world. It is not inferred from
    body motion or from a physiological controller.
    """

    duration_s: float
    mode: PhaseMode
    normal_force_n: float = 0.0

    def __post_init__(self) -> None:
        _require_positive_finite("duration_s", self.duration_s)
        _require_finite("normal_force_n", self.normal_force_n)
        if not isinstance(self.mode, PhaseMode):
            raise ValueError("mode must be a PhaseMode")
        if self.normal_force_n < 0.0:
            raise ValueError("normal_force_n must be nonnegative")
        if self.mode is PhaseMode.FLIGHT and self.normal_force_n != 0.0:
            raise ValueError("ideal flight requires zero ground reaction")


@dataclass(frozen=True, slots=True)
class Scenario:
    """Deterministic initial state and ordered phase schedule."""

    name: str
    masses: MassProperties
    phases: tuple[Phase, ...]
    initial_body_com_height_m: float
    initial_vertical_velocity_m_s: float = 0.0
    gravity_m_s2: float = 9.80665
    sample_interval_s: float = 0.01

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("scenario name must not be empty")
        _require_positive_finite("initial_body_com_height_m", self.initial_body_com_height_m)
        _require_finite("initial_vertical_velocity_m_s", self.initial_vertical_velocity_m_s)
        _require_positive_finite("gravity_m_s2", self.gravity_m_s2)
        _require_positive_finite("sample_interval_s", self.sample_interval_s)
        object.__setattr__(self, "phases", tuple(self.phases))
        if not self.phases:
            raise ValueError("scenario requires at least one phase")
        if self.phases[0].mode is not PhaseMode.STANCE:
            raise ValueError("the pilot starts in declared stance")
        previous_mode = self.phases[0].mode
        for phase in self.phases[1:]:
            if previous_mode is PhaseMode.FLIGHT and phase.mode is PhaseMode.STANCE:
                raise ValueError("flight-to-stance impact is unsupported in this pilot")
            previous_mode = phase.mode


@dataclass(frozen=True, slots=True)
class PhaseTransition:
    """A declared contact-mode transition at an exact schedule boundary."""

    time_s: float
    from_mode: PhaseMode
    to_mode: PhaseMode


@dataclass(frozen=True, slots=True)
class StateSample:
    """One right-continuous state sample in the declared inertial lab frame."""

    time_s: float
    mode: PhaseMode
    normal_force_n: float
    body_com_height_m: float
    body_com_velocity_m_s: float
    body_com_acceleration_m_s2: float
    bar_attachment_height_m: float | None
    bar_attachment_velocity_m_s: float | None
    system_com_height_m: float
    system_com_velocity_m_s: float
    system_com_acceleration_m_s2: float


@dataclass(frozen=True, slots=True)
class Trajectory:
    """Exact sampled state and declared mode-transition events."""

    scenario_name: str
    total_mass_kg: float
    samples: tuple[StateSample, ...]
    transitions: tuple[PhaseTransition, ...]

    @property
    def duration_s(self) -> float:
        return self.samples[-1].time_s - self.samples[0].time_s


def simulate(scenario: Scenario) -> Trajectory:
    """Integrate constant-force phases with exact constant-acceleration steps.

    Sample spacing is at most scenario.sample_interval_s. Every phase boundary
    is included exactly. Contact mode is right-continuous at a boundary;
    position and velocity remain continuous because this model has finite
    forces and no impact impulses.
    """

    samples: list[StateSample] = []
    transitions: list[PhaseTransition] = []
    current_time = 0.0
    body_height = scenario.initial_body_com_height_m
    body_velocity = scenario.initial_vertical_velocity_m_s
    previous_mode: PhaseMode | None = None

    for phase in scenario.phases:
        if previous_mode is not None and phase.mode is not previous_mode:
            transitions.append(
                PhaseTransition(
                    time_s=current_time,
                    from_mode=previous_mode,
                    to_mode=phase.mode,
                )
            )

        acceleration = phase.normal_force_n / scenario.masses.total_mass_kg - scenario.gravity_m_s2
        boundary_sample = _make_sample(
            time_s=current_time,
            mode=phase.mode,
            normal_force_n=phase.normal_force_n,
            acceleration_m_s2=acceleration,
            body_height_m=body_height,
            body_velocity_m_s=body_velocity,
            masses=scenario.masses,
        )
        if samples and samples[-1].time_s == current_time:
            samples[-1] = boundary_sample
        else:
            samples.append(boundary_sample)

        phase_end = current_time + phase.duration_s
        while current_time < phase_end:
            remaining_s = phase_end - current_time
            step_s = min(scenario.sample_interval_s, remaining_s)
            next_time = phase_end if step_s >= remaining_s else current_time + step_s
            actual_step_s = next_time - current_time
            if actual_step_s <= 0.0:
                raise ArithmeticError("phase duration is below time-coordinate precision")
            body_height += body_velocity * actual_step_s + 0.5 * acceleration * actual_step_s**2
            body_velocity += acceleration * actual_step_s
            current_time = next_time
            samples.append(
                _make_sample(
                    time_s=current_time,
                    mode=phase.mode,
                    normal_force_n=phase.normal_force_n,
                    acceleration_m_s2=acceleration,
                    body_height_m=body_height,
                    body_velocity_m_s=body_velocity,
                    masses=scenario.masses,
                )
            )
        previous_mode = phase.mode

    return Trajectory(
        scenario_name=scenario.name,
        total_mass_kg=scenario.masses.total_mass_kg,
        samples=tuple(samples),
        transitions=tuple(transitions),
    )


def _make_sample(
    *,
    time_s: float,
    mode: PhaseMode,
    normal_force_n: float,
    acceleration_m_s2: float,
    body_height_m: float,
    body_velocity_m_s: float,
    masses: MassProperties,
) -> StateSample:
    if masses.has_bar:
        assert masses.bar_mass_kg is not None
        bar_height = body_height_m + masses.bar_offset_m
        system_com_height = (
            masses.body_mass_kg * body_height_m + masses.bar_mass_kg * bar_height
        ) / masses.total_mass_kg
        bar_velocity = body_velocity_m_s
    else:
        bar_height = None
        bar_velocity = None
        system_com_height = body_height_m

    return StateSample(
        time_s=time_s,
        mode=mode,
        normal_force_n=normal_force_n,
        body_com_height_m=body_height_m,
        body_com_velocity_m_s=body_velocity_m_s,
        body_com_acceleration_m_s2=acceleration_m_s2,
        bar_attachment_height_m=bar_height,
        bar_attachment_velocity_m_s=bar_velocity,
        system_com_height_m=system_com_height,
        system_com_velocity_m_s=body_velocity_m_s,
        system_com_acceleration_m_s2=acceleration_m_s2,
    )


def _require_finite(name: str, value: float) -> None:
    if not isfinite(value):
        raise ValueError(f"{name} must be finite")


def _require_positive_finite(name: str, value: float) -> None:
    _require_finite(name, value)
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")
