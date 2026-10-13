"""Declared excitation strategies (control hypotheses, not inferred neural drive).

A strategy maps time and the current contact mode to six excitations in the
order ankle ext, ankle flex, knee ext, knee flex, hip ext, hip flex. The
strategy never sees forces, COM states or event labels. Only the contact
mode is visible, and it is used solely to relax all actuators once ground
contact is lost. Movement phases therefore emerge from the dynamics; they are
not scheduled.
"""

from __future__ import annotations

from dataclasses import dataclass


EXTENSORS = (0, 2, 4)
FLEXORS = (1, 3, 5)


@dataclass(frozen=True, slots=True)
class CountermovementStrategy:
    """Quiet stance -> extensor release -> maximal extension (open loop).

    command_time_s ends quiet stance. Extensor excitations then drop to
    release_fraction times their quiet-stance values. Gravity produces the
    countermovement. At push_time_s (hip) plus per-joint delays, all extensors
    are fully excited and flexors drop to the minimum excitation.
    """

    # Defaults: the best feasible timing found by penalized Nelder-Mead
    # maximizing takeoff velocity (runs/i1/res482/capacity_search.txt).
    command_time_s: float = 0.30
    release_fraction: float = 0.0
    push_time_s: float = 0.8324
    knee_delay_s: float = -0.1938
    ankle_delay_s: float = 0.0468
    relax_in_flight: bool = True
    name: str = "CMJ_RELEASE_THEN_MAXIMAL_EXTENSION"

    def __post_init__(self) -> None:
        if not 0.0 <= self.release_fraction <= 1.0:
            raise ValueError("release_fraction must lie in [0, 1]")
        if self.push_time_s <= self.command_time_s:
            raise ValueError("push must follow the countermovement command")

    def breakpoints(self) -> tuple[float, ...]:
        return tuple(
            sorted(
                {
                    self.command_time_s,
                    self.push_time_s,
                    self.push_time_s + self.knee_delay_s,
                    self.push_time_s + self.ankle_delay_s,
                }
            )
        )

    def excitations(
        self, time_s: float, airborne: bool, quiet: list[float], u_min: float
    ) -> list[float]:
        if airborne and self.relax_in_flight:
            return [u_min] * 6
        if time_s < self.command_time_s:
            return list(quiet)
        u = list(quiet)
        onsets = (
            self.push_time_s + self.ankle_delay_s,
            self.push_time_s + self.knee_delay_s,
            self.push_time_s,
        )
        for joint, ext in enumerate(EXTENSORS):
            if time_s >= onsets[joint]:
                u[ext] = 1.0
                u[ext + 1] = u_min
            else:
                u[ext] = max(u_min, self.release_fraction * quiet[ext])
        return u


@dataclass(frozen=True, slots=True)
class SquatJumpStrategy:
    """Static flexed hold -> maximal extension, with no release phase."""

    push_time_s: float = 0.30
    knee_delay_s: float = 0.0
    ankle_delay_s: float = 0.04
    relax_in_flight: bool = True
    name: str = "SJ_STATIC_HOLD_THEN_MAXIMAL_EXTENSION"

    def breakpoints(self) -> tuple[float, ...]:
        return tuple(
            sorted(
                {
                    self.push_time_s,
                    self.push_time_s + self.knee_delay_s,
                    self.push_time_s + self.ankle_delay_s,
                }
            )
        )

    def excitations(
        self, time_s: float, airborne: bool, quiet: list[float], u_min: float
    ) -> list[float]:
        if airborne and self.relax_in_flight:
            return [u_min] * 6
        u = list(quiet)
        onsets = (
            self.push_time_s + self.ankle_delay_s,
            self.push_time_s + self.knee_delay_s,
            self.push_time_s,
        )
        for joint, ext in enumerate(EXTENSORS):
            if time_s >= onsets[joint]:
                u[ext] = 1.0
                u[ext + 1] = u_min
        return u


@dataclass(frozen=True, slots=True)
class HoldStrategy:
    """Keep the static-equilibrium excitations (quiet-stance verification)."""

    name: str = "HOLD_STATIC_EQUILIBRIUM"

    def breakpoints(self) -> tuple[float, ...]:
        return ()

    def excitations(
        self, time_s: float, airborne: bool, quiet: list[float], u_min: float
    ) -> list[float]:
        return list(quiet)


@dataclass(frozen=True, slots=True)
class ConstantStrategy:
    """Fixed excitations, e.g. a passive (minimum) body for free-fall tests."""

    values: tuple[float, float, float, float, float, float]
    name: str = "CONSTANT_EXCITATION"

    def breakpoints(self) -> tuple[float, ...]:
        return ()

    def excitations(
        self, time_s: float, airborne: bool, quiet: list[float], u_min: float
    ) -> list[float]:
        return list(self.values)
