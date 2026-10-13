"""Latent movement events and phases derived from modeled states (RES-483).

All definitions are predicates on the simulated system-COM state, the ground
reaction and the contact state. None uses a scheduled time. The control
command time is reported separately as a LATENT_CONTROL quantity. It is a
modeling hypothesis about drive, not an observable movement event.

Phase predicates follow the force-time convention used in the CMJ literature
(unweighting, braking, propulsion), applied to exact model quantities:

* weight W = static support force (system mass times g);
* kinetic onset: first time |Fz - W| exceeds a numerical floor (1e-6 W);
* unweighting: onset until Fz first returns to W from below, which is the
  instant of peak downward COM velocity;
* braking: until the system COM vertical velocity returns to 0 (lowest point);
* propulsion: until takeoff, i.e. when the contact normal force reaches 0;
* flight: until a foot point reaches the ground. Landing is not modeled.

State-root events between recorded samples are localized by cubic Hermite
interpolation of the COM velocity, whose derivative (Fz - W)/M is known
exactly at every sample. Their time uncertainty is reported as the gap
between the Hermite root and the linear-interpolation root, plus
contact-event bisection tolerance where applicable.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .simulate import ContactMode, JumpTrajectory


NUMERICAL_ONSET_FRACTION_OF_WEIGHT = 1e-6


@dataclass(frozen=True, slots=True)
class LatentEvent:
    name: str
    time_s: float | None
    uncertainty_s: float | None
    definition: str
    evidence_class: str = "LATENT_PHYSICAL_MODEL_STATE"


@dataclass(frozen=True, slots=True)
class LatentPhase:
    name: str
    start_s: float | None
    end_s: float | None
    definition: str


@dataclass(frozen=True, slots=True)
class PhaseAnalysis:
    task: str
    weight_n: float
    system_mass_kg: float
    events: tuple[LatentEvent, ...]
    phases: tuple[LatentPhase, ...]
    countermovement_depth_m: float
    peak_downward_velocity_m_s: float
    takeoff_velocity_m_s: float | None
    flight_apex_rise_m: float | None
    refusals: tuple[str, ...]

    def event(self, name: str) -> LatentEvent:
        for e in self.events:
            if e.name == name:
                return e
        raise KeyError(name)

    def phase(self, name: str) -> LatentPhase:
        for p in self.phases:
            if p.name == name:
                return p
        raise KeyError(name)


def _hermite_root(t0, t1, v0, v1, a0, a1) -> tuple[float, float]:
    """Root of the cubic Hermite interpolant of v on [t0, t1]; also linear root."""
    h = t1 - t0
    lin = t0 + h * v0 / (v0 - v1) if v0 != v1 else t0
    lo, hi = 0.0, 1.0

    def value(s: float) -> float:
        h00 = 2 * s**3 - 3 * s**2 + 1
        h10 = s**3 - 2 * s**2 + s
        h01 = -2 * s**3 + 3 * s**2
        h11 = s**3 - s**2
        return h00 * v0 + h10 * h * a0 + h01 * v1 + h11 * h * a1

    f_lo = value(lo)
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        f_mid = value(mid)
        if (f_mid < 0.0) == (f_lo < 0.0):
            lo, f_lo = mid, f_mid
        else:
            hi = mid
    return t0 + 0.5 * (lo + hi) * h, lin


def analyze_phases(traj: JumpTrajectory, task: str = "CMJ") -> PhaseAnalysis:
    samples = traj.samples
    first = samples[0]
    weight = first.grf_z_n
    mass = weight / traj.scenario.body.gravity_m_s2
    floor = NUMERICAL_ONSET_FRACTION_OF_WEIGHT * weight
    takeoff = traj.event_time("TAKEOFF")
    contact_end = takeoff if takeoff is not None else samples[-1].time_s
    stance = [s for s in samples if s.time_s <= contact_end and s.mode is not ContactMode.FLIGHT]

    def accel(s) -> float:
        return (s.grf_z_n - weight) / mass if s.mode is not ContactMode.FLIGHT else -traj.scenario.body.gravity_m_s2

    refusals: list[str] = []
    command = next((e.time_s for e in traj.events if e.kind == "CONTROL_BREAKPOINT"), None)

    onset = None
    for prev, cur in zip(stance, stance[1:]):
        if abs(cur.grf_z_n - weight) > floor:
            onset = (prev.time_s, cur.time_s)
            break

    # Peak downward velocity (end of unweighting): Fz crosses W upward.
    v_min_t = v_min_unc = None
    rev_t = rev_unc = None
    v_series = [(s.time_s, s.system_com_velocity_m_s[1], accel(s), s) for s in stance]
    start_index = 0
    if onset is not None:
        start_index = next(i for i, x in enumerate(v_series) if x[0] >= onset[0])
    for i in range(start_index, len(v_series) - 1):
        t0, v0, a0, s0 = v_series[i]
        t1, v1, a1, s1 = v_series[i + 1]
        # A dip within the 1e-9 m/s roundoff floor is not a countermovement.
        if v_min_t is None and a0 < 0.0 <= a1 and v0 < -1e-9:
            # Root of Fz - W by linear interpolation of the exact sample forces.
            frac = -a0 / (a1 - a0)
            v_min_t = t0 + frac * (t1 - t0)
            v_min_unc = t1 - t0
        if v_min_t is not None and rev_t is None and v0 < 0.0 <= v1:
            root, lin = _hermite_root(t0, t1, v0, v1, a0, a1)
            rev_t, rev_unc = root, abs(root - lin)
            break

    vz_all = [s.system_com_velocity_m_s[1] for s in stance]
    z_all = [s.system_com_m[1] for s in stance]
    depth = first.system_com_m[1] - min(z_all) if z_all else 0.0
    peak_down = min(vz_all) if vz_all else 0.0

    takeoff_v = None
    apex = None
    if takeoff is not None:
        to_sample = traj.sample_at_or_after(takeoff)
        takeoff_v = to_sample.system_com_velocity_m_s[1]
        apex = takeoff_v * takeoff_v / (2.0 * traj.scenario.body.gravity_m_s2) if takeoff_v > 0 else 0.0
    else:
        refusals.append("NO_TAKEOFF: propulsion did not release contact; flight phases not evaluable")

    landing = None
    for e in traj.events:
        if e.kind in ("LANDING_CONTACT_NOT_MODELED", "TAKEOFF_NOT_SUSTAINED", "BODY_GROUND_CONTACT_NOT_MODELED"):
            landing = e.time_s
    if traj.termination == "BODY_GROUND_CONTACT_NOT_MODELED":
        refusals.append("BODY_GROUND_CONTACT_NOT_MODELED: a non-foot body point reached the ground; run ended there")
    if traj.termination == "TAKEOFF_NOT_SUSTAINED":
        refusals.append("TAKEOFF_NOT_SUSTAINED: foot re-contacted before the ballistic apex; flight phase refused")

    if task == "SJ":
        if peak_down < -1e-3:
            refusals.append(
                f"SJ_COUNTERMOVEMENT_PRESENT: peak downward COM velocity {peak_down:.4f} m/s before propulsion"
            )
        v_min_t = v_min_unc = None

    heel_off = traj.event_time("HEEL_OFF")
    bisect_tol = traj.scenario.settings.event_time_tolerance_s
    events = (
        LatentEvent("control_command", command, 0.0,
                    "first declared excitation change (control hypothesis)", "LATENT_CONTROL_HYPOTHESIS"),
        LatentEvent("kinetic_onset", onset[1] if onset else None,
                    (onset[1] - onset[0]) if onset else None,
                    f"first |Fz - W| > {NUMERICAL_ONSET_FRACTION_OF_WEIGHT:g} W"),
        LatentEvent("peak_downward_velocity", v_min_t, v_min_unc,
                    "Fz returns to W from below while COM descends (end of unweighting)"),
        LatentEvent("com_velocity_reversal", rev_t, rev_unc,
                    "system COM vertical velocity crosses 0 upward (lowest point)"),
        LatentEvent("heel_off", heel_off, bisect_tol if heel_off is not None else None,
                    "heel normal force reaches 0"),
        LatentEvent("takeoff", takeoff, bisect_tol if takeoff is not None else None,
                    "toe normal force reaches 0 (contact release)"),
        LatentEvent("touchdown", landing, bisect_tol if landing is not None else None,
                    "foot point reaches ground; landing dynamics not modeled"),
    )
    onset_t = onset[1] if onset else None
    if task == "SJ":
        phases = (
            LatentPhase("static_hold", 0.0, onset_t, "flexed static equilibrium"),
            LatentPhase("propulsion", onset_t, takeoff, "kinetic onset to contact release"),
            LatentPhase("flight", takeoff, landing, "no ground contact"),
        )
    else:
        phases = (
            LatentPhase("quiet_stance", 0.0, onset_t, "static equilibrium before kinetic onset"),
            LatentPhase("unweighting", onset_t, v_min_t, "Fz < W, COM accelerating downward"),
            LatentPhase("braking", v_min_t, rev_t, "Fz >= W while COM still descends"),
            LatentPhase("propulsion", rev_t, takeoff, "COM ascends while in contact"),
            LatentPhase("flight", takeoff, landing, "no ground contact; landing not modeled"),
        )
    return PhaseAnalysis(
        task=task,
        weight_n=weight,
        system_mass_kg=mass,
        events=events,
        phases=phases,
        countermovement_depth_m=depth,
        peak_downward_velocity_m_s=peak_down,
        takeoff_velocity_m_s=takeoff_v,
        flight_apex_rise_m=apex,
        refusals=tuple(refusals),
    )


def phase_label_at(analysis: PhaseAnalysis, time_s: float) -> str | None:
    for phase in analysis.phases:
        if phase.start_s is None:
            continue
        end = phase.end_s if phase.end_s is not None else math.inf
        if phase.start_s <= time_s < end:
            return phase.name
    return None
