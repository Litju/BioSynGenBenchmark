"""Latent jump events and phases derived from forces and states (RES-492).

Every definition below is a predicate on recorded latent quantities: the
summed foot-to-ground forces, the system COM state, and the foot contact
modes. None of them reads the controller. Control stage switches are reported
separately as LATENT_CONTROL_HYPOTHESIS. Forces are interval averages from
the velocity-level stepper, and impacts are kept as impulses.

* weight W = system mass x g (body plus any carried load);
* kinetic_onset: first interval with |Fz - W| > 1e-6 W;
* peak_downward_velocity: the first interval after onset where Fz >= W while
  the COM descends (end of unweighting);
* com_velocity_reversal: COM vertical velocity crosses 0 upward (lowest point);
* left/right_foot_release: both contact points of that foot open; takeoff is
  the later of the two;
* touchdown: the first foot contact after takeoff, from an impact or a
  touchdown; left/right_foot_contact are reported per foot;
* landing_lowest_point: COM vertical velocity crosses 0 upward after touchdown.

Event time uncertainty is the length of the bracketing interval.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..multibody import ContactMode
from .simulate import ExerciseTrajectory

ONSET_FRACTION = 1e-6


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


@dataclass(frozen=True, slots=True)
class JumpPhaseAnalysis:
    subtype: str
    weight_n: float
    events: dict[str, LatentEvent]
    phases: tuple[LatentPhase, ...]
    control_commands: tuple[tuple[float, str, str], ...]
    takeoff_velocity_m_s: float | None
    takeoff_impulse_n_s: float | None
    flight_time_s: float | None
    com_rise_after_takeoff_m: float | None
    countermovement_depth_m: float
    peak_downward_velocity_m_s: float
    refusals: tuple[str, ...]

    def time(self, name: str) -> float | None:
        return self.events[name].time_s


def foot_force_z(trajectory: ExerciseTrajectory, frame) -> float:
    model = trajectory.model
    return sum(
        frame.contact_forces[c].force_world_n[1]
        for side in model.contact_ids
        for c in model.contact_ids[side]
        if c in frame.contact_forces
    )


def _foot_open(trajectory: ExerciseTrajectory, state, side: str) -> bool:
    return all(state.contacts[c].mode is ContactMode.OPEN for c in trajectory.model.contact_ids[side])


def analyze_jump(trajectory: ExerciseTrajectory) -> JumpPhaseAnalysis:
    model = trajectory.model
    system = model.system
    mass = model.total_mass_kg()
    weight = mass * model.gravity_m_s2
    frames = trajectory.frames
    refusals: list[str] = []
    if not trajectory.termination.startswith("DURATION_REACHED"):
        refusals.append(f"RUN_TERMINATED: {trajectory.termination}")
    fz = [foot_force_z(trajectory, f) for f in frames]
    vz = [system.linear_momentum(f.state)[1] / mass for f in frames]
    z = [system.center_of_mass(f.state)[1] for f in frames]
    z0 = system.center_of_mass(trajectory.initial_state)[1]

    onset_i = next((i for i, value in enumerate(fz) if abs(value - weight) > ONSET_FRACTION * weight), None)

    # Per-foot release and takeoff.
    release: dict[str, float | None] = {"left": None, "right": None}
    release_i: dict[str, int | None] = {"left": None, "right": None}
    for i, f in enumerate(frames):
        for side in ("left", "right"):
            if release[side] is None and _foot_open(trajectory, f.state, side):
                if all(_foot_open(trajectory, g.state, side) for g in frames[i:i + 3]):
                    release[side], release_i[side] = f.t_end_s, i
        if all(r is not None for r in release.values()):
            break
    takeoff_i = max(release_i.values()) if all(r is not None for r in release_i.values()) else None
    takeoff = frames[takeoff_i].t_end_s if takeoff_i is not None else None

    # Touchdown: first impact or touchdown event on a foot after takeoff.
    foot_contacts = {c for side in model.contact_ids for c in model.contact_ids[side]}
    contact_time: dict[str, float | None] = {"left": None, "right": None}
    touchdown = None
    if takeoff is not None:
        for event in trajectory.events:
            if event.time_s <= takeoff or event.event_id not in ("CONTACT_IMPACT", "CONTACT_TOUCHDOWN"):
                continue
            hit = foot_contacts.intersection(event.contact_ids)
            if not hit:
                continue
            touchdown = event.time_s if touchdown is None else touchdown
            for side in ("left", "right"):
                if contact_time[side] is None and hit.intersection(model.contact_ids[side]):
                    contact_time[side] = event.time_s
    else:
        refusals.append("NO_TAKEOFF: no sustained release of both feet; flight phases not evaluable")

    peak_down_t = reversal_t = None
    stance_end = takeoff_i if takeoff_i is not None else len(frames) - 1
    if onset_i is not None:
        for i in range(onset_i, stance_end):
            if peak_down_t is None and fz[i] >= weight and vz[i] < -1e-9 and i > onset_i and fz[i - 1] < weight:
                peak_down_t = frames[i].t_start_s
            if vz[i] < 0.0 <= vz[i + 1] and vz[i] < -1e-6:
                w = -vz[i] / (vz[i + 1] - vz[i])
                reversal_t = frames[i].t_end_s + w * (frames[i + 1].t_end_s - frames[i].t_end_s)
                break
    landing_low = None
    if touchdown is not None:
        start = next(i for i, f in enumerate(frames) if f.t_end_s > touchdown)
        for i in range(start, len(frames) - 1):
            if vz[i] < 0.0 <= vz[i + 1]:
                w = -vz[i] / (vz[i + 1] - vz[i])
                landing_low = frames[i].t_end_s + w * (frames[i + 1].t_end_s - frames[i].t_end_s)
                break

    stance_vz = vz[: stance_end + 1]
    peak_down = min(stance_vz) if stance_vz else 0.0
    depth = z0 - min(z[: stance_end + 1]) if stance_end >= 0 else 0.0
    if trajectory.exercise_id == "SJ":
        if peak_down < -1e-3:
            refusals.append(f"SJ_COUNTERMOVEMENT_PRESENT: peak downward COM velocity {peak_down:.4f} m/s")
        peak_down_t = None
        reversal_t = None

    takeoff_v = takeoff_impulse = flight_time = rise = None
    if takeoff_i is not None:
        takeoff_v = vz[takeoff_i]
        start_i = onset_i if onset_i is not None else 0
        takeoff_impulse = sum(
            (fz[i] - weight) * (frames[i].t_end_s - frames[i].t_start_s) for i in range(start_i, takeoff_i + 1)
        ) + sum(
            impact.impulse_world_n_s[1] for impact in trajectory.impacts
            if impact.contact_id in foot_contacts and impact.time_s <= takeoff
        )
        later = [z[i] for i, f in enumerate(frames) if f.t_end_s >= takeoff and (touchdown is None or f.t_end_s <= touchdown)]
        rise = max(later) - z[takeoff_i] if later else None
        if touchdown is not None:
            flight_time = touchdown - takeoff

    dt = frames[0].t_end_s - frames[0].t_start_s if frames else 0.0

    def ev(name, t, definition, unc=None):
        return LatentEvent(name, t, (unc if unc is not None else dt) if t is not None else None, definition)

    events = {
        "kinetic_onset": ev("kinetic_onset", frames[onset_i].t_start_s if onset_i is not None else None,
                            "first interval with |Fz - W| > 1e-6 W"),
        "peak_downward_velocity": ev("peak_downward_velocity", peak_down_t,
                                     "Fz returns to W from below while the COM descends"),
        "com_velocity_reversal": ev("com_velocity_reversal", reversal_t, "COM vertical velocity crosses 0 upward"),
        "left_foot_release": ev("left_foot_release", release["left"], "both left contact points open"),
        "right_foot_release": ev("right_foot_release", release["right"], "both right contact points open"),
        "takeoff": ev("takeoff", takeoff, "later of the two foot releases"),
        "touchdown": ev("touchdown", touchdown, "first foot impact or touchdown after takeoff", 0.0),
        "left_foot_contact": ev("left_foot_contact", contact_time["left"], "first left foot contact after takeoff", 0.0),
        "right_foot_contact": ev("right_foot_contact", contact_time["right"], "first right foot contact after takeoff", 0.0),
        "landing_lowest_point": ev("landing_lowest_point", landing_low, "COM vertical velocity crosses 0 upward after touchdown"),
    }
    onset_t = events["kinetic_onset"].time_s
    end_t = frames[-1].t_end_s if frames else 0.0
    if trajectory.exercise_id == "SJ":
        phases = (
            LatentPhase("STATIC_HOLD", 0.0, onset_t),
            LatentPhase("PROPULSION", onset_t, takeoff),
        )
    else:
        phases = (
            LatentPhase("QUIET_STANCE", 0.0, onset_t),
            LatentPhase("UNWEIGHTING", onset_t, peak_down_t),
            LatentPhase("BRAKING", peak_down_t, reversal_t),
            LatentPhase("PROPULSION", reversal_t, takeoff),
        )
    phases += (
        LatentPhase("FLIGHT", takeoff, touchdown),
        LatentPhase("LANDING", touchdown, landing_low),
        LatentPhase("POST_LANDING", landing_low, end_t if landing_low is not None else None),
    )
    return JumpPhaseAnalysis(
        trajectory.exercise_id, weight, events, phases, tuple(trajectory.control_commands),
        takeoff_v, takeoff_impulse, flight_time, rise, depth, peak_down, tuple(refusals),
    )
