"""Sensor-only event detectors for jump FORCE and cable-LPT exports (RES-483).

These functions see only public device exports: native timestamps and values.
They do not import the generator and cannot see latent states or events.
Every threshold is a declared DETECTOR_CONVENTION (an analysis choice), not a
physiological boundary. Outputs are DETECTOR_ESTIMATED events. When the data
cannot support a detector, it returns a refusal reason instead of a number.

FORCE detectors (vertical force, +z up, N):

* Baseline: mean and SD over the declared quiet window, which must be complete
  and quiet (SD below a declared fraction of the mean).
* Onset: first sample with |F - BW| > k SD. The conventional backdated onset
  subtracts 30 ms.
* Takeoff: first post-peak sample with F below max(20 N, k SD of baseline).
  The time is linearly interpolated.
* COM velocity: trapezoidal forward integration of (F - BW)/m from the
  backdated onset, assuming v = 0 at quiet stance. This is a derived estimate,
  not a measurement.
* Peak downward velocity, velocity reversal and takeoff velocity come from
  that integration.
* Landing: not observable in these exports (simulation ends at touchdown).

LPT detectors (cable displacement, m): lowest point, onset, and velocity
reversal from the provider-derived velocity. A cable cannot observe contact,
so LPT takeoff is NOT_EVALUABLE.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


STANDARD_GRAVITY = 9.80665


@dataclass(frozen=True, slots=True)
class DetectorConvention:
    quiet_window_s: float = 0.25
    max_quiet_cv: float = 0.02
    onset_sd_multiple: float = 5.0
    onset_backdate_s: float = 0.030
    takeoff_floor_n: float = 20.0
    takeoff_sd_multiple: float = 5.0
    lpt_onset_m: float = 0.005


@dataclass(frozen=True, slots=True)
class DetectedEvent:
    name: str
    native_time_s: float | None
    value: float | None
    refusal: str | None
    evidence_class: str = "DETECTOR_ESTIMATED"


def _baseline(times, values, window_s):
    t0 = times[0]
    window = [(t, v) for t, v in zip(times, values) if t - t0 <= window_s]
    if any(v is None for _, v in window):
        return None, None, "MISSING_SAMPLE_IN_QUIET_WINDOW"
    if len(window) < 20:
        return None, None, "QUIET_WINDOW_TOO_SHORT"
    xs = [v for _, v in window]
    mean = sum(xs) / len(xs)
    sd = math.sqrt(sum((x - mean) ** 2 for x in xs) / (len(xs) - 1))
    return mean, sd, None


def detect_force_events(
    times: list[float], fz: list[float | None], convention: DetectorConvention = DetectorConvention()
) -> dict[str, DetectedEvent]:
    out: dict[str, DetectedEvent] = {}

    def refuse(names, reason):
        for name in names:
            out[name] = DetectedEvent(name, None, None, reason)

    names = (
        "body_weight", "onset_threshold", "onset_backdated", "peak_downward_velocity",
        "velocity_reversal", "takeoff", "takeoff_velocity", "landing",
    )
    if len(times) != len(fz) or len(times) < 3:
        refuse(names, "INVALID_SERIES")
        return out
    bw, sd, reason = _baseline(times, fz, convention.quiet_window_s)
    if reason is not None:
        refuse(names, reason)
        return out
    if bw <= 0.0 or sd / bw > convention.max_quiet_cv:
        refuse(names, "NO_QUIET_STANCE_BASELINE")
        return out
    sd = max(sd, 1e-9)
    out["body_weight"] = DetectedEvent("body_weight", None, bw, None)
    mass = bw / STANDARD_GRAVITY
    onset_i = None
    for i, (t, v) in enumerate(zip(times, fz)):
        if t - times[0] <= convention.quiet_window_s:
            continue
        if v is not None and abs(v - bw) > convention.onset_sd_multiple * sd:
            onset_i = i
            break
    if onset_i is None:
        refuse(names[1:], "NO_THRESHOLD_CROSSING")
        return out
    t_onset = times[onset_i]
    out["onset_threshold"] = DetectedEvent("onset_threshold", t_onset, None, None)
    t_back = t_onset - convention.onset_backdate_s
    out["onset_backdated"] = DetectedEvent("onset_backdated", t_back, None, None)

    peak_i = max(range(onset_i, len(fz)), key=lambda i: fz[i] if fz[i] is not None else -math.inf)
    threshold = max(convention.takeoff_floor_n, convention.takeoff_sd_multiple * sd)
    takeoff_i = None
    for i in range(peak_i, len(fz)):
        if fz[i] is not None and fz[i] < threshold:
            takeoff_i = i
            break
    t_takeoff = None
    if takeoff_i is None or takeoff_i == 0:
        out["takeoff"] = DetectedEvent("takeoff", None, None, "NO_TAKEOFF_THRESHOLD_CROSSING")
    else:
        a, b = fz[takeoff_i - 1], fz[takeoff_i]
        ta, tb = times[takeoff_i - 1], times[takeoff_i]
        if a is None:
            t_takeoff = tb
        else:
            t_takeoff = ta + (tb - ta) * (a - threshold) / (a - b) if a != b else tb
        out["takeoff"] = DetectedEvent("takeoff", t_takeoff, threshold, None)

    # Forward integration from the backdated onset (v = 0 assumed there).
    start = next(i for i, t in enumerate(times) if t >= t_back)
    end = takeoff_i if takeoff_i is not None else len(times) - 1
    segment = list(range(start, end + 1))
    if any(fz[i] is None for i in segment):
        refuse(("peak_downward_velocity", "velocity_reversal", "takeoff_velocity"), "MISSING_SAMPLE_IN_INTEGRATION_WINDOW")
    else:
        v = 0.0
        vel = [(times[start], 0.0)]
        for i, j in zip(segment, segment[1:]):
            v += 0.5 * ((fz[i] - bw) + (fz[j] - bw)) / mass * (times[j] - times[i])
            vel.append((times[j], v))
        t_vmin, v_min = min(vel, key=lambda p: p[1])
        # Significance floor: integrating white force noise of SD s over a
        # window T with sample spacing dt gives a velocity random walk with
        # SD (s/m) sqrt(dt T). A smaller dip is not a detectable
        # countermovement (e.g. a squat jump) and both events are refused.
        dt_mean = (times[end] - times[start]) / max(1, end - start)
        floor = convention.onset_sd_multiple * (sd / mass) * math.sqrt(dt_mean * max(t_vmin - t_back, dt_mean))
        if v_min > -floor:
            refuse(("peak_downward_velocity", "velocity_reversal"), "NO_SIGNIFICANT_COUNTERMOVEMENT")
            out["takeoff_velocity"] = (
                DetectedEvent("takeoff_velocity", t_takeoff, vel[-1][1], None)
                if t_takeoff is not None
                else DetectedEvent("takeoff_velocity", None, None, "NO_TAKEOFF")
            )
            out["landing"] = DetectedEvent("landing", None, None, "LANDING_NOT_IN_EXPORT")
            return out
        out["peak_downward_velocity"] = DetectedEvent("peak_downward_velocity", t_vmin, v_min, None)
        rev = None
        for (t0, v0), (t1, v1) in zip(vel, vel[1:]):
            if t0 >= t_vmin and v0 < 0.0 <= v1:
                rev = t0 + (t1 - t0) * (-v0) / (v1 - v0)
                break
        out["velocity_reversal"] = (
            DetectedEvent("velocity_reversal", rev, None, None)
            if rev is not None
            else DetectedEvent("velocity_reversal", None, None, "NO_REVERSAL")
        )
        if t_takeoff is not None:
            out["takeoff_velocity"] = DetectedEvent("takeoff_velocity", t_takeoff, vel[-1][1], None)
        else:
            out["takeoff_velocity"] = DetectedEvent("takeoff_velocity", None, None, "NO_TAKEOFF")
    out["landing"] = DetectedEvent("landing", None, None, "LANDING_NOT_IN_EXPORT")
    return out


def detect_lpt_events(
    times: list[float],
    displacement: list[float | None],
    velocity: list[float | None],
    convention: DetectorConvention = DetectorConvention(),
) -> dict[str, DetectedEvent]:
    out: dict[str, DetectedEvent] = {}
    valid = [(t, d) for t, d in zip(times, displacement) if d is not None]
    if len(valid) < 3:
        for name in ("lpt_onset", "lpt_lowest_point", "lpt_velocity_reversal", "lpt_takeoff"):
            out[name] = DetectedEvent(name, None, None, "INSUFFICIENT_LPT_SAMPLES")
        return out
    onset = next(((t, d) for t, d in valid if abs(d) > convention.lpt_onset_m), None)
    out["lpt_onset"] = (
        DetectedEvent("lpt_onset", onset[0], onset[1], None)
        if onset
        else DetectedEvent("lpt_onset", None, None, "NO_LPT_THRESHOLD_CROSSING")
    )
    lowest = min(valid, key=lambda p: p[1])
    out["lpt_lowest_point"] = DetectedEvent("lpt_lowest_point", lowest[0], lowest[1], None)
    rev = None
    pairs = [(t, v) for t, v in zip(times, velocity) if v is not None]
    for (t0, v0), (t1, v1) in zip(pairs, pairs[1:]):
        if t0 >= (onset[0] if onset else times[0]) and v0 < 0.0 <= v1:
            rev = t0 + (t1 - t0) * (-v0) / (v1 - v0)
            break
    out["lpt_velocity_reversal"] = (
        DetectedEvent("lpt_velocity_reversal", rev, None, None)
        if rev is not None
        else DetectedEvent("lpt_velocity_reversal", None, None, "NO_LPT_REVERSAL")
    )
    out["lpt_takeoff"] = DetectedEvent(
        "lpt_takeoff", None, None, "NOT_IDENTIFIABLE_FROM_CABLE: a cable does not observe ground contact"
    )
    return out
