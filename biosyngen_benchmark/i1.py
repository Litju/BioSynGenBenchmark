"""I1 candidate-neutral evaluator over public jump exports (RES-489).

Inputs are plain dicts with a strict, documented schema; see ``SCHEMA``. The
evaluator imports nothing from the generator. Unknown fields are refused,
never read, and listed, so a hidden-truth canary (latent events, phase
labels, solver internals) cannot influence any outcome.

Independent reference formulations (not the generator's Lagrangian/KKT path):

* NEWTON_LINEAR_MOMENTUM: over event-free windows of exported frames,
  sum_i m_i (v_i(tb) - v_i(ta)) = integral(sum contact F + M g) dt.
  Trapezoid refinement differences are reported as estimates, not bounds:
  samples alone cannot rule out an aliased or unresolved between-sample event.
* EULER_ANGULAR_MOMENTUM: dH_O/dt = sum r_i x m_i g + sum r_c x F_c about the
  lab origin, using the same nested-grid estimate and event-free windows.
* CONTACT_ADMISSIBILITY: no tensile normal force; zero contact force when
  there are no contacts; COP inside the declared foot support.
* BALLISTIC_FLIGHT: with no contacts, the system COM accelerates at exactly
  -g and angular momentum about the COM is constant at the sampled times.
* WORK_ENERGY_LEDGER: KE + PE_g + declared elastic energy changes by the
  integral of declared joint + dissipation power plus the contact power
  F . v (recomputed).
* ACTIVATION_WITHIN_DECLARED_SCOPE: activation changes respect the rate
  bounds implied by the declared first-order time constants.
* LPT_CABLE_GEOMETRY: displacement = |p - a| - |p0 - a| at the device sample
  times, within separately supplied, admitted metrology bounds; slack samples
  are excluded and counted. Candidate metadata cannot supply those bounds.
* CLOCK_INTEGRITY: contiguous indices and a finite, positive nominal clock.
* FORCE_FLIGHT_ZERO: plate force after the detected takeoff lies inside the
  baseline-derived noise floor plus an admitted zero-offset bound.
* BILATERAL_FORCE, CROSS_DEVICE_SYNC, BIOLOGICAL_VALIDITY, SENSOR_METROLOGY and
  REAL_PAIRED_FORCE_LPT: refused without the required qualified reference.

Outcomes are claim-level: PASS, REJECTED, NOT_EVALUABLE or OUT_OF_CONTEXT. No
aggregate realism score is computed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import re

from .detectors import detect_force_events


NUMERICAL = "NUMERICAL_VERIFICATION_ONLY"
SIMULATED = "SIMULATED_MEASUREMENT_ONLY"
BIOLOGICAL = "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED"
REAL_PAIRED = "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED"

CLAIMS = (
    "NEWTON_LINEAR_MOMENTUM",
    "EULER_ANGULAR_MOMENTUM",
    "CONTACT_ADMISSIBILITY",
    "BALLISTIC_FLIGHT",
    "WORK_ENERGY_LEDGER",
    "ACTIVATION_WITHIN_DECLARED_SCOPE",
    "LPT_CABLE_GEOMETRY",
    "CLOCK_INTEGRITY",
    "FORCE_FLIGHT_ZERO",
    "PLATE_WEIGHT_CONSISTENCY",
    "BILATERAL_FORCE",
    "CROSS_DEVICE_SYNC",
    "BIOLOGICAL_VALIDITY",
    "SENSOR_METROLOGY",
    "REAL_PAIRED_FORCE_LPT",
)

SCHEMA = {
    "top": {"candidate_id", "claims", "kinematics", "force", "lpt", "empirical_reference"},
    "kinematics": {
        "gravity_m_s2", "bodies", "frames", "declared_impulsive_events", "declared_event_boundaries", "activation_scope",
        "declared_friction_coefficient",
    },
    "body": {"name", "mass_kg", "inertia_kg_m2"},
    "frame": {
        "t", "body_states", "contacts", "foot_support_x_m", "joint_power_w",
        "internal_dissipation_power_w", "internal_elastic_energy_j", "activations", "lpt_attachment_m",
    },
    "contact": {"name", "x", "z", "vx", "vz", "fx", "fz"},
    "event": {"t", "kind"},
    "activation_scope": {"status", "tau_act_s", "tau_deact_s", "u_min", "u_max"},
    "force": {
        "device_class", "sample_period_s", "native_time_offset_s", "support_coverage", "units", "positive_z",
        "samples", "left_fz", "right_fz",
    },
    "force_sample": {"index", "t", "fx", "fz", "my", "cop_x", "saturated"},
    "lpt": {
        "device_class", "sample_period_s", "native_time_offset_s", "anchor_m", "attachment", "quantity", "units", "samples",
    },
    "lpt_sample": {"index", "t", "displacement", "provider_velocity", "slack"},
}

WINDOW_FRAMES = 21  # odd, so the 2h trapezoid uses every other frame
ROUNDOFF_FLOOR = 1e-9


@dataclass(frozen=True, slots=True)
class MetrologyAuthority:
    """Out-of-band calibration authority supplied by the benchmark operator.

    This object must be sourced from an independently admitted calibration
    record. The submission schema intentionally has no corresponding field.
    The repository currently admits no real device calibration records.
    """

    reference_id: str
    evidence_sha256: str
    device: str
    device_class: str
    sample_period_s: float
    native_time_offset_s: float
    timing_uncertainty_bound_s: float
    offset_uncertainty_bound_s: float
    gain_uncertainty_bound: float | None = None
    zero_offset_bound_n: float | None = None
    encoder_resolution_m: float | None = None


@dataclass(frozen=True, slots=True)
class Finding:
    claim: str
    outcome: str
    code: str
    reason: str
    ceiling: str
    metrics: dict = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class Report:
    candidate_id: str
    findings: tuple[Finding, ...]
    refused_fields: tuple[str, ...]
    counts: dict

    def outcome(self, claim: str) -> str:
        return next(f.outcome for f in self.findings if f.claim == claim)

    def finding(self, claim: str) -> Finding:
        return next(f for f in self.findings if f.claim == claim)


def _ne(claim, code, reason, ceiling=NUMERICAL, metrics=None):
    return Finding(claim, "NOT_EVALUABLE", code, reason, ceiling, metrics or {})


def _rej(claim, code, reason, ceiling=NUMERICAL, metrics=None):
    return Finding(claim, "REJECTED", code, reason, ceiling, metrics or {})


def _pass(claim, code, reason, ceiling=NUMERICAL, metrics=None):
    return Finding(claim, "PASS", code, reason, ceiling, metrics or {})


# Schema filtering -------------------------------------------------------------
def _filter(submission: dict) -> tuple[dict, list[str]]:
    refused: list[str] = []
    clean: dict = {}
    for key, value in submission.items():
        if key not in SCHEMA["top"]:
            refused.append(key)
            continue
        clean[key] = value
    kin = clean.get("kinematics")
    if isinstance(kin, dict):
        k2 = {}
        for key, value in kin.items():
            if key in SCHEMA["kinematics"]:
                k2[key] = value
            else:
                refused.append(f"kinematics.{key}")
        bodies = k2.get("bodies")
        if isinstance(bodies, list):
            filtered_bodies = []
            for body in bodies:
                if not isinstance(body, dict):
                    filtered_bodies.append(body)
                    continue
                b2 = {key: value for key, value in body.items() if key in SCHEMA["body"]}
                refused.extend(f"kinematics.bodies[*].{key}" for key in body if key not in SCHEMA["body"]
                               and f"kinematics.bodies[*].{key}" not in refused)
                filtered_bodies.append(b2)
            k2["bodies"] = filtered_bodies
        frames = []
        raw_frames = k2.get("frames", [])
        for i, frame in enumerate(raw_frames if isinstance(raw_frames, list) else []):
            if not isinstance(frame, dict):
                frames.append(frame)
                continue
            f2 = {}
            for key, value in frame.items():
                if key in SCHEMA["frame"]:
                    f2[key] = value
                elif f"kinematics.frames[*].{key}" not in refused:
                    refused.append(f"kinematics.frames[*].{key}")
            contacts = f2.get("contacts")
            if isinstance(contacts, list):
                filtered_contacts = []
                for contact in contacts:
                    if not isinstance(contact, dict):
                        filtered_contacts.append(contact)
                        continue
                    c2 = {key: value for key, value in contact.items() if key in SCHEMA["contact"]}
                    refused.extend(f"kinematics.frames[*].contacts[*].{key}" for key in contact
                                   if key not in SCHEMA["contact"]
                                   and f"kinematics.frames[*].contacts[*].{key}" not in refused)
                    filtered_contacts.append(c2)
                f2["contacts"] = filtered_contacts
            frames.append(f2)
        k2["frames"] = frames
        scope = k2.get("activation_scope")
        if isinstance(scope, dict):
            k2["activation_scope"] = {key: value for key, value in scope.items()
                                       if key in SCHEMA["activation_scope"]}
            refused.extend(f"kinematics.activation_scope.{key}" for key in scope
                           if key not in SCHEMA["activation_scope"]
                           and f"kinematics.activation_scope.{key}" not in refused)
        event_key = "declared_event_boundaries" if "declared_event_boundaries" in k2 else "declared_impulsive_events"
        events = k2.get(event_key)
        if isinstance(events, list):
            filtered_events = []
            for event in events:
                if not isinstance(event, dict):
                    filtered_events.append(event)
                    continue
                e2 = {key: value for key, value in event.items() if key in SCHEMA["event"]}
                refused.extend(f"kinematics.{event_key}[*].{key}" for key in event
                               if key not in SCHEMA["event"]
                               and f"kinematics.{event_key}[*].{key}" not in refused)
                filtered_events.append(e2)
            k2[event_key] = filtered_events
        clean["kinematics"] = k2
    for dev in ("force", "lpt"):
        d = clean.get(dev)
        if isinstance(d, dict):
            d2 = {}
            for key, value in d.items():
                if key in SCHEMA[dev]:
                    d2[key] = value
                else:
                    refused.append(f"{dev}.{key}")
            sample_key = "force_sample" if dev == "force" else "lpt_sample"
            samples = d2.get("samples")
            if isinstance(samples, list):
                filtered_samples = []
                for sample in samples:
                    if not isinstance(sample, dict):
                        filtered_samples.append(sample)
                        continue
                    s2 = {key: value for key, value in sample.items() if key in SCHEMA[sample_key]}
                    refused.extend(f"{dev}.samples[*].{key}" for key in sample if key not in SCHEMA[sample_key]
                                   and f"{dev}.samples[*].{key}" not in refused)
                    filtered_samples.append(s2)
                d2["samples"] = filtered_samples
            clean[dev] = d2
    return clean, refused


# Mechanics helpers -------------------------------------------------------------
def _trap(ts, ys):
    return sum(0.5 * (ys[i] + ys[i + 1]) * (ts[i + 1] - ts[i]) for i in range(len(ts) - 1))


def _quadrature(ts, ys):
    """Return sampled trapezoid integral, refinement estimate and stability.

    The two nested-grid differences are diagnostics, not mathematical error
    bounds for arbitrary trajectories. A window is usable only when the
    finer-grid difference does not grow materially relative to the coarser
    one. Even a stable result cannot detect a signal aliased by every sample.
    """
    def nested_trapezoid(stride):
        indices = list(range(0, len(ts), stride))
        if indices[-1] != len(ts) - 1:
            indices.append(len(ts) - 1)
        return _trap([ts[i] for i in indices], [ys[i] for i in indices])

    fine = _trap(ts, ys)
    middle = nested_trapezoid(2)
    coarse = nested_trapezoid(4)
    fine_delta = abs(fine - middle)
    coarse_delta = abs(middle - coarse)
    scale = max(1.0, abs(fine), abs(middle), abs(coarse))
    roundoff = 64.0 * math.ulp(scale) * len(ts)
    stable = fine_delta <= 1.25 * coarse_delta + roundoff
    return fine, fine_delta, stable, coarse_delta


def _windows(frames, impulses, *, stride=WINDOW_FRAMES - 1):
    """Return event-free windows and excluded count.

    Besides independently detectable contact-topology changes, declared
    impulsive events split windows. These windows are local sampled checks;
    their number is not evidence of complete trajectory coverage.
    """
    impulse_times = [e["t"] for e in impulses]
    contact_signatures = [tuple(sorted(c["name"] for c in f["contacts"])) for f in frames]
    transition_times = [frames[i]["t"] for i in range(1, len(frames))
                        if contact_signatures[i] != contact_signatures[i - 1]]
    event_times = impulse_times + transition_times
    out, excluded = [], 0
    start = 0
    n = len(frames)
    while start + WINDOW_FRAMES <= n:
        idx = list(range(start, start + WINDOW_FRAMES))
        t0, t1 = frames[idx[0]]["t"], frames[idx[-1]]["t"]
        if any(t0 - 1e-12 <= ti <= t1 + 1e-12 for ti in event_times):
            excluded += 1
        else:
            out.append(idx)
        start += stride
    return out, excluded


def _momentum(kin, frame):
    px = pz = 0.0
    for b, st in zip(kin["bodies"], frame["body_states"]):
        px += b["mass_kg"] * st[3]
        pz += b["mass_kg"] * st[4]
    return px, pz


def _ang_momentum(kin, frame, origin=(0.0, 0.0)):
    h = 0.0
    for b, st in zip(kin["bodies"], frame["body_states"]):
        x, z, _, vx, vz, w = st
        h += b["mass_kg"] * ((x - origin[0]) * vz - (z - origin[1]) * vx) + b["inertia_kg_m2"] * w
    return h


def _total_mass(kin):
    return sum(b["mass_kg"] for b in kin["bodies"])


def _com(kin, frame):
    m = _total_mass(kin)
    x = sum(b["mass_kg"] * st[0] for b, st in zip(kin["bodies"], frame["body_states"])) / m
    z = sum(b["mass_kg"] * st[1] for b, st in zip(kin["bodies"], frame["body_states"])) / m
    vx = sum(b["mass_kg"] * st[3] for b, st in zip(kin["bodies"], frame["body_states"])) / m
    vz = sum(b["mass_kg"] * st[4] for b, st in zip(kin["bodies"], frame["body_states"])) / m
    return x, z, vx, vz


def _finite_number(value) -> bool:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, TypeError, ValueError):
        return False


def _finite_vector(value, size: int) -> bool:
    return isinstance(value, (list, tuple)) and len(value) == size and all(_finite_number(v) for v in value)


def _finite_tree(value) -> bool:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return True
    if isinstance(value, (int, float)):
        return _finite_number(value)
    if isinstance(value, dict):
        return all(_finite_tree(k) and _finite_tree(v) for k, v in value.items())
    if isinstance(value, (list, tuple)):
        return all(_finite_tree(v) for v in value)
    return False


def _validate_kinematics(kin) -> str | None:
    if not isinstance(kin, dict):
        return "KINEMATIC_SCHEMA_INCOMPLETE"
    try:
        g = kin["gravity_m_s2"]
        if not _finite_number(g):
            return "NONFINITE_PHYSICAL_PARAMETER"
        if not 9.7 < g < 9.9:
            return "GRAVITY_OUT_OF_CONTEXT"
        bodies = kin["bodies"]
        if not isinstance(bodies, list) or not bodies:
            return "INVALID_INERTIAL_PROPERTIES"
        names = []
        for body in bodies:
            if not isinstance(body, dict) or not isinstance(body.get("name"), str):
                return "INVALID_INERTIAL_PROPERTIES"
            mass, inertia = body["mass_kg"], body["inertia_kg_m2"]
            if not _finite_number(mass) or not _finite_number(inertia):
                return "NONFINITE_PHYSICAL_PARAMETER"
            if mass <= 0 or inertia < 0:
                return "INVALID_INERTIAL_PROPERTIES"
            names.append(body["name"])
        if len(set(names)) != len(names):
            return "DUPLICATE_BODY_NAME"

        mu = kin.get("declared_friction_coefficient")
        if mu is not None and (not _finite_number(mu) or mu < 0):
            return "INVALID_FRICTION_PARAMETER"
        frames = kin["frames"]
        if not isinstance(frames, list):
            return "KINEMATIC_SCHEMA_INCOMPLETE"
        if len(frames) < WINDOW_FRAMES:
            return "TOO_FEW_FRAMES"
        previous = None
        for frame in frames:
            if not isinstance(frame, dict):
                return "FRAME_STRUCTURE_INVALID"
            t = frame["t"]
            if not _finite_number(t):
                return "NONFINITE_KINEMATIC_TIME"
            if t < 0:
                return "KINEMATIC_TIME_OUT_OF_RANGE"
            if previous is not None and t <= previous:
                return "KINEMATIC_TIME_NOT_STRICTLY_INCREASING"
            previous = t

            states = frame["body_states"]
            if not isinstance(states, list) or len(states) != len(bodies):
                return "FRAME_STRUCTURE_INVALID"
            if any(not _finite_vector(state, 6) for state in states):
                return "NONFINITE_OR_MALFORMED_STATE"

            contacts = frame["contacts"]
            if not isinstance(contacts, list):
                return "CONTACT_SCHEMA_INVALID"
            contact_names = set()
            for contact in contacts:
                if not isinstance(contact, dict) or not isinstance(contact.get("name"), str):
                    return "CONTACT_SCHEMA_INVALID"
                if contact["name"] in contact_names:
                    return "DUPLICATE_CONTACT_NAME"
                contact_names.add(contact["name"])
                if any(not _finite_number(contact.get(key)) for key in ("x", "z", "vx", "vz", "fx", "fz")):
                    return "NONFINITE_OR_MALFORMED_CONTACT"

            support = frame.get("foot_support_x_m")
            if support is not None and not _finite_vector(support, 2):
                return "FOOT_SUPPORT_SCHEMA_INVALID"
            for key in ("joint_power_w", "internal_dissipation_power_w", "internal_elastic_energy_j"):
                if key in frame and not _finite_number(frame[key]):
                    return "NONFINITE_POWER_OR_ENERGY"
            if "internal_elastic_energy_j" in frame and frame["internal_elastic_energy_j"] < 0:
                return "NEGATIVE_ELASTIC_ENERGY"
            if "activations" in frame:
                activations = frame["activations"]
                if not isinstance(activations, list) or any(
                    not _finite_number(a) or a < 0 or a > 1 for a in activations
                ):
                    return "ACTIVATION_SCHEMA_INVALID"
            if "lpt_attachment_m" in frame and not _finite_vector(frame["lpt_attachment_m"], 3):
                return "LPT_ATTACHMENT_DIMENSION_INVALID"

        events = kin.get("declared_event_boundaries", kin.get("declared_impulsive_events", []))
        if not isinstance(events, list):
            return "IMPULSE_EVENT_SCHEMA_INVALID"
        event_times = []
        for event in events:
            if not isinstance(event, dict) or not _finite_number(event.get("t")):
                return "IMPULSE_EVENT_SCHEMA_INVALID"
            if not isinstance(event.get("kind"), str):
                return "IMPULSE_EVENT_SCHEMA_INVALID"
            event_times.append(event["t"])
        if any(a > b for a, b in zip(event_times, event_times[1:])):
            return "IMPULSE_EVENT_ORDER_INVALID"

        scope = kin.get("activation_scope")
        if scope is not None:
            if not isinstance(scope, dict):
                return "ACTIVATION_SCOPE_INVALID"
            numeric = tuple(scope.get(key) for key in ("tau_act_s", "tau_deact_s", "u_min", "u_max"))
            if not all(_finite_number(v) for v in numeric):
                return "ACTIVATION_SCOPE_INVALID"
            tau_act, tau_deact, u_min, u_max = numeric
            if tau_act <= 0 or tau_deact <= 0 or not 0 <= u_min <= u_max <= 1:
                return "ACTIVATION_SCOPE_INVALID"

        total_mass = math.fsum(body["mass_kg"] for body in bodies)
        if not _finite_number(total_mass) or not _finite_number(total_mass * g):
            return "INERTIAL_ARITHMETIC_OVERFLOW"
        for frame in frames:
            for body, state in zip(bodies, frame["body_states"]):
                mass, inertia = body["mass_kg"], body["inertia_kg_m2"]
                x, z, _, vx, vz, omega = state
                derived = (mass * x, mass * z, mass * vx, mass * vz,
                           inertia * omega, mass * g * z,
                           0.5 * mass * (vx * vx + vz * vz), 0.5 * inertia * omega * omega,
                           mass * (x * vz - z * vx))
                if not all(_finite_number(value) for value in derived):
                    return "KINEMATIC_ARITHMETIC_OVERFLOW"
            for contact in frame["contacts"]:
                derived = (contact["x"] * contact["fz"], contact["z"] * contact["fx"],
                           contact["fx"] * contact["vx"], contact["fz"] * contact["vz"])
                if not all(_finite_number(value) for value in derived):
                    return "CONTACT_ARITHMETIC_OVERFLOW"
            if not all(_finite_number(sum(contact[key] for contact in frame["contacts"]))
                       for key in ("fx", "fz")):
                return "CONTACT_SUM_OVERFLOW"
        if not _finite_number(g * (frames[-1]["t"] - frames[0]["t"])):
            return "KINEMATIC_TIME_SPAN_OVERFLOW"
    except (KeyError, TypeError, ValueError, OverflowError):
        return "KINEMATIC_SCHEMA_INCOMPLETE"
    return None


def _newton(kin):
    claim = "NEWTON_LINEAR_MOMENTUM"
    frames, g = kin["frames"], kin["gravity_m_s2"]
    mass = _total_mass(kin)
    windows, excluded = _windows(frames, kin.get("declared_event_boundaries", kin.get("declared_impulsive_events", [])))
    worst_ratio, worst_res = 0.0, 0.0
    max_qerr = max_coarse_delta = 0.0
    under_resolved = 0
    for idx in windows:
        ts = [frames[i]["t"] for i in idx]
        if ts[-1] - ts[0] <= 0.0:
            continue
        for axis in (0, 1):
            fs = [sum(c["fx" if axis == 0 else "fz"] for c in frames[i]["contacts"]) - (mass * g if axis == 1 else 0.0)
                  for i in idx]
            ih, qerr, stable, coarse_delta = _quadrature(ts, fs)
            max_qerr = max(max_qerr, qerr)
            max_coarse_delta = max(max_coarse_delta, coarse_delta)
            if not stable:
                under_resolved += 1
                continue
            p0 = _momentum(kin, frames[idx[0]])[axis]
            p1 = _momentum(kin, frames[idx[-1]])[axis]
            res = (p1 - p0) - ih
            scale = mass * g * (ts[-1] - ts[0]) + abs(p0) + abs(p1)
            tol = qerr + 1e-7 * scale + ROUNDOFF_FLOOR
            worst_ratio = max(worst_ratio, abs(res) / tol)
            worst_res = max(worst_res, abs(res))
    metrics = {"windows": len(windows), "excluded_event_windows": excluded,
               "under_resolved_quadratures": under_resolved,
               "max_quadrature_error_estimate_n_s": max_qerr if windows else None,
               "max_coarse_refinement_delta_n_s": max_coarse_delta if windows else None,
               "max_residual_n_s": worst_res, "max_residual_over_sampled_tolerance": worst_ratio,
               "quadrature_error_is_bound": False}
    if not windows:
        return _ne(claim, "NO_ADMISSIBLE_WINDOW", "No impulse-free window of frames.", metrics=metrics)
    if worst_ratio > 1.0:
        return _rej(claim, "MOMENTUM_IMBALANCE",
                    "Exported contact impulses do not account for the exported momentum change.", metrics=metrics)
    if under_resolved:
        return _ne(claim, "QUADRATURE_REFINEMENT_UNSTABLE",
                   "At least one event-free window has unstable refinement; remaining sampled windows do not show an imbalance.", metrics=metrics)
    return _pass(claim, "MOMENTUM_BALANCED_SAMPLED",
                 "Sampled linear momentum change agrees with contact plus gravity impulse; refinement is an estimate, not a bound.",
                 metrics=metrics)


def _euler(kin):
    claim = "EULER_ANGULAR_MOMENTUM"
    frames, g = kin["frames"], kin["gravity_m_s2"]
    windows, excluded = _windows(frames, kin.get("declared_event_boundaries", kin.get("declared_impulsive_events", [])))
    worst_ratio, worst_res = 0.0, 0.0
    max_qerr = max_coarse_delta = 0.0
    under_resolved = 0
    for idx in windows:
        ts = [frames[i]["t"] for i in idx]
        if ts[-1] - ts[0] <= 0.0:
            continue
        torque = []
        for i in idx:
            f = frames[i]
            tq = sum(-b["mass_kg"] * g * st[0] for b, st in zip(kin["bodies"], f["body_states"]))
            tq += sum(c["x"] * c["fz"] - c["z"] * c["fx"] for c in f["contacts"])
            torque.append(tq)
        ih, qerr, stable, coarse_delta = _quadrature(ts, torque)
        max_qerr = max(max_qerr, qerr)
        max_coarse_delta = max(max_coarse_delta, coarse_delta)
        if not stable:
            under_resolved += 1
            continue
        h0, h1 = _ang_momentum(kin, frames[idx[0]]), _ang_momentum(kin, frames[idx[-1]])
        res = (h1 - h0) - ih
        tol = qerr + 1e-7 * (abs(h0) + abs(h1) + sum(abs(t) for t in torque) * (ts[-1] - ts[0]) / len(ts)) + ROUNDOFF_FLOOR
        worst_ratio = max(worst_ratio, abs(res) / tol)
        worst_res = max(worst_res, abs(res))
    metrics = {"windows": len(windows), "excluded_event_windows": excluded,
               "under_resolved_quadratures": under_resolved,
               "max_quadrature_error_estimate_n_m": max_qerr if windows else None,
               "max_coarse_refinement_delta_n_m": max_coarse_delta if windows else None,
               "max_residual_n_m_s": worst_res, "max_residual_over_sampled_tolerance": worst_ratio,
               "quadrature_error_is_bound": False}
    if not windows:
        return _ne(claim, "NO_ADMISSIBLE_WINDOW", "No impulse-free window of frames.", metrics=metrics)
    if worst_ratio > 1.0:
        return _rej(claim, "ANGULAR_MOMENTUM_IMBALANCE",
                    "External moments do not account for the angular momentum change.", metrics=metrics)
    if under_resolved:
        return _ne(claim, "QUADRATURE_REFINEMENT_UNSTABLE",
                   "At least one event-free window has unstable refinement; remaining sampled windows do not show an imbalance.", metrics=metrics)
    return _pass(claim, "ANGULAR_MOMENTUM_BALANCED_SAMPLED",
                 "Sampled angular momentum change agrees with external moment impulse; refinement is an estimate, not a bound.",
                 metrics=metrics)


def _contact(kin):
    claim = "CONTACT_ADMISSIBILITY"
    g = kin["gravity_m_s2"]
    weight = _total_mass(kin) * g
    tol = 1e-6 * weight
    mu = kin.get("declared_friction_coefficient")
    tensile = cop_out = friction = 0
    for f in kin["frames"]:
        contacts = f["contacts"]
        fz = sum(c["fz"] for c in contacts)
        if any(c["fz"] < -tol for c in contacts):
            tensile += 1
        if mu is not None and abs(sum(c["fx"] for c in contacts)) > mu * max(fz, 0.0) + tol:
            friction += 1
        support = f.get("foot_support_x_m")
        if contacts and fz > 1e-3 * weight and support is not None:
            cop = sum(c["fz"] * c["x"] for c in contacts) / fz
            lo, hi = min(support), max(support)
            if cop < lo - 1e-6 or cop > hi + 1e-6:
                cop_out += 1
    metrics = {"frames": len(kin["frames"]), "tensile_frames": tensile, "cop_outside_support_frames": cop_out,
               "friction_cone_violations": friction, "declared_mu": mu}
    if tensile:
        return _rej(claim, "TENSILE_CONTACT", "A normal contact force pulls the body toward the ground.", metrics=metrics)
    if friction:
        return _rej(claim, "FRICTION_CONE_VIOLATED", "Tangential force exceeds the declared friction cone.", metrics=metrics)
    if cop_out:
        return _rej(claim, "COP_OUTSIDE_SUPPORT", "COP lies outside the declared foot support.", metrics=metrics)
    return _pass(claim, "CONTACT_ADMISSIBLE", "Nonadhesive contact with COP inside the support.", metrics=metrics)


def _flight(kin):
    claim = "BALLISTIC_FLIGHT"
    frames, g = kin["frames"], kin["gravity_m_s2"]
    mass = _total_mass(kin)
    flight = [f for f in frames if not f["contacts"]]
    if len(flight) < 3:
        return _ne(claim, "NO_FLIGHT_FRAMES", "No contact-free interval was exported.")
    t0 = flight[0]["t"]
    _, _, vx0, vz0 = _com(kin, flight[0])
    x0, z0, _, _ = _com(kin, flight[0])
    worst_v = worst_h = 0.0
    for f in flight:
        x, z, vx, vz = _com(kin, f)
        dt = f["t"] - t0
        worst_v = max(worst_v, abs(vz - (vz0 - g * dt)), abs(vx - vx0))
        # H about the moving COM = H_O - M (r_com x v_com) with O = lab origin.
        h = _ang_momentum(kin, f) - mass * (x * vz - z * vx)
        h0 = _ang_momentum(kin, flight[0]) - mass * (x0 * vz0 - z0 * vx0)
        worst_h = max(worst_h, abs(h - h0))
    span = flight[-1]["t"] - t0
    tol_v = 1e-6 * (1.0 + g * span)
    tol_h = 1e-6 * (1.0 + max(abs(_ang_momentum(kin, f)) for f in flight))
    metrics = {"flight_frames": len(flight), "max_com_velocity_error_m_s": worst_v, "max_h_com_drift": worst_h}
    if worst_v > tol_v:
        return _rej(claim, "NON_BALLISTIC_COM", "System COM does not follow ballistic motion without contact.", metrics=metrics)
    if worst_h > tol_h:
        return _rej(claim, "ANGULAR_MOMENTUM_NOT_CONSERVED", "Angular momentum about the COM changes in flight.", metrics=metrics)
    return _pass(claim, "BALLISTIC", "COM ballistic and angular momentum about COM conserved.", metrics=metrics)


def _energy(kin):
    claim = "WORK_ENERGY_LEDGER"
    frames, g = kin["frames"], kin["gravity_m_s2"]
    needed = ("joint_power_w", "internal_dissipation_power_w", "internal_elastic_energy_j")
    if any(k not in f for f in frames for k in needed):
        return _ne(claim, "POWER_LEDGER_NOT_DECLARED", "Declared internal power/storage is required.")
    # Check every sampled start, not one partition whose endpoints can miss
    # a localized energy change. Budgets/refinement policy remain unchanged.
    windows, excluded = _windows(frames, kin.get("declared_event_boundaries", kin.get("declared_impulsive_events", [])), stride=1)

    def energy(f):
        e = f["internal_elastic_energy_j"]
        for b, st in zip(kin["bodies"], f["body_states"]):
            e += 0.5 * b["mass_kg"] * (st[3] ** 2 + st[4] ** 2) + 0.5 * b["inertia_kg_m2"] * st[5] ** 2
            e += b["mass_kg"] * g * st[1]
        return e

    def power(f):
        p = f["joint_power_w"] + f["internal_dissipation_power_w"]
        return p + sum(c["fx"] * c["vx"] + c["fz"] * c["vz"] for c in f["contacts"])

    worst_ratio = worst = 0.0
    max_qerr = max_coarse_delta = 0.0
    under_resolved = 0
    for idx in windows:
        ts = [frames[i]["t"] for i in idx]
        ps = [power(frames[i]) for i in idx]
        ih, qerr, stable, coarse_delta = _quadrature(ts, ps)
        max_qerr = max(max_qerr, qerr)
        max_coarse_delta = max(max_coarse_delta, coarse_delta)
        if not stable:
            under_resolved += 1
            continue
        res = energy(frames[idx[-1]]) - energy(frames[idx[0]]) - ih
        tol = qerr + 1e-7 * (sum(abs(p) for p in ps) * (ts[-1] - ts[0]) / len(ts) + abs(energy(frames[idx[0]]))) + ROUNDOFF_FLOOR
        worst_ratio = max(worst_ratio, abs(res) / tol)
        worst = max(worst, abs(res))
    metrics = {"windows": len(windows), "excluded_event_windows": excluded,
               "under_resolved_quadratures": under_resolved,
               "max_quadrature_error_estimate_j": max_qerr if windows else None,
               "max_coarse_refinement_delta_j": max_coarse_delta if windows else None,
               "max_residual_j": worst, "max_residual_over_sampled_tolerance": worst_ratio,
               "quadrature_error_is_bound": False}
    if not windows:
        return _ne(claim, "NO_ADMISSIBLE_WINDOW", "No impulse-free window.", metrics=metrics)
    if worst_ratio > 1.0:
        return _rej(claim, "ENERGY_NOT_ACCOUNTED", "Mechanical energy changes without declared or contact work.", metrics=metrics)
    if under_resolved:
        return _ne(claim, "QUADRATURE_REFINEMENT_UNSTABLE",
                   "At least one event-free window has unstable refinement; remaining sampled windows do not show an energy imbalance.", metrics=metrics)
    return _pass(claim, "ENERGY_ACCOUNTED_SAMPLED",
                 "Sampled energy change agrees with declared and contact work; refinement is an estimate, not a bound.",
                 metrics=metrics)


def _activation(kin):
    claim = "ACTIVATION_WITHIN_DECLARED_SCOPE"
    scope = kin.get("activation_scope")
    frames = kin["frames"]
    if not scope or any("activations" not in f for f in frames):
        return _ne(claim, "NO_DECLARED_ACTIVATION_SCOPE", "Activations and a declared dynamic scope are required.", NUMERICAL)
    ta, td, umin, umax = scope["tau_act_s"], scope["tau_deact_s"], scope["u_min"], scope["u_max"]
    violations = 0
    worst = 0.0
    for f0, f1 in zip(frames, frames[1:]):
        dt = f1["t"] - f0["t"]
        if dt <= 0:
            continue
        for a0, a1 in zip(f0["activations"], f1["activations"]):
            if a0 < umin - 1e-9 or a1 > umax + 1e-9 or a1 < umin - 1e-9:
                violations += 1
                continue
            lo = min(a0, a1)
            rise_bound = dt * (umax - lo) / (0.5 * ta)
            fall_bound = dt * 2.0 * (max(a0, a1) - umin) / td
            change = a1 - a0
            ratio = change / rise_bound if change > 0 else -change / fall_bound if change < 0 else 0.0
            worst = max(worst, ratio)
            if ratio > 1.0 + 1e-9:
                violations += 1
    metrics = {"scope_status": scope.get("status", "UNDECLARED"), "violations": violations, "max_rate_ratio": worst}
    if violations:
        return _rej(claim, "ACTIVATION_RATE_OUTSIDE_SCOPE",
                    "Activation changes faster than the declared first-order dynamics allow.", NUMERICAL, metrics)
    return _pass(claim, "WITHIN_DECLARED_SCOPE",
                 "Activation respects the declared numerical scope; this is not physiological validation.",
                 NUMERICAL, metrics)


def _clock(dev, name):
    if not isinstance(dev, dict):
        return f"{name}: device schema incomplete"
    try:
        period = dev["sample_period_s"]
        offset = dev["native_time_offset_s"]
        samples = dev["samples"]
        if not _finite_number(period) or period <= 0:
            return f"{name}: sample period must be finite and positive"
        if not _finite_number(offset):
            return f"{name}: native offset must be finite"
        if not isinstance(samples, list) or not samples:
            return f"{name}: sample list is empty or malformed"
        previous = None
        for k, sample in enumerate(samples):
            if not isinstance(sample, dict):
                return f"{name}: malformed sample at index {k}"
            index, t = sample.get("index"), sample.get("t")
            if isinstance(index, bool) or not isinstance(index, int) or index != k:
                return f"{name}: indices not contiguous"
            if not _finite_number(t):
                return f"{name}: nonfinite timestamp at index {k}"
            if previous is not None and t <= previous:
                return f"{name}: native timestamps are not strictly increasing"
            expected = offset + k * period
            if not _finite_number(expected):
                return f"{name}: nominal clock arithmetic overflow"
            tolerance = 1e-9 * max(1.0, abs(t), abs(expected))
            if abs(t - expected) > tolerance:
                return f"{name}: native time off the declared clock at index {k}"
            previous = t
    except (KeyError, TypeError, ValueError, OverflowError):
        return f"{name}: clock schema incomplete"
    return None


def _validate_device(dev, name):
    problem = _clock(dev, name)
    if problem:
        return problem
    if not isinstance(dev.get("device_class"), str) or not dev["device_class"]:
        return f"{name}: device class missing"
    if name == "FORCE":
        if dev.get("units") != {"force": "N", "moment": "N m", "length": "m"}:
            return "FORCE: units must be N, N m and m"
        if dev.get("positive_z") != "up (reaction on subject)":
            return "FORCE: sign convention is missing or unsupported"
        if not isinstance(dev.get("support_coverage"), str) or not dev["support_coverage"]:
            return "FORCE: support coverage is missing"
        for index, sample in enumerate(dev["samples"]):
            for key in ("fx", "fz", "my", "cop_x"):
                value = sample.get(key)
                if value is not None and not _finite_number(value):
                    return f"FORCE: nonfinite or malformed {key} at index {index}"
            if not isinstance(sample.get("saturated"), bool):
                return f"FORCE: saturation flag malformed at index {index}"
    else:
        if dev.get("units") != {"displacement": "m", "velocity": "m/s"}:
            return "LPT: units must be m and m/s"
        if not _finite_vector(dev.get("anchor_m"), 3):
            return "LPT: anchor must be a finite 3-vector"
        if not isinstance(dev.get("attachment"), str) or not dev["attachment"]:
            return "LPT: attachment label missing"
        if dev.get("quantity") != "cable displacement from first sample":
            return "LPT: unsupported measured quantity"
        for index, sample in enumerate(dev["samples"]):
            for key in ("displacement", "provider_velocity"):
                value = sample.get(key)
                if value is not None and not _finite_number(value):
                    return f"LPT: nonfinite or malformed {key} at index {index}"
            if not isinstance(sample.get("slack"), bool):
                return f"LPT: slack flag malformed at index {index}"
    return None


def _authority_problem(authority, dev, name):
    if authority is None:
        return "NO_ADMITTED_CALIBRATION_REFERENCE"
    if not isinstance(authority, MetrologyAuthority):
        return "INVALID_AUTHORITY_OBJECT"
    if (not isinstance(authority.reference_id, str) or not authority.reference_id.strip()
            or not isinstance(authority.evidence_sha256, str)
            or re.fullmatch(r"[0-9a-fA-F]{64}", authority.evidence_sha256) is None):
        return "INVALID_AUTHORITY_PROVENANCE"
    if authority.device != name or authority.device_class != dev.get("device_class"):
        return "AUTHORITY_DEVICE_MISMATCH"
    for value in (authority.sample_period_s, authority.native_time_offset_s,
                  authority.timing_uncertainty_bound_s, authority.offset_uncertainty_bound_s):
        if not _finite_number(value):
            return "INVALID_AUTHORITY_NUMERIC_VALUE"
    if (authority.sample_period_s <= 0 or authority.timing_uncertainty_bound_s < 0
            or authority.offset_uncertainty_bound_s < 0):
        return "INVALID_AUTHORITY_NUMERIC_VALUE"
    if name == "FORCE":
        values = (authority.gain_uncertainty_bound, authority.zero_offset_bound_n)
        if not all(_finite_number(value) and value >= 0 for value in values):
            return "INVALID_AUTHORITY_FORCE_BOUNDS"
    else:
        value = authority.encoder_resolution_m
        if not _finite_number(value) or value <= 0:
            return "INVALID_AUTHORITY_LPT_BOUNDS"
    tolerance = 1e-9 * max(1.0, abs(authority.sample_period_s), abs(dev["sample_period_s"]))
    if abs(authority.sample_period_s - dev["sample_period_s"]) > tolerance:
        return "AUTHORITY_SAMPLE_PERIOD_MISMATCH"
    if abs(authority.native_time_offset_s - dev["native_time_offset_s"]) > authority.offset_uncertainty_bound_s + tolerance:
        return "AUTHORITY_TIME_OFFSET_MISMATCH"
    return None


def _authority_finding(claim, problem):
    if problem == "NO_ADMITTED_CALIBRATION_REFERENCE" or problem.startswith("INVALID_AUTHORITY"):
        return _ne(claim, problem, "An independently admitted calibration reference is required.", SIMULATED)
    return _rej(claim, "METROLOGY_AUTHORITY_MISMATCH", problem, SIMULATED)


def _lpt(kin, lpt, authority):
    claim = "LPT_CABLE_GEOMETRY"
    frames = kin["frames"]
    if any("lpt_attachment_m" not in f for f in frames):
        return _ne(claim, "ATTACHMENT_TRAJECTORY_MISSING", "The attachment trajectory must be exported.")
    anchor = lpt["anchor_m"]
    ts = [f["t"] for f in frames]
    pts = [f["lpt_attachment_m"] for f in frames]

    def point_at(t):
        lo, hi = 0, len(ts) - 1
        if t <= ts[0]:
            return pts[0]
        if t >= ts[-1]:
            return pts[-1]
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ts[mid] <= t:
                lo = mid
            else:
                hi = mid
        w = 0.0 if ts[hi] == ts[lo] else (t - ts[lo]) / (ts[hi] - ts[lo])
        return [a + w * (b - a) for a, b in zip(pts[lo], pts[hi])]

    def length(p):
        return math.sqrt(sum((pi - ai) ** 2 for pi, ai in zip(p, anchor)))

    speeds = [
        math.sqrt(sum((b - a) ** 2 for a, b in zip(pts[i], pts[i + 1]))) / max(ts[i + 1] - ts[i], 1e-12)
        for i in range(len(ts) - 1)
        if ts[i + 1] > ts[i]
    ]
    vmax = max(speeds) if speeds else 0.0
    tol = authority.encoder_resolution_m * 2 + vmax * (
        authority.timing_uncertainty_bound_s + authority.offset_uncertainty_bound_s
    )
    samples = lpt["samples"]
    first_time = samples[0]["t"] - authority.native_time_offset_s
    if not ts[0] <= first_time <= ts[-1]:
        return _ne(claim, "LPT_REFERENCE_TIME_OUT_OF_RANGE", "The calibrated LPT time does not overlap the kinematic reference.", SIMULATED)
    l0 = length(point_at(first_time))
    worst, compared, slack = 0.0, 0, 0
    reference_displacements = []
    for s in samples:
        if s["displacement"] is None:
            continue
        if s.get("slack"):
            slack += 1
            continue
        t = s["t"] - authority.native_time_offset_s
        if not ts[0] <= t <= ts[-1]:
            return _ne(claim, "LPT_REFERENCE_TIME_OUT_OF_RANGE", "A calibrated LPT sample falls outside the kinematic reference.", SIMULATED)
        expected = length(point_at(t)) - l0
        worst = max(worst, abs(s["displacement"] - expected))
        reference_displacements.append(expected)
        compared += 1
    excursion = max(reference_displacements) - min(reference_displacements) if reference_displacements else 0.0
    metrics = {"compared_samples": compared, "slack_excluded": slack, "max_error_m": worst,
               "tolerance_m": tol, "reference_excursion_m": excursion,
               "authority_reference_id": authority.reference_id, "tolerance_from_admitted_authority": True}
    if compared == 0:
        return _ne(claim, "NO_COMPARABLE_LPT_SAMPLE", "No taut, present LPT sample.", SIMULATED, metrics)
    if tol >= excursion:
        return _ne(claim, "LPT_TOLERANCE_NONIDENTIFYING", "Admitted uncertainty is not smaller than the reference motion excursion.", SIMULATED, metrics)
    if worst > tol:
        return _rej(claim, "CABLE_GEOMETRY_MISMATCH", "Cable displacement conflicts with the calibrated 3-D geometry.", SIMULATED, metrics)
    return _pass(claim, "CABLE_GEOMETRY_CONSISTENT", "Cable displacement matches the 3-D geometry within admitted simulated-device uncertainty.", SIMULATED, metrics)


def _force_flight(force, authority, kin=None):
    """Plate force over the flight interval must sit at the noise floor.

    The interval is the contact-free span of the kinematic export when one is
    supplied (cross-modal check, mapped with the admitted nominal offset and
    shrunk by the admitted timing uncertainty). Otherwise it starts after the
    force-detected takeoff.
    """
    claim = "FORCE_FLIGHT_ZERO"
    samples = force["samples"]
    times = [s["t"] - authority.native_time_offset_s for s in samples]
    det = detect_force_events(times, [s["fz"] for s in samples])
    if det["body_weight"].refusal is not None:
        return _ne(claim, "NO_QUIET_BASELINE", det["body_weight"].refusal, SIMULATED)
    source = "force_detected_takeoff"
    if kin is not None and _validate_kinematics(kin) is None:
        free = [f["t"] for f in kin["frames"] if not f["contacts"]]
        if not free:
            return _ne(claim, "NO_DECLARED_FLIGHT", "Kinematic export has no contact-free frames.", SIMULATED)
        margin = authority.timing_uncertainty_bound_s + authority.offset_uncertainty_bound_s + 2 * authority.sample_period_s
        lo, hi = min(free) + margin, max(free) - margin
        source = "kinematic_contact_free_interval"
    else:
        if det["takeoff"].refusal is not None:
            return _ne(claim, "NO_DETECTABLE_TAKEOFF", f"Detector refused: {det['takeoff'].refusal}", SIMULATED)
        lo, hi = det["takeoff"].native_time_s + max(0.02, authority.timing_uncertainty_bound_s), math.inf
    # Baseline SD from the quiet window (same convention as the detector).
    quiet = [s["fz"] for s, t in zip(samples, times) if t - times[0] <= 0.25 and s["fz"] is not None]
    if len(quiet) < 2:
        return _ne(claim, "INSUFFICIENT_QUIET_BASELINE", "At least two quiet baseline samples are required.", SIMULATED)
    mean_q = sum(quiet) / len(quiet)
    sd = math.sqrt(sum((v - mean_q) ** 2 for v in quiet) / (len(quiet) - 1))
    flight = [s["fz"] for s, t in zip(samples, times) if lo < t < hi and s["fz"] is not None]
    if len(flight) < 10:
        return _ne(claim, "TOO_FEW_FLIGHT_SAMPLES", "Fewer than 10 flight samples.", SIMULATED)
    mean_f = sum(flight) / len(flight)
    bound = 6.0 * sd + authority.zero_offset_bound_n
    if kin is not None and bound >= _total_mass(kin) * kin["gravity_m_s2"]:
        return _ne(claim, "FORCE_TOLERANCE_NONIDENTIFYING", "Admitted zero and baseline uncertainty spans body weight.", SIMULATED)
    metrics = {"flight_samples": len(flight), "flight_mean_n": mean_f, "bound_n": bound, "baseline_sd_n": sd,
               "interval_source": source, "authority_reference_id": authority.reference_id,
               "tolerance_from_admitted_authority": True}
    if abs(mean_f) > bound or max(abs(v - mean_f) for v in flight) > 6.0 * sd + 1e-9:
        return _rej(claim, "SUPPORT_FORCE_DURING_FLIGHT", "Plate reports support beyond noise and offset bounds after takeoff.",
                    SIMULATED, metrics)
    return _pass(claim, "FLIGHT_FORCE_AT_NOISE_FLOOR",
                 "Post-takeoff force within baseline noise and declared zero offset (simulated device only).",
                 SIMULATED, metrics)


def _plate_weight(force, kin, authority):
    claim = "PLATE_WEIGHT_CONSISTENCY"
    samples = force["samples"]
    times = [s["t"] - authority.native_time_offset_s for s in samples]
    det = detect_force_events(times, [s["fz"] for s in samples])
    if det["body_weight"].refusal is not None:
        return _ne(claim, "NO_QUIET_BASELINE", det["body_weight"].refusal, SIMULATED)
    expected = _total_mass(kin) * kin["gravity_m_s2"]
    bound = authority.gain_uncertainty_bound * expected + authority.zero_offset_bound_n
    measured = det["body_weight"].value
    metrics = {"plate_weight_n": measured, "model_weight_n": expected, "bound_n": bound,
               "authority_reference_id": authority.reference_id, "tolerance_from_admitted_authority": True}
    if bound >= expected:
        return _ne(claim, "PLATE_TOLERANCE_NONIDENTIFYING", "Admitted gain and zero uncertainty spans body weight.", SIMULATED, metrics)
    if abs(measured - expected) > bound:
        return _rej(claim, "PLATE_WEIGHT_INCONSISTENT",
                    "Plate quiet-stance force disagrees with the exported system weight beyond admitted calibration bounds.",
                    SIMULATED, metrics)
    return _pass(claim, "PLATE_WEIGHT_CONSISTENT",
                 "Plate weight agrees with exported mass within admitted simulated-device calibration bounds.", SIMULATED, metrics)


def evaluate_i1(submission: dict, *, force_authority: MetrologyAuthority | None = None,
                lpt_authority: MetrologyAuthority | None = None) -> Report:
    if not isinstance(submission, dict):
        finding = _rej("SUBMISSION_SCHEMA", "TOP_LEVEL_NOT_MAPPING", "Submission must be a mapping.")
        return Report("", (finding,), (),
                      {"attempted": 1, "eligible": 1, "admitted": 0, "rejected": 1,
                       "not_evaluable": 0, "out_of_context": 0})
    clean, refused = _filter(submission)
    claims = clean.get("claims", [])
    if (not isinstance(claims, list) or any(not isinstance(c, str) for c in claims)
            or len(set(claims)) != len(claims)):
        finding = _rej("SUBMISSION_SCHEMA", "INVALID_CLAIMS", "Claims must be unique strings.")
        return Report("", (finding,), tuple(refused),
                      {"attempted": 1, "eligible": 1, "admitted": 0, "rejected": 1,
                       "not_evaluable": 0, "out_of_context": 0})
    findings: list[Finding] = []
    kin = clean.get("kinematics")
    kin_problem = _validate_kinematics(kin) if kin is not None else "KINEMATICS_ABSENT"
    force = clean.get("force")
    lpt = clean.get("lpt")
    force_problem = _validate_device(force, "FORCE") if force is not None else "FORCE_ABSENT"
    lpt_problem = _validate_device(lpt, "LPT") if lpt is not None else "LPT_ABSENT"

    mechanics = {
        "NEWTON_LINEAR_MOMENTUM": _newton,
        "EULER_ANGULAR_MOMENTUM": _euler,
        "CONTACT_ADMISSIBILITY": _contact,
        "BALLISTIC_FLIGHT": _flight,
        "WORK_ENERGY_LEDGER": _energy,
        "ACTIVATION_WITHIN_DECLARED_SCOPE": _activation,
    }
    for claim in claims:
        if claim not in CLAIMS:
            findings.append(Finding(claim, "OUT_OF_CONTEXT", "UNKNOWN_CLAIM",
                                    "Claim not defined in this benchmark.", NUMERICAL))
            continue
        if claim in mechanics:
            if kin_problem is not None:
                abstain = kin_problem in ("KINEMATICS_ABSENT", "TOO_FEW_FRAMES")
                findings.append((_ne if abstain else _rej)(claim, kin_problem,
                    "Kinematic export unusable for this claim.", NUMERICAL))
                continue
            try:
                finding = mechanics[claim](kin)
            except (ArithmeticError, KeyError, TypeError, ValueError, IndexError):
                finding = _rej(claim, "EVALUATION_INPUT_INVALID",
                               "Validated input caused an invalid numerical operation.", NUMERICAL)
            findings.append(finding if _finite_tree(finding.metrics) else
                            _rej(claim, "NONFINITE_EVALUATION_RESULT", "A numerical result was nonfinite.", NUMERICAL))
        elif claim == "LPT_CABLE_GEOMETRY":
            if lpt is None or kin_problem is not None:
                findings.append(_ne(claim, "LPT_OR_KINEMATICS_ABSENT",
                                    "LPT export and attachment kinematics required.", SIMULATED))
                continue
            if lpt_problem:
                findings.append(_rej(claim, "DEVICE_SCHEMA_INVALID", lpt_problem, SIMULATED))
                continue
            authority_problem = _authority_problem(lpt_authority, lpt, "LPT")
            if authority_problem:
                findings.append(_authority_finding(claim, authority_problem))
                continue
            try:
                finding = _lpt(kin, lpt, lpt_authority)
            except (ArithmeticError, KeyError, TypeError, ValueError, IndexError):
                finding = _rej(claim, "EVALUATION_INPUT_INVALID",
                               "Validated LPT data caused an invalid numerical operation.", SIMULATED)
            findings.append(finding if _finite_tree(finding.metrics) else
                            _rej(claim, "NONFINITE_EVALUATION_RESULT", "A numerical result was nonfinite.", SIMULATED))
        elif claim == "CLOCK_INTEGRITY":
            devices = [(n, clean[n]) for n in ("force", "lpt") if clean.get(n) is not None]
            if not devices:
                findings.append(_ne(claim, "NO_DEVICE_EXPORT", "No device export supplied.", SIMULATED))
                continue
            problems = [p for p in (force_problem if n == "force" else lpt_problem for n, _ in devices) if p]
            findings.append(
                _rej(claim, "CLOCK_INCONSISTENT", "; ".join(problems), SIMULATED) if problems
                else _pass(claim, "NOMINAL_CLOCKS_CONSISTENT",
                           "Indices and timestamps agree with the finite positive nominal clocks.",
                           SIMULATED, metrics={"devices": len(devices)})
            )
        elif claim == "FORCE_FLIGHT_ZERO":
            if force is None:
                findings.append(_ne(claim, "FORCE_ABSENT", "No FORCE export.", SIMULATED))
                continue
            if force_problem:
                findings.append(_rej(claim, "DEVICE_SCHEMA_INVALID", force_problem, SIMULATED))
                continue
            authority_problem = _authority_problem(force_authority, force, "FORCE")
            if authority_problem:
                findings.append(_authority_finding(claim, authority_problem))
                continue
            try:
                finding = _force_flight(force, force_authority, kin if kin_problem is None else None)
            except (ArithmeticError, KeyError, TypeError, ValueError, IndexError):
                finding = _rej(claim, "EVALUATION_INPUT_INVALID",
                               "Validated FORCE data caused an invalid numerical operation.", SIMULATED)
            findings.append(finding if _finite_tree(finding.metrics) else
                            _rej(claim, "NONFINITE_EVALUATION_RESULT", "A numerical result was nonfinite.", SIMULATED))
        elif claim == "PLATE_WEIGHT_CONSISTENCY":
            if force is None or kin_problem is not None:
                findings.append(_ne(claim, "FORCE_OR_KINEMATICS_ABSENT",
                                    "FORCE export and kinematic masses required.", SIMULATED))
                continue
            if force_problem:
                findings.append(_rej(claim, "DEVICE_SCHEMA_INVALID", force_problem, SIMULATED))
                continue
            authority_problem = _authority_problem(force_authority, force, "FORCE")
            if authority_problem:
                findings.append(_authority_finding(claim, authority_problem))
                continue
            try:
                finding = _plate_weight(force, kin, force_authority)
            except (ArithmeticError, KeyError, TypeError, ValueError, IndexError):
                finding = _rej(claim, "EVALUATION_INPUT_INVALID",
                               "Validated FORCE data caused an invalid numerical operation.", SIMULATED)
            findings.append(finding if _finite_tree(finding.metrics) else
                            _rej(claim, "NONFINITE_EVALUATION_RESULT", "A numerical result was nonfinite.", SIMULATED))
        elif claim == "BILATERAL_FORCE":
            force_data = force if isinstance(force, dict) else {}
            has_lr = "left_fz" in force_data or "right_fz" in force_data
            lumped = "LUMPED" in str(force_data.get("support_coverage", "")).upper()
            if has_lr and lumped:
                findings.append(_rej(claim, "BILATERAL_FROM_LUMPED_SUPPORT",
                                     "Left/right channels cannot come from a single lumped-support plate.", SIMULATED))
            else:
                findings.append(_ne(claim, "BILATERAL_NOT_IDENTIFIABLE",
                                    "No independently measured per-foot channels.", SIMULATED))
        elif claim == "CROSS_DEVICE_SYNC":
            findings.append(_ne(claim, "NO_SHARED_TRIGGER",
                                "Independent native clocks without a shared trigger cannot establish synchronization.", SIMULATED))
        elif claim == "BIOLOGICAL_VALIDITY":
            findings.append(_ne(claim, "NO_QUALIFIED_HUMAN_REFERENCE",
                                "No protocol-matched, rights-qualified human reference admitted.", BIOLOGICAL))
        elif claim == "SENSOR_METROLOGY":
            findings.append(_ne(claim, "NO_CALIBRATION_REFERENCE",
                                "No independently admitted calibration reference; simulated device hypothesis only.", SIMULATED))
        elif claim == "REAL_PAIRED_FORCE_LPT":
            findings.append(_ne(claim, "NO_SYNCHRONIZED_REAL_PAIR",
                                "No calibrated synchronized real FORCE/LPT pair admitted.", REAL_PAIRED))
    counts = {
        "attempted": len(findings),
        "eligible": sum(f.outcome in ("PASS", "REJECTED") for f in findings),
        "admitted": sum(f.outcome == "PASS" for f in findings),
        "rejected": sum(f.outcome == "REJECTED" for f in findings),
        "not_evaluable": sum(f.outcome == "NOT_EVALUABLE" for f in findings),
        "out_of_context": sum(f.outcome == "OUT_OF_CONTEXT" for f in findings),
    }
    candidate_id = clean.get("candidate_id", "")
    if not isinstance(candidate_id, str):
        candidate_id = ""
        refused.append("candidate_id: expected string")
    return Report(candidate_id, tuple(findings), tuple(refused), counts)
