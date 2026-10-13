"""I1 candidate-neutral evaluator over public jump exports (RES-485).

Inputs are plain dicts with a strict, documented schema; see ``SCHEMA``. The
evaluator imports nothing from the generator. Unknown fields are refused,
never read, and listed, so a hidden-truth canary (latent events, phase
labels, solver internals) cannot influence any outcome.

Independent reference formulations (not the generator's Lagrangian/KKT path):

* NEWTON_LINEAR_MOMENTUM: over windows of exported frames,
  sum_i m_i (v_i(tb) - v_i(ta)) = integral(sum contact F + M g) dt.
  The integral uses the trapezoid rule. Its truncation error is bounded from
  the data: the larger of the Richardson difference 4|I_h - I_2h| and a
  sample-curvature bound sum h^2 |delta slope| / 12, which stays valid across
  kinks. The tolerance is therefore data-derived, not a tuned constant.
* EULER_ANGULAR_MOMENTUM: dH_O/dt = sum r_i x m_i g + sum r_c x F_c about the
  lab origin, in the same window/Richardson form.
* CONTACT_ADMISSIBILITY: no tensile normal force; zero contact force when
  there are no contacts; COP inside the declared foot support.
* BALLISTIC_FLIGHT: with no contacts, the system COM accelerates at exactly
  -g and angular momentum about the COM is constant (Richardson-bounded).
* WORK_ENERGY_LEDGER: KE + PE_g + declared elastic energy changes by the
  integral of declared joint + dissipation power plus the contact power
  F . v (recomputed).
* ACTIVATION_WITHIN_DECLARED_SCOPE: activation changes respect the rate
  bounds implied by the declared first-order time constants.
* LPT_CABLE_GEOMETRY: displacement = |p - a| - |p0 - a| at the device sample
  times, within encoder + timing-uncertainty bounds; slack samples are
  excluded and counted.
* CLOCK_INTEGRITY: contiguous indices and nominal native timestamps.
* FORCE_FLIGHT_ZERO: plate force after the detected takeoff lies inside the
  baseline-derived noise floor plus the declared zero-offset bound.
* BILATERAL_FORCE, CROSS_DEVICE_SYNC, BIOLOGICAL_VALIDITY, SENSOR_METROLOGY and
  REAL_PAIRED_FORCE_LPT: refused without the required qualified reference.

Outcomes are claim-level: PASS, REJECTED, NOT_EVALUABLE or OUT_OF_CONTEXT. No
aggregate realism score is computed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from .detectors import detect_force_events


KNOWN_ANSWER = "KNOWN_ANSWER_VERIFICATION"
MODELED = "MODELED_NOT_EMPIRICALLY_VALIDATED"
EMPIRICAL_BIO = "EMPIRICAL_BIOLOGICAL_VALIDATION"
METROLOGY = "SENSOR_METROLOGY_VALIDATION"

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
        "gravity_m_s2", "bodies", "frames", "declared_impulsive_events", "activation_scope",
        "declared_friction_coefficient",
    },
    "frame": {
        "t", "body_states", "contacts", "foot_support_x_m", "joint_power_w",
        "internal_dissipation_power_w", "internal_elastic_energy_j", "activations", "lpt_attachment_m",
    },
    "force": {
        "device_class", "sample_period_s", "native_time_offset_s", "timing_uncertainty_bound_s",
        "zero_offset_bound_n", "gain_uncertainty_bound", "support_coverage", "units", "positive_z",
        "samples", "left_fz", "right_fz",
    },
    "lpt": {
        "device_class", "sample_period_s", "native_time_offset_s", "timing_uncertainty_bound_s",
        "encoder_resolution_m", "anchor_m", "attachment", "quantity", "units", "samples",
    },
}

WINDOW_FRAMES = 21  # odd, so the 2h trapezoid uses every other frame
ROUNDOFF_FLOOR = 1e-9


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


def _ne(claim, code, reason, ceiling=KNOWN_ANSWER, metrics=None):
    return Finding(claim, "NOT_EVALUABLE", code, reason, ceiling, metrics or {})


def _rej(claim, code, reason, ceiling=KNOWN_ANSWER, metrics=None):
    return Finding(claim, "REJECTED", code, reason, ceiling, metrics or {})


def _pass(claim, code, reason, ceiling=KNOWN_ANSWER, metrics=None):
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
        frames = []
        for i, frame in enumerate(k2.get("frames", [])):
            f2 = {}
            for key, value in frame.items():
                if key in SCHEMA["frame"]:
                    f2[key] = value
                elif f"kinematics.frames[*].{key}" not in refused:
                    refused.append(f"kinematics.frames[*].{key}")
            frames.append(f2)
        k2["frames"] = frames
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
            clean[dev] = d2
    return clean, refused


# Mechanics helpers -------------------------------------------------------------
def _trap(ts, ys):
    return sum(0.5 * (ys[i] + ys[i + 1]) * (ts[i + 1] - ts[i]) for i in range(len(ts) - 1))


def _quadrature(ts, ys):
    """Trapezoid integral and a data-derived bound on its truncation error.

    The bound is the larger of (i) the Richardson estimate 4|I_h - I_2h| and
    (ii) a curvature bound sum_i h_i^2 |delta slope_i| / 12 from the exported
    samples. Term (ii) stays valid across kinks (C0 but not C1 signals, e.g.
    at an excitation switch), where Richardson alone underestimates the error.
    """
    ih = _trap(ts, ys)
    rich = 4.0 * abs(ih - _trap(ts[::2], ys[::2]))
    slopes = []
    for i in range(len(ts) - 1):
        h = ts[i + 1] - ts[i]
        slopes.append((ys[i + 1] - ys[i]) / h if h > 0.0 else 0.0)
    curvature = 0.0
    for i in range(len(slopes)):
        h = ts[i + 1] - ts[i]
        left = abs(slopes[i] - slopes[i - 1]) if i > 0 else 0.0
        right = abs(slopes[i + 1] - slopes[i]) if i + 1 < len(slopes) else 0.0
        curvature += h * h * max(left, right) / 12.0
    return ih, max(rich, curvature)


def _windows(frames, impulses):
    """Yield index windows that do not straddle a declared impulsive event."""
    impulse_times = [e["t"] for e in impulses]
    out, excluded = [], 0
    start = 0
    n = len(frames)
    while start + WINDOW_FRAMES <= n:
        idx = list(range(start, start + WINDOW_FRAMES))
        t0, t1 = frames[idx[0]]["t"], frames[idx[-1]]["t"]
        if any(t0 - 1e-12 <= ti <= t1 + 1e-12 for ti in impulse_times):
            excluded += 1
        else:
            out.append(idx)
        start += WINDOW_FRAMES - 1
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


def _validate_kinematics(kin) -> str | None:
    try:
        g = kin["gravity_m_s2"]
        if not (isinstance(g, (int, float)) and 9.7 < g < 9.9):
            return "GRAVITY_OUT_OF_CONTEXT"
        bodies = kin["bodies"]
        if not bodies or any(b["mass_kg"] <= 0 or b["inertia_kg_m2"] < 0 for b in bodies):
            return "INVALID_INERTIAL_PROPERTIES"
        frames = kin["frames"]
        if len(frames) < WINDOW_FRAMES:
            return "TOO_FEW_FRAMES"
        prev = -math.inf
        for f in frames:
            if f["t"] < prev or len(f["body_states"]) != len(bodies):
                return "FRAME_STRUCTURE_INVALID"
            prev = f["t"]
            for st in f["body_states"]:
                if len(st) != 6 or not all(math.isfinite(v) for v in st):
                    return "NONFINITE_OR_MALFORMED_STATE"
    except (KeyError, TypeError):
        return "KINEMATIC_SCHEMA_INCOMPLETE"
    return None


def _newton(kin):
    claim = "NEWTON_LINEAR_MOMENTUM"
    frames, g = kin["frames"], kin["gravity_m_s2"]
    mass = _total_mass(kin)
    windows, excluded = _windows(frames, kin.get("declared_impulsive_events", []))
    worst_ratio, worst_res = 0.0, 0.0
    for idx in windows:
        ts = [frames[i]["t"] for i in idx]
        if ts[-1] - ts[0] <= 0.0:
            continue
        for axis in (0, 1):
            fs = [sum(c["fx" if axis == 0 else "fz"] for c in frames[i]["contacts"]) - (mass * g if axis == 1 else 0.0)
                  for i in idx]
            ih, qerr = _quadrature(ts, fs)
            p0 = _momentum(kin, frames[idx[0]])[axis]
            p1 = _momentum(kin, frames[idx[-1]])[axis]
            res = (p1 - p0) - ih
            scale = mass * g * (ts[-1] - ts[0]) + abs(p0) + abs(p1)
            tol = qerr + 1e-7 * scale + ROUNDOFF_FLOOR
            worst_ratio = max(worst_ratio, abs(res) / tol)
            worst_res = max(worst_res, abs(res))
    metrics = {"windows": len(windows), "excluded_impulse_windows": excluded,
               "max_residual_n_s": worst_res, "max_residual_over_tolerance": worst_ratio}
    if not windows:
        return _ne(claim, "NO_ADMISSIBLE_WINDOW", "No impulse-free window of frames.", metrics=metrics)
    if worst_ratio > 1.0:
        return _rej(claim, "MOMENTUM_IMBALANCE",
                    "Exported contact impulses do not account for the exported momentum change.", metrics=metrics)
    return _pass(claim, "MOMENTUM_BALANCED",
                 "Linear momentum change equals contact plus gravity impulse within the Richardson bound.",
                 metrics=metrics)


def _euler(kin):
    claim = "EULER_ANGULAR_MOMENTUM"
    frames, g = kin["frames"], kin["gravity_m_s2"]
    windows, excluded = _windows(frames, kin.get("declared_impulsive_events", []))
    worst_ratio, worst_res = 0.0, 0.0
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
        ih, qerr = _quadrature(ts, torque)
        h0, h1 = _ang_momentum(kin, frames[idx[0]]), _ang_momentum(kin, frames[idx[-1]])
        res = (h1 - h0) - ih
        tol = qerr + 1e-7 * (abs(h0) + abs(h1) + sum(abs(t) for t in torque) * (ts[-1] - ts[0]) / len(ts)) + ROUNDOFF_FLOOR
        worst_ratio = max(worst_ratio, abs(res) / tol)
        worst_res = max(worst_res, abs(res))
    metrics = {"windows": len(windows), "excluded_impulse_windows": excluded,
               "max_residual_n_m_s": worst_res, "max_residual_over_tolerance": worst_ratio}
    if not windows:
        return _ne(claim, "NO_ADMISSIBLE_WINDOW", "No impulse-free window of frames.", metrics=metrics)
    if worst_ratio > 1.0:
        return _rej(claim, "ANGULAR_MOMENTUM_IMBALANCE",
                    "External moments do not account for the angular momentum change.", metrics=metrics)
    return _pass(claim, "ANGULAR_MOMENTUM_BALANCED",
                 "Angular momentum change equals external moment impulse within the Richardson bound.", metrics=metrics)


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
    windows, excluded = _windows(frames, kin.get("declared_impulsive_events", []))

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
    for idx in windows:
        ts = [frames[i]["t"] for i in idx]
        ps = [power(frames[i]) for i in idx]
        ih, qerr = _quadrature(ts, ps)
        res = energy(frames[idx[-1]]) - energy(frames[idx[0]]) - ih
        tol = qerr + 1e-7 * (sum(abs(p) for p in ps) * (ts[-1] - ts[0]) / len(ts) + abs(energy(frames[idx[0]]))) + ROUNDOFF_FLOOR
        worst_ratio = max(worst_ratio, abs(res) / tol)
        worst = max(worst, abs(res))
    metrics = {"windows": len(windows), "excluded_impulse_windows": excluded,
               "max_residual_j": worst, "max_residual_over_tolerance": worst_ratio}
    if not windows:
        return _ne(claim, "NO_ADMISSIBLE_WINDOW", "No impulse-free window.", metrics=metrics)
    if worst_ratio > 1.0:
        return _rej(claim, "ENERGY_NOT_ACCOUNTED", "Mechanical energy changes without declared or contact work.", metrics=metrics)
    return _pass(claim, "ENERGY_ACCOUNTED", "Energy change equals declared plus contact work.", metrics=metrics)


def _activation(kin):
    claim = "ACTIVATION_WITHIN_DECLARED_SCOPE"
    scope = kin.get("activation_scope")
    frames = kin["frames"]
    if not scope or any("activations" not in f for f in frames):
        return _ne(claim, "NO_DECLARED_ACTIVATION_SCOPE", "Activations and a declared dynamic scope are required.", MODELED)
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
                    "Activation changes faster than the declared first-order dynamics allow.", MODELED, metrics)
    return _pass(claim, "WITHIN_DECLARED_SCOPE",
                 "Activation respects the declared (uncalibrated) dynamics; this is not physiological validation.",
                 MODELED, metrics)


def _clock(dev, name):
    try:
        period = dev["sample_period_s"]
        offset = dev["native_time_offset_s"]
        for k, s in enumerate(dev["samples"]):
            if s["index"] != k:
                return f"{name}: indices not contiguous"
            if abs(s["t"] - (offset + k * period)) > 1e-9 * max(1.0, abs(s["t"])):
                return f"{name}: native time off the declared clock at index {k}"
    except (KeyError, TypeError):
        return f"{name}: clock schema incomplete"
    return None


def _lpt(kin, lpt):
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
    tol = lpt["encoder_resolution_m"] * 2 + vmax * lpt["timing_uncertainty_bound_s"] + 1e-6
    samples = lpt["samples"]
    l0 = length(point_at(samples[0]["t"] - lpt["native_time_offset_s"]))
    worst, compared, slack = 0.0, 0, 0
    for s in samples:
        if s["displacement"] is None:
            continue
        if s.get("slack"):
            slack += 1
            continue
        t = s["t"] - lpt["native_time_offset_s"]
        expected = length(point_at(t)) - l0
        worst = max(worst, abs(s["displacement"] - expected))
        compared += 1
    metrics = {"compared_samples": compared, "slack_excluded": slack, "max_error_m": worst, "tolerance_m": tol}
    if compared == 0:
        return _ne(claim, "NO_COMPARABLE_LPT_SAMPLE", "No taut, present LPT sample.", metrics=metrics)
    if worst > tol:
        return _rej(claim, "CABLE_GEOMETRY_MISMATCH", "Cable displacement conflicts with anchor/attachment geometry.", metrics=metrics)
    return _pass(claim, "CABLE_GEOMETRY_CONSISTENT", "Cable displacement matches the declared 3-D geometry.", metrics=metrics)


def _force_flight(force, kin=None):
    """Plate force over the flight interval must sit at the noise floor.

    The interval is the contact-free span of the kinematic export when one is
    supplied (cross-modal check, mapped with the declared nominal offset and
    shrunk by the timing bound). Otherwise it starts 20 ms after the
    force-detected takeoff.
    """
    claim = "FORCE_FLIGHT_ZERO"
    samples = force["samples"]
    times = [s["t"] - force["native_time_offset_s"] for s in samples]
    det = detect_force_events(times, [s["fz"] for s in samples])
    if det["body_weight"].refusal is not None:
        return _ne(claim, "NO_QUIET_BASELINE", det["body_weight"].refusal, METROLOGY)
    source = "force_detected_takeoff"
    if kin is not None and _validate_kinematics(kin) is None:
        free = [f["t"] for f in kin["frames"] if not f["contacts"]]
        if not free:
            return _ne(claim, "NO_DECLARED_FLIGHT", "Kinematic export has no contact-free frames.", METROLOGY)
        margin = force.get("timing_uncertainty_bound_s", 0.0) + 2 * force["sample_period_s"]
        lo, hi = min(free) + margin, max(free) - margin
        source = "kinematic_contact_free_interval"
    else:
        if det["takeoff"].refusal is not None:
            return _ne(claim, "NO_DETECTABLE_TAKEOFF", f"Detector refused: {det['takeoff'].refusal}", METROLOGY)
        lo, hi = det["takeoff"].native_time_s + 0.02, math.inf
    # Baseline SD from the quiet window (same convention as the detector).
    quiet = [s["fz"] for s, t in zip(samples, times) if t - times[0] <= 0.25 and s["fz"] is not None]
    mean_q = sum(quiet) / len(quiet)
    sd = math.sqrt(sum((v - mean_q) ** 2 for v in quiet) / (len(quiet) - 1))
    flight = [s["fz"] for s, t in zip(samples, times) if lo < t < hi and s["fz"] is not None]
    if len(flight) < 10:
        return _ne(claim, "TOO_FEW_FLIGHT_SAMPLES", "Fewer than 10 flight samples.", METROLOGY)
    mean_f = sum(flight) / len(flight)
    bound = 6.0 * sd + force.get("zero_offset_bound_n", 0.0)
    metrics = {"flight_samples": len(flight), "flight_mean_n": mean_f, "bound_n": bound, "baseline_sd_n": sd,
               "interval_source": source}
    if abs(mean_f) > bound or max(abs(v - mean_f) for v in flight) > 6.0 * sd + 1e-9:
        return _rej(claim, "SUPPORT_FORCE_DURING_FLIGHT", "Plate reports support beyond noise and offset bounds after takeoff.",
                    METROLOGY, metrics)
    return _pass(claim, "FLIGHT_FORCE_AT_NOISE_FLOOR",
                 "Post-takeoff force within baseline noise and declared zero offset (simulated device only).",
                 KNOWN_ANSWER, metrics)


def _plate_weight(force, kin):
    claim = "PLATE_WEIGHT_CONSISTENCY"
    samples = force["samples"]
    times = [s["t"] - force["native_time_offset_s"] for s in samples]
    det = detect_force_events(times, [s["fz"] for s in samples])
    if det["body_weight"].refusal is not None:
        return _ne(claim, "NO_QUIET_BASELINE", det["body_weight"].refusal, METROLOGY)
    expected = _total_mass(kin) * kin["gravity_m_s2"]
    bound = force.get("gain_uncertainty_bound", 0.0) * expected + force.get("zero_offset_bound_n", 0.0)
    measured = det["body_weight"].value
    metrics = {"plate_weight_n": measured, "model_weight_n": expected, "bound_n": bound}
    if abs(measured - expected) > bound:
        return _rej(claim, "PLATE_WEIGHT_INCONSISTENT",
                    "Plate quiet-stance force disagrees with the exported system weight beyond declared calibration bounds.",
                    KNOWN_ANSWER, metrics)
    return _pass(claim, "PLATE_WEIGHT_CONSISTENT",
                 "Plate weight agrees with exported mass within declared (assumed) calibration bounds.", KNOWN_ANSWER, metrics)


def evaluate_i1(submission: dict) -> Report:
    if not isinstance(submission, dict):
        raise TypeError("submission must be a dict following SCHEMA")
    clean, refused = _filter(submission)
    claims = clean.get("claims", [])
    findings: list[Finding] = []
    kin = clean.get("kinematics")
    kin_problem = _validate_kinematics(kin) if isinstance(kin, dict) else "KINEMATICS_ABSENT"
    for claim in claims:
        if claim not in CLAIMS:
            findings.append(Finding(claim, "OUT_OF_CONTEXT", "UNKNOWN_CLAIM", "Claim not defined in this benchmark.", KNOWN_ANSWER))
            continue
        if claim in ("NEWTON_LINEAR_MOMENTUM", "EULER_ANGULAR_MOMENTUM", "CONTACT_ADMISSIBILITY",
                     "BALLISTIC_FLIGHT", "WORK_ENERGY_LEDGER", "ACTIVATION_WITHIN_DECLARED_SCOPE"):
            if kin_problem is not None:
                findings.append(
                    (_rej if kin_problem not in ("KINEMATICS_ABSENT", "TOO_FEW_FRAMES") else _ne)(
                        claim, kin_problem, "Kinematic export unusable for this claim."
                    )
                )
                continue
            fn = {
                "NEWTON_LINEAR_MOMENTUM": _newton, "EULER_ANGULAR_MOMENTUM": _euler,
                "CONTACT_ADMISSIBILITY": _contact, "BALLISTIC_FLIGHT": _flight,
                "WORK_ENERGY_LEDGER": _energy, "ACTIVATION_WITHIN_DECLARED_SCOPE": _activation,
            }[claim]
            findings.append(fn(kin))
        elif claim == "LPT_CABLE_GEOMETRY":
            lpt = clean.get("lpt")
            if lpt is None or kin_problem is not None:
                findings.append(_ne(claim, "LPT_OR_KINEMATICS_ABSENT", "LPT export and attachment kinematics required."))
                continue
            problem = _clock(lpt, "LPT")
            findings.append(_rej(claim, "CLOCK_INCONSISTENT", problem) if problem else _lpt(kin, lpt))
        elif claim == "CLOCK_INTEGRITY":
            devices = [(n, clean[n]) for n in ("force", "lpt") if clean.get(n) is not None]
            if not devices:
                findings.append(_ne(claim, "NO_DEVICE_EXPORT", "No device export supplied."))
                continue
            problems = [p for p in (_clock(d, n.upper()) for n, d in devices) if p]
            findings.append(
                _rej(claim, "CLOCK_INCONSISTENT", "; ".join(problems)) if problems
                else _pass(claim, "NOMINAL_CLOCKS_CONSISTENT", "Indices contiguous and timestamps on the declared native clocks.",
                           metrics={"devices": len(devices)})
            )
        elif claim == "FORCE_FLIGHT_ZERO":
            force = clean.get("force")
            if force is None:
                findings.append(_ne(claim, "FORCE_ABSENT", "No FORCE export.", METROLOGY))
                continue
            problem = _clock(force, "FORCE")
            findings.append(
                _rej(claim, "CLOCK_INCONSISTENT", problem) if problem
                else _force_flight(force, kin if kin_problem is None else None)
            )
        elif claim == "PLATE_WEIGHT_CONSISTENCY":
            force = clean.get("force")
            if force is None or kin_problem is not None:
                findings.append(_ne(claim, "FORCE_OR_KINEMATICS_ABSENT", "FORCE export and kinematic masses required.", METROLOGY))
                continue
            problem = _clock(force, "FORCE")
            findings.append(_rej(claim, "CLOCK_INCONSISTENT", problem) if problem else _plate_weight(force, kin))
        elif claim == "BILATERAL_FORCE":
            force = clean.get("force") or {}
            has_lr = "left_fz" in force or "right_fz" in force
            lumped = "LUMPED" in str(force.get("support_coverage", "")).upper()
            if has_lr and lumped:
                findings.append(_rej(claim, "BILATERAL_FROM_LUMPED_SUPPORT",
                                     "Left/right channels cannot come from a single lumped-support plate.", METROLOGY))
            else:
                findings.append(_ne(claim, "BILATERAL_NOT_IDENTIFIABLE",
                                    "No independently measured per-foot channels.", METROLOGY))
        elif claim == "CROSS_DEVICE_SYNC":
            findings.append(_ne(claim, "NO_SHARED_TRIGGER",
                                "Independent native clocks without a shared trigger cannot establish synchronization.", METROLOGY))
        elif claim == "BIOLOGICAL_VALIDITY":
            findings.append(_ne(claim, "NO_QUALIFIED_HUMAN_REFERENCE",
                                "No protocol-matched, rights-qualified human reference admitted.", EMPIRICAL_BIO))
        elif claim == "SENSOR_METROLOGY":
            findings.append(_ne(claim, "NO_CALIBRATION_REFERENCE",
                                "No device calibration reference; simulated device hypothesis only.", METROLOGY))
        elif claim == "REAL_PAIRED_FORCE_LPT":
            findings.append(_ne(claim, "NO_SYNCHRONIZED_REAL_PAIR",
                                "No calibrated synchronized real FORCE/LPT pair admitted.", METROLOGY))
    counts = {
        "attempted": len(findings),
        "eligible": sum(f.outcome in ("PASS", "REJECTED") for f in findings),
        "admitted": sum(f.outcome == "PASS" for f in findings),
        "rejected": sum(f.outcome == "REJECTED" for f in findings),
        "not_evaluable": sum(f.outcome == "NOT_EVALUABLE" for f in findings),
        "out_of_context": sum(f.outcome == "OUT_OF_CONTEXT" for f in findings),
    }
    return Report(str(clean.get("candidate_id", "")), tuple(findings), tuple(refused), counts)
