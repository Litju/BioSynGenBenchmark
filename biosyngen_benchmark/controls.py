"""Benchmark-authored analytic positive controls and mutation negatives (RES-489).

The positive controls below are written from closed-form mechanics inside the
benchmark package. They share no code, parameters or model family with the
generator. The mutations operate on deep copies of public exports; they never
modify the evaluator.
"""

from __future__ import annotations

import copy
import math


G = 9.80665


def prismatic_push_control(duration_s: float = 0.4, dt: float = 0.001) -> dict:
    """Two bodies: a grounded base A and a body B pushed up by an internal force.

    A (12 kg) rests on two contacts at x = -0.1 and +0.1. An internal actuator
    applies F = 1.4 m_B g between A and B, so z_B = z0 + 0.5 (F/m_B - g) t^2
    exactly. Each contact carries (m_A g + F)/2 and the COP is at x = 0.
    """
    ma, mb, z0 = 12.0, 60.0, 1.0
    force = 1.4 * mb * G
    acc = force / mb - G
    frames = []
    n = int(round(duration_s / dt))
    for k in range(n + 1):
        t = k * dt
        zb = z0 + 0.5 * acc * t * t
        vb = acc * t
        each = 0.5 * (ma * G + force)
        frames.append({
            "t": t,
            "body_states": [[0.0, 0.05, 0.0, 0.0, 0.0, 0.0], [0.0, zb, 0.0, 0.0, vb, 0.0]],
            "contacts": [
                {"name": "a_left", "x": -0.1, "z": 0.0, "vx": 0.0, "vz": 0.0, "fx": 0.0, "fz": each},
                {"name": "a_right", "x": 0.1, "z": 0.0, "vx": 0.0, "vz": 0.0, "fx": 0.0, "fz": each},
            ],
            "foot_support_x_m": [-0.1, 0.1],
            "joint_power_w": force * vb,
            "internal_dissipation_power_w": 0.0,
            "internal_elastic_energy_j": 0.0,
        })
    return {
        "gravity_m_s2": G,
        "declared_friction_coefficient": 0.5,
        "bodies": [{"name": "base", "mass_kg": ma, "inertia_kg_m2": 0.3},
                   {"name": "pushed", "mass_kg": mb, "inertia_kg_m2": 4.0}],
        "frames": frames,
        "declared_impulsive_events": [],
    }


def tumbling_ballistic_control(duration_s: float = 0.5, dt: float = 0.001) -> dict:
    """One rigid body thrown with spin; no contact. Closed-form state."""
    m, inertia = 70.0, 9.0
    x0, z0, vx, vz0, th0, w = -0.3, 1.1, 0.4, 2.6, 0.2, 3.0
    frames = []
    n = int(round(duration_s / dt))
    for k in range(n + 1):
        t = k * dt
        frames.append({
            "t": t,
            "body_states": [[x0 + vx * t, z0 + vz0 * t - 0.5 * G * t * t, th0 + w * t, vx, vz0 - G * t, w]],
            "contacts": [],
            "joint_power_w": 0.0,
            "internal_dissipation_power_w": 0.0,
            "internal_elastic_energy_j": 0.0,
        })
    return {"gravity_m_s2": G, "bodies": [{"name": "rigid", "mass_kg": m, "inertia_kg_m2": inertia}],
            "frames": frames, "declared_impulsive_events": []}


def two_body_free_fall_control(duration_s: float = 0.4, dt: float = 0.001) -> dict:
    """Unseen two-body family with closed-form free flight and independent motion.

    Distinct masses, horizontal velocities, elevations and spins make the
    aggregate checks exercise multi-body COM and angular-momentum arithmetic
    without using generator topology or parameters.
    """
    bodies = [
        {"name": "falling_left", "mass_kg": 2.0, "inertia_kg_m2": 0.04},
        {"name": "falling_right", "mass_kg": 3.5, "inertia_kg_m2": 0.09},
    ]
    initial = [(-0.25, 1.35, 0.32, 0.40, 0.2, 1.1),
               (0.18, 1.10, -0.17, -0.05, -0.3, -0.7)]
    frames = []
    n = int(round(duration_s / dt))
    for k in range(n + 1):
        t = k * dt
        states = []
        for x0, z0, vx, vz0, theta0, omega in initial:
            states.append([x0 + vx * t, z0 + vz0 * t - 0.5 * G * t * t,
                           theta0 + omega * t, vx, vz0 - G * t, omega])
        frames.append({
            "t": t, "body_states": states, "contacts": [],
            "joint_power_w": 0.0, "internal_dissipation_power_w": 0.0,
            "internal_elastic_energy_j": 0.0,
        })
    return {"gravity_m_s2": G, "bodies": bodies, "frames": frames,
            "declared_impulsive_events": []}


# Mutations (each returns a modified deep copy) ---------------------------------
def scale_contact_forces(sub: dict, factor: float) -> dict:
    out = copy.deepcopy(sub)
    for f in out["kinematics"]["frames"]:
        for c in f["contacts"]:
            c["fx"] *= factor
            c["fz"] *= factor
    return out


def scale_body_masses(sub: dict, factor: float) -> dict:
    out = copy.deepcopy(sub)
    for b in out["kinematics"]["bodies"]:
        b["mass_kg"] *= factor
    return out


def inject_tensile_contact(sub: dict, newtons: float = -50.0) -> dict:
    out = copy.deepcopy(sub)
    frames = [f for f in out["kinematics"]["frames"] if f["contacts"]]
    for f in frames[len(frames) // 3 : len(frames) // 3 + 5]:
        f["contacts"][0]["fz"] = newtons
    return out


def shift_contacts_outside_support(sub: dict, dx: float = 0.4) -> dict:
    out = copy.deepcopy(sub)
    for f in out["kinematics"]["frames"]:
        for c in f["contacts"]:
            c["x"] += dx
    return out


def excessive_tangential_force(sub: dict) -> dict:
    out = copy.deepcopy(sub)
    for f in out["kinematics"]["frames"]:
        for c in f["contacts"]:
            c["fx"] = 2.0 * abs(c["fz"]) + 1.0
    return out


def inject_energy_in_flight(sub: dict, factor: float = 1.03) -> dict:
    out = copy.deepcopy(sub)
    frames = out["kinematics"]["frames"]
    flight = [i for i, f in enumerate(frames) if not f["contacts"]]
    if not flight:
        return out
    start = flight[len(flight) // 2]
    for f in frames[start:]:
        if f["contacts"]:
            continue
        for st in f["body_states"]:
            st[3] *= factor
            st[4] *= factor
            st[5] *= factor
    return out


def activation_step(sub: dict) -> dict:
    out = copy.deepcopy(sub)
    frames = [f for f in out["kinematics"]["frames"] if "activations" in f]
    for f in frames[len(frames) // 4 :]:
        f["activations"][2] = 0.95  # instantaneous jump of one actuator
    return out


def lpt_wrong_anchor(sub: dict, dx: float = 0.25) -> dict:
    out = copy.deepcopy(sub)
    out["lpt"]["anchor_m"] = [out["lpt"]["anchor_m"][0] + dx] + list(out["lpt"]["anchor_m"][1:])
    return out


def lpt_vertical_substitution(sub: dict) -> dict:
    """Replace cable displacement by vertical attachment displacement."""
    out = copy.deepcopy(sub)
    frames = out["kinematics"]["frames"]
    ts = [f["t"] for f in frames]
    zs = [f["lpt_attachment_m"][2] for f in frames]
    offset = out["lpt"]["native_time_offset_s"]

    def z_at(t):
        i = max(0, min(len(ts) - 2, next((k for k, tk in enumerate(ts) if tk >= t), len(ts) - 1) - 1))
        w = 0.0 if ts[i + 1] == ts[i] else (t - ts[i]) / (ts[i + 1] - ts[i])
        return zs[i] + w * (zs[i + 1] - zs[i])

    z0 = z_at(out["lpt"]["samples"][0]["t"] - offset)
    for s in out["lpt"]["samples"]:
        if s["displacement"] is not None:
            s["displacement"] = z_at(s["t"] - offset) - z0
    return out


def clock_skew(sub: dict, device: str = "force", skew_s: float = 0.003) -> dict:
    out = copy.deepcopy(sub)
    for s in out[device]["samples"][len(out[device]["samples"]) // 2 :]:
        s["t"] += skew_s
    return out


def force_in_flight(sub: dict, newtons: float = 40.0) -> dict:
    out = copy.deepcopy(sub)
    samples = out["force"]["samples"]
    peak = max(range(len(samples)), key=lambda i: samples[i]["fz"] if samples[i]["fz"] is not None else -1e9)
    for s in samples[peak:]:
        if s["fz"] is not None and s["fz"] < 30.0:
            s["fz"] += newtons
    return out


def force_in_kilonewtons(sub: dict) -> dict:
    out = copy.deepcopy(sub)
    for s in out["force"]["samples"]:
        for k in ("fx", "fz", "my"):
            if s[k] is not None:
                s[k] /= 1000.0
    return out


def fake_bilateral_split(sub: dict) -> dict:
    out = copy.deepcopy(sub)
    out["force"]["left_fz"] = [None if s["fz"] is None else 0.5 * s["fz"] for s in out["force"]["samples"]]
    out["force"]["right_fz"] = [None if s["fz"] is None else 0.5 * s["fz"] for s in out["force"]["samples"]]
    return out


def hidden_truth_canary(sub: dict) -> dict:
    """Add latent labels that an honest evaluator must refuse to read."""
    out = copy.deepcopy(sub)
    out["latent_events"] = {"takeoff_s": -1.0, "phase_labels": ["POISON"]}
    out["generator_parameters"] = {"poison": True}
    for f in out["kinematics"]["frames"]:
        f["phase"] = "POISON"
        f["latent_takeoff_flag"] = True
    return out
