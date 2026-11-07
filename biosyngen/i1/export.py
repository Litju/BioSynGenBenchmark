"""Convert I1 simulations into the public evaluator schema (plain dicts).

The export carries only quantities a measurement or a declared model would
expose: per-body inertial properties and states, contact point forces and
kinematics, declared internal power and storage, activations, device
exports and declared device geometry. It deliberately carries no phase
labels, latent events, solver internals or generator parameters.
"""

from __future__ import annotations

from .body import PlanarBody
from .instruments import ForceExport, LptExport, lpt_attachment_point
from .simulate import ContactMode, JumpTrajectory


def kinematic_export(traj: JumpTrajectory, lpt_model=None) -> dict:
    body = PlanarBody(traj.scenario.body, traj.scenario.bar)
    bodies = [
        {"name": seg.name, "mass_kg": seg.mass_kg, "inertia_kg_m2": seg.inertia_kg_m2}
        for seg in body.segments
    ]
    if body.bar is not None:
        bodies.append({"name": "bar", "mass_kg": body.bar.mass_kg, "inertia_kg_m2": 0.0})
    frames = []
    collapsed_times = []
    for s in traj.samples:
        q, qd = list(s.q), list(s.qd)
        states = []
        for seg in body.segments:
            x, z = body.point(q, seg.com_terms)
            vx, vz = body.point_velocity(q, qd, seg.com_terms)
            states.append([x, z, q[seg.angle_index], vx, vz, qd[seg.angle_index]])
        if body.bar is not None:
            states.append([s.bar_m[0], s.bar_m[1], 0.0, s.bar_velocity_m_s[0], s.bar_velocity_m_s[1], 0.0])
        contacts = []
        if s.mode is not ContactMode.FLIGHT:
            hv = body.point_velocity(q, qd, body.points["heel"])
            tv = body.point_velocity(q, qd, body.points["toe"])
            if s.mode is ContactMode.FLAT:
                contacts.append({"name": "heel", "x": s.heel_m[0], "z": s.heel_m[1], "vx": hv[0], "vz": hv[1],
                                 "fx": s.heel_force_n[0], "fz": s.heel_force_n[1]})
            contacts.append({"name": "toe", "x": s.toe_m[0], "z": s.toe_m[1], "vx": tv[0], "vz": tv[1],
                             "fx": s.toe_force_n[0], "fz": s.toe_force_n[1]})
        frame = {
            "t": s.time_s,
            "body_states": states,
            "contacts": contacts,
            "foot_support_x_m": [s.heel_m[0], s.toe_m[0]],
            "joint_power_w": sum(tq * w for tq, w in zip(s.joint_torque_n_m, s.joint_velocity_rad_s)),
            "internal_dissipation_power_w": 0.0,
            "internal_elastic_energy_j": 0.0,
            "activations": list(s.activation),
        }
        if body.bar is not None:
            sx, sz = body.point(q, body.points["shoulder"])
            k = body.bar.coupling_stiffness_n_m
            frame["internal_elastic_energy_j"] = 0.5 * k * ((sx - s.bar_m[0]) ** 2 + (sz - s.bar_m[1]) ** 2)
            frame["internal_dissipation_power_w"] = body.coupling_damping_power(q, qd)
        if lpt_model is not None:
            frame["lpt_attachment_m"] = list(lpt_attachment_point(s, traj, lpt_model))
        if frames and frame["t"] == frames[-1]["t"]:
            # Solver event handling can emit both one-sided contact records at
            # the same time. Keep the right-continuous record and expose the
            # boundary so the sampled-data evaluator never integrates across it.
            collapsed_times.append(frame["t"])
            frames[-1] = frame
        else:
            frames.append(frame)
    impulses = [
        {"t": e.time_s, "kind": e.kind} for e in traj.events if e.kind == "HEEL_STRIKE"
    ]
    event_boundaries = [{"t": e.time_s, "kind": e.kind} for e in traj.events
                        if e.kind in {"HEEL_OFF", "TAKEOFF", "HEEL_STRIKE", "SLIP_ONSET", "SLIP_STOP",
                                      "CONTROL_BREAKPOINT"}]
    event_boundaries.extend({"t": t, "kind": "SAME_TIME_SAMPLE_COLLAPSE"} for t in collapsed_times)
    event_boundaries.sort(key=lambda event: (event["t"], event["kind"]))
    nm = traj.scenario.nm
    return {
        "gravity_m_s2": traj.scenario.body.gravity_m_s2,
        "declared_friction_coefficient": traj.scenario.body.friction_coefficient,
        "bodies": bodies,
        "frames": frames,
        "declared_impulsive_events": impulses,
        "declared_event_boundaries": event_boundaries,
        "activation_scope": {
            "status": "ASSUMED_UNCALIBRATED",
            "tau_act_s": nm.activation_time_constant_s,
            "tau_deact_s": nm.deactivation_time_constant_s,
            "u_min": nm.minimum_excitation,
            "u_max": 1.0,
        },
    }


def force_export_dict(export: ForceExport) -> dict:
    m = export.model
    return {
        "device_class": export.metadata["output_class"],
        "sample_period_s": m.clock.sample_period_s,
        "native_time_offset_s": m.clock.offset_s,
        "timing_uncertainty_bound_s": 1e-4 * 2.0 + 6 * m.clock.jitter_sd_s,
        "zero_offset_bound_n": 5.0,
        "gain_uncertainty_bound": 0.01,
        "support_coverage": m.support_coverage,
        "units": {"force": "N", "moment": "N m", "length": "m"},
        "positive_z": "up (reaction on subject)",
        "samples": [
            {"index": s.index, "t": s.native_time_s, "fx": s.fx_n, "fz": s.fz_n, "my": s.my_n_m,
             "cop_x": s.cop_x_m, "saturated": s.saturated}
            for s in export.samples
        ],
    }


def lpt_export_dict(export: LptExport) -> dict:
    m = export.model
    return {
        "device_class": export.metadata["output_class"],
        "sample_period_s": m.clock.sample_period_s,
        "native_time_offset_s": m.clock.offset_s,
        "timing_uncertainty_bound_s": 1e-4 * 2.0 + 6 * m.clock.jitter_sd_s,
        "encoder_resolution_m": m.encoder_resolution_m,
        "anchor_m": list(m.anchor_m),
        "attachment": m.attachment,
        "quantity": "cable displacement from first sample",
        "units": {"displacement": "m", "velocity": "m/s"},
        "samples": [
            {"index": s.index, "t": s.native_time_s, "displacement": s.displacement_m,
             "provider_velocity": s.provider_velocity_m_s, "slack": s.slack}
            for s in export.samples
        ],
    }
