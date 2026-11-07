"""Export an I2 trajectory to the evaluator submission schema (RES-498).

The submission carries what an independent checker may see: body inertial
parameters, states at interval ends, the interval-average forces at each
contact point, and impact impulses. Each contact is tagged with its support
group (FOOT_LEFT, FOOT_RIGHT, BENCH, BAR_FLOOR, RACK) or INTERNAL for
body-to-body contacts. Latent event labels, controller stages and solver
internals are not exported.
"""

from __future__ import annotations

from ..multibody import ContactKind
from .simulate import ExerciseTrajectory

SCHEMA_ID = "BIOSYNGEN_I2_SUBMISSION_V1"


def support_group(trajectory: ExerciseTrajectory, contact_id: str) -> str:
    spec = trajectory.model.system._contact_by_id[contact_id]
    if spec.body_b_id is not None:
        return "INTERNAL"
    if spec.kind in (ContactKind.FOOT, ContactKind.HEEL, ContactKind.TOE):
        return f"FOOT_{spec.side}"
    return {ContactKind.BENCH: "BENCH", ContactKind.BAR_FLOOR: "BAR_FLOOR", ContactKind.RACK: "RACK"}.get(spec.kind, "OTHER")


def export_submission(trajectory: ExerciseTrajectory) -> dict:
    system = trajectory.model.system
    body_ids = [b.body_id for b in system.bodies]
    groups = {spec.contact_id: support_group(trajectory, spec.contact_id) for spec in system.contacts}
    friction = {}
    for spec in system.contacts:
        group = groups[spec.contact_id]
        if group != "INTERNAL":
            friction[group] = spec.friction_coefficient
    frames = []
    for frame in trajectory.frames:
        states = []
        for body_id in body_ids:
            b = frame.state.bodies[body_id]
            states.append([b.position_m[0], b.position_m[1], b.angle_rad, b.velocity_m_s[0], b.velocity_m_s[1],
                           b.angular_velocity_rad_s])
        contacts = [
            {"name": c, "group": groups[c], "x": s.point_world_m[0], "z": s.point_world_m[1],
             "fx": s.force_world_n[0], "fz": s.force_world_n[1]}
            for c, s in sorted(frame.contact_forces.items())
        ]
        frames.append({"t0": frame.t_start_s, "t1": frame.t_end_s, "body_states": states, "contacts": contacts})
    initial = []
    for body_id in body_ids:
        b = trajectory.initial_state.bodies[body_id]
        initial.append([b.position_m[0], b.position_m[1], b.angle_rad, b.velocity_m_s[0], b.velocity_m_s[1],
                        b.angular_velocity_rad_s])
    impulses = [
        {"t": i.time_s, "name": i.contact_id, "group": groups[i.contact_id], "x": i.point_world_m[0],
         "z": i.point_world_m[1], "jx": i.impulse_world_n_s[0], "jz": i.impulse_world_n_s[1]}
        for i in trajectory.impacts
    ]
    declared = sorted({g for g in groups.values() if g != "INTERNAL"})
    return {
        "schema": SCHEMA_ID,
        "exercise_id": trajectory.exercise_id,
        "gravity_m_s2": system.gravity_m_s2,
        "bodies": [{"name": b.body_id, "mass_kg": b.mass_kg, "inertia_kg_m2": b.inertia_kg_m2} for b in system.bodies],
        "initial_body_states": initial,
        "frames": frames,
        "impulses": impulses,
        "external_support_groups": declared,
        "friction_by_group": friction,
        "claims": ["LINEAR_MOMENTUM_CLOSURE", "CONTACT_ADMISSIBILITY", "BILATERAL_FORCE", "ANATOMICAL_PHASES"],
    }
