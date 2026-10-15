"""Context-of-use refusals over generator trajectories, not a benchmark oracle.

This inspects modeled topology/support only. It cannot admit human/device
validation or infer anatomy from exported force/LPT channels.
"""
from collections import defaultdict
from ..multibody import ConnectionKind, ContactMode
from .bench import BENCH_CONTACTS
from .human import LoadKind


def qualify_context(trajectory) -> dict:
    model = trajectory.model
    findings = {
        "biological_validation": {"outcome": "NOT_EVALUABLE", "code": "NO_EMPIRICAL_REFERENCE"},
        "real_paired_force_lpt": {"outcome": "NOT_EVALUABLE", "code": "NO_ADMITTED_REAL_PAIRED_REFERENCE"},
        "physiological_peak_force_rfd": {"outcome": "NOT_EVALUABLE", "code": "RIGID_CONTACT_AND_UNCALIBRATED_CONTROL_SENSOR_STAGES"},
    }
    result = {"termination": trajectory.termination, "findings": findings,
              "scope": "declared modeled topology only; not empirical qualification"}
    if trajectory.exercise_id == "LOADED_JUMP_SQUAT" and model.load.kind is LoadKind.BACK_BAR:
        gripped = any(j.connection_kind is ConnectionKind.GRIP
                      and model.load_body_id in (j.body_a_id, j.body_b_id) for j in model.system.joints)
        release = [f for f in trajectory.frames if f.state.contacts[model.load_contact_id].mode is ContactMode.OPEN]
        findings["ordinary_hand_stabilized_jump_squat"] = {
            "outcome": "NOT_EVALUABLE",
            "code": "HAND_STABILIZATION_NOT_IMPLEMENTED" if not gripped else "HAND_SUPPORT_NOT_EMPIRICALLY_QUALIFIED",
            "first_bar_open_state_s": release[0].t_end_s if release else None,
            "open_intervals": len(release),
        }
        result["supported_context"] = "separable shoulder-shelf contact research fixture"
    if trajectory.exercise_id == "BENCH_PRESS":
        losses = defaultdict(lambda: defaultdict(int))
        maximum_gap = dict.fromkeys(BENCH_CONTACTS, 0.0)
        for frame in trajectory.frames:
            for contact in BENCH_CONTACTS:
                if frame.state.contacts[contact].mode is ContactMode.OPEN:
                    losses[frame.controller_stage][contact] += 1
                gap = model.system.contact_kinematics(frame.state, contact).gap_m
                maximum_gap[contact] = max(maximum_gap[contact], gap)
        findings["stable_supine_support_whole_record"] = {
            "outcome": "NOT_EVALUABLE" if losses else "NUMERICAL_SUPPORT_MAINTAINED",
            "code": "BENCH_SUPPORT_TRANSITION_REQUIRES_QUALIFICATION" if losses else "DECLARED_CONTACTS_MAINTAINED",
            "open_intervals_by_stage": {stage: dict(counts) for stage, counts in losses.items()},
            "maximum_modeled_gap_m": maximum_gap,
            "note": "zero reaction/open mode is distinct from positive geometric separation; neither establishes human technique",
        }
    return result
