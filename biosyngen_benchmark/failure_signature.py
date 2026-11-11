"""RES-624 Gate D: predeclared physical failure signatures for negative cases.

A generic refusal on both platforms is not evidence that both platforms failed
the same way. This module derives, from the exported frames and the
independent evaluator's own result, a compact signature that distinguishes the
*required* failure semantics from incidental post-impact consequences:

* expected and observed acceptance category and the primary failure reason;
* the first failure event (category, reason, time) and, where applicable, the
  first contact-loss time;
* the longest pre-failure interval that still qualifies, with its support
  ratio, per-hand shares, bar displacement and relative pose;
* the model/solver/source identity that produced it.

Only these fields are compared across platforms. Secondary failure categories,
post-impact trajectories, force chatter and the exact impact time after the
required mechanism are reported but not gated; a different primary mechanism,
a different first contact-loss event beyond the declared tolerance, a
different pre-failure support state, a different source/solver identity or a
missing signature is a comparison failure.

NUMERICAL_VERIFICATION_ONLY.
"""
from __future__ import annotations

from .grasp_contract import classify_frame
from .negative_signature import reason_code
from .receiving_acceptance import evaluate_receiving

GRIP_ACTUATORS = ("_finger_flex", "_finger_distal", "_thumb_opposition")
SETTLE_S = .1


def _first_contact_loss(frames):
    for f in frames:
        ledger = classify_frame(f)
        if not any(r["active"] for r in ledger["contacts"]):
            return float(f["t"])
    return None


def _first_saturation(frames):
    for f in frames:
        a = f.get("actuators")
        if not a:
            continue
        for name, force, cap in zip(a["names"], a["force"], a["capacity"]):
            if abs(force) >= cap*(1-1e-9) and not name.endswith(GRIP_ACTUATORS):
                return float(f["t"])
    return None


def _longest_qualified_prefix(frames, dt_s, static_verdict, onset_s):
    """Evaluate the prefix before the first failure event; return duration and metrics."""
    if onset_s is None:
        return dict(duration_s=None, outcome=None, support_ratio=None, bar_displacement_m=None)
    end = float(onset_s)-2.*dt_s
    if end <= SETTLE_S+dt_s:
        return dict(duration_s=0.0, outcome=None, support_ratio=None, bar_displacement_m=None)
    prefix = [f for f in frames if f["t"] <= end]
    if len(prefix) < 2:
        return dict(duration_s=0.0, outcome=None, support_ratio=None, bar_displacement_m=None)
    r = evaluate_receiving(prefix, dt_s, static_verdict)
    ev = r.get("evidence", {})
    pose = ev.get("relative_bar_pose") or {}
    return dict(duration_s=float(prefix[-1]["t"]-prefix[0]["t"]),
                outcome=r["outcome"], support_ratio=ev.get("hand_support_impulse_ratio"),
                support_share=ev.get("hand_support_share"),
                bar_displacement_m=ev.get("bar_displacement_in_hand_m"),
                relative_tilt_deg=(pose.get("max_tilt_deg") or None),
                active_frame_fraction=ev.get("hand_active_frame_fraction"))


def case_failure_signature(name, frames, dt_s, static_verdict, result, expected, receipt, *, onset_s=None, required_code=None):
    """Signature of one dynamic negative case (release, capacity, ...)."""
    failures = result.get("failures") or []
    primary = failures[0] if failures else {}
    first_events = (result.get("evidence") or {}).get("first_events") or []
    code = required_code or (str(expected).split(":")[-1] if str(expected).startswith("REFUSED:") else None)
    matches = [e for e in first_events if e.get("code") == code] if code else []
    required_event = min(matches, key=lambda e: e.get("t_s", 0.)) if matches else None
    first = required_event or result.get("first_failure") or {}
    contact_loss = _first_contact_loss(frames)
    saturation = _first_saturation(frames)
    onset = float(onset_s) if onset_s is not None else (first.get("t_s") if first.get("t_s") is not None else
                                                        min([t for t in (contact_loss, saturation) if t is not None], default=None))
    prefix = _longest_qualified_prefix(frames, dt_s, static_verdict, onset)
    code = primary.get("outcome") or result.get("outcome")
    reason = reason_code(primary.get("reason", result.get("reason")))
    first_code = first.get("code") or code
    first_reason = reason_code(first.get("reason") or reason)
    models = (receipt or {}).get("models") or {}
    model = models.get(name) or {}
    return dict(case=name, expected=expected, observed=result.get("outcome"), reason=reason,
                first_event=dict(code=first_code, reason=first_reason, t_s=first.get("t_s", onset)),
                contact_loss_s=contact_loss, actuator_saturation_s=saturation,
                qualified_prefix=prefix,
                secondary_failures=[f"{f.get('outcome')}:{f.get('reason')}" for f in failures[1:]],
                commanded_transition_s=onset_s,
                identity=dict(head=(receipt or {}).get("head"), mujoco=(receipt or {}).get("mujoco"),
                              scipy=(receipt or {}).get("scipy"), numpy=(receipt or {}).get("numpy"),
                              evaluator=(receipt or {}).get("evaluator", {}).get("sha256"),
                              model_parameter_set_sha256=model.get("parameter_set_sha256"),
                              controller=(receipt or {}).get("controller", {}).get("identity")))


def adversarial_signature(entry):
    """Signature of one frame-level adversarial control (no trajectory)."""
    evidence = entry.get("evidence") or {}
    primary = entry.get("reason")
    return dict(expected=entry.get("expected"), observed=entry.get("observed"), reason=reason_code(primary),
                identity_fields=dict(min_grasp_contacts=evidence.get("min_grasp_contacts"),
                                     min_grasp_active_contacts=evidence.get("min_grasp_active_contacts"),
                                     support_ratio=evidence.get("hand_support_impulse_ratio"),
                                     identity_problems=len(evidence.get("grasp_identity_problems") or [])))
