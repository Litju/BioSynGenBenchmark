"""RES-624 outcome-derived Human V2 receiving qualification (replaces no frozen witness).

    python -m scripts.res624_receiving_qualification [--out runs/i3/res624/qualification] [--quick]

The frozen RES-618 ``scripts/res618_fullbody_qualification.py`` (hardcoded
FAIL) is untouched and runs separately as a RED canary. This runner derives
every verdict from evidence through the frame-only evaluator
(biosyngen_benchmark.receiving_acceptance):

  1. RED canary: the frozen RES-618 candidate and hold, exported with the same
     frame schema, must be refused (static and dynamic).
  2. Forward cases of the Gate E campaign, each re-simulated from its certified
     state and evaluated independently.
  3. Adversarial frame/scene manipulations that must be refused: contacts
     deleted, declared friction violated, capacity exceeded, undeclared bar
     support, non-foot floor contact, uncertified static state.

Final verdict RECEIVING_HOLD_QUALIFIED requires: nominal holds (0.3 s RED
window and 2 s; 10 s unless --quick) qualified, the RED canary refused and
every adversarial manipulation refused with its declared code. Exit 0 iff so.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

import mujoco

from biosyngen.spatial.model_v2 import HumanV2Spec
from biosyngen.spatial.plant_v2 import HumanV2Plant
from biosyngen.spatial.receiving_grasp_v2 import (actuator_export, bar_frame, forward_receiving_hold, hand_frames,
                                                   qualify_static)
from biosyngen.spatial.receiving_v2 import solve_candidate
from biosyngen_benchmark.failure_signature import adversarial_signature, case_failure_signature
from biosyngen_benchmark.qualification_scope import REQUIRED_NOMINAL, qualify_scope
from biosyngen_benchmark.receiving_acceptance import evaluate_receiving
from scripts.res624_grasp_diagnosis import CLAIMS, source_receipt
from scripts.res624_receipt import full_receipt
from scripts.res624_receiving_campaign import build_nominal, cases, clone, run_case
from scripts import res624_adversarial_scenes as scenes

EXPECTED_QUALIFIED = ("nominal_0p3s_red_window", "nominal_2s", "nominal_10s")
# Preregistered expectation per case. "QUALIFIED": the independent evaluator must qualify the hold.
# ("REFUSED", code): the evaluator must report that failure category (a physical negative control).
# finger_release_at_1s is judged in two parts: the pre-release hold must qualify, and the full run must
# show the contact loss (GRASP_SLIP:CONTACT_LOST) caused by the commanded release.
NEGATIVE_EXPECTATION = {"finger_release_at_1s": "GRASP_SLIP", "arm_capacity_half_threshold": "ACTUATOR_INFEASIBLE"}
ANTHROPOMETRIC = ("anthropometry_70kg_1p70m", "anthropometry_80kg_1p80m")


def red_frames(duration_s=.3):
    """Exact replay of receiving_v2.forward_hold with the evaluator's frame schema."""
    plant = HumanV2Plant.build()
    solve_candidate(plant, "OVERHEAD", "POWER", 1000)
    static = qualify_static(plant)
    m, d = plant.model, plant.data
    targets = plant.joint_angles()
    frames = []
    for _ in range(round(duration_s/m.opt.timestep)):
        mujoco.mj_forward(m, d)
        ff = {name: float(d.qfrc_bias[dof]) for name, dof, _ in plant.actuated}
        plant.set_targets(targets, ff)
        d.xfrc_applied[plant.body_id("barbell"), 0] = 0
        mujoco.mj_forward(m, d)
        f = plant.export_frame()
        f["hand_frames"], f["bar_frame"], f["actuators"] = hand_frames(plant), bar_frame(plant), actuator_export(plant)
        frames.append(f)
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    f = plant.export_frame()
    f["hand_frames"], f["bar_frame"], f["actuators"] = hand_frames(plant), bar_frame(plant), actuator_export(plant)
    frames.append(f)
    return frames, static["verdict"], float(frames[0]["bodies"][-1]["com"][2]-frames[-1]["bodies"][-1]["com"][2])


def _is_hand_bar(c):
    return "barbell" in (c["body1"], c["body2"]) and (c["body1"] if c["body2"] == "barbell" else c["body2"])[:2] in ("l_", "r_")


def adversarial(frames, dt, verdict, nominal):
    out = {}

    def run(name, expected, fr, v=verdict):
        r = evaluate_receiving(fr, dt, v)
        ev = r.get("evidence", {})
        out[name] = dict(expected=expected, observed=r["outcome"], reason=str(r.get("reason")),
                         refused=r["outcome"] == expected,
                         evidence={k: ev.get(k) for k in ("hand_support_impulse_ratio", "hand_support_share",
                                                          "hand_support_force_residual_ratio",
                                                          "hand_support_moment_residual_ratio",
                                                          "min_grasp_contacts", "min_grasp_active_contacts",
                                                          "grasp_identity_problems", "nongrasp_bar_contacts_max",
                                                          "bar_angular_reference", "bar_angular_bound")},
                         first_failure=r.get("first_failure"))
        out[name]["signature"] = adversarial_signature(out[name])
    deleted = copy.deepcopy(frames)
    for f in deleted:
        f["contacts"] = [c for c in f["contacts"] if not _is_hand_bar(c)]
    run("hand_contacts_deleted", "NUMERICAL_NOT_QUALIFIED", deleted)
    friction = copy.deepcopy(frames)
    for f in friction:
        for c in f["contacts"]:
            if _is_hand_bar(c):
                c["friction"] = 1e-3
    run("declared_friction_violated", "CONTACT_INADMISSIBLE", friction)
    capacity = copy.deepcopy(frames)
    for f in capacity:
        f["actuators"]["capacity"] = [.05*c for c in f["actuators"]["capacity"]]
    run("actuator_capacity_exceeded", "ACTUATOR_INFEASIBLE", capacity)
    shank = copy.deepcopy(frames)
    for f in shank:
        for c in f["contacts"]:
            if "world" in (c["body1"], c["body2"]):
                if c["body2"] == "l_foot":
                    c["body2"] = "l_shank"
                    break
    run("non_foot_floor_contact", "RECEIVING_UNSTABLE", shank)
    run("static_not_certified", "STATIC_NOT_CERTIFIED", frames, "STATIC_GRASP_INFEASIBLE")
    # Undeclared support: a real simulation with a hidden upward bar force not exported.
    plant = clone(HumanV2Spec(), nominal["qpos"])
    static = qualify_static(plant)
    hidden = forward_receiving_hold(plant, static["certificate"], .2, push=("barbell", 0.0, 1.0, (0, 0, 30.0)))
    for f in hidden:
        f["external"] = []
    run("undeclared_bar_support", "NUMERICAL_NOT_QUALIFIED", hidden, static["verdict"])
    # Gate A false-support family: the bar-side wrench, the system force balance,
    # contact admissibility and the static verdict are preserved; only the body
    # claimed to carry the load changes.
    run("external_support_hand_load_5pct", "GRASP_UNSUPPORTED",
        scenes.redistribute_hand_load(frames, {"l": .05, "r": .05}))
    run("unilateral_support_left_only", "GRASP_NOT_BILATERAL",
        scenes.redistribute_hand_load(frames, {"l": 2., "r": 0.}))
    for segment in ("forearm", "thigh", "torso"):
        run(f"{segment}_relabelled_support", "GRASP_UNSUPPORTED", scenes.move_load_to_segment(frames, segment))
    run("segment_only_no_grasp_contact", "GRASP_GEOMETRY_INFEASIBLE", scenes.segment_only(frames, "thigh"))
    run("forged_pad_body_identity", "GRASP_IDENTITY_UNADMISSIBLE", scenes.forge_grasp_geom_body(frames))
    run("forged_pad_contact_point", "GRASP_IDENTITY_UNADMISSIBLE", scenes.forge_contact_point(frames))
    run("misassigned_contact_moment", "NUMERICAL_NOT_QUALIFIED", scenes.swap_contact_sites(frames))
    run("reversed_hand_forces", "NUMERICAL_NOT_QUALIFIED", scenes.reverse_one_hand(frames, "l"))
    # Gate B relative bar pose: the COM can stay put while the bar tilts, slips
    # along its axis or the reported hand frame is wrong.
    run("bar_tilt_10deg_zero_translation", "NUMERICAL_NOT_QUALIFIED", scenes.rotate_reported_bar_ramp(frames, tilt_deg=10.))
    run("bar_axial_slip_5mm", "GRASP_SLIP", scenes.translate_bar_axially(frames, .005))
    run("right_hand_frame_10deg_offset", "GRASP_SLIP", scenes.offset_hand_frame(frames, "r", 10., "x"))
    run("hand_frames_ramp_5deg", "GRASP_SLIP", scenes.ramp_hand_frames(frames, "r", 4., "z"))
    run("right_grip_zero_load_left_carries_all", "GRASP_NOT_BILATERAL",
        scenes.redistribute_hand_load(frames, {"l": 2., "r": 0.}))
    # Gate F: an incorrect declared bar inertia must not pass the independent angular oracle.
    run("incorrect_bar_inertia", "NUMERICAL_NOT_QUALIFIED", scenes.scale_bar_inertia(frames, 1.2))
    return out


def positive_controls(frames, dt, verdict):
    """Controls that must keep qualifying: asymmetric sharing and permitted grip motion."""
    out = {}
    asymmetric = scenes.redistribute_hand_load(frames, {"l": .6, "r": 1.4})
    r = evaluate_receiving(asymmetric, dt, verdict)
    ev = r.get("evidence", {})
    out["asymmetric_30_70_bilateral_support"] = dict(
        observed=r["outcome"], qualified=r["outcome"] == "RECEIVING_HOLD_QUALIFIED",
        support_ratio=ev.get("hand_support_impulse_ratio"), support_share=ev.get("hand_support_share"),
        note="asymmetric hand load sharing is physically admissible; only the bilateral minimum share is gated")
    small = scenes.small_legitimate_relative_motion(frames)
    r = evaluate_receiving(small, dt, verdict)
    pose = r.get("evidence", {}).get("relative_bar_pose", {})
    out["small_relative_motion_within_grip"] = dict(
        observed=r["outcome"], qualified=r["outcome"] == "RECEIVING_HOLD_QUALIFIED",
        tilt_deg=pose.get("max_tilt_deg"), contact_site_drift_m=pose.get("max_contact_site_drift_m"),
        note="0.5 deg reported tilt and 0.5 mm hand-frame translation are inside the declared grip bounds")
    return out


def relative_pose_controls(frames, dt, verdict):
    """Pose-only controls: a shaft roll is permitted by the declared grip even though a *forged* roll is
    refused end-to-end by the angular-velocity/attitude consistency check."""
    from biosyngen_benchmark.grasp_contract import classify_frame
    from biosyngen_benchmark.receiving_acceptance import relative_pose_check
    window = [f for f in frames if f["t"] >= .1]
    twist = [f for f in scenes.rotate_reported_bar_ramp(frames, twist_deg=30.) if f["t"] >= .1]
    pose = relative_pose_check(twist, "barbell", [classify_frame(f) for f in twist])
    return dict(bar_shaft_twist_30deg_pose_control=dict(
        max_twist_deg=pose["max_twist_deg"], max_tilt_deg=pose["max_tilt_deg"], twist_gated=pose["twist_gated"],
        first=pose["first"], pose_within_bounds=not pose["first"],
        note="the unplated shaft/sleeve body is axisymmetric about its local y axis, so the relative-pose criterion "
             "does not gate a shaft roll; the forged 30 deg roll is additionally refused end-to-end because the "
             "reported roll is not driven by an exported torque (NUMERICAL/attitude inconsistency)."))


def run(out, quick=False):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    _, nominal = build_nominal()
    red, red_static, red_drop = red_frames()
    red_eval = evaluate_receiving(red, .001, red_static)
    results, nominal_frames, plants, negative_frames = {}, None, {}, {}
    for name, spec, mod, dur, kw in cases(quick):
        frames, static, plant = run_case(name, spec, mod, dur, kw, nominal)
        dt = plant.model.opt.timestep
        release = kw.get("release_at_s")
        r = evaluate_receiving(frames, dt, static["verdict"])
        codes = [f"{x['outcome']}:{x['reason']}" for x in r.get("failures", [])]
        entry = dict(static_verdict=static["verdict"], outcome=r["outcome"], reason=str(r.get("reason")), all_failures=codes,
                     evidence={k: v for k, v in r["evidence"].items()
                               if k in ("bar_displacement_in_hand_m", "hand_support_ratio_mean", "hand_support_ratio_min",
                                        "min_hand_contacts", "min_cop_margin_m", "min_com_projection_margin_m",
                                        "floor_vertical_over_weight_mean", "max_penetration_m", "posture_saturation_samples",
                                        "contact_problems", "over_capacity", "min_grasp_contacts", "min_grasp_active_contacts",
                                        "hand_support_impulse_ratio", "hand_support_share",
                                        "hand_support_force_residual_ratio", "hand_support_moment_residual_ratio",
                                        "hand_active_frame_fraction", "nongrasp_bar_contacts_max",
                                        "grasp_identity_problems", "grasp_contract_problems", "relative_bar_pose",
                                        "bar_angular_reference", "bar_angular_bound")},
                     first_failure=r.get("first_failure"),
                     bar_linear_residual_n_s=r["evidence"].get("bar_newton_euler", {}).get("max_step_linear_residual_n_s"),
                     bar_angular_residual_n_m_s=r["evidence"].get("bar_angular_reference", {}).get("max_residual_n_m_s"),
                     bar_angular_bound_residual_n_m_s=r["evidence"].get("bar_angular_bound", {}).get("max_residual_n_m_s"),
                     bar_angular_tolerance_n_m_s=r["evidence"].get("bar_angular_reference", {}).get("tolerance_n_m_s"),
                     system_step_residual_n_s=r["evidence"].get("system_newton_euler", {}).get("max_step_linear_residual_n_s"),
                     bar_mass_kg=next(b for b in frames[0]["bodies"] if b["name"] == "barbell")["mass"],
                     model_total_mass_kg=sum(b["mass"] for b in frames[0]["bodies"]))
        profile = plant.spec.profile
        bar_mass = entry["bar_mass_kg"]
        human_mass = entry["model_total_mass_kg"]-bar_mass
        entry["subject"] = dict(body_mass_kg=float(profile.body_mass_kg), stature_m=float(profile.stature_m),
                                model_human_mass_kg=float(human_mass), bar_mass_kg=float(bar_mass),
                                mass_consistent=bool(abs(human_mass-float(profile.body_mass_kg)) <= 1e-3),
                                source="declared subject profile" if spec is not None else "nominal profile")
        if release is not None:
            pre = [f for f in frames if f["t"] <= release]
            rp = evaluate_receiving(pre, dt, static["verdict"])
            entry["pre_release"] = dict(outcome=rp["outcome"], all_failures=[f"{x['outcome']}:{x['reason']}" for x in rp.get("failures", [])])
            entry["expectation"] = "pre-release QUALIFIED and post-release GRASP_SLIP"
            entry["expectation_met"] = rp["outcome"] == "RECEIVING_HOLD_QUALIFIED" and any(c.startswith("GRASP_SLIP:CONTACT_LOST") for c in codes)
            negative_frames[name] = (frames, dt, static["verdict"], r, entry["expectation"], release, "GRASP_SLIP")
        elif name in NEGATIVE_EXPECTATION:
            entry["expectation"] = "REFUSED:" + NEGATIVE_EXPECTATION[name]
            entry["expectation_met"] = (r["outcome"] != "RECEIVING_HOLD_QUALIFIED"
                                        and any(c.startswith(NEGATIVE_EXPECTATION[name]+":") for c in codes)
                                        and static["verdict"] != "STATIC_EQUILIBRIUM_CERTIFIED")
            negative_frames[name] = (frames, dt, static["verdict"], r, entry["expectation"], None, NEGATIVE_EXPECTATION[name])
        else:
            entry["expectation"] = "QUALIFIED"
            entry["expectation_met"] = r["outcome"] == "RECEIVING_HOLD_QUALIFIED"
        results[name] = entry
        plants[name] = plant
        if name == "nominal_2s":
            nominal_frames = (frames, dt, static["verdict"])
    adv = adversarial(*nominal_frames, nominal)
    pos = positive_controls(*nominal_frames)
    pose_controls = relative_pose_controls(*nominal_frames)
    required = [n for n in EXPECTED_QUALIFIED if n in results]
    red_refused = red_eval["outcome"] != "RECEIVING_HOLD_QUALIFIED" and red_static != "STATIC_EQUILIBRIUM_CERTIFIED"
    anthro = {n: results[n]["expectation_met"] for n in ANTHROPOMETRIC if n in results}
    receipt = full_receipt(plants)
    for name, (fr, dtc, sv, res, exp, onset, code) in negative_frames.items():
        results[name]["failure_signature"] = case_failure_signature(
            name, fr, dtc, sv, res, exp, receipt, onset_s=onset, required_code=code)
    report = dict(issue="RES-624", **source_receipt(), receipt=receipt, scope="declared Human V2 computational model; "
                  "20 kg unplated bar; static overhead POWER receiving hold, not a lift or catch",
                  red_canary=dict(static_verdict=red_static, outcome=red_eval["outcome"], reason=str(red_eval.get("reason")),
                                  bar_drop_m=red_drop, refused=red_refused),
                  cases=results, adversarial=adv, adversarial_positive=pos, relative_pose_controls=pose_controls,
                  required_qualified=required,
                  expectations_met=sum(v["expectation_met"] for v in results.values()), expectations_total=len(results),
                  adversarial_refused=sum(a["refused"] for a in adv.values()), adversarial_total=len(adv),
                  adversarial_positive_qualified=sum(p["qualified"] for p in pos.values()), adversarial_positive_total=len(pos),
                  anthropometric_generalization=("QUALIFIED_BOTH" if anthro and all(anthro.values()) else
                                                 "NOT_CERTIFIED" if anthro else "NOT_RUN"), claims=CLAIMS,
                  quick=bool(quick))
    required_nominal = REQUIRED_NOMINAL if not quick else tuple(n for n in REQUIRED_NOMINAL if n in results)
    verdicts = qualify_scope(report, required_nominal=required_nominal)
    report["verdicts"] = verdicts
    report["final_verdict"] = verdicts["overall"]["verdict"]
    report["scope_statement"] = verdicts["scope_statement"]
    report["anthropometric_generalization"] = ("QUALIFIED_BOTH" if verdicts["anthropometric_generalization"]["qualified"]
                                               else "NOT_CERTIFIED" if anthro else "NOT_RUN")
    (out/"receiving_qualification.json").write_text(json.dumps(report, indent=2, sort_keys=True, default=float)+"\n", encoding="utf-8")
    return report


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="runs/i3/res624/qualification")
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args(argv)
    r = run(a.out, a.quick)
    print("FINAL", r["final_verdict"], "|", r["scope_statement"], "| RED canary", r["red_canary"])
    print("VERDICTS", json.dumps({k: v.get("verdict") for k, v in r["verdicts"].items() if isinstance(v, dict) and "verdict" in v}))
    for k, v in r["cases"].items():
        print(f'{k:28s} {v["static_verdict"]:30s} {v["outcome"]:26s} expect[{v["expectation"]}]={v["expectation_met"]} {v["all_failures"][:3]}')
    for k, v in r["adversarial"].items():
        print(f'ADV {k:28s} expected={v["expected"]:24s} observed={v["observed"]:24s} refused={v["refused"]}')
    nominal_ok = r["verdicts"]["nominal"]["qualified"]
    general_ok = r["verdicts"]["anthropometric_generalization"]["qualified"]
    return 0 if nominal_ok and (a.quick or general_ok) else 1


if __name__ == "__main__":
    raise SystemExit(main())
