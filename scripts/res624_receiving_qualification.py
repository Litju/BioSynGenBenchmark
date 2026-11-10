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
from biosyngen.spatial.receiving_grasp_v2 import actuator_export, forward_receiving_hold, hand_frames, qualify_static
from biosyngen.spatial.receiving_v2 import solve_candidate
from biosyngen_benchmark.receiving_acceptance import evaluate_receiving
from scripts.res624_grasp_diagnosis import CLAIMS, source_receipt
from scripts.res624_receiving_campaign import build_nominal, cases, clone, run_case

EXPECTED_QUALIFIED = ("nominal_0p3s_red_window", "nominal_2s", "nominal_10s")


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
        f["hand_frames"], f["actuators"] = hand_frames(plant), actuator_export(plant)
        frames.append(f)
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    f = plant.export_frame()
    f["hand_frames"], f["actuators"] = hand_frames(plant), actuator_export(plant)
    frames.append(f)
    return frames, static["verdict"], float(frames[0]["bodies"][-1]["com"][2]-frames[-1]["bodies"][-1]["com"][2])


def _is_hand_bar(c):
    return "barbell" in (c["body1"], c["body2"]) and (c["body1"] if c["body2"] == "barbell" else c["body2"])[:2] in ("l_", "r_")


def adversarial(frames, dt, verdict, nominal):
    out = {}

    def run(name, expected, fr, v=verdict):
        r = evaluate_receiving(fr, dt, v)
        out[name] = dict(expected=expected, observed=r["outcome"], reason=str(r.get("reason")), refused=r["outcome"] == expected)
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
    return out


def run(out, quick=False):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    _, nominal = build_nominal()
    red, red_static, red_drop = red_frames()
    red_eval = evaluate_receiving(red, .001, red_static)
    results, nominal_frames = {}, None
    for name, spec, mod, dur, kw in cases(quick):
        frames, static, plant = run_case(name, spec, mod, dur, kw, nominal)
        r = evaluate_receiving(frames, plant.model.opt.timestep, static["verdict"],
                               end_s=kw.get("release_at_s"))
        results[name] = dict(static_verdict=static["verdict"], outcome=r["outcome"], reason=str(r.get("reason")),
                             all_failures=[f"{x['outcome']}:{x['reason']}" for x in r.get("failures", [])],
                             evidence={k: v for k, v in r["evidence"].items()
                                       if k in ("bar_displacement_in_hand_m", "hand_support_ratio_mean", "hand_support_ratio_min",
                                                "min_hand_contacts", "min_cop_margin_m", "min_com_projection_margin_m",
                                                "floor_vertical_over_weight_mean", "max_penetration_m", "posture_saturation_samples",
                                                "contact_problems", "over_capacity")},
                             bar_linear_residual_n_s=r["evidence"].get("bar_newton_euler", {}).get("max_step_linear_residual_n_s"),
                             bar_angular_excess_n_m_s=r["evidence"].get("bar_angular_discretization", {}).get("max_excess_over_discretization_n_m_s"),
                             system_step_residual_n_s=r["evidence"].get("system_newton_euler", {}).get("max_step_linear_residual_n_s"))
        if name == "nominal_2s":
            nominal_frames = (frames, plant.model.opt.timestep, static["verdict"])
    adv = adversarial(*nominal_frames, nominal)
    required = [n for n in EXPECTED_QUALIFIED if n in results]
    red_refused = red_eval["outcome"] != "RECEIVING_HOLD_QUALIFIED" and red_static != "STATIC_EQUILIBRIUM_CERTIFIED"
    final = ("RECEIVING_HOLD_QUALIFIED" if all(results[n]["outcome"] == "RECEIVING_HOLD_QUALIFIED" for n in required)
             and red_refused and all(a["refused"] for a in adv.values()) else "RECEIVING_NOT_QUALIFIED")
    report = dict(issue="RES-624", **source_receipt(), final_verdict=final, scope="declared Human V2 computational model; "
                  "20 kg unplated bar; static overhead POWER receiving hold, not a lift or catch",
                  red_canary=dict(static_verdict=red_static, outcome=red_eval["outcome"], reason=str(red_eval.get("reason")),
                                  bar_drop_m=red_drop, refused=red_refused),
                  cases=results, adversarial=adv, required_qualified=required, claims=CLAIMS)
    (out/"receiving_qualification.json").write_text(json.dumps(report, indent=2, sort_keys=True, default=float)+"\n", encoding="utf-8")
    return report


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="runs/i3/res624/qualification")
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args(argv)
    r = run(a.out, a.quick)
    print("FINAL", r["final_verdict"], "| RED canary", r["red_canary"])
    for k, v in r["cases"].items():
        print(f'{k:28s} {v["static_verdict"]:30s} {v["outcome"]:26s} {v["all_failures"]}')
    for k, v in r["adversarial"].items():
        print(f'ADV {k:28s} expected={v["expected"]:24s} observed={v["observed"]:24s} refused={v["refused"]}')
    return 0 if r["final_verdict"] == "RECEIVING_HOLD_QUALIFIED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
