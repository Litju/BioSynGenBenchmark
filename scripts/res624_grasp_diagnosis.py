"""RES-624 Gate A: reproduce the RES-618 RED and diagnose the grasp failure.

    python -m scripts.res624_grasp_diagnosis [--out runs/i3/res624/gateA]

Never writes runs/i3/res618/* (the frozen RED witness). Steps:
  1. the frozen RES-618 falsifier ``run()`` reproduces the bar drop;
  2. an instrumented replay of the identical hold (bitwise-identical state);
  3. evidence-ordered initial-support diagnosis (eligibility, closure,
     contact-topology LP, actual forces, saturation);
  4. deliberately broken controls that the instrumentation must classify:
     ineligible shaft masks, unreachable bar, hidden external support,
     and a positive plated-bar-on-floor support control.
Exit 0 means the diagnosis ran and every control was classified as declared;
it is not a receiving PASS (the RED remains a FAIL witness).
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import mujoco
import numpy as np

from biosyngen.spatial.grasp_diagnostics import GraspRecorder, diagnose_initial_support, support_chronology
from biosyngen.spatial.model import AthleteBarbellSpec
from biosyngen.spatial.model_v2 import HumanV2Spec
from biosyngen.spatial.plant_v2 import HumanV2Plant
from biosyngen.spatial.receiving_v2 import forward_hold, solve_candidate
from scripts.svgplot import PALETTE, Figure

CLAIMS = ["NUMERICAL_VERIFICATION_ONLY", "SIMULATED_MEASUREMENT_ONLY",
          "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED", "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED"]
SOURCES = ["biosyngen/spatial/grasp_diagnostics.py", "biosyngen/spatial/grasp_mechanics.py",
           "biosyngen/spatial/receiving_v2.py", "biosyngen/spatial/model_v2.py", "biosyngen/spatial/plant_v2.py",
           "scripts/res624_grasp_diagnosis.py", "scripts/res618_fullbody_qualification.py"]


def source_receipt():
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    dirty = bool(subprocess.check_output(["git", "status", "--porcelain", "--"] + SOURCES, text=True).strip())
    hashes = {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in SOURCES}
    return dict(source_sha=sha, source_dirty=dirty, source_sha256=hashes)


def instrumented_hold(plant, duration_s=.3, push_n=0, hidden_support_n=None):
    """Exact replay of receiving_v2.forward_hold with a passive recorder.

    hidden_support_n: if given, an *undeclared* upward bar force (negative
    control for false support); the recorder must still report contact-only
    support and flag the applied external load.
    """
    m, d = plant.model, plant.data
    targets = plant.joint_angles()
    rec = GraspRecorder(plant)
    bar = plant.body_id("barbell")
    for _ in range(round(duration_s/m.opt.timestep)):
        mujoco.mj_forward(m, d)
        ff = {name: float(d.qfrc_bias[dof]) for name, dof, _ in plant.actuated}
        plant.set_targets(targets, ff)
        d.xfrc_applied[bar, 0] = push_n if .1 <= d.time < .15 else 0
        if hidden_support_n is not None:
            d.xfrc_applied[bar, 2] = hidden_support_n
        mujoco.mj_forward(m, d)
        rec.record()
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    rec.record()
    return rec, dict(final_bar_z_m=float(d.xpos[bar, 2]), final_pelvis_z_m=float(d.qpos[2]))


def _fresh_candidate():
    plant = HumanV2Plant.build()
    candidate = solve_candidate(plant, "OVERHEAD", "POWER", 1000)
    return plant, candidate


def _summary_series(records):
    return dict(t=[r["t"] for r in records], bar_z=[r["bar"]["position"][2] for r in records],
                bar_vz=[r["bar"]["linear_velocity"][2] for r in records],
                support_ratio=[r["bar"]["support_ratio"] for r in records],
                hand_contacts=[r["bar"]["active_hand_contacts"] for r in records],
                bar_contact_fz=[r["bar"]["contact_force_n"][2] for r in records],
                saturation=[r["saturation_count"] for r in records],
                max_cone_utilization=[max((c["cone_utilization"] for c in r["bar_contacts"]), default=0.0) for r in records],
                pad_normals={f"{s}_{p}": [sum(c["normal_force_n"] for c in r["bar_contacts"] if c["other"] == f"{s}_{p}")
                                          for r in records]
                             for s in ("l",) for p in ("palm_collision", "finger_flex_pad", "finger_distal_pad", "thumb_opposition_pad")},
                min_pad_distance={f"l_{p}": [r["hands"]["l"]["pad_shaft_distance_m"][p] for r in records]
                                  for p in ("palm_collision", "finger_flex_pad", "finger_distal_pad", "thumb_opposition_pad")})


def plot(series, chronology, out):
    fig = Figure(980, 760, "RES-618 overhead RED: hand–bar contact chronology (RES-624 Gate A)",
                 "Support ratio, contact count, pad forces, pad distances and bar height over the original 0.3 s hold.",
                 "Left hand shown (right is mirror-identical). NUMERICAL_VERIFICATION_ONLY; diagnostic markers, not acceptance limits.")
    t = series["t"]
    xr = (0, max(t))
    p = fig.panel(70, 80, 380, 170, xr, (-0.2, 1.2), "Hand-contact vertical support / bar weight", "t (s)", "ratio")
    p.line(t, series["support_ratio"], PALETTE["blue"], label="ΣF_z(contacts on bar)/W")
    p.hline(1.0, PALETTE["green"], "full support")
    p.hline(0.5, PALETTE["grey"], "diagnostic 0.5")
    if chronology["first_zero_hand_contact_t"] is not None:
        p.vline(chronology["first_zero_hand_contact_t"], PALETTE["vermillion"], "hand contact lost")
    p = fig.panel(560, 80, 380, 170, xr, (-0.5, 4.5), "Active hand–bar contacts", "t (s)", "count")
    p.line(t, series["hand_contacts"], PALETTE["black"], label="active hand/bar contacts")
    p.line(t, [s/1.0 for s in series["saturation"]], PALETTE["orange"], dash="4 3", label="saturated actuators")
    colors = [PALETTE["blue"], PALETTE["vermillion"], PALETTE["green"], PALETTE["purple"]]
    hi = max(1.0, max(max(v) for v in series["pad_normals"].values()))
    p = fig.panel(70, 320, 380, 170, xr, (0, hi*1.1), "Left pad normal force on shaft", "t (s)", "N")
    for (k, v), c in zip(series["pad_normals"].items(), colors):
        p.line(t, v, c, label=k)
    p.hline(98.1, PALETTE["grey"], "W/2 = 98.1 N")
    p = fig.panel(560, 320, 380, 170, xr, (-0.002, 0.03), "Left pad signed distance to shaft", "t (s)", "m")
    for (k, v), c in zip(series["min_pad_distance"].items(), colors):
        p.line(t, v, c, label=k)
    p = fig.panel(70, 560, 380, 150, xr, (min(series["bar_z"])-.02, max(series["bar_z"])+.02), "Bar COM height", "t (s)", "m")
    p.line(t, series["bar_z"], PALETTE["black"], label="bar z")
    p = fig.panel(560, 560, 380, 150, xr, (0, 1.2), "Max friction-cone utilization on bar", "t (s)", "|f_t|/(μ f_n)")
    p.line(t, series["max_cone_utilization"], PALETTE["purple"], label="max over bar contacts")
    Path(out).write_text(fig.render(), encoding="utf-8")


def control_ineligible():
    plant, _ = _fresh_candidate()
    m = plant.model
    shaft = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "bar_shaft")
    m.geom_contype[shaft] = 0
    m.geom_conaffinity[shaft] = 0
    diag = diagnose_initial_support(plant)
    rec, _ = instrumented_hold(plant, .05)
    return dict(expected="NO_ELIGIBLE_CONTACT_PAIR", observed=diag["failure_class"],
                hand_contacts_any_step=max(r["bar"]["active_hand_contacts"] for r in rec.records))


def control_unreachable():
    plant, _ = _fresh_candidate()
    q = plant.joint_qpos("bar_free")
    plant.data.qpos[q+2] += .05
    mujoco.mj_forward(plant.model, plant.data)
    diag = diagnose_initial_support(plant)
    return dict(expected="CONTACT_GEOMETRY_CANNOT_CLOSE", observed=diag["failure_class"],
                pads_within_margin=diag["pads_within_margin"])


def control_hidden_support():
    plant, _ = _fresh_candidate()
    w = float(plant.model.body_mass[plant.body_id("barbell")]*9.81)
    rec, final = instrumented_hold(plant, .3, hidden_support_n=w)
    chron = support_chronology(rec.records)
    ext = max(r["bar"]["applied_external_n"][2] for r in rec.records)
    drop = rec.records[0]["bar"]["position"][2]-rec.records[-2]["bar"]["position"][2]
    return dict(expected="BAR_HELD_BY_UNDECLARED_EXTERNAL_FORCE_NOT_CONTACT", contact_support_state=chron["state"],
                max_contact_support_ratio=chron["max_support_ratio"], applied_external_max_n=ext,
                bar_drop_m=float(drop),
                observed="BAR_HELD_BY_UNDECLARED_EXTERNAL_FORCE_NOT_CONTACT"
                if chron["state"] == "NEVER_ESTABLISHED" and ext > 0 and abs(drop) < .05 else "MISCLASSIFIED")


def control_plated_floor():
    """Positive control: plated bar resting on the floor is supported by plate contacts."""
    spec = HumanV2Spec(equipment=AthleteBarbellSpec(plate_mass_kg=20))
    plant = HumanV2Plant.build(spec)
    m, d = plant.model, plant.data
    q = plant.joint_qpos("bar_free")
    radius = max(float(m.geom_size[g, 0]) for g in range(m.ngeom)
                 if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith("plate_"))
    d.qpos[q:q+3] = [1.5, 0, radius-1e-4]
    rec = GraspRecorder(plant)
    for _ in range(300):
        mujoco.mj_forward(m, d)
        rec.record()
        mujoco.mj_step(m, d)
    chron = support_chronology(rec.records)
    tail = [r["bar"]["support_ratio"] for r in rec.records[-50:]]
    return dict(expected="SUSTAINED", observed=chron["state"], final_support_ratio_mean=float(np.mean(tail)),
                floor_eligible_via="plate geoms (contype 2, conaffinity 5)")


def run(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    from scripts.res618_fullbody_qualification import run as frozen_run
    frozen = frozen_run()
    plant, candidate = _fresh_candidate()
    initial_bar_z = float(plant.data.xpos[plant.body_id("barbell"), 2])
    topology = diagnose_initial_support(plant)
    # Re-solve on a fresh plant so that the recorder replays from the same state as forward_hold.
    plant, _ = _fresh_candidate()
    rec, final = instrumented_hold(plant, .3)
    plant_ref, _ = _fresh_candidate()
    ref = forward_hold(plant_ref, .3)
    identical = final["final_bar_z_m"] == ref["final_bar_z_m"] and final["final_pelvis_z_m"] == ref["final_pelvis_z_m"]
    chron = support_chronology(rec.records)
    first = rec.records[0]
    series = _summary_series(rec.records)
    plot(series, chron, out/"red_chronology.svg")
    controls = dict(ineligible=control_ineligible(), unreachable=control_unreachable(),
                    hidden_support=control_hidden_support(), plated_floor_positive=control_plated_floor())
    controls_ok = all(c["expected"] == c["observed"] for c in controls.values())
    step0_cone = [dict(pad=c["other"], normal_n=c["normal_force_n"], tangent_n=c["tangent_force_n"],
                       utilization=c["cone_utilization"], force_on_bar=c["force_on_bar"]) for c in first["bar_contacts"]]
    report = dict(
        issue="RES-624", gate="A", outcome="RED_REPRODUCED_AND_DIAGNOSED" if controls_ok and identical else "DIAGNOSIS_INCOMPLETE",
        **source_receipt(),
        frozen_red=dict(code=frozen["code"], outcome=frozen["outcome"], initial_bar_z_m=frozen["initial_bar_z_m"],
                        final_bar_z_m=frozen["hold"]["final_bar_z_m"], bar_drop_m=frozen["bar_drop_m"],
                        final_contacts=[(c["geom1"], c["geom2"]) for c in frozen["final_contacts"]],
                        historical_windows_drop_m=0.3760531196714312),
        candidate=dict(optimizer_success=candidate["optimizer_success"], residual_norm=candidate["residual_norm"],
                       min_contact_distance_m=candidate["min_contact_distance_m"], initial_bar_z_m=initial_bar_z),
        instrumentation_bitwise_identical_to_forward_hold=bool(identical),
        initial_support_diagnosis=topology,
        step0_bar_contacts=step0_cone, step0_bar_contact_force_n=first["bar"]["contact_force_n"],
        step0_support_ratio=first["bar"]["support_ratio"], bar_weight_n=rec.weight,
        chronology=chron,
        terminal=dict(bar_contacts=[c["other"] for c in rec.records[-1]["bar_contacts"]],
                      floor_contacts=rec.records[-1]["floor"]["count"],
                      bar_linear_momentum=rec.records[-1]["bar"]["linear_momentum"],
                      bar_angular_momentum_about_com=rec.records[-1]["bar"]["angular_momentum_about_com"],
                      contact_work_j=rec.contact_work_j, actuator_work_j=rec.actuator_work_j),
        thumb_transient=dict(steps=[r["step"] for r in rec.records
                                    if any(c["other"].endswith("thumb_opposition_pad") for c in r["bar_contacts"])][:1]
                             + [r["step"] for r in rec.records
                                if any(c["other"].endswith("thumb_opposition_pad") for c in r["bar_contacts"])][-1:]),
        controls=controls, controls_classified=controls_ok, claims=CLAIMS)
    (out/"red_diagnosis.json").write_text(json.dumps(report, indent=2, sort_keys=True)+"\n", encoding="utf-8")
    (out/"red_trace.json").write_text(json.dumps(dict(series=series, records=rec.records), sort_keys=True)+"\n",
                                      encoding="utf-8")
    return report


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="runs/i3/res624/gateA")
    args = ap.parse_args(argv)
    report = run(args.out)
    print(json.dumps({k: report[k] for k in ("outcome", "frozen_red", "instrumentation_bitwise_identical_to_forward_hold",
                                             "chronology", "controls_classified", "step0_support_ratio")}, indent=2))
    print("initial failure class:", report["initial_support_diagnosis"]["failure_class"])
    return 0 if report["outcome"] == "RED_REPRODUCED_AND_DIAGNOSED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
