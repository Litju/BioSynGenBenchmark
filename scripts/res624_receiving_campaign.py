"""RES-624 Gates D/E: equilibrium-certified Human V2 overhead pose and forward holds.

    python -m scripts.res624_receiving_campaign [--out runs/i3/res624/gateDE] [--quick]

Gate D: the support-aware pose (Gate C hand-bar relation) is solved, then the
generalized static-equilibrium certificate, COP, ROM and self-intersection
checks decide. Adversarial static cases: the frozen RES-618 kinematic RED
candidate, the nominal pose at reduced capacity, the bar lifted off the hands.

Gate E: bounded-actuation MuJoCo forward holds from the certified state only
(nominal 0.3 s RED window, 2 s, 10 s; bar and pelvis pushes; 15/25 kg bar;
friction 0.6; two anthropometries; finger release; insufficient arm capacity).
Generator-side metrics are reported here; the independent frame-only verdict
is produced by scripts.res624_receiving_qualification.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from biosyngen.spatial.anthropometry import HumanSubjectProfile
from biosyngen.spatial.model_v2 import HumanV2Spec
from biosyngen.spatial.plant_v2 import HumanV2Plant
from biosyngen.spatial.receiving_grasp_v2 import (build_certified_candidate, forward_receiving_hold, qualify_static)
from biosyngen.spatial.receiving_v2 import solve_candidate
from scripts.res624_grasp_diagnosis import CLAIMS, source_receipt
from scripts.svgplot import PALETTE, Figure

ARM_ACTUATORS = ("_sh_flex", "_sh_abd", "_sh_rot", "_elbow_flex", "_forearm_sup", "_wrist_flex", "_wrist_dev",
                 "_girdle_elev", "_girdle_prot")


def _set_bar_mass(plant, mass):
    m = plant.model
    b = plant.body_id("barbell")
    k = mass/m.body_mass[b]
    m.body_mass[b] = mass
    m.body_inertia[b] *= k


def _set_hand_friction(plant, mu):
    m = plant.model
    for g in range(m.ngeom):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        if name == "bar_shaft" or name.endswith(("_pad", "_palm_collision")):
            m.geom_friction[g, 0] = mu


def _scale_arm_capacity(plant, scale):
    m = plant.model
    for a in range(m.nu):
        if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or "").endswith(ARM_ACTUATORS):
            m.actuator_forcerange[a] *= scale
            m.actuator_ctrlrange[a] *= scale if m.actuator_biastype[a] != mujoco.mjtBias.mjBIAS_AFFINE else 1


def frame_metrics(frames, settle_s=.1):
    bar = "barbell"
    w = next(b for b in frames[0]["bodies"] if b["name"] == bar)["mass"]*9.81
    total = sum(b["mass"] for b in frames[0]["bodies"])*9.81
    win = [f for f in frames if f["t"] >= settle_s]
    hand, support, sat, floor = [], [], 0, []
    for f in win:
        hc = [c for c in f["contacts"] if bar in (c["body1"], c["body2"])
              and (c["body1"] if c["body2"] == bar else c["body2"])[:2] in ("l_", "r_")]
        hand.append(len(hc))
        support.append(sum(c["force_on_body2"][2]*(1 if c["body2"] == bar else -1) for c in hc)/w)
        floor.append(sum(c["force_on_body2"][2]*(1 if c["body1"] == "world" else -1) for c in f["contacts"]
                         if "world" in (c["body1"], c["body2"]))/total)
        a = f["actuators"]
        sat += sum(1 for n, F, cap in zip(a["names"], a["force"], a["capacity"])
                   if abs(F) >= cap*(1-1e-9) and not n.endswith(("_finger_flex", "_finger_distal", "_thumb_opposition")))
    ref = win[0]

    def bar_in(f, side):
        r = np.asarray(f["hand_frames"][side]["rotation"])
        p = np.asarray(f["hand_frames"][side]["position"])
        return r.T @ (np.asarray(next(b for b in f["bodies"] if b["name"] == bar)["com"]) - p)
    disp = max(float(np.linalg.norm(bar_in(f, s)-bar_in(ref, s))) for f in win for s in "lr")
    com = [sum(np.asarray(b["com"])*b["mass"] for b in f["bodies"])/sum(b["mass"] for b in f["bodies"]) for f in frames]
    bz = [next(b for b in f["bodies"] if b["name"] == bar)["com"][2] for f in frames]
    return dict(duration_s=frames[-1]["t"], min_hand_contacts=min(hand), hand_support_mean=float(np.mean(support)),
                hand_support_min=float(np.min(support)), bar_displacement_in_hand_m=disp,
                posture_saturation_samples=sat, floor_vertical_over_weight_mean=float(np.mean(floor)),
                com_x_drift_m=float(com[-1][0]-com[0][0]), bar_z_change_m=float(bz[-1]-bz[0]))


def static_adversarials(nominal_plant):
    out = {}
    red = HumanV2Plant.build()
    solve_candidate(red, "OVERHEAD", "POWER", 1000)
    out["frozen_res618_kinematic_candidate"] = qualify_static(red)["verdict"]
    out["nominal_at_20pct_capacity"] = qualify_static(nominal_plant, actuator_scale=.2)["verdict"]
    lifted = HumanV2Plant.build()
    lifted.data.qpos[:] = nominal_plant.data.qpos
    q = lifted.joint_qpos("bar_free")
    lifted.data.qpos[q+2] += .02
    mujoco.mj_forward(lifted.model, lifted.data)
    out["bar_lifted_20mm_into_fingers"] = qualify_static(lifted)["verdict"]
    moved = HumanV2Plant.build()
    moved.data.qpos[:] = nominal_plant.data.qpos
    moved.data.qpos[q] += .05
    mujoco.mj_forward(moved.model, moved.data)
    out["bar_displaced_50mm_out_of_hands"] = qualify_static(moved)["verdict"]
    return out


def arm_capacity_threshold(qpos, iterations=10):
    """Smallest arm/shoulder/wrist capacity scale with a feasible certificate (bisection)."""
    lo, hi = 0.0, 1.0
    for _ in range(iterations):
        mid = .5*(lo+hi)
        p = clone(HumanV2Spec(), qpos)
        _scale_arm_capacity(p, mid)
        if qualify_static(p)["verdict"] == "STATIC_EQUILIBRIUM_CERTIFIED":
            hi = mid
        else:
            lo = mid
    return hi


def clone(plant_spec, qpos):
    p = HumanV2Plant.build(plant_spec)
    p.data.qpos[:] = qpos
    mujoco.mj_forward(p.model, p.data)
    return p


def cases(quick=False):
    """(name, spec, modifier, duration, kwargs, expected_family)."""
    base = [("nominal_0p3s_red_window", None, None, .3, {}),
            ("nominal_2s", None, None, 2.0, {}),
            ("bar_push_40N_50ms", None, None, 2.0, dict(push=("barbell", 1.0, 1.05, (40.0, 0, 0)))),
            ("pelvis_push_60N_100ms", None, None, 2.0, dict(push=("pelvis", 1.0, 1.1, (60.0, 0, 0)))),
            ("bar_mass_15kg", None, ("mass", 15.0), 2.0, {}),
            ("bar_mass_25kg", None, ("mass", 25.0), 2.0, {}),
            ("hand_friction_0p6", None, ("friction", .6), 2.0, {}),
            ("finger_release_at_1s", None, None, 2.0, dict(release_at_s=1.0)),
            ("arm_capacity_half_threshold", None, ("arm_capacity", None), 1.0, {})]
    if quick:
        return base[:2]
    return base + [("nominal_10s", None, None, 10.0, {}),
                   ("anthropometry_70kg_1p70m", HumanV2Spec(profile=HumanSubjectProfile(70, 1.70)), None, 2.0, {}),
                   ("anthropometry_80kg_1p80m", HumanV2Spec(profile=HumanSubjectProfile(80, 1.80)), None, 2.0, {})]


def run_case(name, spec, modifier, duration, kwargs, nominal):
    """Simulate one case from a certified state; returns (frames, static, plant)."""
    if spec is None:
        plant = clone(HumanV2Spec(), nominal["qpos"])
    else:
        plant, _, _, _ = build_certified_candidate(spec)
    if modifier and modifier[0] == "arm_capacity" and modifier[1] is None:
        modifier = ("arm_capacity", .5*nominal["arm_capacity_threshold"])
    if modifier:
        {"mass": _set_bar_mass, "friction": _set_hand_friction, "arm_capacity": _scale_arm_capacity}[modifier[0]](plant, modifier[1])
        mujoco.mj_forward(plant.model, plant.data)
    static = qualify_static(plant)
    cert = static["certificate"]
    if "actuator_force" not in cert:  # no admissible feed-forward exists: hold with the nominal one
        cert = dict(cert, actuator_force=nominal["actuator_force"])
    frames = forward_receiving_hold(plant, cert, duration, **kwargs)
    return frames, static, plant


def plot(series, out):
    fig = Figure(980, 520, "RES-624 Gates D/E: certified Human V2 overhead hold (forward dynamics)",
                 "Hand-contact support ratio and bar displacement in the hand frame for forward holds.",
                 "Bounded actuator commands only; no bar/root state writes. NUMERICAL_VERIFICATION_ONLY.")
    p = fig.panel(70, 80, 870, 160, (0, 2.0), (-.2, 1.4), "Hand-contact vertical support / bar weight", "t (s)", "ratio")
    colors = [PALETTE["blue"], PALETTE["green"], PALETTE["vermillion"], PALETTE["purple"], PALETTE["orange"], PALETTE["black"]]
    for (name, s), c in zip(series.items(), colors):
        p.line(s["t"], s["support"], c, label=name)
    p = fig.panel(70, 310, 870, 160, (0, 2.0), (0, .03), "Bar COM displacement in left-hand frame", "t (s)", "m")
    for (name, s), c in zip(series.items(), colors):
        p.line(s["t"], s["disp"], c, label=name)
    p.hline(.014, PALETTE["grey"], "shaft radius 14 mm")
    Path(out).write_text(fig.render(), encoding="utf-8")


def _series(frames):
    bar = "barbell"
    w = next(b for b in frames[0]["bodies"] if b["name"] == bar)["mass"]*9.81
    out = dict(t=[], support=[], disp=[])
    ref = None
    for f in frames[::10]:
        hc = [c for c in f["contacts"] if bar in (c["body1"], c["body2"])
              and (c["body1"] if c["body2"] == bar else c["body2"])[:2] in ("l_", "r_")]
        r = np.asarray(f["hand_frames"]["l"]["rotation"])
        rel = r.T @ (np.asarray(next(b for b in f["bodies"] if b["name"] == bar)["com"])-np.asarray(f["hand_frames"]["l"]["position"]))
        ref = rel if ref is None else ref
        out["t"].append(f["t"])
        out["support"].append(sum(c["force_on_body2"][2]*(1 if c["body2"] == bar else -1) for c in hc)/w)
        out["disp"].append(float(np.linalg.norm(rel-ref)))
    return out


def build_nominal():
    plant, candidate, static, relation = build_certified_candidate()
    nominal = dict(candidate=candidate, static=static, relation=relation.__dict__, qpos=plant.data.qpos.tolist(),
                   actuator_force=static["certificate"].get("actuator_force"))
    nominal["arm_capacity_threshold"] = arm_capacity_threshold(nominal["qpos"])
    return plant, nominal


def run(out, quick=False):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    plant, nominal = build_nominal()
    adversarial = static_adversarials(plant)
    results, series = {}, {}
    for name, spec, mod, dur, kw in cases(quick):
        frames, static, _ = run_case(name, spec, mod, dur, kw, nominal)
        results[name] = dict(static_verdict=static["verdict"], static_code=static["certificate"]["code"],
                             max_utilization=static["certificate"].get("max_utilization"), metrics=frame_metrics(frames))
        if dur >= 2.0 and len(series) < 6 and name in ("nominal_2s", "bar_push_40N_50ms", "pelvis_push_60N_100ms",
                                                       "bar_mass_25kg", "hand_friction_0p6", "finger_release_at_1s"):
            series[name] = _series(frames)
    if series:
        plot(series, out/"receiving_hold.svg")
    cert = nominal["static"]["certificate"]
    report = dict(issue="RES-624", gate="D/E", **source_receipt(),
                  nominal=dict(candidate={k: nominal["candidate"][k] for k in ("optimizer_success", "residual_norm",
                                                                              "evaluations", "variables", "palm_tilt_from_vertical_deg")},
                               relation=nominal["relation"], static_verdict=nominal["static"]["verdict"],
                               arm_capacity_threshold_scale=nominal["arm_capacity_threshold"],
                               certificate={k: v for k, v in cert.items() if k not in ("actuator_force",)},
                               self_intersection=nominal["static"]["self_intersection"], qpos=nominal["qpos"]),
                  static_adversarials=adversarial, forward=results, claims=CLAIMS)
    (out/"receiving_campaign.json").write_text(json.dumps(report, indent=2, sort_keys=True, default=float)+"\n", encoding="utf-8")
    return report


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="runs/i3/res624/gateDE")
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args(argv)
    report = run(a.out, a.quick)
    print("nominal static:", report["nominal"]["static_verdict"], "adversarial:", report["static_adversarials"])
    for k, v in report["forward"].items():
        mt = v["metrics"]
        print(f'{k:28s} static={v["static_verdict"]:30s} hc_min={mt["min_hand_contacts"]:2d} support={mt["hand_support_mean"]:.4f}/{mt["hand_support_min"]:.3f} '
              f'disp={mt["bar_displacement_in_hand_m"]:.4f} sat={mt["posture_saturation_samples"]} floor={mt["floor_vertical_over_weight_mean"]:.4f} '
              f'comdx={mt["com_x_drift_m"]:.4f} barz={mt["bar_z_change_m"]:.4f}')
    return 0 if report["nominal"]["static_verdict"] == "STATIC_EQUILIBRIUM_CERTIFIED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
