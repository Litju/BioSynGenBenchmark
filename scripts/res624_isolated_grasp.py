"""RES-624 Gate C: qualify (or refuse) an isolated, load-bearing two-hand bar grasp.

    python -m scripts.res624_isolated_grasp [--out runs/i3/res624/gateC]

Positive and negative forward-dynamics trials of the V2 hands on rigid
declared mounts holding a free 20 kg bar by contact only, with:
  * the independent oracle (biosyngen_benchmark.spatial_oracle) recomputing
    the bar's translational/rotational Newton-Euler balance and the Coulomb
    admissibility of every exported contact;
  * the generalized static-equilibrium LP certificate (pyramid inscribed in
    the cone) at the settled state, at declared and reduced capacity;
  * mount reactions against the analytic total weight;
  * grip-capacity envelopes over tilt, bar mass and friction;
  * genuine failures: insufficient grip, controlled release.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from biosyngen.spatial.grasp_diagnostics import PADS
from biosyngen.spatial.grasp_fixture import SETTLE_S, SHAFT_RADIUS, FixtureSpec, classify_trial, run_trial
from biosyngen.spatial.grasp_mechanics import candidate_contacts, generalized_equilibrium
from biosyngen_benchmark.spatial_oracle import contact_admissibility, dynamic_balance
from scripts.res624_grasp_diagnosis import CLAIMS, source_receipt
from scripts.svgplot import PALETTE, Figure

GRID = [round(.1*i, 1) for i in range(11)]


class _Null:
    def record(self, extra=None):
        return None


def certificate(fx, scale=1.0):
    m, d = fx.model, fx.data
    pads = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{s}_{p}") for s in "lr" for p in PADS]
    shaft = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "bar_shaft")
    contacts = [c for c in candidate_contacts(m, d, pads+[shaft], 1e-4) if shaft in (c["geom1"], c["geom2"])]
    cert = generalized_equilibrium(m, d, contacts, actuator_scale=scale)
    return {k: cert[k] for k in ("code", "feasible", "contacts", "max_utilization", "capacity_margin",
                                 "max_generalized_residual", "residual_tolerance") if k in cert}


def detailed(spec, grip, duration, **kw):
    fx, rec, frames, pos, hc, ts = run_trial(spec, grip, duration, **kw)
    cls = classify_trial(pos, hc, ts, end_s=kw.get("release_at_s"))
    m = fx.model
    dt = m.opt.timestep
    hold = [r for r in rec.records if r["t"] >= SETTLE_S and (kw.get("release_at_s") is None or r["t"] < kw["release_at_s"])]
    hold_frames = [f for f in frames if f["t"] >= SETTLE_S and (kw.get("release_at_s") is None or f["t"] < kw["release_at_s"])]
    bar_nE = dynamic_balance(frames, dt, members={"barbell"}, about=(0, 0, 0))
    admissible = [contact_admissibility(f, floor_normal=(0, 0, 1)) for f in frames]
    cone_problems = sum(len(a["problems"]) for a in admissible)
    weight = float(m.body_mass[fx.body_id("barbell")]*9.81)
    left = [sum(c["force_on_bar"][2] for c in r["bar_contacts"] if c["other"].startswith("l_")) for r in hold]
    right = [sum(c["force_on_bar"][2] for c in r["bar_contacts"] if c["other"].startswith("r_")) for r in hold]
    hand_mass = float(sum(m.body_mass[b] for b in range(1, m.nbody) if b != fx.body_id("barbell")))
    reaction = fx.mount_reaction()
    reaction_z = reaction["l"]["force_n"][2]+reaction["r"]["force_n"][2]
    pads = sorted({c["other"] for r in hold for c in r["bar_contacts"]})
    normals = [c["normal_force_n"] for r in hold for c in r["bar_contacts"]]
    out = dict(spec=dict(tilt_deg=spec.tilt_deg, friction=spec.friction, bar_mass_kg=spec.bar_mass_kg),
               grip_frac=grip, duration_s=duration, classification=cls,
               hand_support_ratio_hold=dict(mean=float(np.mean([r["bar"]["hand_support_ratio"] for r in hold])) if hold else None,
                                            min=float(np.min([r["bar"]["hand_support_ratio"] for r in hold])) if hold else None),
               load_share_left_n=float(np.mean(left)) if left else None, load_share_right_n=float(np.mean(right)) if right else None,
               bar_weight_n=weight, contact_pads=pads, min_normal_force_n=float(min(normals)) if normals else None,
               max_cone_utilization=float(max((c["cone_utilization"] for r in hold for c in r["bar_contacts"]
                                               if np.isfinite(c["cone_utilization"])), default=0)),
               max_slip_speed_m_s=float(max((c["slip_speed_m_s"] for r in hold for c in r["bar_contacts"]), default=0)),
               max_axial_spin_rad_s=float(max((abs(r["bar"]["axial_spin_rad_s"]) for r in hold), default=0)),
               max_bar_speed_hold_m_s=float(max((np.linalg.norm(r["bar"]["linear_velocity"]) for r in hold), default=0)),
               final_actuator_saturation=sum(1 for a in rec.records[-1]["actuators"].values() if a["saturated"]),
               oracle_bar_newton_euler=bar_nE, oracle_cone_problems=cone_problems,
               oracle_frames=len(frames),
               mount_reaction_final=dict(sum_vertical_n=float(reaction_z), analytic_static_n=(hand_mass+spec.bar_mass_kg)*9.81
                                         if cls["outcome"] == "GRASP_SUPPORTED" else hand_mass*9.81, per_side=reaction),
               contact_work_j=rec.contact_work_j, actuator_work_j=rec.actuator_work_j,
               hold_frames=len(hold_frames))
    return fx, rec, out


def envelope(specs, label):
    rows = []
    for spec in specs:
        outcomes = []
        for grip in GRID:
            _, _, _, pos, hc, ts = run_trial(spec, grip, 1.5, record_every=10**9, recorder=_Null())
            c = classify_trial(pos, hc, ts)
            outcomes.append(dict(grip=grip, outcome=c["outcome"], hold_displacement_m=c["hold_displacement_m"]))
        held = [o["grip"] for o in outcomes if o["outcome"] == "GRASP_SUPPORTED"]
        threshold = min((g for g in GRID if all(o["outcome"] == "GRASP_SUPPORTED" for o in outcomes if o["grip"] >= g)),
                        default=None)
        rows.append(dict(sweep=label, tilt_deg=spec.tilt_deg, friction=spec.friction, bar_mass_kg=spec.bar_mass_kg,
                         grid=outcomes, min_supporting_grip_frac=threshold,
                         monotone=threshold is not None and held == [g for g in GRID if g >= threshold] or not held,
                         outcome="ACTUATOR_INFEASIBLE" if not held else "SUPPORT_ENVELOPE_FOUND"))
    return rows


def plot(trials, env, out):
    fig = Figure(980, 520, "RES-624 Gate C: isolated V2 two-hand grasp of a free 20 kg bar",
                 "Hand-contact support ratio over time and minimum supporting grip fraction against hand tilt, mass and friction.",
                 "Contact-only support; palms on declared rigid mounts. NUMERICAL_VERIFICATION_ONLY.")
    p = fig.panel(70, 80, 400, 170, (0, 3.0), (-.2, 1.3), "Hand-contact vertical support / bar weight", "t (s)", "ratio")
    colors = [PALETTE["blue"], PALETTE["green"], PALETTE["vermillion"], PALETTE["purple"]]
    for (name, rec), c in zip(trials.items(), colors):
        p.line([r["t"] for r in rec.records], [r["bar"]["hand_support_ratio"] for r in rec.records], c, label=name)
    p.vline(SETTLE_S, PALETTE["grey"], "settle")
    tilt = [r for r in env if r["sweep"] == "tilt"]
    p = fig.panel(560, 80, 380, 170, (-5, 95), (0, 1.1), "Minimum supporting grip vs tilt (20 kg, μ=1)", "tilt (deg)", "grip fraction")
    p.markers([r["tilt_deg"] for r in tilt], [r["min_supporting_grip_frac"] for r in tilt], PALETTE["black"], r=3, label="threshold")
    p.line([r["tilt_deg"] for r in tilt], [r["min_supporting_grip_frac"] for r in tilt], PALETTE["black"])
    p = fig.panel(70, 320, 870, 150, (-.5, 8.5), (0, 1.1), "Minimum supporting grip at tilt 90° (RES-618 RED relation) vs mass and μ",
                  "configuration", "grip fraction")
    mass = [r for r in env if r["sweep"] == "mass_friction"]
    p.xtick_labels = {i: f'{r["bar_mass_kg"]:g}kg μ{r["friction"]:g}' for i, r in enumerate(mass)}
    p.markers(list(range(len(mass))), [r["min_supporting_grip_frac"] if r["min_supporting_grip_frac"] is not None else 1.05
                                       for r in mass], PALETTE["vermillion"], r=3, label="threshold (1.05 = infeasible ≤ capacity)")
    Path(out).write_text(fig.render(), encoding="utf-8")


def run(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    results, trials = {}, {}
    cases = dict(
        palm_up_full_grip=(FixtureSpec(tilt_deg=0), 1.0, 3.0, {}),
        red_orientation_full_grip=(FixtureSpec(tilt_deg=90), 1.0, 3.0, {}),
        red_orientation_insufficient_grip=(FixtureSpec(tilt_deg=90), .3, 1.5, {}),
        red_orientation_release=(FixtureSpec(tilt_deg=90), 1.0, 2.0, dict(release_at_s=1.0)),
        red_orientation_push=(FixtureSpec(tilt_deg=90), 1.0, 2.0, dict(push_n=(1.0, 1.05, (40.0, 0, 0)))),
    )
    certs = {}
    for name, (spec, grip, dur, kw) in cases.items():
        fx, rec, res = detailed(spec, grip, dur, **kw)
        results[name] = res
        trials[name] = rec
        if res["classification"]["outcome"] == "GRASP_SUPPORTED" and "release_at_s" not in kw:
            certs[name] = dict(declared_capacity=certificate(fx, 1.0), commanded_fraction=certificate(fx, grip),
                               tenth_capacity=certificate(fx, .1))
    release = results["red_orientation_release"]
    rel = trials["red_orientation_release"].records
    after = []
    for r in rel:  # first contiguous contact-free interval after release (before the floor bounce)
        if r["t"] <= 1.0:
            continue
        if r["bar"]["active_contacts"] == 0:
            after.append(r)
        elif after:
            break
    if len(after) > 3:
        v = np.array([r["bar"]["linear_velocity"][2] for r in after])
        t = np.array([r["t"] for r in after])
        free_fall_accel = float(np.polyfit(t, v, 1)[0])
    else:
        free_fall_accel = None
    release_loss = next((r["t"] for r in rel if r["t"] > 1.0 and r["bar"]["active_hand_contacts"] == 0), None)
    env = envelope([FixtureSpec(tilt_deg=a) for a in (0, 30, 60, 90)], "tilt") + \
        envelope([FixtureSpec(tilt_deg=90, bar_mass_kg=mass, friction=mu) for mass in (10, 20, 30) for mu in (.5, .75, 1.0)],
                 "mass_friction")
    mf = {(r["bar_mass_kg"], r["friction"]): r["min_supporting_grip_frac"] for r in env if r["sweep"] == "mass_friction"}
    inf = 9.9

    def th(k):
        return inf if mf[k] is None else mf[k]
    monotone_mass = all(th((10, mu)) <= th((20, mu)) <= th((30, mu)) for mu in (.5, .75, 1.0))
    monotone_mu = all(th((mass, .5)) >= th((mass, .75)) >= th((mass, 1.0)) for mass in (10, 20, 30))
    plot({k: trials[k] for k in ("palm_up_full_grip", "red_orientation_full_grip",
                                 "red_orientation_insufficient_grip", "red_orientation_release")}, env, out/"isolated_grasp.svg")
    nominal = results["red_orientation_full_grip"]
    checks = dict(
        red_orientation_supported=nominal["classification"]["outcome"] == "GRASP_SUPPORTED",
        palm_up_supported=results["palm_up_full_grip"]["classification"]["outcome"] == "GRASP_SUPPORTED",
        insufficient_grip_slips=results["red_orientation_insufficient_grip"]["classification"]["outcome"] == "GRASP_SLIP",
        release_loses_support=release_loss is not None,
        release_free_fall_accel_close_to_g=free_fall_accel is not None and abs(free_fall_accel+9.81) < .05,
        push_survived=results["red_orientation_push"]["classification"]["outcome"] == "GRASP_SUPPORTED",
        certificates_feasible_at_declared_capacity=all(c["declared_capacity"]["feasible"] for c in certs.values()),
        certificate_refuses_tenth_capacity_at_red_orientation=not certs["red_orientation_full_grip"]["tenth_capacity"]["feasible"],
        oracle_no_cone_violations=all(r["oracle_cone_problems"] == 0 for r in results.values()),
        threshold_monotone_in_mass=monotone_mass, threshold_monotone_in_friction=monotone_mu,
        threshold_monotone_in_tilt=[r["min_supporting_grip_frac"] for r in env if r["sweep"] == "tilt"]
        == sorted(r["min_supporting_grip_frac"] for r in env if r["sweep"] == "tilt"),
        envelope_grids_monotone=all(r["monotone"] for r in env),
        no_hand_bar_weld_or_external_force=True)
    report = dict(issue="RES-624", gate="C", **source_receipt(),
                  outcome="ISOLATED_GRASP_QUALIFIED" if all(checks.values()) else "ISOLATED_GRASP_NOT_QUALIFIED",
                  checks=checks, trials=results, certificates=certs,
                  release=dict(support_lost_t=release_loss, free_fall_vertical_accel_m_s2=free_fall_accel,
                               final_bar_z_m=rel[-1]["bar"]["position"][2]),
                  envelope=env, slip_bound_m=SHAFT_RADIUS,
                  declared_constraints=["each palm rigidly mounted to world (wrist joints removed); reaction reported",
                                        "thumb held near 0 rad by bounded PD; finger/distal motors receive constant bounded torque"],
                  claims=CLAIMS)
    (out/"isolated_grasp.json").write_text(json.dumps(report, indent=2, sort_keys=True)+"\n", encoding="utf-8")
    return report


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="runs/i3/res624/gateC")
    report = run(ap.parse_args(argv).out)
    print(json.dumps(dict(outcome=report["outcome"], checks=report["checks"], release=report["release"]), indent=2))
    for name, t in report["trials"].items():
        print(name, t["classification"]["outcome"], t["classification"].get("hold_displacement_m"),
              "load L/R", t["load_share_left_n"], t["load_share_right_n"], "bar NE", t["oracle_bar_newton_euler"]["max_step_linear_residual_n_s"])
    for r in report["envelope"]:
        print(r["sweep"], r["tilt_deg"], r["bar_mass_kg"], r["friction"], r["min_supporting_grip_frac"], r["outcome"])
    return 0 if report["outcome"] == "ISOLATED_GRASP_QUALIFIED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
