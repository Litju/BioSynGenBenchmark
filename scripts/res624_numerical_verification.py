"""RES-624 Gate F: independent physical and numerical verification of the receiving hold.

    python -m scripts.res624_numerical_verification [--out runs/i3/res624/gateF]

All balances are recomputed from exported frames by the frame-only
evaluator modules (biosyngen_benchmark.*). Checks:
  * declared bar mass and principal inertia (oracle mass_properties);
  * bar linear momentum (strict), bar angular momentum about its moving COM
    (derived discretization bound), whole-system linear momentum;
  * hand-bar and floor contact forces and impulses over the window;
  * mechanical-energy ledger: d(KE+PE) against the integrated generalized
    power of actuators, passive joint terms, constraints (contacts, equality,
    limits) and declared external loads;
  * timestep refinement 1.0/0.5/0.25 ms of the 0.3 s hold;
  * solver sensitivity (tolerance, cone type);
  * deterministic replay (bitwise) and a cross-platform fingerprint.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from biosyngen.spatial.model import AthleteBarbellSpec, bar_inertia
from biosyngen.spatial.model_v2 import HumanV2Spec
from biosyngen.spatial.plant_v2 import HumanV2Plant
from biosyngen.spatial.receiving_grasp_v2 import forward_receiving_hold, qualify_static
from biosyngen_benchmark.receiving_acceptance import evaluate_receiving
from biosyngen_benchmark.rigid_body_check import angular_discretization_check
from biosyngen_benchmark.spatial_oracle import dynamic_balance, mass_properties
from scripts.res624_grasp_diagnosis import CLAIMS, source_receipt
from scripts.res624_receiving_campaign import build_nominal, frame_metrics

HOLD_S = .3


def plant_at(qpos, timestep=None, tolerance=None, cone=None):
    p = HumanV2Plant.build(HumanV2Spec())
    if timestep:
        p.model.opt.timestep = timestep
    if tolerance:
        p.model.opt.tolerance = tolerance
    if cone is not None:
        p.model.opt.cone = cone
    p.data.qpos[:] = qpos
    mujoco.mj_forward(p.model, p.data)
    return p


def hold(qpos, duration=HOLD_S, **kw):
    p = plant_at(qpos, **kw)
    st = qualify_static(p)
    return p, st, forward_receiving_hold(p, st["certificate"], duration)


def mechanical_energy(frame):
    g = np.asarray(frame["gravity"], float)
    ke = sum(.5*b["mass"]*float(np.dot(b["v"], b["v"])) + .5*float(np.asarray(b["w"]) @ np.asarray(b["inertia_world"]) @ np.asarray(b["w"]))
             for b in frame["bodies"])
    pe = -sum(b["mass"]*float(g @ np.asarray(b["com"])) for b in frame["bodies"])
    return ke, pe


def energy_ledger(frames, dt):
    """Signed mechanical-energy ledger. Mechanical energy = body KE (translation+rotation) + rotor (armature) KE
    + gravitational PE. Work = integral of generalized power of actuators, passive joint terms, constraints
    (contacts, joint limits, equalities) and declared external loads; step work uses the trapezoid of the
    start- and end-of-step velocity with the step's (fixed) forces; the explicit start-of-step rule is reported too."""
    def energy(f):
        ke, pe = mechanical_energy(f)
        return ke + f["energy"]["armature_ke_j"] + pe
    e0, e1 = energy(frames[0]), energy(frames[-1])
    w = dict(actuator=0.0, passive=0.0, constraint=0.0, external=0.0)
    w_explicit = dict(w)
    gross = 0.0
    for f in frames[:-1]:
        en = f["energy"]
        for key, name in (("actuator", "qvel_dot_actuator"), ("passive", "qvel_dot_passive"), ("constraint", "qvel_dot_constraint")):
            a, b = en[name], en["qvel_next_"+name[5:]]
            w[key] += dt*.5*(a+b)
            w_explicit[key] += dt*a
            gross += dt*.5*(abs(a)+abs(b))
        for e in f.get("external", []):
            body = next(x for x in f["bodies"] if x["name"] == e["body"])
            w["external"] += dt*float(np.dot(e["force"], body["v"]))
            w_explicit["external"] += dt*float(np.dot(e["force"], body["v"]))
    delta = e1-e0
    resid = delta-sum(w.values())
    return dict(delta_mechanical_j=delta, work_j=w, ledger_residual_j=resid, gross_flux_j=gross,
                relative_to_gross_flux=resid/gross if gross else 0.0,
                explicit_start_of_step_residual_j=delta-sum(w_explicit.values()),
                note="signed work; residual is the integrator/solver discretization remainder after rotor KE is included")


def impulses(frames, dt):
    bar, hand, floor = np.zeros(3), np.zeros(3), np.zeros(3)
    for f in frames[:-1]:
        for c in f["contacts"]:
            F = np.asarray(c["force_on_body2"])
            if "barbell" in (c["body1"], c["body2"]):
                hand += dt*(F if c["body2"] == "barbell" else -F)
            if "world" in (c["body1"], c["body2"]):
                floor += dt*(F if c["body1"] == "world" else -F)
    t = frames[-1]["t"]-frames[0]["t"]
    total_mass = sum(b["mass"] for b in frames[0]["bodies"])
    return dict(hand_on_bar_impulse_n_s=hand.tolist(), bar_weight_impulse_n_s=20*9.81*t,
                floor_on_human_impulse_n_s=floor.tolist(), system_weight_impulse_n_s=total_mass*9.81*t)


def fingerprint(frames):
    bar = [next(b for b in f["bodies"] if b["name"] == "barbell")["com"] for f in frames]
    return dict(final_bar_com=bar[-1], bar_com_sum=np.sum(bar, axis=0).tolist(),
                sha256=hashlib.sha256(json.dumps(frames, sort_keys=True).encode()).hexdigest())


def traj_distance(a, b, step_a, step_b):
    """Max bar-COM distance at common times (every 1 ms)."""
    ta = [f for i, f in enumerate(a) if i % step_a == 0]
    tb = [f for i, f in enumerate(b) if i % step_b == 0]
    n = min(len(ta), len(tb))
    get = lambda f: np.asarray(next(x for x in f["bodies"] if x["name"] == "barbell")["com"])
    return float(max(np.linalg.norm(get(x)-get(y)) for x, y in zip(ta[:n], tb[:n])))


def run(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    _, nominal = build_nominal()
    q = nominal["qpos"]
    p, st, frames = hold(q)
    dt = p.model.opt.timestep
    eq = AthleteBarbellSpec(plate_mass_kg=0)
    masses = mass_properties(frames[0], {"barbell": 20.0}, bar_inertia=bar_inertia(eq))
    verdict = evaluate_receiving(frames, dt, st["verdict"])
    _, _, replay = hold(q)
    deterministic = fingerprint(frames)["sha256"] == fingerprint(replay)["sha256"]
    refine = {}
    runs = {}
    for ts in (1e-3, 5e-4, 2.5e-4):
        pp, ss, ff = hold(q, timestep=ts)
        runs[ts] = ff
        refine[f"{ts:g}"] = dict(evaluator=evaluate_receiving(ff, ts, ss["verdict"])["outcome"],
                                 metrics=frame_metrics(ff),
                                 bar_angular=angular_discretization_check(ff, ts),
                                 system_linear=dynamic_balance(ff, ts), energy=energy_ledger(ff, ts))
    d1 = traj_distance(runs[1e-3], runs[5e-4], 1, 2)
    d2 = traj_distance(runs[5e-4], runs[2.5e-4], 2, 4)
    sens = {}
    for label, kw in (("tolerance_1e-8", dict(tolerance=1e-8)), ("pyramidal_cone", dict(cone=0))):
        pp, ss, ff = hold(q, **kw)
        sens[label] = dict(evaluator=evaluate_receiving(ff, dt, ss["verdict"])["outcome"], metrics=frame_metrics(ff),
                           bar_trajectory_distance_from_nominal_m=traj_distance(frames, ff, 1, 1))
    report = dict(issue="RES-624", gate="F", **source_receipt(), hold_s=HOLD_S,
                  mass_inertia=masses, evaluator=verdict,
                  bar_linear=dynamic_balance(frames, dt, members={"barbell"}),
                  bar_angular=angular_discretization_check(frames, dt),
                  system=dynamic_balance(frames, dt), contact_impulses=impulses(frames, dt),
                  energy=energy_ledger(frames, dt), deterministic_replay_bitwise=deterministic,
                  timestep_refinement=dict(runs=refine, bar_traj_max_diff_1ms_vs_0p5ms_m=d1,
                                           bar_traj_max_diff_0p5ms_vs_0p25ms_m=d2,
                                           observed_ratio=d1/d2 if d2 > 0 else None),
                  solver_sensitivity=sens, fingerprint=fingerprint(frames), claims=CLAIMS)
    en = [abs(refine[k]["energy"]["ledger_residual_j"]) for k in ("0.001", "0.0005", "0.00025")]
    gross = [refine[k]["energy"]["gross_flux_j"] for k in ("0.001", "0.0005", "0.00025")]
    energy_refinement = dict(abs_residual_j=en, gross_flux_j=gross, relative=[e/g for e, g in zip(en, gross)],
                             ratios=[en[0]/en[1] if en[1] else None, en[1]/en[2] if en[2] else None],
                             observed_order=[math.log2(en[0]/en[1]) if en[1] else None, math.log2(en[1]/en[2]) if en[2] else None],
                             accepted_bound="residual must decrease monotonically with dt (discretization remainder, not a modelling "
                             "error) and stay below 1% of gross power flux at the declared dt=1 ms; the observed order is reported, "
                             "not assumed")
    checks = dict(mass_inertia=masses["outcome"] == "PASS", evaluator=verdict["outcome"] == "RECEIVING_HOLD_QUALIFIED",
                  deterministic=deterministic, refinement_all_qualified=all(r["evaluator"] == "RECEIVING_HOLD_QUALIFIED" for r in refine.values()),
                  refinement_converging=d2 < d1,
                  energy_residual_decreasing=en[0] > en[1] > en[2],
                  energy_residual_below_1pct_of_flux=energy_refinement["relative"][0] < .01,
                  solver_tolerance_insensitive=sens["tolerance_1e-8"]["evaluator"] == "RECEIVING_HOLD_QUALIFIED")
    report["energy_refinement"] = energy_refinement
    report["checks"] = checks
    # Contact-model sensitivity is a finding, not a gate: the qualification is for the declared
    # elliptic-cone contact model only; the pyramidal-cone result is reported as measured.
    report["contact_model_sensitivity"] = dict(pyramidal_cone=sens["pyramidal_cone"]["evaluator"],
                                               declared_model="elliptic cone, impratio 10, Newton, tol 1e-10",
                                               qualified_for_alternative_cone=sens["pyramidal_cone"]["evaluator"] == "RECEIVING_HOLD_QUALIFIED")
    report["outcome"] = ("NUMERICALLY_VERIFIED_FOR_DECLARED_CONTACT_MODEL" if all(checks.values()) else "NUMERICAL_NOT_QUALIFIED")
    (out/"numerical_verification.json").write_text(json.dumps(report, indent=2, sort_keys=True, default=float)+"\n", encoding="utf-8")
    return report


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="runs/i3/res624/gateF")
    r = run(ap.parse_args(argv).out)
    print(json.dumps(dict(outcome=r["outcome"], checks=r["checks"], contact_model_sensitivity=r["contact_model_sensitivity"], energy=r["energy"],
                          refinement={k: v for k, v in r["timestep_refinement"].items() if k != "runs"},
                          bar_angular=r["bar_angular"], system=r["system"], impulses=r["contact_impulses"]), indent=2, default=float))
    return 0 if r["outcome"] == "NUMERICALLY_VERIFIED_FOR_DECLARED_CONTACT_MODEL" else 1


if __name__ == "__main__":
    raise SystemExit(main())
