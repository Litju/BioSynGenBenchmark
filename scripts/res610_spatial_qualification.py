"""RES-610 spatial receiving-configuration qualification and retained RED witnesses.

    python -m scripts.res610_spatial_qualification --out runs/i3/res598/working/res610-spatial

For each declared configuration: symmetric pose solve (feet, grips, shelf,
clearance), static quasi-equilibrium QP, independent oracle checks of that
static solution (whole system, bar alone, contact admissibility, centre of
pressure), and a 1.5 s bounded-torque hold with a 6 N s bar push checked
frame by frame by the oracle (discrete Newton-Euler for system and bar,
admissibility, undeclared contacts), deterministic replay and a timestep
refinement. RED witnesses: the closed-palm front rack and the uniform-servo
front-rack hold. A static pose or hold is not a clean or snatch.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import time

CONFIGS = (
    ("FRONT_RACK", "POWER", "OPEN_FINGER_RACK_V1", "RELAXED_RACK_ARMS_V1"),
    ("FRONT_RACK", "FULL", "OPEN_FINGER_RACK_V1", "RELAXED_RACK_ARMS_V1"),
    ("OVERHEAD", "POWER", "CLOSED_PALM_V1", "UNIFORM_V1"),
    ("OVERHEAD", "FULL", "CLOSED_PALM_V1", "UNIFORM_V1"),
    # Retained RED witnesses.
    ("FRONT_RACK", "POWER", "CLOSED_PALM_V1", "RELAXED_RACK_ARMS_V1"),
    ("FRONT_RACK", "POWER", "OPEN_FINGER_RACK_V1", "UNIFORM_V1"),
)


def run_config(args):
    receiving, depth, grip, servo, dt = args
    import mujoco
    import numpy as np
    from biosyngen.spatial.model import bar_inertia, mass_table
    from biosyngen.spatial.plant import SpatialPlant
    from biosyngen.spatial import receiving as R
    from biosyngen_benchmark import spatial_oracle as O
    started = time.perf_counter()
    spec = replace(R.default_spec(receiving, grip), timestep_s=dt)
    plant = SpatialPlant.build(spec)
    x, closure, solve = R.solve_pose(plant, receiving, depth)
    lo, hi = R._limits()
    at_bounds = [k for k, a, b, c in zip(R.VARIABLES, x, lo, hi) if abs(a - b) < 1e-4 or abs(a - c) < 1e-4]
    thigh, trunk = R.thigh_inclination(plant), R.trunk_inclination(plant)
    clearance = min(R.bar_clearances(plant).values())
    certificate = R.static_equilibrium(plant, receiving, depth=depth)
    closed = closure["max_grip_separation_m"] <= R.POSE_TOLERANCE_M and all(
        abs(g + 5e-4) <= R.POSE_TOLERANCE_M for g in closure["shelf_gaps_m"])
    static_frame = plant.export_frame()
    static_frame["contacts"], static_frame["grips"] = certificate.contacts, certificate.grips
    declared = {"barbell": spec.total_load_kg, **mass_table(spec)}
    oracle_static = dict(
        system=O.static_balance(static_frame), bar=O.static_balance(static_frame, members={"barbell"}),
        contacts=O.contact_admissibility(static_frame), support=O.support_polygon(static_frame, margin_m=.03),
        mass=O.mass_properties(static_frame, declared, bar_inertia=bar_inertia(spec)))
    feasible = closed and certificate.feasible and all(
        v["outcome"] == "PASS" for v in oracle_static.values())
    reason = certificate.reason
    if not closed:
        reason = "POSE_NOT_CLOSED"
    elif certificate.feasible and not feasible:
        reason = "INDEPENDENT_ORACLE_REJECTED"
    hold = R.hold_trial(plant, x, certificate, servo_profile=servo)
    frames = hold["frames"]
    undeclared = sorted({(c["geom1"], c["geom2"]) for f in frames for c in f["contacts"]
                         if not R.declared_support(receiving, c["geom1"], c["geom2"])
                         and np.linalg.norm(c["force_on_body2"]) > 1e-6})
    admissible = sum(O.contact_admissibility(f)["outcome"] == "PASS" for f in frames)
    support = [O.support_polygon(f)["outcome"] for f in frames]
    dyn = O.dynamic_balance(frames, dt)
    dyn_bar = O.dynamic_balance(frames, dt, members={"barbell"})
    violations = R.joint_range_violations(plant, hold["joint_angles"])
    final_hash = hashlib.sha256(json.dumps([round(v, 12) for v in plant.data.qpos.tolist()]).encode()).hexdigest()
    stable = (hold["saturated_steps"] == 0 and not violations and not undeclared and admissible == len(frames)
              and support.count("PASS") == len(support) and hold["final_max_speed"] < .5
              and hold["max_grip_separation_m"] < 1e-3)
    return dict(
        receiving=receiving, depth=depth, grip=grip, servo=servo, dt_s=dt, grip_model=spec.grip_model,
        grip_half_width_m=spec.grip_half_width_m, grip_site_distal_m=spec.grip_site_distal_m,
        pose=dict(variables=dict(zip(R.VARIABLES, map(float, x))), closure=closure, closed=closed,
                  at_margin_bounds=at_bounds, thigh_inclination_rad=thigh, depth_label=R.depth_label(thigh),
                  trunk_inclination_rad=trunk, min_bar_clearance_m=clearance,
                  solve_cost=float(solve.cost), solve_nfev=int(solve.nfev)),
        static=dict(feasible=feasible, reason=reason, certificate_reason=certificate.reason,
                    max_torque_ratio=certificate.max_torque_ratio, free_residual=certificate.free_residual,
                    torques_n_m=certificate.torques_n_m, details=certificate.details,
                    contact_forces=[dict(geom1=c["geom1"], geom2=c["geom2"], force=c["force_on_body2"]) for c in certificate.contacts],
                    grip_forces=[dict(name=g["name"], force=g["force_on_body1"]) for g in certificate.grips]),
        oracle_static=oracle_static,
        hold=dict(stable=stable, saturated_steps=hold["saturated_steps"], steps=hold["steps"],
                  final_max_speed=hold["final_max_speed"], max_grip_separation_m=hold["max_grip_separation_m"],
                  range_violations=violations, undeclared_contacts=undeclared,
                  admissible_frames=admissible, frames=len(frames), support_pass_frames=support.count("PASS"),
                  newton_euler_system=dyn, newton_euler_bar=dyn_bar,
                  energy=dict(initial_j=hold["energy0_j"], final_j=hold["energy_final_j"],
                              actuator_work_start_force_j=hold["actuator_work_j"],
                              external_work_j=hold["external_work_j"], joint_damping_work_j=hold["damping_work_j"]),
                  final_qpos_sha256=final_hash),
        wall_s=time.perf_counter() - started, mujoco=mujoco.__version__)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True)
    parser.add_argument("--workers", type=int, default=6)
    args = parser.parse_args(argv)
    out = Path(args.out)
    if out.exists():
        parser.error("output must be a fresh directory")
    out.mkdir(parents=True)
    started = time.monotonic()
    jobs = [(*c, .001) for c in CONFIGS]
    jobs += [("OVERHEAD", "POWER", "CLOSED_PALM_V1", "UNIFORM_V1", .001),     # replay
             ("OVERHEAD", "POWER", "CLOSED_PALM_V1", "UNIFORM_V1", .0005),    # refinement
             ("FRONT_RACK", "POWER", "OPEN_FINGER_RACK_V1", "RELAXED_RACK_ARMS_V1", .0005)]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(run_config, jobs))
    main_rows, extra = rows[:len(CONFIGS)], rows[len(CONFIGS):]
    replay = dict(first=main_rows[2]["hold"]["final_qpos_sha256"], second=extra[0]["hold"]["final_qpos_sha256"])
    replay["identical"] = replay["first"] == replay["second"]
    refinement = [dict(config=f"{r['receiving']}/{r['depth']}", dt_s=r["dt_s"], stable=r["hold"]["stable"],
                       max_step_linear_residual_n_s=r["hold"]["newton_euler_system"]["max_step_linear_residual_n_s"],
                       max_step_angular_residual_n_m_s=r["hold"]["newton_euler_system"]["max_step_angular_residual_n_m_s"])
                  for r in (main_rows[0], extra[2], main_rows[2], extra[1])]
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    dirty = bool(subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], text=True).strip())
    receipt = dict(issue="RES-610", source_sha=sha, source_dirty=dirty,
                   host=dict(platform=platform.platform(), python=sys.version.split()[0]),
                   configurations=main_rows, replay=replay, refinement=refinement,
                   feasible_static=[f"{r['receiving']}/{r['depth']}/{r['grip']}" for r in main_rows if r["static"]["feasible"]],
                   stable_holds=[f"{r['receiving']}/{r['depth']}/{r['grip']}/{r['servo']}" for r in main_rows
                                 if r["static"]["feasible"] and r["hold"]["stable"]],
                   wall_s=time.monotonic() - started,
                   claim_ceilings=["NUMERICAL_VERIFICATION_ONLY", "SIMULATED_MEASUREMENT_ONLY",
                                   "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED", "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED"],
                   scope="Static receiving equilibria and perturbed holds only; no catch, clean or snatch simulated")
    (out / "spatial_qualification.json").write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")
    for r in main_rows:
        h = r["hold"]
        print(f"{r['receiving']:10s} {r['depth']:5s} {r['grip']:20s} {r['servo']:21s} static={r['static']['reason']:26s} "
              f"ratio={r['static']['max_torque_ratio']:.3f} grip={r['pose']['closure']['max_grip_separation_m']:.2e} "
              f"hold_stable={h['stable']} sat={h['saturated_steps']} undeclared={len(h['undeclared_contacts'])} "
              f"NE={h['newton_euler_system']['max_step_linear_residual_n_s']:.2e}")
    print("replay identical:", replay["identical"])
    for row in refinement:
        print("refinement", row)
    print("feasible static:", receipt["feasible_static"])
    print("stable holds:", receipt["stable_holds"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
