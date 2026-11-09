"""RES-609 drop-jump energy RED/GREEN and final-recovery witnesses.

    python -m scripts.res609_energy_and_recovery --out runs/i3/res598/working/res609-energy

Energy: the 0.5 s drop candidate (through first impact, rebound and second
flight) under the historical EXPLICIT_INTERVAL_START_V1 contact scheme and
the IMPLICIT_DISCRETE_GRADIENT_V2 scheme at 0.4/0.2/0.1 ms. The independent
evaluator recomputes contact work from exported forces and contact-point
displacements, the passive-dissipation sign test and the discrete ledger.

Recovery: full 3 s trials at 0.4 ms with V2 contact and V2 projection under
the historical stages and the two new repair attempts. Each must report its
actual termination; none is relabelled as completed. The script asserts no
success; it records witnesses. NUMERICAL_VERIFICATION_ONLY.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

SCHEMES = ("EXPLICIT_INTERVAL_START_V1", "IMPLICIT_DISCRETE_GRADIENT_V2")
STEPS = (.0004, .0002, .0001)
ENERGY_CONFIGS = ((SCHEMES[0], "MASS_METRIC_V1"), (SCHEMES[1], "MASS_METRIC_V1"),
                  (SCHEMES[1], "MASS_METRIC_ANGULAR_MOMENTUM_RESTORING_V2"))
RECOVERY = ("HISTORICAL_STAGES", "ATTEMPT_1_CMJ_FLIGHT_ABSORB_STAND", "ATTEMPT_2_BALANCED_PROPULSION_PLUS_1")


def stored(trial, law, state):
    system = trial.model.system
    return sum(law.potential(system.contact_kinematics(state, cid).gap_m)
               for ids in trial.model.contact_ids.values() for cid in ids)


def energy_case(args):
    scheme, dt, projection = args
    from biosyngen.i2.drop_jump import build_drop_jump_candidate
    from biosyngen.i2.export import export_submission
    from biosyngen_benchmark.i2_evaluator import discrete_energy_ledger, energy_diagnostics, evaluate_i2
    scenario = build_drop_jump_candidate(duration_s=.5)
    scenario.model.system = scenario.model.system.with_projection_policy(projection)
    started = time.perf_counter()
    trial = scenario.run(dt, compliant_scheme=scheme)
    wall = time.perf_counter() - started
    law = scenario.contact_law
    submission = export_submission(trial)
    submission["claims"].append("ANGULAR_MOMENTUM_CLOSURE")
    findings = evaluate_i2(submission)["findings"]
    h = [f.t_end_s - f.t_start_s for f in trial.frames]
    u0, u1 = stored(trial, law, trial.initial_state), stored(trial, law, trial.frames[-1].state)
    legacy = energy_diagnostics(
        submission, joint_work_j=sum(f.joint_power_w*dt_ for f, dt_ in zip(trial.frames, h)),
        contact_dissipation_j=sum(sum(f.contact_dissipated_power_w.values())*dt_ for f, dt_ in zip(trial.frames, h)),
        initial_stored_energy_j=u0, final_stored_energy_j=u1,
        numerical_projection_work_j=sum(f.constraint_projection_work_j for f in trial.frames),
        rigid_contact_work_estimate_j=sum(f.rigid_contact_work_estimate_j for f in trial.frames))
    ledger = discrete_energy_ledger(
        submission, groups=("FOOT_LEFT", "FOOT_RIGHT"),
        joint_work_discrete_j=sum(f.joint_work_discrete_j for f in trial.frames),
        initial_stored_energy_j=u0, final_stored_energy_j=u1,
        numerical_projection_work_j=sum(f.constraint_projection_work_j for f in trial.frames),
        declared_contact_dissipation_j=legacy.get("contact_dissipation_j"))
    return dict(scheme=scheme, projection=projection, dt_s=dt, termination=trial.termination, wall_s=wall,
                generator_declared_dissipation_j=legacy.get("contact_dissipation_j"),
                legacy_raw_residual_j=legacy.get("raw_energy_residual_j"),
                independent=ledger, linear=findings["LINEAR_MOMENTUM_CLOSURE"]["outcome"],
                angular=findings["ANGULAR_MOMENTUM_CLOSURE"]["outcome"],
                contact=findings["CONTACT_ADMISSIBILITY"]["outcome"])


def recovery_case(name):
    from biosyngen.i2.drop_jump import build_drop_jump_candidate
    from biosyngen.i2.human import balanced_posture
    from biosyngen.i2.jumps import JumpStrategy, _gains, feet_released, foot_touched
    from biosyngen.i2.simulate import ControlStage
    from biosyngen.multibody import PROJECTION_ANGULAR_NEUTRAL_V2
    scenario = build_drop_jump_candidate(duration_s=3.0)
    model = scenario.model
    model.system = model.system.with_projection_policy(PROJECTION_ANGULAR_NEUTRAL_V2)
    if name != "HISTORICAL_STAGES":
        st, x = JumpStrategy(), .05
        stance = balanced_posture(model, *st.stance_ankle_knee, x)
        absorb = balanced_posture(model, *st.absorb_ankle_knee, x)
        kb, db = st.balance_gain, st.balance_damping
        stages = list(scenario.controller.stages)
        names = [s.name for s in stages]
        tail = [
            ControlStage("FLIGHT", st.flight, _gains(600, 600, 600), _gains(40, 40, 40), False, foot_touched, "foot contact sensed"),
            ControlStage("LANDING", absorb, _gains(st.landing_ankle_kp, 1500, 1500), _gains(st.landing_ankle_kd, 250, 250),
                         True, lambda m, s, t: t > st.landing_time_s, "declared absorption time", .15, kb, db),
            ControlStage("STAND", stance, _gains(st.landing_ankle_kp, 800, 800), _gains(100, 100, 100), True, None, "", .5, kb, db)]
        if name == "ATTEMPT_2_BALANCED_PROPULSION_PLUS_1":
            b, r = names.index("BRAKING"), names.index("REACTIVE_PROPULSION")
            stages[b] = replace(stages[b], balance_gain_n_m_per_m=kb, balance_damping_n_m_s_per_m=db)
            stages[r] = ControlStage("REACTIVE_PROPULSION", st.extension, _gains(5000, 5000, 5000), _gains(20, 20, 20), True,
                                     feet_released, "both feet released (touch)", 0.0, kb, db, {"ankle": st.ankle_effort})
            stages[r + 1:] = tail
        else:
            stages[names.index("FLIGHT"):] = tail
        scenario.controller.stages = stages
        scenario.controller.balance_reference_x_m = x
    trial = scenario.run(.0004, compliant_scheme="IMPLICIT_DISCRETE_GRADIENT_V2")
    system = model.system
    flight = [f for f in trial.frames if f.controller_stage == "FLIGHT"]
    takeoff = flight[0].state if flight else None
    touchdown = next((f.state for f in trial.frames if f.controller_stage in ("FINAL_LANDING", "LANDING")), None)
    out = dict(recovery=name, termination=trial.termination, end_time_s=trial.frames[-1].t_end_s,
               commands=[[t, s, r] for t, s, r in trial.control_commands])
    if takeoff is not None:
        out["takeoff_centroidal_angular_momentum_kg_m2_s"] = system.angular_momentum(takeoff, system.center_of_mass(takeoff))
    if touchdown is not None:
        ankle = sum(model.point_world(touchdown, f"{side}_foot", model.foot_points["ankle"])[0] for side in ("left", "right")) / 2
        out["touchdown_mean_ankle_minus_com_x_m"] = ankle - system.center_of_mass(touchdown)[0]
    final = trial.frames[-1].state
    out["terminal_linear_momentum_kg_m_s"] = list(system.linear_momentum(final))
    out["completed"] = trial.termination == "DURATION_REACHED"
    return out


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
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    dirty = bool(subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], text=True).strip())
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        recovery = pool.map(recovery_case, RECOVERY)
        energy = list(pool.map(energy_case, [(s, dt, p) for s, p in ENERGY_CONFIGS for dt in STEPS]))
        recovery = list(recovery)
    receipt = dict(
        issue="RES-609", source_sha=sha, source_dirty=dirty, python=sys.version.split()[0],
        evaluator_sha256=hashlib.sha256(Path("biosyngen_benchmark/i2_evaluator.py").read_bytes()).hexdigest(),
        contact_sha256=hashlib.sha256(Path("biosyngen/compliant_contact.py").read_bytes()).hexdigest(),
        energy=energy, recovery=recovery, completed_drop_jumps=sum(r["completed"] for r in recovery),
        wall_s=time.monotonic() - started,
        claim_ceilings=["NUMERICAL_VERIFICATION_ONLY", "SIMULATED_MEASUREMENT_ONLY",
                        "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED", "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED"])
    (out / "energy_and_recovery.json").write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")
    print("scheme / projection                         dt      legacy_res  W_contact   D_indep   D_num    closure   passive declared")
    for e in energy:
        i = e["independent"]
        print(f"{e['scheme'][:29]:29s}/{e['projection'][:14]:14s} {e['dt_s']:.4f} {e['legacy_raw_residual_j']:10.4f} "
              f"{i['passive_contact']['contact_work_j']:10.4f} {i['passive_contact']['contact_dissipation_j']:9.4f} "
              f"{i['ledger']['numerical_dissipation_j']:8.4f} {i['ledger']['closure_residual_j']:9.5f} "
              f"{i['passive_contact']['outcome']} {i['declared_dissipation']['outcome']}  lin={e['linear']} ang={e['angular']}")
    for r in recovery:
        print(r["recovery"], r["termination"], round(r["end_time_s"], 4),
              "L_takeoff", round(r.get("takeoff_centroidal_angular_momentum_kg_m2_s", float("nan")), 3),
              "ankle-COM", round(r.get("touchdown_mean_ankle_minus_com_x_m", float("nan")), 3))
    print("completed drop jumps:", receipt["completed_drop_jumps"], "/", len(recovery))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
