"""RES-507 raw artifact/control qualification; all outputs are synthetic.

Predeclared CMJ grid: masses 67.5/75/82.5 kg × symmetric capacity scales
0.9/1.0/1.1, no gain/posture retuning. All failures enter the denominator.
Step controls use 0.002/0.001/0.0005 s for jump-squat, bench and clean pull.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
from dataclasses import replace

from biosyngen.i2.human import Anthropometry, StrengthHypothesis
from biosyngen.i2.jumps import build_jump
from biosyngen.i2.bench import BENCH_CONTACTS, build_bench_press
from biosyngen.i2.olympic import build_pull, PullProtocol
from biosyngen.i2.export import export_submission
from biosyngen.i2.instruments import observe_dual_plates
from biosyngen.i2.phases import analyze_jump, foot_force_z
from biosyngen.i2.qualification import qualify_context
from biosyngen_benchmark.i2_evaluator import evaluate_i2
from scripts.run_i2_benchmark import families

CEILINGS = ["NUMERICAL_VERIFICATION_ONLY", "SIMULATED_MEASUREMENT_ONLY",
            "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED", "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED"]


def centroid(model, state, include_load=True):
    bodies = [b for b in model.system.bodies if include_load or b.body_id != model.load_body_id]
    mass = sum(b.mass_kg for b in bodies)
    return (sum(b.mass_kg * state.bodies[b.body_id].position_m[1] for b in bodies) / mass,
            sum(b.mass_kg * state.bodies[b.body_id].velocity_m_s[1] for b in bodies) / mass)


def stage_diagnostics(frames, forces):
    stages = {}
    for frame, force in zip(frames, forces):
        stages.setdefault(frame.controller_stage, []).append((frame.t_end_s, force))
    result = {}
    for stage, rows in stages.items():
        values = [f for _, f in rows]
        info = {"n": len(rows), "peak_interval_force_n": max(values), "range_n": max(values)-min(values),
                "sd_n": statistics.pstdev(values), "duration_s": rows[-1][0]-rows[0][0]}
        if "LOCKOUT" in stage and len(rows) >= 50:
            # Raw, unwindowed irregular-time Fourier diagnostic; no filtering,
            # interpolation or claim of physiological tremor frequency.
            mean = statistics.fmean(values)
            scores = [(sum((v-mean)*math.cos(2*math.pi*f*t) for t,v in rows)**2
                       + sum((v-mean)*math.sin(2*math.pi*f*t) for t,v in rows)**2, f) for f in range(2,151)]
            info["raw_periodogram_peak_hz"] = max(scores)[1]
            info["frequency_scope"] = "unwindowed raw controller-stage diagnostic, not a human frequency estimate"
        result[stage] = info
    return result


def inspect_case(name, scenario, out, dt=0.001):
    trajectory = scenario.run(dt_s=dt)
    model = trajectory.model
    frames = trajectory.frames
    evaluation = evaluate_i2(export_submission(trajectory))
    foot = {c for contacts in model.contact_ids.values() for c in contacts}
    forces = [foot_force_z(trajectory, f) for f in frames]
    plates = observe_dual_plates(trajectory) if frames else None
    result = {"dt_s": dt, "termination": trajectory.termination, "context": qualify_context(trajectory),
              "numerical_findings": evaluation.get("findings", {}), "numerical_status": evaluation["status"],
              "model_task_completed": trajectory.termination == "DURATION_REACHED",
              "stage_diagnostics": stage_diagnostics(frames, forces) if frames else {},
              "foot_impact_impulse_z_n_s": sum(i.impulse_world_n_s[1] for i in trajectory.impacts if i.contact_id in foot),
              "foot_impacts": sum(i.contact_id in foot for i in trajectory.impacts),
              "peak_interval_foot_force_n": max(forces) if forces else None,
              "virtual_adc_peak_fz_total_n": max(s.fz_total_n for s in plates.samples) if plates else None,
              "virtual_adc_saturated_samples": sum(s.saturated for s in plates.samples) if plates else None}
    if trajectory.exercise_id in {"CMJ","SJ","LOADED_CMJ","LOADED_JUMP_SQUAT"} and frames:
        analysis = analyze_jump(trajectory)
        result["jump"] = {"takeoff_s": analysis.time("takeoff"), "touchdown_s": analysis.time("touchdown"),
                          "takeoff_system_vz_m_s": analysis.takeoff_velocity_m_s, "refusals": analysis.refusals}
        result["model_task_completed"] &= (analysis.time("takeoff") is not None and analysis.time("touchdown") is not None)
    result["initial_activations"] = dict(trajectory.initial_state.activations)
    result["controller_id"] = scenario.controller.controller_id
    trace = out / (name + ".csv")
    with trace.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["t_start_s","t_end_s","controller_stage","foot_interval_force_z_n","bench_force_z_n",
                         "rack_force_z_n","bar_floor_force_z_n","body_com_z_m","body_com_vz_m_s",
                         "system_com_z_m","system_com_vz_m_s","bar_z_m","bar_vz_m_s","bar_shelf_gap_m"])
        for frame, force in zip(frames, forces):
            b_z,b_v=centroid(model,frame.state,False)
            s_z,s_v=centroid(model,frame.state)
            load=frame.state.bodies.get(model.load_body_id)
            gap=(model.system.contact_kinematics(frame.state,model.load_contact_id).gap_m
                 if model.load_contact_id else None)
            reaction=lambda c: frame.contact_forces[c].force_world_n[1] if c in frame.contact_forces else 0
            writer.writerow([frame.t_start_s,frame.t_end_s,frame.controller_stage,force,
                             sum(reaction(c) for c in BENCH_CONTACTS),reaction("rack_hooks"),reaction("bar_floor"),
                             b_z,b_v,s_z,s_v,load.position_m[1] if load else None,load.velocity_m_s[1] if load else None,gap])
    impacts=out/(name+"-impulses.csv")
    with impacts.open("w",encoding="utf-8",newline="") as stream:
        writer=csv.writer(stream,lineterminator="\n")
        writer.writerow(["time_s","contact_id","impulse_x_n_s","impulse_z_n_s"])
        writer.writerows((i.time_s,i.contact_id,*i.impulse_world_n_s) for i in trajectory.impacts)
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out",default="runs/i3/res507/artifacts")
    args=parser.parse_args(argv)
    project=Path(__file__).resolve().parents[1]
    out=(project/args.out).resolve()
    if not out.is_relative_to(project) or out==project:
        parser.error("outputs must remain in a project subdirectory")
    if out.exists() and any(out.iterdir()):
        parser.error("use a fresh output directory; existing evidence is preserved")
    out.mkdir(parents=True,exist_ok=True)
    report={"issue":"RES-507","claim_ceilings":CEILINGS,"cases":{},"grid":[],
            "grid_definition":{"mass_kg":[67.5,75.0,82.5],"capacity_scale":[0.9,1.0,1.1],"retuning":False}}
    for name,scenario in families():
        print("BASELINE",name,flush=True)
        report["cases"][name]=inspect_case(name,scenario,out)
    for mass in (67.5,75.0,82.5):
        for scale in (0.9,1.0,1.1):
            name=f"CMJ_M{mass}_S{scale}"
            if mass==75 and scale==1:
                result=report["cases"]["CMJ"]
                name="CMJ"
            else:
                print("GRID",name,flush=True)
                try:
                    scenario=build_jump("CMJ",anthropometry=Anthropometry(body_mass_kg=mass),
                                        strength=StrengthHypothesis(left_scale=scale,right_scale=scale))
                    result=inspect_case(name,scenario,out)
                except (ValueError, ArithmeticError) as exc:
                    result={"model_task_completed":False,"termination":"CASE_REFUSED_OR_FAILED", "error":str(exc)}
                report["cases"][name]=result
            report["grid"].append({"case":name,"mass_kg":mass,"capacity_scale":scale,"completed":result["model_task_completed"]})
    for kind in ("LOADED_JUMP_SQUAT","BENCH_PRESS","CLEAN_PULL"):
        for dt in (0.002,0.0005):
            name=kind+f"_DT{dt}"
            print("REFINEMENT",name,flush=True)
            scenario=(build_jump(kind) if kind=="LOADED_JUMP_SQUAT" else
                      build_bench_press() if kind=="BENCH_PRESS" else build_pull(PullProtocol(variant=kind)))
            report["cases"][name]=inspect_case(name,scenario,out,dt)
    report["grid_completed"]={"n":len(report["grid"]),"completed":sum(c["completed"] for c in report["grid"]),
                              "interpretation":"curated deterministic model grid, not a human population feasibility rate"}
    (out/"qualification.json").write_text(json.dumps(report,indent=2,sort_keys=True)+"\n",encoding="utf-8",newline="\n")
    print(json.dumps({"grid":report["grid_completed"],"cases":len(report["cases"])}))
    return 0


if __name__=="__main__":
    raise SystemExit(main())
