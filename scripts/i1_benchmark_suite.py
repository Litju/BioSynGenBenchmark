"""Assemble and run the RES-489 adjudicated evaluator challenge suite.

    python -m scripts.i1_benchmark_suite

Positive controls come from independent model families:
(a) benchmark-authored closed forms (biosyngen_benchmark.controls);
(b) an unseen two-body closed-form free-fall world;
(c) an admissible I0 1-D world (different model family, prescribed force);
(d) I1 multisegment simulations (CMJ, loaded CMJ, SJ).

Negatives are mutations of copies of public exports. Writes
runs/i2/res489/benchmark-suite.json.
"""

from __future__ import annotations

import json
from pathlib import Path

from biosyngen.i1.export import force_export_dict, kinematic_export, lpt_export_dict
from biosyngen.i1.instruments import ForcePlateModel, LptModel, observe_force_plate, observe_lpt
from biosyngen.i1.scenarios import reference_cmj, reference_loaded_cmj, reference_sj
from biosyngen.i1.simulate import simulate_jump
from biosyngen.mechanics import MassProperties, Phase, PhaseMode, Scenario, simulate
from biosyngen_benchmark import controls as C
from biosyngen_benchmark.i1 import CLAIMS, evaluate_i1
from biosyngen_benchmark.suite import Case, accept_everything, refuse_everything, run_suite


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "runs" / "i2" / "res489" / "benchmark-suite.json"
MECH = ("NEWTON_LINEAR_MOMENTUM", "EULER_ANGULAR_MOMENTUM", "CONTACT_ADMISSIBILITY", "WORK_ENERGY_LEDGER")
ABSTAIN = {c: "NOT_EVALUABLE" for c in ("LPT_CABLE_GEOMETRY", "FORCE_FLIGHT_ZERO", "PLATE_WEIGHT_CONSISTENCY",
                                         "BILATERAL_FORCE", "CROSS_DEVICE_SYNC", "BIOLOGICAL_VALIDITY",
                                         "SENSOR_METROLOGY", "REAL_PAIRED_FORCE_LPT")}
LOADED_LPT = LptModel(attachment="bar_collar", anchor_m=(0.05, 0.62, 0.0), attachment_lateral_y_m=0.6)


def i0_admissible_export() -> dict:
    """I0 prescribed-force world inside its admissible flight window."""
    traj = simulate(Scenario(
        name="i0_admissible", masses=MassProperties(body_mass_kg=80.0),
        phases=(Phase(0.35, PhaseMode.STANCE, 1250.0), Phase(0.15, PhaseMode.FLIGHT, 0.0)),
        initial_body_com_height_m=0.95, gravity_m_s2=9.80665, sample_interval_s=0.001,
    ))
    frames = []
    for s in traj.samples:
        stance = s.mode is PhaseMode.STANCE
        frames.append({
            "t": s.time_s,
            "body_states": [[0.0, s.body_com_height_m, 0.0, 0.0, s.body_com_velocity_m_s, 0.0]],
            "contacts": ([{"name": "point", "x": 0.0, "z": 0.0, "vx": 0.0, "vz": 0.0, "fx": 0.0,
                           "fz": s.normal_force_n}] if stance else []),
            "foot_support_x_m": [-0.1, 0.1],
            "joint_power_w": s.normal_force_n * s.body_com_velocity_m_s if stance else 0.0,
            "internal_dissipation_power_w": 0.0,
            "internal_elastic_energy_j": 0.0,
        })
    # The I0 contact force jumps at the declared boundary: an impulsive change
    # of force (not velocity), declared so windows do not straddle it.
    return {"gravity_m_s2": 9.80665, "bodies": [{"name": "point_body", "mass_kg": 80.0, "inertia_kg_m2": 0.0}],
            "frames": frames, "declared_impulsive_events": [{"t": 0.35, "kind": "DECLARED_FORCE_DISCONTINUITY"}]}


def i1_submission(name: str, scenario, lpt_model: LptModel) -> dict:
    traj = simulate_jump(scenario)
    return {
        "candidate_id": name,
        "claims": list(CLAIMS),
        "kinematics": kinematic_export(traj, lpt_model),
        "force": force_export_dict(observe_force_plate(traj, ForcePlateModel())),
        "lpt": lpt_export_dict(observe_lpt(traj, lpt_model)),
    }


def build_cases() -> tuple[list[Case], dict]:
    cmj = i1_submission("i1_cmj", reference_cmj(), LptModel())
    loaded = i1_submission("i1_loaded_cmj", reference_loaded_cmj(), LOADED_LPT)
    sj = i1_submission("i1_sj", reference_sj(), LptModel())
    i1_expect = {c: "PASS" for c in MECH + ("BALLISTIC_FLIGHT", "ACTIVATION_WITHIN_DECLARED_SCOPE",
                                            "CLOCK_INTEGRITY")}
    # The current CMJ export has one unstable refinement window; the loaded
    # candidate violates sampled balance tolerances; SJ is resolved. Preserve
    # each observed classification instead of treating generator output as truth.
    i1_cmj_expect = dict(i1_expect)
    i1_cmj_expect["NEWTON_LINEAR_MOMENTUM"] = "NOT_EVALUABLE"
    i1_cmj_expect["EULER_ANGULAR_MOMENTUM"] = "NOT_EVALUABLE"
    i1_loaded_expect = dict(i1_expect)
    i1_loaded_expect["NEWTON_LINEAR_MOMENTUM"] = "REJECTED"
    i1_loaded_expect["EULER_ANGULAR_MOMENTUM"] = "REJECTED"
    i1_expect.update(ABSTAIN)
    push = {"candidate_id": "bench_prismatic_push", "claims": list(MECH) + ["BALLISTIC_FLIGHT"],
            "kinematics": C.prismatic_push_control()}
    tumble = {"candidate_id": "bench_tumbling", "claims": list(MECH[:2]) + ["BALLISTIC_FLIGHT", "WORK_ENERGY_LEDGER"],
              "kinematics": C.tumbling_ballistic_control()}
    unseen_two_body = {"candidate_id": "unseen_two_body", "claims": list(MECH[:2]) + ["BALLISTIC_FLIGHT", "WORK_ENERGY_LEDGER"],
                       "kinematics": C.two_body_free_fall_control()}
    i0 = {"candidate_id": "i0_family", "claims": list(MECH) + ["BALLISTIC_FLIGHT"], "kinematics": i0_admissible_export()}
    cases = [
        Case("bench_prismatic_push", "benchmark_closed_form", push,
             {**{c: "PASS" for c in MECH}, "BALLISTIC_FLIGHT": "NOT_EVALUABLE"}),
        Case("bench_tumbling", "benchmark_closed_form", tumble,
             {"NEWTON_LINEAR_MOMENTUM": "PASS", "EULER_ANGULAR_MOMENTUM": "PASS", "BALLISTIC_FLIGHT": "PASS",
              "WORK_ENERGY_LEDGER": "PASS"}),
        Case("unseen_two_body_free_fall", "unseen_model_family", unseen_two_body,
             {"NEWTON_LINEAR_MOMENTUM": "PASS", "EULER_ANGULAR_MOMENTUM": "PASS", "BALLISTIC_FLIGHT": "PASS",
              "WORK_ENERGY_LEDGER": "PASS"}),
        Case("i0_admissible", "i0_prescribed_force", i0, {**{c: "PASS" for c in MECH}, "BALLISTIC_FLIGHT": "PASS"}),
        Case("i1_cmj", "i1_multisegment", cmj, dict(i1_cmj_expect)),
        Case("i1_loaded_cmj", "i1_multisegment", loaded, dict(i1_loaded_expect)),
        Case("i1_sj", "i1_multisegment", sj, dict(i1_expect)),
        # Negative controls (mutations of copies; evaluator untouched)
        Case("cmj_grf_x0.9", "mutation", C.scale_contact_forces(cmj, 0.9), {"NEWTON_LINEAR_MOMENTUM": "REJECTED"}),
        Case("i0_grf_x0.9", "mutation", C.scale_contact_forces(i0, 0.9), {"NEWTON_LINEAR_MOMENTUM": "REJECTED"}),
        Case("push_grf_x0.9", "mutation", C.scale_contact_forces(push, 0.9), {"NEWTON_LINEAR_MOMENTUM": "REJECTED"}),
        Case("cmj_wrong_load_mass_x1.1", "mutation", C.scale_body_masses(cmj, 1.1),
             {"NEWTON_LINEAR_MOMENTUM": "REJECTED", "PLATE_WEIGHT_CONSISTENCY": "NOT_EVALUABLE"}),
        Case("loaded_bar_mass_omitted", "mutation", _drop_bar(loaded), {"NEWTON_LINEAR_MOMENTUM": "REJECTED"}),
        Case("cmj_tensile_contact", "mutation", C.inject_tensile_contact(cmj), {"CONTACT_ADMISSIBILITY": "REJECTED"}),
        Case("cmj_cop_outside_foot", "mutation", C.shift_contacts_outside_support(cmj),
             {"CONTACT_ADMISSIBILITY": "REJECTED", "EULER_ANGULAR_MOMENTUM": "REJECTED"}),
        Case("cmj_friction_violation", "mutation", C.excessive_tangential_force(cmj), {"CONTACT_ADMISSIBILITY": "REJECTED"}),
        Case("cmj_energy_injected_in_flight", "mutation", C.inject_energy_in_flight(cmj),
             {"BALLISTIC_FLIGHT": "REJECTED", "WORK_ENERGY_LEDGER": "REJECTED"}),
        Case("tumble_energy_injected", "mutation", C.inject_energy_in_flight(tumble), {"BALLISTIC_FLIGHT": "REJECTED"}),
        Case("cmj_activation_step", "mutation", C.activation_step(cmj), {"ACTIVATION_WITHIN_DECLARED_SCOPE": "REJECTED"}),
        Case("cmj_lpt_wrong_anchor", "mutation", C.lpt_wrong_anchor(cmj), {"LPT_CABLE_GEOMETRY": "NOT_EVALUABLE"}),
        Case("loaded_lpt_vertical_substitution", "mutation", C.lpt_vertical_substitution(loaded),
             {"LPT_CABLE_GEOMETRY": "NOT_EVALUABLE"}),
        Case("cmj_force_clock_skew", "mutation", C.clock_skew(cmj, "force"), {"CLOCK_INTEGRITY": "REJECTED"}),
        Case("cmj_lpt_clock_skew", "mutation", C.clock_skew(cmj, "lpt"),
             {"CLOCK_INTEGRITY": "REJECTED", "LPT_CABLE_GEOMETRY": "REJECTED"}),
        Case("cmj_force_in_flight", "mutation", C.force_in_flight(cmj), {"FORCE_FLIGHT_ZERO": "NOT_EVALUABLE"}),
        Case("cmj_force_in_kN", "mutation", C.force_in_kilonewtons(cmj), {"PLATE_WEIGHT_CONSISTENCY": "NOT_EVALUABLE"}),
        Case("cmj_fake_bilateral", "mutation", C.fake_bilateral_split(cmj), {"BILATERAL_FORCE": "REJECTED"}),
    ]
    return cases, {"cmj": cmj}


def _drop_bar(sub: dict) -> dict:
    import copy

    out = copy.deepcopy(sub)
    kin = out["kinematics"]
    keep = [i for i, b in enumerate(kin["bodies"]) if b["name"] != "bar"]
    kin["bodies"] = [kin["bodies"][i] for i in keep]
    for f in kin["frames"]:
        f["body_states"] = [f["body_states"][i] for i in keep]
    return out


def canary_invariance(cmj: dict) -> dict:
    clean = evaluate_i1(cmj)
    poisoned = evaluate_i1(C.hidden_truth_canary(cmj))
    same = [(a.claim, a.outcome, a.code) for a in clean.findings] == [(b.claim, b.outcome, b.code) for b in poisoned.findings]
    return {"outcomes_identical": same, "refused_fields": list(poisoned.refused_fields)}


def main() -> None:
    cases, base = build_cases()
    result = run_suite(cases)
    refuse = run_suite(cases, refuse_everything)
    accept = run_suite(cases, accept_everything)
    canary = canary_invariance(base["cmj"])
    summary = {
        "claim_ceiling": ["NUMERICAL_VERIFICATION_ONLY", "SIMULATED_MEASUREMENT_ONLY",
                          "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED", "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED"],
        "evaluator": {"gate": result["gate"], "gate_failures": result["gate_failures"], "totals": result["totals"],
                      "per_claim": result["per_claim"]},
        "baseline_refuse_everything": {"gate": refuse["gate"], "gate_failures": refuse["gate_failures"]},
        "baseline_accept_everything": {"gate": accept["gate"], "gate_failures": accept["gate_failures"]},
        "hidden_truth_canary": canary,
        "case_families": sorted({c.family for c in cases}),
        "rows": result["rows"],
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_bytes((json.dumps(summary, indent=1, sort_keys=True, default=str) + "\n").encode("utf-8"))
    t = result["totals"]
    print(f"evaluator gate {result['gate']}: {t['agreements']}/{t['adjudicated_claim_instances']} adjudicated instances agree")
    for d in t["disagreements"]:
        print("  DISAGREE", d)
    print(f"refuse-everything gate {refuse['gate']} ({len(refuse['gate_failures'])} failures)")
    print(f"accept-everything gate {accept['gate']} ({len(accept['gate_failures'])} failures)")
    print(f"canary: outcomes identical={canary['outcomes_identical']}, refused={canary['refused_fields']}")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
