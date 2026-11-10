"""RES-624 Gate G: decompose the static-certificate refusals of the anthropometric variants.

    python -m scripts.res624_anthropometric_diagnosis [--out runs/i3/res624/gateG]

For each subject (nominal, 70 kg/1.70 m, 80 kg/1.80 m) this rebuilds the certified candidate and reports
exactly which clause of ``qualify_static`` decides: generalized-equilibrium feasibility, hand contacts, COP margin,
self-intersections, undeclared penetrations, joints at limit, contact penetration. It also verifies that the
compiled model mass, subtree mass and exported parameter identity agree (guards the cached-dynamics regression).
Nothing here changes human limits, strengths or anthropometry. NUMERICAL_VERIFICATION_ONLY.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from biosyngen.spatial.anthropometry import HumanSubjectProfile
from biosyngen.spatial.model_v2 import HumanV2Spec
from biosyngen.spatial.receiving_grasp_v2 import MAX_PENETRATION_M, build_certified_candidate
from scripts.res624_grasp_diagnosis import CLAIMS, source_receipt
from scripts.res624_receiving_campaign import _set_bar_mass, clone, build_nominal

SUBJECTS = (("nominal", None), ("70kg_1p70m", HumanV2Spec(profile=HumanSubjectProfile(70, 1.70))),
            ("80kg_1p80m", HumanV2Spec(profile=HumanSubjectProfile(80, 1.80))))


def clause_report(plant, static):
    c = static["certificate"]
    si = static["self_intersection"]
    return dict(verdict=static["verdict"], generalized_code=c["code"], feasible=c["feasible"],
                hand_contacts=c["hand_contacts"], floor_contacts=c["floor_contacts"],
                cop_hull_margin_m=c.get("cop_hull_margin_m"), max_utilization=c.get("max_utilization"),
                max_generalized_residual=c.get("max_generalized_residual"), residual_tolerance=c.get("residual_tolerance"),
                self_intersections=si["penetrating"], min_geom_distance_m=si["min_distance_m"],
                undeclared_penetrations=c["undeclared_penetrations"], joints_at_limit=c.get("joints_at_limit"),
                min_joint_limit_margin_rad=c.get("min_joint_limit_margin_rad"),
                max_contact_penetration_m=c["max_penetration_m"], penetration_bound_m=MAX_PENETRATION_M,
                most_loaded_actuators=c.get("most_loaded_actuators"),
                floor_vertical_n=c.get("floor_vertical_n"), total_weight_n=c.get("total_weight_n"))


def mass_identity(plant, expected_bar_kg=20.0):
    m, d = plant.model, plant.data
    bar = plant.body_id("barbell")
    human = float(m.body_mass[1:].sum()-m.body_mass[bar])
    return dict(bar_body_mass=float(m.body_mass[bar]), human_mass_kg=human, total_mass=float(m.body_mass[1:].sum()),
                subtree_mass_root=float(m.body_subtreemass[1]), human_subtree_matches=bool(abs(m.body_subtreemass[1]-human) < 1e-9 and abs(m.body_subtreemass[bar]-m.body_mass[bar]) < 1e-9),
                bar_matches_declared=bool(abs(m.body_mass[bar]-expected_bar_kg) < 1e-9))


def run(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    rep = {}
    for name, spec in SUBJECTS:
        plant, cand, static, _ = build_certified_candidate(spec)
        rep[name] = dict(optimizer_success=cand["optimizer_success"], residual_norm=cand["residual_norm"],
                         variables=cand["variables"], clauses=clause_report(plant, static), mass=mass_identity(plant))
    # Bar-mass edits: compiled mass, subtree mass, applied gravity load and static support must agree.
    _, nominal = build_nominal()
    mass_cases = {}
    for kg in (15.0, 20.0, 25.0):
        p = clone(HumanV2Spec(), nominal["qpos"])
        _set_bar_mass(p, kg)
        qpos, qvel = p.data.qpos.copy(), p.data.qvel.copy()
        mujoco.mj_setConst(p.model, p.data)
        p.data.qpos[:], p.data.qvel[:] = qpos, qvel
        mujoco.mj_forward(p.model, p.data)
        bar = p.body_id("barbell")
        mujoco.mj_forward(p.model, p.data)
        # gravity load on the bar as the engine sees it: bias force of the free joint, vertical component
        dof = p.model.jnt_dofadr[p.model.body_jntadr[bar]]
        bias_z = float(p.data.qfrc_bias[dof+2])
        mass_cases[f"bar_{kg:g}kg"] = dict(identity=mass_identity(p, kg), engine_gravity_bias_z_n=bias_z,
                                           expected_weight_n=kg*9.81, qpos_preserved=bool(np.array_equal(p.data.qpos, qpos)),
                                           static_verdict=__import__("biosyngen.spatial.receiving_grasp_v2", fromlist=["x"]).qualify_static(p)["verdict"])
    report = dict(issue="RES-624", gate="G", **source_receipt(), subjects=rep, bar_mass_identity=mass_cases, claims=CLAIMS)
    (out/"anthropometric_diagnosis.json").write_text(json.dumps(report, indent=2, sort_keys=True, default=float)+"\n", encoding="utf-8")
    return report


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="runs/i3/res624/gateG")
    r = run(ap.parse_args(argv).out)
    for k, v in r["subjects"].items():
        c = v["clauses"]
        print(k, c["verdict"], "| code", c["generalized_code"], "| hands", c["hand_contacts"], "| cop", c["cop_hull_margin_m"],
              "| selfX", [(a, b, round(d, 5)) for a, b, d in c["self_intersections"]][:4], "| undecl", c["undeclared_penetrations"][:3],
              "| limits", c["joints_at_limit"], "| pen", c["max_contact_penetration_m"], "| util", c["max_utilization"], "| opt", v["optimizer_success"], v["residual_norm"])
    for k, v in r["bar_mass_identity"].items():
        print(k, v)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
