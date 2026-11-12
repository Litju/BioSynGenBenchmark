"""RES-624 Gate B: compiled collision-eligibility audit of Human V2 and V1.

    python -m scripts.res624_contact_audit [--out runs/i3/res624/gateB]

For each declared pair the audit records (a) the compiled-mask rule
evaluation and (b) whether MuJoCo *actually* generates a contact when the
two geoms are made to overlap in the compiled model. The two must agree, and
each must match the declared mechanical policy. A drop test shows the
unplated V2 bar now lands on the floor, while the untouched V1 bar (same
inherited masks, kept for RES-610 reproducibility) still passes through.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from biosyngen.spatial.grasp_diagnostics import pair_eligibility
from biosyngen.spatial.model import AthleteBarbellSpec, build_mjcf
from biosyngen.spatial.plant_v2 import HumanV2Plant

# (label, geom A, geom B, intended eligible, role)
POLICY = [
    ("barbell-floor", "bar_shaft", "floor", True, "load-bearing when dropped/missed"),
    ("barbell_sleeve-floor", "bar_sleeve_l", "floor", True, "load-bearing when dropped/missed"),
    ("barbell-palm", "bar_shaft", "l_palm_collision", True, "load-bearing grasp"),
    ("barbell-proximal_fingers", "bar_shaft", "l_finger_flex_pad", True, "load-bearing grasp"),
    ("barbell-distal_fingers", "bar_shaft", "r_finger_distal_pad", True, "load-bearing grasp"),
    ("barbell-thumb", "bar_shaft", "l_thumb_opposition_pad", True, "load-bearing grasp"),
    ("barbell-forearm", "bar_shaft", "geom:l_forearm", True, "realistic collision"),
    ("barbell-head", "bar_shaft", "head_collision", True, "realistic collision"),
    ("barbell-neck", "bar_shaft", "neck_collision", True, "realistic collision"),
    ("barbell-thorax", "bar_shaft", "thorax_collision", True, "realistic collision"),
    ("barbell-thigh", "bar_shaft", "geom:l_thigh", True, "realistic collision"),
    ("barbell-shin", "bar_shaft", "geom:r_shank", True, "realistic collision"),
    ("barbell-foot", "bar_shaft", "l_forefoot_sole", True, "realistic collision"),
    ("foot-ground", "l_hindfoot_sole", "floor", True, "load-bearing support"),
    ("forefoot-ground", "r_mtp_sole", "floor", True, "load-bearing support"),
    ("shin-ground", "geom:l_shank", "floor", True, "realistic collision (kneeling/fall)"),
    ("hand-ground", "r_palm_collision", "floor", True, "realistic collision (fall)"),
    ("palm-own_fingers", "l_palm_collision", "l_finger_flex_pad", False, "parent/child joint neighbours"),
    ("thigh-shin_self", "geom:l_thigh", "geom:l_shank", False, "human self-collision excluded (inherited policy)"),
    ("hand-head_self", "l_palm_collision", "head_collision", False, "human self-collision excluded (inherited policy)"),
    ("forearm-thorax_self", "geom:l_forearm", "thorax_collision", False, "human self-collision excluded (inherited policy)"),
]


def _geom(m, name):
    if name.startswith("geom:"):
        body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name[5:])
        return next(g for g in range(m.ngeom) if m.geom_bodyid[g] == body)
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)


def actual_detection(plant, a, b):
    """Force geom overlap in the compiled model and report whether a contact is generated.

    The bar is moved onto the other geom when the pair involves the bar; the
    other pairs use a temporarily large detection margin on both geoms and
    the engine's own broad/narrow phase, so the compiled masks and filters
    alone decide.
    """
    m, d = plant.model, plant.data
    ga, gb = _geom(m, a), _geom(m, b)
    saved = d.qpos.copy()
    margin = m.geom_margin.copy()
    try:
        mujoco.mj_forward(m, d)
        bar = plant.body_id("barbell")
        if m.geom_bodyid[ga] == bar and m.geom_bodyid[gb] != 0:
            q = plant.joint_qpos("bar_free")
            local = m.geom_pos[ga]
            # Partial, non-concentric overlap: concentric deep penetration is a
            # degenerate convex-collision query (reported distance 0), not a mask test.
            target = d.geom_xpos[gb] + .9*m.geom_size[gb, 0]*d.geom_xmat[gb].reshape(3, 3)[:, 0]
            d.qpos[q:q+3] = target - local
            d.qpos[q+3:q+7] = [1, 0, 0, 0]
        elif m.geom_bodyid[ga] == bar:
            q = plant.joint_qpos("bar_free")
            d.qpos[q:q+3] = [1.5, 0, m.geom_size[ga, 0]-.002]
            d.qpos[q+3:q+7] = [1, 0, 0, 0]
        m.geom_margin[[ga, gb]] = 1.0
        mujoco.mj_forward(m, d)
        hit = any({int(c.geom1), int(c.geom2)} == {ga, gb} for c in d.contact[:d.ncon])
        return bool(hit)
    finally:
        m.geom_margin[:] = margin
        d.qpos[:] = saved
        mujoco.mj_forward(m, d)


def eligibility_matrix(plant):
    rows = []
    for label, a, b, intended, role in POLICY:
        m = plant.model
        rule = pair_eligibility(m, _geom(m, a), _geom(m, b))
        actual = actual_detection(plant, a, b)
        rows.append(dict(pair=label, geom_a=rule["geom1"], geom_b=rule["geom2"], role=role, intended=intended,
                         compiled_rule=rule["eligible"], engine_detected=actual, bitmask=rule["bitmask"],
                         parent_filtered=rule["parent_filtered"], contype=rule["contype"], conaffinity=rule["conaffinity"],
                         consistent=rule["eligible"] == actual == intended))
    return rows


def drop_test(model, duration_s=1.0, start_z=.3):
    d = mujoco.MjData(model)
    bar = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "barbell")
    j = model.body_jntadr[bar]
    q = model.jnt_qposadr[j]
    mujoco.mj_forward(model, d)
    d.qpos[q:q+3] = [1.5, 0, start_z]
    d.qpos[q+3:q+7] = [1, 0, 0, 0]
    floor_contacts = 0
    for _ in range(round(duration_s/model.opt.timestep)):
        mujoco.mj_step(model, d)
        floor_contacts = max(floor_contacts, sum(1 for c in d.contact[:d.ncon]
                                                 if 0 in (model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2])
                                                 and bar in (model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2])))
    return dict(final_bar_z_m=float(d.xipos[bar, 2]), max_bar_floor_contacts=int(floor_contacts),
                final_speed_m_s=float(np.linalg.norm(d.qvel[model.jnt_dofadr[j]:model.jnt_dofadr[j]+3])))


def solver_audit(m):
    return dict(timestep_s=float(m.opt.timestep), integrator=int(m.opt.integrator), cone="elliptic" if m.opt.cone == 1 else "pyramidal",
                impratio=float(m.opt.impratio), solver=int(m.opt.solver), iterations=int(m.opt.iterations),
                tolerance=float(m.opt.tolerance), noslip_iterations=int(m.opt.noslip_iterations),
                filterparent_enabled=not bool(m.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_FILTERPARENT),
                explicit_excludes=int(m.nexclude), explicit_pairs=int(m.npair),
                geom_solref_unique=sorted({tuple(map(float, r)) for r in m.geom_solref}),
                geom_solimp_unique=sorted({tuple(map(float, r[:3])) for r in m.geom_solimp}),
                contact_friction_rule="max of the two geoms' sliding friction (MuJoCo solmix default)",
                hand_shaft_mu=float(max(m.geom_friction[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "bar_shaft"), 0],
                                        m.geom_friction[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "l_palm_collision"), 0])),
                foot_floor_mu=float(m.geom_friction[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor"), 0]))


def run(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    plant = HumanV2Plant.build()
    matrix = eligibility_matrix(plant)
    v2_drop = drop_test(plant.model)
    v1 = mujoco.MjModel.from_xml_string(build_mjcf(AthleteBarbellSpec(plate_mass_kg=0)))
    v1_rule = pair_eligibility(v1, "bar_shaft", "floor")
    v1_plant = type("V1", (), {})()
    v1_plant.model, v1_plant.data = v1, mujoco.MjData(v1)
    v1_plant.body_id = lambda n: mujoco.mj_name2id(v1, mujoco.mjtObj.mjOBJ_BODY, n)
    v1_plant.joint_qpos = lambda n: int(v1.jnt_qposadr[mujoco.mj_name2id(v1, mujoco.mjtObj.mjOBJ_JOINT, n)])
    v1_detect = actual_detection(v1_plant, "bar_shaft", "floor")
    sleeve_radius = float(plant.model.geom_size[_geom(plant.model, "bar_sleeve_l"), 0])
    report = dict(issue="RES-624", gate="B", matrix=matrix, all_consistent=all(r["consistent"] for r in matrix),
                  v2_unplated_drop=dict(v2_drop, sleeve_radius_m=sleeve_radius,
                                        lands=v2_drop["max_bar_floor_contacts"] > 0 and v2_drop["final_bar_z_m"] > .02),
                  v1_unchanged=dict(shaft_floor_rule_eligible=v1_rule["eligible"], shaft_floor_engine_detected=v1_detect,
                                    note="RES-610 V1 masks intentionally untouched (its grip welds hold the bar); "
                                         "the inherited shaft/floor pair remains ineligible there"),
                  solver=solver_audit(plant.model),
                  claims=["NUMERICAL_VERIFICATION_ONLY", "SIMULATED_MEASUREMENT_ONLY",
                          "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED", "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED"])
    report["outcome"] = "PASS" if report["all_consistent"] and report["v2_unplated_drop"]["lands"] else "FAIL"
    (out/"contact_eligibility.json").write_text(json.dumps(report, indent=2, sort_keys=True)+"\n", encoding="utf-8")
    return report


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="runs/i3/res624/gateB")
    report = run(ap.parse_args(argv).out)
    for r in report["matrix"]:
        print(f'{r["pair"]:28s} intended={r["intended"]!s:5} rule={r["compiled_rule"]!s:5} engine={r["engine_detected"]!s:5} ok={r["consistent"]}')
    print("V2 drop", report["v2_unplated_drop"], "\nV1", report["v1_unchanged"])
    return 0 if report["outcome"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
