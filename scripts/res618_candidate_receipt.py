"""Reproduce the bounded foot/inertia checkpoint; never classifies a lift."""
import json
from pathlib import Path
import subprocess

import mujoco
import numpy as np

from biosyngen.spatial.feet_v2 import build_foot_candidate


def main():
    output = Path("runs/i3/res618")
    output.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_string(build_foot_candidate())
    data = mujoco.MjData(model)
    data.eq_active[:] = False
    j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "bar_free")
    q = model.jnt_qposadr[j]
    data.qpos[q:q+3] = [1.5, 0, .3]
    mujoco.mj_forward(model, data)
    initial = min((float(c.dist) for c in data.contact), default=0)
    for _ in range(1000):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    result = dict(
        source_sha=subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        source_dirty=bool(subprocess.check_output(["git", "status", "--porcelain"], text=True).strip()),
        model="human-v2.0-b_FOOT_CANDIDATE_UNQUALIFIED",
        body_count_excluding_world=model.nbody-1, generalized_dof=model.nv,
        actuator_count=model.nu, total_system_mass_kg=float(sum(model.body_mass)),
        initial_min_contact_distance_m=initial, standing_final_pelvis_height_m=float(data.qpos[2]),
        standing_final_max_generalized_speed=float(np.max(np.abs(data.qvel))),
        standing_final_min_contact_distance_m=min((float(c.dist) for c in data.contact), default=0),
        receiving="NOT_EVALUABLE", anatomical_gates="INCOMPLETE",
        limitations=["legacy upper body and grip remain", "shoe equipment mass omitted",
                     "manufactured anthropometry", "no grip/rack transition", "standing probe only"],
        claims=["NUMERICAL_VERIFICATION_ONLY", "SIMULATED_MEASUREMENT_ONLY",
                "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED", "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED"])
    (output / "candidate.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    # Visualization is an inspection aid, separate from quantitative evidence.
    from PIL import Image
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, .9]
    camera.distance, camera.azimuth, camera.elevation = 3.2, 125, -15
    with mujoco.Renderer(model, height=480, width=640) as renderer:
        renderer.update_scene(data, camera=camera)
        Image.fromarray(renderer.render()).save(output / "standing_candidate.png")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
