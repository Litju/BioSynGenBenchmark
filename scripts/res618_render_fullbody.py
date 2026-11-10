"""Render actual neutral, candidate and failed receiving states for inspection."""
import json
from pathlib import Path

import mujoco
from PIL import Image

from biosyngen.spatial.plant_v2 import HumanV2Plant
from biosyngen.spatial.receiving_v2 import forward_hold


def main():
    output = Path("runs/i3/res618")
    plant = HumanV2Plant.build()
    m, d = plant.model, plant.data
    m.vis.headlight.ambient[:] = [.6, .6, .6]
    m.vis.headlight.diffuse[:] = [.7, .7, .7]
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, 1]
    camera.distance, camera.azimuth, camera.elevation = 3.1, 115, -12
    with mujoco.Renderer(m, height=480, width=640) as renderer:
        def save(name):
            mujoco.mj_forward(m, d)
            renderer.update_scene(d, camera=camera)
            Image.fromarray(renderer.render()).save(output/(name+".png"))
        save("fullbody_neutral")
        candidate = json.loads((output/"fullbody_receiving_failure.json").read_text(encoding="utf-8"))["candidate"]
        d.qpos[:] = candidate["qpos"]
        d.qvel[:] = 0
        save("overhead_candidate_NOT_QUALIFIED")
        forward_hold(plant, .3)
        save("overhead_after_300ms_FAIL")


if __name__ == "__main__":
    main()
