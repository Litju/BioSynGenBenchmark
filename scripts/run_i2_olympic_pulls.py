"""Reproduce the RES-496 Olympic-pull evidence (summary JSON and SVG figure).

    python -m scripts.run_i2_olympic_pulls --out runs/i2/res496

Synthetic only: NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path

from biosyngen.i2 import CLAIM_CEILINGS
from biosyngen.i2.instruments import observe_dual_plates, observe_lpt
from biosyngen.i2.olympic import PULL_VARIANTS, PullProtocol, analyze_pull, build_pull, pull_lpt
from biosyngen.i2.phases import foot_force_z
from scripts.svgplot import PALETTE, Figure


def _r(v, d=6):
    return None if v is None else round(v, d)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="runs/i2/res496")
    out = Path(parser.parse_args(argv).out)
    out.mkdir(parents=True, exist_ok=True)
    summary = {"issue": "RES-496", "claim_ceilings": list(CLAIM_CEILINGS), "cases": {}}
    figure = Figure(980, 560, "RES-496 Olympic pulls: bar velocity, bar height and foot force",
                    "Bar vertical velocity (blue), bar height above the floor position (green) and summed foot force "
                    "over system weight (orange), with latent lift-off and second-pull-end markers.",
                    "Synthetic only. Joint phases (double knee bend, triple extension) are not identifiable from FORCE/LPT.")
    for index, variant in enumerate(PULL_VARIANTS):
        scenario = build_pull(replace(PullProtocol(), variant=variant))
        trajectory = scenario.run()
        analysis = analyze_pull(trajectory)
        plates = observe_dual_plates(trajectory)
        lpt = observe_lpt(trajectory, pull_lpt(trajectory))
        summary["cases"][variant] = {
            "protocol": asdict(scenario.protocol),
            "variant": {k: (_r(v) if isinstance(v, float) else v) for k, v in PULL_VARIANTS[variant].items()},
            "setup_posture": {k: _r(v) for k, v in asdict(scenario.setup_posture).items()},
            "termination": trajectory.termination,
            "analysis": {k: (_r(v) if isinstance(v, float) else v) for k, v in asdict(analysis).items()},
            "plate_samples": len(plates.samples),
            "lpt_samples": len(lpt.samples),
        }
        weight = scenario.model.total_mass_kg() * scenario.model.gravity_m_s2
        radius = scenario.model.load.plate_radius_m
        times = [f.t_end_s for f in trajectory.frames]
        velocities = [f.state.bodies["load"].velocity_m_s[1] for f in trajectory.frames]
        heights = [f.state.bodies["load"].position_m[1] - radius for f in trajectory.frames]
        forces = [foot_force_z(trajectory, f) / weight for f in trajectory.frames]
        limits = (min(-1.2, min(velocities + heights + forces) * 1.05), max(2.2, max(velocities + heights + forces) * 1.05))
        panel = figure.panel(70, 70 + index * 240, 880, 180, (0.0, times[-1]), limits,
                             title=variant, xlabel="model time (s)", ylabel="m/s ; m ; F/W")
        panel.line(times, velocities, PALETTE["blue"], label="bar vz")
        panel.line(times, heights, PALETTE["green"], label="bar height")
        panel.line(times, forces, PALETTE["orange"], label="feet Fz / W")
        if analysis.lift_off_s is not None:
            panel.vline(analysis.lift_off_s, PALETTE["black"], label="lift-off")
        if analysis.second_pull_end_s is not None:
            panel.vline(analysis.second_pull_end_s, PALETTE["grey"], label="peak bar velocity", position=1)
    (out / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8", newline="\n")
    (out / "olympic_pulls.svg").write_text(figure.render(), encoding="utf-8", newline="\n")
    print(json.dumps({k: v["analysis"]["refusals"] for k, v in summary["cases"].items()}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
