"""Reproduce the RES-494 deadlift evidence (summary JSON and SVG figure).

    python -m scripts.run_i2_deadlift --out runs/i2/res494

Synthetic only: NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path

from biosyngen.i2 import CLAIM_CEILINGS
from biosyngen.i2.deadlift import DeadliftProtocol, analyze_deadlift, build_deadlift, deadlift_lpt
from biosyngen.i2.instruments import observe_dual_plates, observe_lpt
from biosyngen.i2.phases import foot_force_z
from scripts.svgplot import PALETTE, Figure

CASES = (("DEADLIFT_100KG", {}), ("DEADLIFT_320KG_NOT_LIFTED", {"load_mass_kg": 320.0}))


def _r(v, d=6):
    return None if v is None else round(v, d)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="runs/i2/res494")
    out = Path(parser.parse_args(argv).out)
    out.mkdir(parents=True, exist_ok=True)
    summary = {"issue": "RES-494", "claim_ceilings": list(CLAIM_CEILINGS), "cases": {}}
    figure = Figure(980, 560, "RES-494 deadlift: plate force, bar-floor reaction and bar height",
                    "Summed foot force (green) and bar-floor reaction (orange) over system weight, and bar height "
                    "above its floor position (blue, metres), with latent pre-tension onset and lift-off markers.",
                    "Synthetic only. Plate force rises while the bar is still on the floor; lift-off is the bar-floor contact opening.")
    for index, (label, kwargs) in enumerate(CASES):
        scenario = build_deadlift(replace(DeadliftProtocol(), **kwargs))
        trajectory = scenario.run()
        analysis = analyze_deadlift(trajectory)
        plates = observe_dual_plates(trajectory)
        lpt = observe_lpt(trajectory, deadlift_lpt(trajectory))
        summary["cases"][label] = {
            "protocol": asdict(scenario.protocol),
            "setup_posture": asdict(scenario.setup_posture),
            "termination": trajectory.termination,
            "analysis": {k: _r(v) if isinstance(v, float) else v for k, v in asdict(analysis).items()},
            "plate_samples": len(plates.samples),
            "lpt_samples": len(lpt.samples),
            "lpt_slack_samples": sum(s.slack for s in lpt.samples),
        }
        times = [f.t_end_s for f in trajectory.frames]
        weight = analysis.system_weight_n
        feet = [foot_force_z(trajectory, f) / weight for f in trajectory.frames]
        floor = [(f.contact_forces["bar_floor"].force_world_n[1] if "bar_floor" in f.contact_forces else 0.0) / weight
                 for f in trajectory.frames]
        radius = scenario.model.load.plate_radius_m
        height = [f.state.bodies["load"].position_m[1] - radius for f in trajectory.frames]
        panel = figure.panel(70, 70 + index * 240, 880, 180, (0.0, times[-1]), (-0.05, 1.3),
                             title=label, xlabel="model time (s)", ylabel="F/W ; bar height (m)")
        panel.line(times, [min(v, 1.28) for v in feet], PALETTE["green"], label="feet Fz / W")
        panel.line(times, [min(v, 1.28) for v in floor], PALETTE["orange"], label="bar-floor reaction / W")
        panel.line(times, height, PALETTE["blue"], label="bar height (m)")
        if analysis.pretension_onset_s is not None:
            panel.vline(analysis.pretension_onset_s, PALETTE["grey"], label="pre-tension onset")
        if analysis.lift_off_s is not None:
            panel.vline(analysis.lift_off_s, PALETTE["black"], label="lift-off", position=1)
    (out / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8", newline="\n")
    (out / "deadlift.svg").write_text(figure.render(), encoding="utf-8", newline="\n")
    print(json.dumps({k: v["analysis"]["refusals"] for k, v in summary["cases"].items()}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
