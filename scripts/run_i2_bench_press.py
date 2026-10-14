"""Reproduce the RES-495 bench-press evidence (summary JSON and SVG figure).

    python -m scripts.run_i2_bench_press --out runs/i2/res495

Synthetic only: NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path

from biosyngen.i2 import CLAIM_CEILINGS
from biosyngen.i2.bench import BenchProtocol, analyze_bench, bench_lpt, build_bench_press, support_reaction
from biosyngen.i2.instruments import observe_dual_plates, observe_lpt
from biosyngen.i2.qualification import qualify_context
from scripts.svgplot import PALETTE, Figure

CASES = (("BENCH_60KG_2REPS", {}), ("BENCH_60KG_PAUSED_0.8S", {"repetitions": 1, "pause_s": 0.8}))


def _r(v, d=6):
    return None if v is None else round(v, d)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="runs/i2/res495")
    out = Path(parser.parse_args(argv).out)
    out.mkdir(parents=True, exist_ok=True)
    summary = {"issue": "RES-495", "claim_ceilings": list(CLAIM_CEILINGS), "cases": {}}
    figure = Figure(980, 560, "RES-495 bench press: support reactions and bar height",
                    "Bench reaction (purple), summed foot force (green) and rack-hook reaction (orange) over system "
                    "weight, and bar height change from setup (blue, metres), with chest-touch markers.",
                    "Synthetic only. Feet-only plates are incomplete: most of the supine lifter's weight is on the bench.")
    for index, (label, kwargs) in enumerate(CASES):
        scenario = build_bench_press(replace(BenchProtocol(), **kwargs))
        trajectory = scenario.run()
        analysis = analyze_bench(trajectory)
        plates = observe_dual_plates(trajectory)
        lpt = observe_lpt(trajectory, bench_lpt(trajectory))
        summary["cases"][label] = {
            "protocol": asdict(scenario.protocol),
            "termination": trajectory.termination,
            "context_qualification": qualify_context(trajectory),
            "unrack_s": _r(analysis.unrack_s), "rerack_s": _r(analysis.rerack_s),
            "refusals": list(analysis.refusals),
            "support_events": [[_r(t), e, list(c)] for t, e, c in analysis.support_events],
            "reps": [{k: (_r(v) if isinstance(v, float) else v) for k, v in asdict(r).items()} for r in analysis.reps],
            "plate_samples": len(plates.samples),
            "lpt_samples": len(lpt.samples),
        }
        weight = analysis.system_weight_n
        times = [f.t_end_s for f in trajectory.frames]
        reactions = [support_reaction(trajectory, f) for f in trajectory.frames]
        z0 = trajectory.initial_state.bodies["load"].position_m[1]
        height = [f.state.bodies["load"].position_m[1] - z0 for f in trajectory.frames]
        forces = [r[key] / weight for r in reactions for key in ("bench", "feet", "rack")]
        limits = (min(-0.6, min(forces + height) * 1.05), max(1.2, max(forces + height) * 1.05))
        panel = figure.panel(70, 70 + index * 240, 880, 180, (0.0, times[-1]), limits,
                             title=label, xlabel="model time (s)", ylabel="F/W ; bar dz (m)")
        panel.line(times, [r["bench"] / weight for r in reactions], PALETTE["purple"], label="bench / W")
        panel.line(times, [r["feet"] / weight for r in reactions], PALETTE["green"], label="feet / W")
        panel.line(times, [r["rack"] / weight for r in reactions], PALETTE["orange"], label="rack / W")
        panel.line(times, height, PALETTE["blue"], label="bar height change (m)")
        for rep in analysis.reps:
            if rep.chest_touch_s is not None:
                panel.vline(rep.chest_touch_s, PALETTE["black"], label="chest touch" if rep.index == 1 else None)
    (out / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8", newline="\n")
    (out / "bench_press.svg").write_text(figure.render(), encoding="utf-8", newline="\n")
    print(json.dumps({k: [r["refusals"] for r in v["reps"]] for k, v in summary["cases"].items()}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
