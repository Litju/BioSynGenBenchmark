"""Reproduce the RES-493 back-squat evidence (summary JSON and SVG figure).

    python -m scripts.run_i2_back_squat --out runs/i2/res493

Synthetic only: NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path

from biosyngen.i2 import CLAIM_CEILINGS
from biosyngen.i2.instruments import observe_dual_plates, observe_lpt
from biosyngen.i2.phases import foot_force_z
from biosyngen.i2.squat import SquatProtocol, analyze_squat, bar_state, build_back_squat, squat_lpt
from scripts.svgplot import PALETTE, Figure

CASES = (
    ("SET_60KG_3REPS", {"repetitions": 3}),
    ("SINGLE_160KG", {"repetitions": 1, "load_mass_kg": 160.0}),
    ("SINGLE_60KG_PAUSE_0.6S", {"repetitions": 1, "bottom_pause_s": 0.6}),
    ("SINGLE_60KG_PARTIAL_DEPTH", {"repetitions": 1, "bottom_ankle_knee": (0.35, -1.0)}),
)


def _r(v, d=6):
    return None if v is None else round(v, d)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="runs/i2/res493")
    out = Path(parser.parse_args(argv).out)
    out.mkdir(parents=True, exist_ok=True)
    summary = {"issue": "RES-493", "claim_ceilings": list(CLAIM_CEILINGS), "cases": {}}
    figure = Figure(980, 1000, "RES-493 back squat: bar velocity and plate force",
                    "Latent bar vertical velocity (blue) and summed foot force over system weight (green, right scale "
                    "shifted by -1), with latent repetition descent and ascent bands.",
                    "Synthetic only. Repetition events come from bar/COM mechanics, not from the controller.")
    for index, (label, kwargs) in enumerate(CASES):
        scenario = build_back_squat(replace(SquatProtocol(), **kwargs))
        trajectory = scenario.run()
        analysis = analyze_squat(trajectory)
        plates = observe_dual_plates(trajectory)
        lpt = observe_lpt(trajectory, squat_lpt(trajectory))
        summary["cases"][label] = {
            "protocol": {k: v for k, v in asdict(scenario.protocol).items()},
            "termination": trajectory.termination,
            "refusals": list(analysis.refusals),
            "system_weight_n": _r(analysis.system_weight_n),
            "reps": [
                {
                    "index": r.index,
                    "descent_onset_s": _r(r.descent_onset_s), "bottom_reversal_s": _r(r.bottom_reversal_s),
                    "ascent_onset_s": _r(r.ascent_onset_s), "rep_end_s": _r(r.rep_end_s),
                    "sticking_region_s": None if r.sticking_region_s is None else [_r(x) for x in r.sticking_region_s],
                    "bar_displacement_m": _r(r.bar_displacement_m),
                    "peak_descent_velocity_m_s": _r(r.peak_descent_velocity_m_s),
                    "peak_ascent_velocity_m_s": _r(r.peak_ascent_velocity_m_s),
                    "mean_ascent_velocity_m_s": _r(r.mean_ascent_velocity_m_s),
                    "bar_slip_m": _r(r.bar_slip_m, 9), "refusals": list(r.refusals),
                }
                for r in analysis.reps
            ],
            "joint_work_j": _r(analysis.joint_work_j, 3),
            "bar_work_on_bar_j": _r(analysis.bar_work_on_bar_j, 3),
            "system_com_work_j": _r(analysis.system_com_work_j, 3),
            "plate_samples": len(plates.samples),
            "lpt_samples": len(lpt.samples),
            "lpt_span_m": _r(max(s.displacement_m for s in lpt.samples) - min(s.displacement_m for s in lpt.samples)),
        }
        times = [f.t_end_s for f in trajectory.frames]
        bar_v = [bar_state(trajectory, f.state)[1] for f in trajectory.frames]
        force = [foot_force_z(trajectory, f) / analysis.system_weight_n - 1.0 for f in trajectory.frames]
        panel = figure.panel(70, 70 + index * 225, 880, 175, (0.0, times[-1]), (-1.2, 1.2),
                             title=label, xlabel="model time (s)", ylabel="m/s ; Fz/W - 1")
        for rep in analysis.reps:
            if rep.bottom_reversal_s is not None:
                panel.band(rep.descent_onset_s, rep.bottom_reversal_s, "#DCEBF7")
            if rep.ascent_onset_s is not None and rep.rep_end_s is not None:
                panel.band(rep.ascent_onset_s, rep.rep_end_s, "#DDF1E4")
            if rep.sticking_region_s is not None:
                panel.band(*rep.sticking_region_s, "#FBE3CF")
        panel.line(times, bar_v, PALETTE["blue"], label="bar vertical velocity")
        panel.line(times, [max(-1.15, min(1.15, v)) for v in force], PALETTE["green"], label="Fz/W - 1")
    (out / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8", newline="\n")
    (out / "back_squat.svg").write_text(figure.render(), encoding="utf-8", newline="\n")
    print(json.dumps({k: [r["bar_displacement_m"] for r in v["reps"]] for k, v in summary["cases"].items()}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
