"""Reproduce the RES-492 jump-family evidence (summary JSON and SVG figure).

    python -m scripts.run_i2_jumps --out runs/i2/res492

Outputs are deterministic for a fixed source tree. All quantities are
synthetic: NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from biosyngen.i2 import CLAIM_CEILINGS
from biosyngen.i2.human import StrengthHypothesis
from biosyngen.i2.instruments import default_lpt, observe_dual_plates, observe_lpt
from biosyngen.i2.jumps import REFUSED_SUBTYPES, build_jump
from biosyngen.i2.phases import analyze_jump, foot_force_z
from biosyngen.i2.qualification import qualify_context
from scripts.svgplot import PALETTE, Figure

PHASE_COLOR = {
    "QUIET_STANCE": "#F2F2F2", "STATIC_HOLD": "#F2F2F2", "UNWEIGHTING": "#DCEBF7", "BRAKING": "#FBE3CF",
    "PROPULSION": "#DDF1E4", "FLIGHT": "#FFF6CC", "LANDING": "#F6DDE9", "POST_LANDING": "#EEEEEE",
}
CASES = (
    ("CMJ", {}),
    ("SJ", {}),
    ("LOADED_CMJ", {}),
    ("LOADED_JUMP_SQUAT", {}),
    ("SJ_RIGHT_STRENGTH_0.80", {"strength": StrengthHypothesis(right_scale=0.80)}),
)


def _r(value, digits=6):
    return None if value is None else round(value, digits)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="runs/i2/res492")
    args = parser.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    summary = {"issue": "RES-492", "claim_ceilings": list(CLAIM_CEILINGS), "refused_subtypes": REFUSED_SUBTYPES, "cases": {}}
    figure = Figure(980, 1180, "RES-492 jump family: latent foot force and simulated plates",
                    "Latent summed foot-to-plate vertical force (line) and simulated FZ_TOTAL samples (dots), "
                    "normalised by system weight, with latent force/state phases as background bands.",
                    "Synthetic only. Landing peaks are rigid-impact impulses through an assumed filter: NOT_EVALUABLE.")
    for index, (label, kwargs) in enumerate(CASES):
        subtype = label.split("_RIGHT")[0]
        scenario = build_jump(subtype, **kwargs)
        trajectory = scenario.run()
        analysis = analyze_jump(trajectory)
        plates = observe_dual_plates(trajectory)
        lpt = observe_lpt(trajectory, default_lpt(trajectory))
        weight = analysis.weight_n
        mass = scenario.model.total_mass_kg()
        summary["cases"][label] = {
            "termination": trajectory.termination,
            "context_qualification": qualify_context(trajectory),
            "refusals": list(analysis.refusals),
            "system_mass_kg": _r(mass),
            "load": scenario.metadata["load_kind"],
            "load_mass_kg": scenario.metadata["load_mass_kg"],
            "events_s": {k: _r(e.time_s) for k, e in analysis.events.items()},
            "phases_s": [[p.name, _r(p.start_s), _r(p.end_s)] for p in analysis.phases],
            "control_commands": [[_r(t), stage, reason] for t, stage, reason in analysis.control_commands],
            "takeoff_velocity_m_s": _r(analysis.takeoff_velocity_m_s),
            "takeoff_impulse_over_mass_m_s": _r(analysis.takeoff_impulse_n_s / mass if analysis.takeoff_impulse_n_s else None),
            "flight_time_s": _r(analysis.flight_time_s),
            "com_rise_after_takeoff_m": _r(analysis.com_rise_after_takeoff_m),
            "countermovement_depth_m": _r(analysis.countermovement_depth_m),
            "peak_downward_velocity_m_s": _r(analysis.peak_downward_velocity_m_s),
            "foot_impacts": len([i for i in trajectory.impacts if i.contact_id in {c for v in scenario.model.contact_ids.values() for c in v}]),
            "plate_samples": len(plates.samples),
            "plate_saturated_samples": sum(s.saturated for s in plates.samples),
            "lpt_samples": len(lpt.samples),
            "lpt_attachment": lpt.metadata["attachment"],
            "lpt_peak_displacement_m": _r(max(s.displacement_m for s in lpt.samples)),
        }
        times = [f.t_end_s for f in trajectory.frames]
        latent = [foot_force_z(trajectory, f) / weight for f in trajectory.frames]
        virtual_force = [s.fz_total_n / weight for s in plates.samples]
        offset = plates.metadata["native_time_offset_s"]
        limits = (min(-0.2, min(latent + virtual_force) * 1.05), max(4.5, max(latent + virtual_force) * 1.05))
        panel = figure.panel(70, 70 + index * 215, 880, 170, (0.0, times[-1]), limits,
                             title=label, xlabel="model time (s)", ylabel="Fz / W")
        for phase in analysis.phases:
            if phase.start_s is not None and phase.end_s is not None and phase.end_s > phase.start_s:
                panel.band(phase.start_s, phase.end_s, PHASE_COLOR.get(phase.name, "#FFFFFF"))
        panel.line(times, latent, PALETTE["blue"], label="latent foot force")
        sampled = plates.samples[::5]
        panel.markers([s.native_time_s - offset for s in sampled], [s.fz_total_n / weight for s in sampled],
                      PALETTE["vermillion"], r=1.0, label="simulated FZ_TOTAL (every 5th)")
        if analysis.time("takeoff") is not None:
            panel.vline(analysis.time("takeoff"), PALETTE["black"], label="latent takeoff")
    (out / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8", newline="\n")
    (out / "jump_family.svg").write_text(figure.render(), encoding="utf-8", newline="\n")
    print(json.dumps({k: {"takeoff_v": v["takeoff_velocity_m_s"], "termination": v["termination"]} for k, v in summary["cases"].items()}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
