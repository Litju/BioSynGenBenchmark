"""Reproduce the full-body receiving falsifier; process fails on unqualified hold."""
import json
from pathlib import Path
import subprocess

from biosyngen.spatial.plant_v2 import HumanV2Plant
from biosyngen.spatial.receiving_v2 import solve_candidate, forward_hold
from biosyngen_benchmark.spatial_oracle import dynamic_balance, contact_admissibility


def run():
    plant = HumanV2Plant.build()
    candidate = solve_candidate(plant, "OVERHEAD", "POWER", 1000)
    initial_bar_z = float(plant.data.xpos[plant.body_id("barbell"), 2])
    hold = forward_hold(plant, .3)
    frames = hold.pop("frames")
    # No universal tolerance authorizes receiving. This campaign retains the
    # observed loss-of-support witness and has not passed the acceptance gates.
    report = dict(outcome="FAIL", code="RECEIVING_NOT_QUALIFIED_AFTER_TWO_REPAIRS",
                  source_sha=subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
                  source_dirty=bool(subprocess.check_output(["git", "status", "--porcelain"], text=True).strip()),
                  candidate=candidate, hold=hold, initial_bar_z_m=initial_bar_z,
                  bar_drop_m=initial_bar_z-hold["final_bar_z_m"],
                  dynamic_balance=dynamic_balance(frames, plant.model.opt.timestep),
                  contact_admissibility_final=contact_admissibility(frames[-1]),
                  final_contacts=frames[-1]["contacts"], repair_attempts=2,
                  claims=["NUMERICAL_VERIFICATION_ONLY", "SIMULATED_MEASUREMENT_ONLY",
                          "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED", "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED"])
    return report


def main():
    report = run()
    output = Path("runs/i3/res618/fullbody_receiving_failure.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k not in ("candidate", "final_contacts")}, indent=2))
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
