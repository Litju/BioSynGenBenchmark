"""Reference-only back-squat geometry preflight, before any Plate2bar export.

Run: python -m scripts.probe_acquisition_geometry --out <fresh-json-file>
Exit 2 means geometry is unqualified. This is neither a source bundle nor a
production event/conformance test. No latent data is passed to Plate2bar.
"""

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import time

from biosyngen.i2 import CLAIM_CEILINGS
from biosyngen.i2.acquisition import DUAL_1KHZ_V1
from biosyngen.i2.instruments import attachment_world, observe_dual_plates, observe_lpt
from biosyngen.i2.squat import SquatProtocol, build_back_squat, squat_lpt


def geometry_diagnostics(points, anchor):
    """Euclidean geometry independent of the sensor's cable-length function.

    The residual is cable travel minus vertical attachment displacement.
    No empirical angle/error acceptance threshold is asserted here.
    """
    lengths = [math.hypot(*(p - a for p, a in zip(point, anchor))) for point in points]
    residuals = [(length - lengths[0]) - (point[2] - points[0][2])
                 for length, point in zip(lengths, points)]
    lateral = [math.hypot(point[0] - anchor[0], point[1] - anchor[1]) for point in points]
    angles = [math.degrees(math.atan2(d, point[2] - anchor[2])) for d, point in zip(lateral, points)]
    worst = max(range(len(points)), key=lambda i: abs(residuals[i]))
    return {
        "initial_angle_deg": angles[0],
        "max_angle_deg": max(angles),
        "max_horizontal_offset_m": max(lateral),
        "max_abs_cable_minus_vertical_travel_m": abs(residuals[worst]),
        "worst_state_index": worst,
        "max_abs_cable_minus_vertical_travel_index": worst,
        "error_acceptance_authority": "NONE; diagnostics do not establish vertical equivalence",
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    # Reserve the fresh output before expensive computation; never overwrite evidence.
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("x", encoding="utf-8", newline="\n") as output:
        started = time.perf_counter()
        trajectory = build_back_squat(SquatProtocol(repetitions=1)).run()
        force_model, lpt_model = DUAL_1KHZ_V1.instruments(squat_lpt(trajectory))
        force = observe_dual_plates(trajectory, force_model)
        lpt = observe_lpt(trajectory, lpt_model)
        points = [attachment_world(trajectory, s, lpt_model.attachment) for s in trajectory.states()]
        geometry = geometry_diagnostics(points, lpt_model.anchor_m)
        report = {
            "schema_id": "BSMB-ACQUISITION-GEOMETRY-PREFLIGHT-1.0",
            "claim_ceilings": list(CLAIM_CEILINGS),
            "profile_id": DUAL_1KHZ_V1.profile_id,
            "case": "BACK_SQUAT_ONE_REPETITION_DEVELOPMENT_ONLY",
            "trajectory_termination": trajectory.termination,
            "trajectory_frame_count": len(trajectory.frames),
            "physical_duration_s": trajectory.times()[-1],
            "force_clock": asdict(force_model.clock),
            "lpt_clock": asdict(lpt_model.clock),
            "force_count": len(force.samples),
            "lpt_count": len(lpt.samples),
            "force_saturated_samples": sum(s.saturated for s in force.samples),
            "lpt_slack_samples": sum(s.slack for s in lpt.samples),
            "geometry": geometry,
            "elapsed_generation_and_probe_s": time.perf_counter() - started,
            "source_bundle_count": 0,
            "production_execution_count": 0,
            "production_truth_access_count": 0,
            "truth_injection_count": 0,
            "source_hashes": {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (
                Path("biosyngen/i2/acquisition.py"), Path("biosyngen/i2/instruments.py"),
                Path("biosyngen/i2/squat.py"), Path("biosyngen/multibody.py"),
                Path("scripts/probe_acquisition_geometry.py"))},
            "status": "BLOCKED_GEOMETRY_QUALIFICATION",
            "first_blocker": "UNQUALIFIED_CABLE_TO_VERTICAL_COORDINATE",
            "limitation": "Fixed reel is initially vertical but full-path vertical equivalence has no qualification authority. This probe never grants admission, even for zero measured residual.",
        }
        output.write(json.dumps(report, sort_keys=True, indent=2) + "\n")
    print(json.dumps({k: report[k] for k in ("status", "first_blocker", "force_count", "lpt_count", "geometry")}))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
