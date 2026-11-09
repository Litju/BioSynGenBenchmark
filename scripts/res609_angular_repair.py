"""RES-609 angular-momentum RED/GREEN replay for the ten baseline fixtures.

    python -m scripts.res609_angular_repair --out runs/i3/res598/working/res609-angular

Each fixture runs at the original nominal 1 ms step twice: under the
historical MASS_METRIC_V1 position projection (the retained RED) and under the
angular-momentum-neutral V2 projection. The unchanged independent evaluator
(``biosyngen_benchmark.i2_evaluator``) scores both; its budget is not edited.
A spin mutation (+1 rad/s on one body in one mid-trial frame) must be
REJECTED wherever the baseline passes. The kinematic effect of the repair is
measured as the maximum whole-body COM displacement between V1 and V2.
NUMERICAL_VERIFICATION_ONLY; no biological or device claim.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

from biosyngen.multibody import PROJECTION_POLICIES as POLICIES


def _scenario(name):
    from scripts.run_i2_benchmark import families
    for family, scenario in families():
        if family == name:
            return scenario
    raise KeyError(name)


def _fixture_names():
    return ("CMJ", "SJ", "LOADED_CMJ", "LOADED_JUMP_SQUAT", "BACK_SQUAT", "DEADLIFT",
            "BENCH_PRESS", "CLEAN_PULL", "SNATCH_PULL", "POWER_CLEAN_PULL_ONLY")


def run_case(args):
    name, policy = args
    from biosyngen.i2.export import export_submission
    from biosyngen_benchmark.i2_evaluator import evaluate_i2
    scenario = _scenario(name)
    scenario.model.system = scenario.model.system.with_projection_policy(policy)
    started = time.perf_counter()
    trial = scenario.run(dt_s=.001)
    elapsed = time.perf_counter() - started
    submission = export_submission(trial)
    submission["claims"].append("ANGULAR_MOMENTUM_CLOSURE")
    findings = evaluate_i2(submission)["findings"]
    spin = deepcopy(submission)
    spin["frames"][len(spin["frames"]) // 2]["body_states"][0][5] += 1
    spin_finding = evaluate_i2(spin)["findings"]["ANGULAR_MOMENTUM_CLOSURE"]
    com = [list(trial.model.system.center_of_mass(s)) for s in trial.states()]
    return dict(fixture=name, policy=policy, termination=trial.termination, wall_s=elapsed,
                frames=len(trial.frames), angular=findings["ANGULAR_MOMENTUM_CLOSURE"],
                linear=findings["LINEAR_MOMENTUM_CLOSURE"]["outcome"],
                linear_residual_n_s=findings["LINEAR_MOMENTUM_CLOSURE"].get("residual_n_s"),
                contact=findings["CONTACT_ADMISSIBILITY"]["outcome"],
                spin_negative=spin_finding["outcome"], com=com,
                angular_restoration=dict(scenario.model.system.angular_restoration_ledger))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True)
    parser.add_argument("--workers", type=int, default=4)
    args = parser.parse_args(argv)
    out = Path(args.out)
    if out.exists():
        parser.error("output must be a fresh directory")
    out.mkdir(parents=True)
    started = time.monotonic()
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    dirty = bool(subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], text=True).strip())
    jobs = [(name, policy) for name in _fixture_names() for policy in POLICIES]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(run_case, jobs))
    by = {(r["fixture"], r["policy"]): r for r in rows}
    table = []
    for name in _fixture_names():
        v1, v2 = by[(name, POLICIES[0])], by[(name, POLICIES[1])]
        n = min(len(v1["com"]), len(v2["com"]))
        shift = max(((a[0]-b[0])**2 + (a[1]-b[1])**2) ** .5 for a, b in zip(v1["com"][:n], v2["com"][:n]))
        eligible = v2["angular"]["outcome"] == "PASS"
        table.append(dict(
            fixture=name, budget_kg_m2_s=v2["angular"]["tolerance_kg_m2_s"],
            v1_budget_kg_m2_s=v1["angular"]["tolerance_kg_m2_s"],
            v1_outcome=v1["angular"]["outcome"], v1_max_prefix=v1["angular"]["max_prefix_residual_kg_m2_s"],
            v2_outcome=v2["angular"]["outcome"], v2_max_prefix=v2["angular"]["max_prefix_residual_kg_m2_s"],
            v1_termination=v1["termination"], v2_termination=v2["termination"],
            v2_linear=v2["linear"], v2_contact=v2["contact"],
            v2_spin_negative=v2["spin_negative"], v2_spin_detected=(v2["spin_negative"] == "REJECTED") if eligible else None,
            max_com_shift_v1_v2_m=shift, wall_s_v1=v1["wall_s"], wall_s_v2=v2["wall_s"],
            v2_angular_restoration=v2["angular_restoration"]))
    for row in rows:
        row.pop("com")
    receipt = dict(
        issue="RES-609", source_sha=sha, source_dirty=dirty, nominal_step_s=.001,
        evaluator_sha256=hashlib.sha256(Path("biosyngen_benchmark/i2_evaluator.py").read_bytes()).hexdigest(),
        multibody_sha256=hashlib.sha256(Path("biosyngen/multibody.py").read_bytes()).hexdigest(),
        python=sys.version.split()[0], wall_s=time.monotonic() - started, table=table, cases=rows,
        claim_ceilings=["NUMERICAL_VERIFICATION_ONLY", "SIMULATED_MEASUREMENT_ONLY",
                        "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED", "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED"],
        scope="Planar fixed-origin interval-end angular balance; V1 failures are retained RED evidence")
    (out / "angular_repair.json").write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")
    for row in table:
        print(f"{row['fixture']:24s} V1 {row['v1_outcome']:8s} {row['v1_max_prefix']:.6g}  "
              f"V2 {row['v2_outcome']:8s} {row['v2_max_prefix']:.6g}  budget {row['budget_kg_m2_s']:.6g}  "
              f"spin {row['v2_spin_negative']}  dCOM {row['max_com_shift_v1_v2_m']:.3g} m")
    print("V2 angular PASS", sum(r["v2_outcome"] == "PASS" for r in table), "/", len(table),
          "; V1 angular PASS", sum(r["v1_outcome"] == "PASS" for r in table), "/", len(table))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
