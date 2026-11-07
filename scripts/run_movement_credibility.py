"""Bounded RES-598 scientific report using the existing independent evaluator.

Run: python -m scripts.run_movement_credibility --out <fresh-directory>
Domain cells remain pending unless the entire domain is supported; numerical
subclaims never imply anatomical, biological or device qualification.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

from biosyngen_benchmark.scientific_matrix import qualification_matrix, qualification_counts


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def recovery_finding(trial):
    return dict(termination=trial.termination,
        end_time_s=trial.frames[-1].t_end_s if trial.frames else None,
        claim="Complete terminal landing recovery", expected="DURATION_REACHED with supported recovery",
        measured="FAIL" if trial.termination != "DURATION_REACHED" else "EVIDENCE_PENDING",
        scope="Experimental default candidate only; no broad movement qualification")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    out = Path(args.out)
    if out.exists():
        parser.error("output must be a fresh directory")
    out.mkdir(parents=True)
    started = time.monotonic()
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    dirty = bool(subprocess.check_output(["git", "status", "--porcelain"], text=True).strip())
    baseline = subprocess.run([sys.executable, "-m", "scripts.run_i2_benchmark", "--out", str(out/"baseline")],
                              capture_output=True, text=True, timeout=1200)
    (out/"baseline.log").write_text(baseline.stdout + baseline.stderr, encoding="utf-8")
    if baseline.returncode:
        return baseline.returncode
    # Generator orchestration is outside the independent evaluator package.
    from biosyngen.i2.drop_jump import build_drop_jump_candidate
    from biosyngen.i2.export import export_submission
    from biosyngen_benchmark.i2_evaluator import evaluate_i2, FAULTS, fault_outcome
    cases = []
    for dt in (.0004, .0002, .0001):
        trial = build_drop_jump_candidate(duration_s=.5).run(dt)
        submission = export_submission(trial)
        result = evaluate_i2(submission)
        faults = {}
        for name, (mutate, expected, claim) in FAULTS.items():
            verdict = fault_outcome(evaluate_i2(mutate(submission)), claim)
            faults[name] = dict(measured=verdict, expected=list(expected), detected=verdict in expected)
        payload = json.dumps(submission, sort_keys=True, allow_nan=False).encode()
        cases.append(dict(context_of_use="Synthetic planar adult anthropometry; post-release first rebound only",
            population="Manufactured model; no human cohort", claim="LINEAR_MOMENTUM_CLOSURE and CONTACT_ADMISSIBILITY",
            oracle="biosyngen_benchmark.i2_evaluator; distinct force integration; no generator imports",
            conditions=dict(dt_s=dt, duration_s=.5, drop_height_m=.15), expected="PASS for bounded numerical subclaims",
            measured=result, uncertainty="Timestep and assumed material/model parameters; no calibrated device budget",
            tolerance_basis="Existing evaluator MOMENTUM_TOLERANCE and FORCE_TOLERANCE; no candidate tolerance",
            falsifiers=faults, submission_sha256=hashlib.sha256(payload).hexdigest(),
            termination=trial.termination, completion="FULL_EXERCISE_NOT_QUALIFIED"))
    matrix = qualification_matrix()
    recovery = build_drop_jump_candidate().run()
    recovery_evidence = recovery_finding(recovery)
    for cell in matrix["primary_cells"]:
        if cell["movement"] == "DROP_JUMP" and cell["domain"] == "MOVEMENT":
            cell.update(disposition=recovery_evidence["measured"], evidence=recovery_evidence,
                        reason="Measured terminal recovery; see explicit candidate failure evidence")
    counts = qualification_counts(matrix["primary_cells"])
    report = dict(schema="BSMB-MOVEMENT-CREDIBILITY-1", source_sha=sha,
        source_dirty=dirty, runner_sha256=digest(__file__),
        evaluator_sha256=digest("biosyngen_benchmark/i2_evaluator.py"),
        matrix=matrix, denominators=counts, bounded_drop_cases=cases,
        terminal_recovery=recovery_evidence, full_campaign_ready=False,
        baseline_sha256=digest(out/"baseline"/"benchmark.json"),
        baseline=json.loads((out/"baseline"/"benchmark.json").read_text(encoding="utf-8")),
        elapsed_s=time.monotonic()-started, data_rights="OWNED_MANUFACTURED_ONLY_NO_REAL_DATA_ADMITTED",
        independent_final_sha_acceptance="PENDING",
        claim_ceilings=["NUMERICAL_VERIFICATION_ONLY", "SIMULATED_MEASUREMENT_ONLY",
                       "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED", "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED"])
    (out/"credibility.json").write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False)+"\n", encoding="utf-8")
    lines = ["# Movement credibility evidence", "", f"Source SHA: `{sha}`", "",
             "No global scientific acceptance is inferred from the numerical subclaims.", "",
             "| Movement | " + " | ".join(dict.fromkeys(c["domain"] for c in matrix["primary_cells"])) + " |",
             "|---|" + "---|"*12]
    for movement in dict.fromkeys(c["movement"] for c in matrix["primary_cells"]):
        lines.append("| " + movement + " | " + " | ".join(c["disposition"] for c in matrix["primary_cells"] if c["movement"]==movement) + " |")
    lines += ["", "Full drop recovery and Olympic receiving remain unqualified. Biological comparisons and",
              "human/device reliability are NOT_EVALUABLE without licensed matched experimental data."]
    (out/"credibility.md").write_text("\n".join(lines)+"\n", encoding="utf-8")
    size = sum(p.stat().st_size for p in out.rglob("*") if p.is_file())
    if size > 1073741824:
        raise RuntimeError("artifact disk budget exceeded")
    baseline_results = report["baseline"]
    positive = baseline_results["positive_controls"]
    baseline_valid = positive["n"] > 0 and positive["passed"] == positive["n"] and all(
        f["n"] > 0 and f["detected"] == f["n"] for f in baseline_results["fault_detection"].values())
    valid = baseline_valid and all(c["termination"] == "DURATION_REACHED" and
                all(c["measured"]["findings"][k]["outcome"] == "PASS" for k in ("LINEAR_MOMENTUM_CLOSURE", "CONTACT_ADMISSIBILITY")) and
                all(f["detected"] for f in c["falsifiers"].values()) for c in cases)
    print(json.dumps(dict(source_sha=sha, bounded_numerical_gate=valid, bytes=size, elapsed_s=report["elapsed_s"], denominators=counts)))
    return 0 if valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
