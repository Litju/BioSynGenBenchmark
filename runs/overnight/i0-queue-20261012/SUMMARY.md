# BioSynGenBenchmark I0 overnight report

**Date:** 2026-10-12  
**Workspace:** D:\projects\BioSynGenBenchmark  
**Linear project:** P-RES-41 — Biomechanical Synthetic Measurement Generation & Benchmarking  
**Outcome:** Implementation queue verified through RES-479. Issues are kept at In Review for owner review; none are marked Done.

## Issue outcomes

- **RES-475 — In Review:** Added root agent rules, a bounded Windows PowerShell controller, atomic resumable state, a five-issue allowlist, structured result parsing, attempt/deadline limits, and README instructions. Its real dry run validated root/runtime/CLI help/queue construction without launching Codex. The controller refuses live CLI execution while the CLI Linear MCP reports “Not logged in.”
- **RES-476 — In Review:** Added exact 1D stance/flight mechanics for body plus optional rigidly coupled bar/load, explicit prescribed support force, events, SI fields, and refusal of unsupported landing impacts. The independent 41-sample fixture measured maximum absolute state errors of 4.440892098500626e-16 m and m/s against binary64-derived roundoff bounds of 6.864202433121135e-13 m and 1.7344177649647465e-12 m/s.
- **RES-477 — In Review:** Added separate simulated FORCE and cable-LPT series with independent native clocks, geometry projection, explicit optional foot-force partition, offsets/noise/quantization/saturation, and missing-sample records. The deterministic example contains 20 FORCE and 32 LPT samples.
- **RES-478 — In Review:** Added a separate evaluator package that imports no generator or Plate2bar modules. It reports claim-level PASS, REJECTED, and NOT_EVALUABLE findings and counts. The report includes three admitted known-answer checks, four rejected adversarial claims, and five not-evaluable incomplete-evidence claims.
- **RES-479 — In Review:** Added a one-command integrated run, public evaluator input, report, accessible SVG, manifests, and same-seed normalized digest comparison.

## Implemented software

- Controller: AGENTS.md and scripts/overnight.ps1
- Mechanics and tests: biosyngen/mechanics.py, biosyngen/__init__.py, tests/test_mechanics.py
- FORCE/LPT models and tests: biosyngen/observations.py, tests/test_observations.py
- Independent evaluator and tests: biosyngen_benchmark/core.py, biosyngen_benchmark/__init__.py, tests/test_benchmark.py
- Integrated runner and tests: scripts/run_pilot.py, tests/test_run_pilot.py
- Fixture/report runners: scripts/generate_mechanics_fixture.py, scripts/generate_observation_fixture.py, scripts/generate_benchmark_report.py
- Controller tests: tests/test_overnight_controller.ps1
- Mathematical and claim-limit notes: docs/mechanics.md, docs/observations.md, docs/benchmark.md
- Project instructions and reproduction commands: AGENTS.md and README.md

Generated run records and artifacts are listed in the Generated artifacts section below.

## Verification

- python -m unittest discover -s tests -v — exit 0; 43 tests passed.
- pwsh -NoProfile -File tests\test_overnight_controller.ps1 — exit 0; 23 assertions passed, including fake worker logs, atomic resume state, bounded child timeout, and fail-closed gates.
- python -m scripts.generate_mechanics_fixture — verified identical fixture; 41 samples; maximum position/velocity error 4.440892098500626e-16.
- python -m scripts.generate_observation_fixture — verified identical fixture; FORCE=20 samples, LPT=32 samples, independent clock metadata and missing values retained.
- python -m scripts.generate_benchmark_report — verified identical report; valid, adversarial, and incomplete-evidence counts reproduced.
- python -m scripts.run_pilot — exit 0. Two runs with identical seed, configuration, and Python version produced the same normalized digest: b7de6f94886a2420d6f3653eeea49baaa4b624f8b58d0978fb.
- RES-475 dry run: pwsh -NoProfile -File scripts\overnight.ps1 -Root D:\projects\BioSynGenBenchmark -LinearSnapshotPath D:\projects\BioSynGenBenchmark\runs\overnight\linear-snapshot.json -DryRun -SessionId res475-dryrun-acceptance-20261012 — exit 0; Codex was not launched. It inspected 33 project items, found 0 reparse points and no .git, validated the fixed queue and supported CLI flags, and saved a transcript and journal.
- Missing/noncanonical root and missing Linear snapshot probes each failed closed with exit 1.
- The independent evaluator package passes its AST import-boundary test and does not expose a candidate hidden-truth field.

Ruff and Matplotlib were unavailable, so no linter or Matplotlib renderer was used. The figure is a dependency-free vector SVG. The Python and PowerShell test suites, AST independence check, SVG XML parsing, accessible title/description, and figure text-bound checks passed. A rendered screenshot preview was blocked because the session browser disallows local-file URLs, so the owner should make the final visual review of the SVG.

## Reproduction commands

Run the integrated report, figures, and digest comparison:

    python -m scripts.run_pilot

Run the full tests:

    python -m unittest discover -s tests -v
    pwsh -NoProfile -File tests\test_overnight_controller.ps1

Reproduce the individual mechanics, observation, and evaluator artifacts:

    python -m scripts.generate_mechanics_fixture
    python -m scripts.generate_observation_fixture
    python -m scripts.generate_benchmark_report

All commands use Windows-native Python/PowerShell and standard-library code. No package installation, WSL, Docker, cloud service, nested Codex process, or second execution harness was used.

## Generated artifacts

- Controller preflight: runs/overnight/res475-dryrun-acceptance-20261012/dry-run.txt
- Durable issue journal: runs/overnight/i0-queue-20261012/status.json
- RES-476 fixture: runs/overnight/i0-queue-20261012/res476-mechanics-fixture.json
- RES-477 observations: runs/overnight/i0-queue-20261012/res477-observation-fixture.json
- RES-478 evaluation report: runs/overnight/i0-queue-20261012/res478-benchmark-report.json
- RES-479 reproducibility record: runs/overnight/i0-queue-20261012/reproducibility.json
- End-to-end run artifacts and plots: runs/overnight/i0-queue-20261012/end-to-end-run-a/ and end-to-end-run-b/
- Main plot: runs/overnight/i0-queue-20261012/end-to-end-run-a/pilot-figure.svg
- Reproduction input: runs/overnight/i0-queue-20261012/end-to-end-run-a/evaluator-input.json
- Full scientific and numerical report: runs/overnight/i0-queue-20261012/end-to-end-run-a/pilot-report.md

## Scientific and governance limits

Claims remain NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY, BIOLOGICAL_VALIDATION=NOT_ESTABLISHED, and REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED. The mechanics world is one-dimensional with prescribed support force; the sensor models are simplified synthetic assumptions; the evaluator admits numerical known-answer checks only and refuses unsupported empirical claims.

**Boundary audit:** evidence-folder mutations = 0; Plate2bar mutations = 0; historical/locked holdout replays = 0; Git initialization = 0. No private corpus or participant records were accessed.

## Open follow-up

- Owner review of RES-475 through RES-479 and visual review of the SVG.
- The reusable controller remains fail-closed for live CLI runs until the separate Codex CLI Linear MCP is authenticated. This task used the connected app-level Linear integration directly and did not launch the controller as a nested Codex process.
- Scientific validation and any later empirical data access remain under the separate R0–R4 gates.
