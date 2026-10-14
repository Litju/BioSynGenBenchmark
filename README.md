# BioSynGenBenchmark

Canonical research and program workspace for the Research project [Biomechanical Synthetic Measurement Generation & Benchmarking](https://linear.app/alignerr-cmj/project/biomechanical-synthetic-measurement-generation-and-benchmarking-18565cbe99b1).

## Current I3 qualification and release preparation

I3 independently audits the existing I2 implementation under RES-502. Local source/provenance findings are in `docs/i3/RES503_SOURCE_AUDIT.md`; fresh test logs and exact exit receipts are in `runs/i3/`. The I2 implementation remains submitted for review, with scoped mechanics refusals. The owner selected Apache-2.0 for owned code, docs and synthetic artifacts; final publication/content qualification is in progress. No new public I2 branch or hosted I2 CI pass is claimed.

Core code uses the Python standard library. From the project root on Windows (CPython 3.12.10 used for local qualification), or from a reviewed checkout on Linux:

    python -m unittest discover -s tests -v
    python -m scripts.run_i2_benchmark --out runs/i3/local-benchmark
    python -m scripts.verify_i3_reproducibility

The I3 reproduction runner launches separate Python processes, preserves outputs under the project, checks matching complete artifact inventories, and independently gates the authored 10-positive/76-fault benchmark counts. Deterministic bytes alone do not verify physics or cross-platform numerical equivalence. Hosted Windows/Linux qualification remains a release gate; Linux instructions are not a claim of completed hosted verification.

For the optional independent I1 integration check, use a separate Python 3.11 environment with SciPy 1.17.1 and NumPy 2.4.4, then run `python -m unittest tests.test_i1_solver_crosscheck -v`. On Windows the already-installed qualified environment is invoked with `py -3.11`. Record actual versions and true exit codes. The SciPy tests share the I1 derivative and do not independently verify I2 equations.

See [CONTRIBUTING](CONTRIBUTING.md), [SECURITY](SECURITY.md), [data availability](DATA_AVAILABILITY.md), and the [license/rights proposal](docs/i3/RES505_LICENSE_AND_RIGHTS.md). CITATION.cff supplies provisional software citation metadata; cite the exact reviewed commit. Historical reproduction sections below retain their original provenance and reported counts.

Keep program work products here. The sibling D:\projects\BioSynGenBenchmark-evidence is limited to the minimal retained scientific evidence admitted under the [Workspace Authority & Minimal Evidence Policy](https://linear.app/alignerr-cmj/document/workspace-authority-and-minimal-evidence-policy-biosyngenbenchmark-3aa199d05e55); it is not a second project, backup, corpus mirror, or home for routine working files.

## I0 implementation pilot

The RES-475 → RES-476 → RES-477 → RES-478 → RES-479 queue is the completed I0 pilot under the [I0 Overnight Implementation Authority & Scientific Gate Contract](https://linear.app/alignerr-cmj/document/i0-overnight-implementation-authority-and-scientific-gate-contract-ad04c81bd73b). Its files and runs are historical regression material. The active full-system follow-on queue is RES-489 → RES-490 → RES-491 → RES-492 under parent RES-488. Synthetic observations support numerical verification only; biological and real paired FORCE/LPT validation remain unestablished.

### Historical I0 controller

`scripts/overnight.ps1` is retained for the completed I0 queue only. It does not run RES-489 → RES-492; do not start it from an active Codex agent. The current queue is being executed directly in the existing session under each live issue's acceptance criteria.

The Windows PowerShell 7 controller is **scripts/overnight.ps1**. It runs in the foreground, uses one worker, enforces a six-hour wall-clock deadline, limits the session to these five issue IDs, and permits one initial attempt plus two bounded repairs per issue. It writes atomic state and concise records under **runs/overnight/<session-id>**. Ctrl+C interrupts the foreground process; resume with the printed session ID.

The controller requires a fresh, authenticated Linear MCP snapshot containing exactly the fixed queue in order. A Linear-connected agent should create or refresh this small JSON snapshot after reading the live issues; it contains only issue identifiers, statuses, project, source, and check time. A missing, stale, reordered, or disconnected snapshot fails closed. A real run additionally requires authenticated Linear MCP access in the Codex CLI. On the initial preflight for this session, the app-level Linear MCP was connected, while codex mcp list reported the CLI Linear entry as “Not logged in.” The dry run can verify command construction using the fresh app snapshot; a real CLI run must stop until the CLI reports authenticated Linear access.

From a separate, top-level local Codex session, run:

    $root = 'D:\projects\BioSynGenBenchmark'
    $snapshot = Join-Path $root 'runs\overnight\linear-snapshot.json'
    & (Join-Path $root 'scripts\overnight.ps1') -Root $root -LinearSnapshotPath $snapshot -MaxHours 6

To validate setup without starting Codex:

    & (Join-Path $root 'scripts\overnight.ps1') -Root $root -LinearSnapshotPath $snapshot -DryRun

The dry run prints the session ID and exact resume command. Resume a stopped session with:

    & (Join-Path $root 'scripts\overnight.ps1') -Root $root -SessionId '<printed-session-id>' -Resume -LinearSnapshotPath $snapshot

Cancel with Ctrl+C in that PowerShell window. A fresh Linear snapshot is required on resume. Keep Windows awake and connected; host sleep, network loss, unavailable model access, or disconnected Linear MCP interrupts progress. The runner does not start itself recursively. Do not run it from an already active Codex agent; that agent should continue the approved queue directly.

### Controller checks

Run the no-dependency PowerShell checks with:

    pwsh -NoProfile -File tests\test_overnight_controller.ps1

The checks use locally generated fake worker result logs and cover allowlist order, worker exit/result gates, stop conditions, no re-execution of issues already In Review, and atomic state replacement. They make no external service calls.

Run the Windows-native mechanics known-answer and refusal checks with:

    python -m unittest discover -s tests -p test_mechanics.py -v

The RES-476 mechanics fixture and independent exact-solution error summary can be reproduced with:

    python -m scripts.generate_mechanics_fixture

It writes one small manufactured latent mechanics fixture under runs/overnight. Repeating the command verifies identical content and leaves the existing file unchanged.

Run the separate FORCE and cable-LPT observation tests with:

    python -m unittest discover -s tests -p test_observations.py -v

Reproduce their small synthetic observation fixture with:

    python -m scripts.generate_observation_fixture

The FORCE and LPT files retain their own native clocks and time vectors. The fixture contains no generator latent-state fields or real sensor records.

Run the independent benchmark controls with:

    python -m unittest discover -s tests -p test_benchmark.py -v

Generate its known-answer, adversarial, and missing-evidence report with:

    python -m scripts.generate_benchmark_report

The evaluator imports only its separate public input contract and the Python standard library. Its output reports claim-level counts and reasons; it does not compute a universal realism score.

Run the integrated reproducible pilot, independent evaluation, and SVG figure with:

    python -m scripts.run_pilot

It writes two same-seed run directories under runs/overnight/i0-queue-20261012, compares normalized artifact digests, and records the digest comparison. Reproduce its deterministic file checks with:

    python -m unittest discover -s tests -p test_run_pilot.py -v

### Scientific claim ceiling

Artifacts must label the work **NUMERICAL_VERIFICATION_ONLY** and **SIMULATED_MEASUREMENT_ONLY**, with **BIOLOGICAL_VALIDATION=NOT_ESTABLISHED** and **REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED**. No pilot result certifies human biomechanics, instrument fidelity, vendor equivalence, or a universal credibility score.

## I1 neuromechanical reconstruction (RES-480 → RES-481 … RES-487)

I1 audits the I0 pilot, repairs its P0 defects, and adds a forward-dynamical successor. The I0 1-D model stays as a separate known-answer fixture. Owner summary: **I1_FINAL_REVIEW.md**. Details: `docs/i1/` (AUDIT, MODEL_SPEC, PHASE_OBSERVABILITY, EMPIRICAL_EVIDENCE, VANDV, parameter_provenance.json).

- `biosyngen/i1/`:
  - planar 4-segment body (foot, shank, thigh, head-arms-trunk), with an optional gripped bar;
  - Thelen-form activation driving Hill-type joint torque actuators;
  - rigid unilateral Coulomb contact (flat foot / toe / flight). Takeoff happens when the computed toe force reaches zero;
  - latent phase predicates;
  - staged force-plate and 3-D cable-LPT forward models;
  - public export.
- `biosyngen_benchmark/{i1,detectors,controls,suite}.py`: the candidate-neutral evaluator. Standard library only; it never imports the generator.

Reproduce (Windows, CPython 3.12, standard library only):

    python -m unittest discover -s tests -v          # 96 tests (2 SciPy cross-checks skip without SciPy)
    python -m scripts.run_i1 --replicate --out runs/i2/res489/i1/run # fresh two-process digest check; preserve RES-487 files
    python -m scripts.i1_benchmark_suite             # adjudicated evaluator challenge suite
    python -m scripts.i1_provenance_sensitivity      # provenance ledger, sensitivity, counterexamples
    python -m scripts.i1_i0_regression --output runs/i2/res489/i0-regression.json # preserve historical regression record
    python -m scripts.i1_optimize_control --task cmj # declared control-hypothesis search (slow)

Optional independent-solver check, where SciPy is installed:

    py -3.11 -m unittest tests.test_i1_solver_crosscheck -v

**Historical I1 interpretation:** numerical software checks are `NUMERICAL_VERIFICATION_ONLY`; modeled device outputs are `SIMULATED_MEASUREMENT_ONLY`; `BIOLOGICAL_VALIDATION=NOT_ESTABLISHED`; `REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED`. The RES-489 addendum below supersedes the old evaluator results for current review.

## I2 full-system follow-on (RES-488; active queue RES-489 → RES-492)

The binding I2 acceptance contract is the live Linear issue text and the linked [Independent I1 Source Audit & Full Protocol Coverage Gap](https://linear.app/alignerr-cmj/document/biosyngenbenchmark-independent-i1-source-audit-and-full-protocol-2718d3dfb375). Read each live issue before beginning it. RES-489 hardens the evaluator; RES-490 inventories the actual exercise/test registry read-only; RES-491 builds reusable multibody mechanics; RES-492 completes the jumping protocol family. Keep each result in Review until owner acceptance. Do not publish or merge unreviewed claims.

### RES-489 evaluator contract

`biosyngen_benchmark.i1` is still standard-library-only and imports no generator code. Inputs must have finite physical values, dimensioned vectors and strictly increasing kinematic/device timestamps. Duplicate or malformed samples are rejected before numerical calculations. Event/contact transitions split quadrature windows. Nested-grid differences are reported as estimates, never mathematical error bounds; unstable sampled windows return NOT_EVALUABLE. A sampled PASS is a software/numerical check and cannot rule out an aliased between-sample signal.

FORCE/LPT acceptance tolerances are supplied only out of band with a `MetrologyAuthority` created by the benchmark operator from an independently admitted reference. Candidate uncertainty fields are ignored and listed as refused. The repository currently contains no admitted real FORCE/LPT calibration records, so those checks return NOT_EVALUABLE by default. The test-only authority in `tests/test_i2a_evaluator_adversarial.py` is a manufactured software fixture, not calibration evidence. `SENSOR_METROLOGY`, `BIOLOGICAL_VALIDITY` and `REAL_PAIRED_FORCE_LPT` remain NOT_EVALUABLE.

The suite gate requires every adjudicated positive and negative to receive its expected decision, and every abstention control to abstain. It reports decision coverage, refusal and Wilson intervals by denominator; those intervals describe only the finite authored controls. `unseen_two_body_free_fall` is a hand-authored closed-form family outside the generator.

Reproduce RES-489 without overwriting historical RES-481/485/487 outputs:

    python -m unittest discover -s tests -p test_i2a_evaluator_adversarial.py -v
    python -m scripts.i1_benchmark_suite
    python -m scripts.i1_i0_regression --output runs/i2/res489/i0-regression.json
    python -m scripts.run_i1 --replicate --out runs/i2/res489/i1/run
    pwsh -NoProfile -File tests/test_overnight_controller.ps1
    py -3.11 -m unittest tests.test_i1_solver_crosscheck -v

The first command includes the before-repair regressions preserved at `runs/i2/res489/falsifiers-before-repair.txt`; the post-repair run record is `runs/i2/res489/falsifiers-after-repair.txt`. The adjudicated report is `runs/i2/res489/benchmark-suite.json`. Cross-platform CI is defined in `.github/workflows/verification.yml`; its explicit SciPy job fails if dependency installation or the independent-solver checks fail.

**Permitted claim ceilings:** `NUMERICAL_VERIFICATION_ONLY`, `SIMULATED_MEASUREMENT_ONLY`, `BIOLOGICAL_VALIDATION=NOT_ESTABLISHED`, and `REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED`.

## License

Owned project code, documentation and manufactured fixtures/figures, including retained project-authored historical versions in this distribution, are licensed under [Apache-2.0](LICENSE). Copyright 2026 Julio Rodriguez; see [NOTICE](NOTICE). External source implementations, papers, books, participant data and restricted datasets are excluded. Licensing does not establish human biomechanics, device metrology or real paired FORCE/LPT validity.
