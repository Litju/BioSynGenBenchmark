# BioSynGenBenchmark

Canonical research and program workspace for the Research project [Biomechanical Synthetic Measurement Generation & Benchmarking](https://linear.app/alignerr-cmj/project/biomechanical-synthetic-measurement-generation-and-benchmarking-18565cbe99b1).

Keep program work products here. The sibling D:\projects\BioSynGenBenchmark-evidence is limited to the minimal retained scientific evidence admitted under the [Workspace Authority & Minimal Evidence Policy](https://linear.app/alignerr-cmj/document/workspace-authority-and-minimal-evidence-policy-biosyngenbenchmark-3aa199d05e55); it is not a second project, backup, corpus mirror, or home for routine working files.

## I0 implementation pilot

The only authorized implementation queue is RES-475 → RES-476 → RES-477 → RES-478 → RES-479 under the [I0 Overnight Implementation Authority & Scientific Gate Contract](https://linear.app/alignerr-cmj/document/i0-overnight-implementation-authority-and-scientific-gate-contract-ad04c81bd73b). This is a bounded engineering pilot. Synthetic observations support numerical verification only; biological and real paired FORCE/LPT validation remain unestablished.

### Controller

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
    python -m scripts.run_i1 --replicate             # figures, results, manifest; two-process digest check
    python -m scripts.i1_benchmark_suite             # adjudicated evaluator challenge suite
    python -m scripts.i1_provenance_sensitivity      # provenance ledger, sensitivity, counterexamples
    python -m scripts.i1_i0_regression               # I0 artifact preservation check
    python -m scripts.i1_optimize_control --task cmj # declared control-hypothesis search (slow)

Optional independent-solver check, where SciPy is installed:

    py -3.11 -m unittest tests.test_i1_solver_crosscheck -v

**Claim ceiling (I1):**
- Mechanics: VERIFIED_NUMERICALLY. Human model: MODELED_NOT_EMPIRICALLY_VALIDATED.
- BIOLOGICAL_VALIDATION=NOT_ESTABLISHED; REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.
- Simulated devices are SIMULATED_DEVICE_HYPOTHESIS, not vendor equivalents.
- No parameter is SOURCE_VERIFIED.

## License

License pending owner decision. No open-source license is granted; all rights are reserved until a license is selected.
