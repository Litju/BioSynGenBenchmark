# RES-503 source and publication inventory

Baseline: `3322d89499050eea0004f9e8882666e2a40d5a9a` on preserved local `claude/res-488-roadmap-r1`. I3 work branch: `codex/res-502-i3-qualification`. `git ls-remote --heads origin` returned only main at `6115d702ba3ab814982353945b0753f4846aae88`. No fetch, push, history rewrite or main merge was performed. Initial tracked worktree was clean; the only untracked entry was `.claude/launch.json`, a 197-byte local HTTP-server launch configuration. It remains untracked and excluded from staging.

The frozen Git tree is `fa315a4894414025cf245f84f3a1b9df7a7950b4`. The sorted path/NUL/blob inventory SHA-256 is `82fd1547e93e712692311fe96beb00a34f92314b119179cff8daef620d80a340`. The machine-readable manifest at `runs/i3/res503/provenance.json` covers 200 baseline paths, 29 reachable commits and 229 distinct history blobs, with exact SHA-256, Git blob, size, paths, provisional license/provenance and disposition. Its commit ledger includes the full SHA and complete changed-path list for each logical change, including the initial public I1 snapshot.

Reproduce the frozen inventory in Windows PowerShell:

    git switch --detach 3322d89499050eea0004f9e8882666e2a40d5a9a
    git rev-parse HEAD
    git rev-parse HEAD^{tree}

Use a separate review checkout or restore the I3 branch before running `python -m scripts.audit_i3_provenance`; the audit script lives on the I3 branch and defaults to the immutable I2 baseline. It reads Git blobs, not ignored directories. Re-running it yields the same manifest bytes. Reviewers need the local I3 branch until publication gates clear; there is no public I2 PR.

## Commit attribution and negative evidence

RES-489: `895c8e60893337491ad5d030c313fac91b8867fc`, `913f01e2454fd9948ef904d4884f3f203c9a91e4`, `4cf4f6a4bea77ca5a45daf2053c146589968c86b`. RES-490: `eade435c427e7f12349ad370168c0984b6836979`. The ledger maps RES-491–498 and RES-488 checkpoints individually; it does not squash them.

RES-491's foundation at `b5b4c29409a9418d8c89eec4578b30ed22af8f77` is subsequently changed by six RES-492 engine commits, including the Signorini–Coulomb correction `19497a79641ee7ffb2ba4e8c0bee7c910bedc7b7`. Thus the final contact solver cannot be qualified using the RES-491 SHA alone. RES-497's `981248dbc36c6441fe3759d106a9f2e45a95377b` historically carried a failing rack-feasibility test; `229a2790beb61169c0b7202e6058736406ade8fc` repairs its forward-kinematics sign. The failure and correction remain immutable. The original recorded 243 tests and 10/10 + 76/76 results are historical agent reports until fresh execution receipts establish their reproduction.

## Source-level map and audit findings

* `biosyngen/multibody.py`: COM-coordinate diagonal mass matrix, revolute/weld equality constraints, sparse Schur KKT solve, velocity-level Signorini–Coulomb modes, grouped friction patches, simultaneous rigid impulses and separate position/velocity projections. Projection bookkeeping and restitution require independent analytical falsifiers in RES-504; deterministic momentum bookkeeping does not establish energy or angular-momentum accuracy.
* `biosyngen/i2/human.py`: separate planar bilateral legs, recalled segment fractions and uncalibrated torque/activation hypotheses. Back bar is a unilateral shoulder contact, vest a weld, gripped bar a revolute arm link. No human calibration is admitted.
* `biosyngen/i2/simulate.py`: declared staged PD/feedforward; interval-average forces derived from step/projection impulses, impacts stored separately. Controller stages are exported only as model hypotheses. Termination can refuse constraints or unmodeled body-floor contact.
* `biosyngen/i2/{jumps,phases,squat,deadlift,bench,olympic,power_clean}.py`: force/state event definitions, support-specific analyses and existing refusals. COM force-times-velocity is called `system_com_work_j` in the squat analysis; it is pseudo-work, not stationary ground-contact work. Default bench analysis checks support inside repetitions but transitions need separate qualification. A back bar may separate during jump propulsion. Front-rack feasibility clips the law-of-cosines argument, so over-reach and malformed parameter domains need adversarial checks.
* `biosyngen/i2/instruments.py`: independent clocks; plate gain/cross-talk, Butterworth filter, noise/ADC; cable geometry, reel retraction limit, quantization and provider differentiation. Rigid impact peaks explicitly remain unevaluable. Numerical bilateral sum identity is distinct from empirical calibration.
* `biosyngen_benchmark/i2_evaluator.py`: generator-independent imports, strict advertised field sets, total-record linear impulse balance and grouped vertical-ground friction. Source inspection finds unvalidated nested container/initial-state/friction domains, unchecked impulse time/cone, and no interval-local momentum challenge. These are hypotheses for fresh RES-504 falsifiers, not repaired in this audit.
* `biosyngen_benchmark/i2_coverage.py` and the pinned RES-490 inventory: 11 registered exercise rows, six deferred variants and two refused jump subtypes. Full clean/snatch, hang variants, drop/rebound jumps and actual power-clean turnover/catch remain NOT_IMPLEMENTED. Block pulls are explicitly refused by the pull API. All external human/device references, anatomical sensor claims and real bilateral calibration remain NOT_EVALUABLE.
* `scripts/run_i2_benchmark.py`: curated generator positives and authored fault copies, no independently sampled fault population. It currently returns zero regardless of tally failures. `scripts/check_i2_reproducibility.py` executes twice in one process via imported `main`, despite the historical two-process claim. It uses system temporary directories and checks SVG structure rather than rendered scientific content. Fresh independent process runs and meaningful exit gates belong to RES-504/506.
* `.github/workflows/verification.yml`: Windows/Linux definitions exist, with an optional SciPy job. Hosted I2 results do not yet exist; no remote pass is asserted.

This source map records inspected equations, control/measurement contracts, refusal paths and test structure. It is not a completed scientific V&V conclusion. The original I0/I1 source snapshots and run artifacts are retained as software regression material. The independent physics campaign must go beyond the old tests.

## Publication disposition

Pattern screening of every eligible history blob found no private-key, GitHub-token, provider-key or AWS-key pattern matches, no NUL-bearing binaries and no blob above 5 MB. Twenty distinct blobs contain local machine paths. These include preserved failure logs, controller docs/tests and the historical summaries; they require an explicit metadata disposition before publishing the intended history. Pattern screening cannot prove absence of PII, undisclosed copied material or credentials with an unrecognized format.

199 current paths are candidate project code/docs/manufactured outputs pending owner license and content review. `docs/i2/RES490_PROTOCOL_TEST_CAPABILITY_INVENTORY_V1.json` additionally requires rights review: it contains protocol identities and excerpts/interpretations from the external registry. Its recorded `source_code_copied=false` is provenance evidence, not permission to relicense third-party expression. No Plate2bar source was accessed during I3.

All paths currently have NO_LICENSE_OWNER_CHOICE_PENDING. Public visibility of I1 grants no open-source code license. Source fractions/formulas attributed to Winter/Thelen/Hill are recalled hypotheses, not inspected source tables or licensed copied text. Establishing ownership and license scope remains RES-505 work. No actual LICENSE is added in RES-503.

Ignored `research/`, historical external corpora, participant/holdout files, sibling evidence, Plate2bar and DynamisBench were not opened, copied, executed or modified. The audit enumerates only the intended Git ancestry; the restricted-path classifier is tested to refuse reading restricted entries if they ever appear. Private local material is not inventoried by opening it.

## Verification receipt

`python -m unittest discover -s tests -p test_i3_provenance.py -v`: 3 tests pass, exit 0. Synthetic canaries verify credential-pattern detection, metadata/binary flags and restricted-path exclusion. They contain no real secret or participant data.

`python -m scripts.audit_i3_provenance`: exit 0; 200 files / 29 commits / 229 history blobs / 20 machine-path flags. A second fresh Python process wrote an identical manifest; bytes were compared with an assertion, exit 0.

`python -m unittest discover -s tests -v`: 246 tests in 331.099 s, OK, 2 skips, exit 0. This comprises the unchanged original 243 tests (241 passed, 2 optional SciPy skips on Python 3.12.10) and the 3 new RES-503 controls. Output is redirected directly to `runs/i3/res503/baseline-tests.txt`; `$LASTEXITCODE` is captured immediately, persisted and returned to the shell. No output pipe determines success.

`py -3.11 -m unittest tests.test_i1_solver_crosscheck -v`: 2 tests in 5.346 s, OK, no skips, exit 0, Python 3.11.9 / SciPy 1.17.1. This qualifies I1 integration against DOP853 using a shared derivative; it is not an independent I2 equation oracle. Full logs and exit receipts are committed; all subsequent independent scientific findings are separate RES-504 run receipts. Final acceptance is recorded in Linear with full issue commit SHA and staged inventory.

Claim ceilings: NUMERICAL_VERIFICATION_ONLY; SIMULATED_MEASUREMENT_ONLY; BIOLOGICAL_VALIDATION=NOT_ESTABLISHED; REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.
