# RES-532 admission repair (RES-525 re-audit R01-R05) — independent re-audit handoff

Authority: RES-532, owner-authorized bounded repair executed by Opus 5.5.
Repairs the four P1 groups and the P2 R05 of the sealed RES-525 independent
re-audit (local audit-only commit 57eaaf4f78c8d782c10df474fca249ba1849646e, not
modified). Base: PR #3 `work/RES-527-fail-closed-qualification` at
a6614bdf6c6e6cca5d92fe9a94d17aea3220e951, stacked on PR #2 (295f652) and PR #1
(650c797). Branch: `work/RES-532-evidence-clock-admission`. Code-qualified
commit: ea0b26d; this dossier is a documentation-only commit on top of it.

Claim ceilings are unchanged: NUMERICAL_VERIFICATION_ONLY,
SIMULATED_MEASUREMENT_ONLY, BIOLOGICAL_VALIDATION=NOT_ESTABLISHED,
REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED. No merge, tag, release or
Done transition is requested or authorized. INDEPENDENT_REAUDIT=PENDING.

## Commits (atomic, on top of a6614bd)

| Commit | Scope |
|---|---|
| 5aab3ad | `paired_source.py`: R02 pinned-root link admission, R04 interval clock fit, R05 duplicate JSON keys |
| 7d0a928 | `verify_paired_corpus.py`: R03 closed TRUTH admission, R01 mechanics-to-reference binding, pinned TRUTH/DIAGNOSTICS/run.json |
| ad2fb0a | `tests/test_res532_admission.py` (23 methods); RES-527 F03 tensile test also re-endorses the ledger |
| ea0b26d | contract documentation |

No file under `biosyngen/` changed: physics, contact, integrator, controllers,
forward sensor model, sampling constructors and the legacy 500 Hz stage are
untouched. No dependency, workflow, license or data asset was added.

## RED (unmodified a6614bd) and GREEN (ea0b26d)

The re-audit reproducers were copied byte-identically (SHA-256 verified) into a
fresh ignored root with a copy of its manufactured corpus; Linux used a
separate WSL clone at each SHA.

| Finding | Witness | RED a6614bd | GREEN ea0b26d |
|---|---|---|---|
| R01 | SJ `mechanics_submission.json` into CMJ, resealed | direct passed=true; CLI 0; full 1 (Windows and Linux) | `MECHANICS_PROTOCOL_MISMATCH`; CLI 1; full 0 |
| R02 | Windows junction at SOURCE case root | CLI 0; full 1; hook: 17 outside reads | `SOURCE_ROOT_LINK_REFUSED`; zero outside reads |
| R02 | Windows junction at DIAGNOSTICS case root | CLI 0; full 1 | `DIAGNOSTICS_ROOT_LINK_REFUSED`; CLI 1 |
| R02 | Linux symlink at SOURCE case root / TRUTH namespace | CLI 0; full 1; 9 outside reads / full 1 | `SOURCE_ROOT_LINK_REFUSED` / `TRUTH_ROOT_LINK_REFUSED`; zero outside reads |
| R03 | guide duplicate physics time | CLI 0; full 1 | `REFERENCE_TIME_GRID_INVALID` |
| R03 | guide reversed interval + compensating interval | CLI 0; full 1 | `SUPPORT_INTERVAL_INVALID` |
| R03 | CMJ one state for 3504 times | CLI 0; full 1 | `REFERENCE_STATES_MISALIGNED` |
| R03 | `context_qualification.json` = `{garbage` | CLI 0; full 1 | `TRUTH_MALFORMED_JSON` |
| R03 | empty context / empty modeled support ledger | CLI 0; full 1 | `CONTEXT_LEDGER_INVALID` / `SUPPORT_LEDGER_MALFORMED` |
| R04 | LPT edges [0,1e-300,2e-300,3e-300], bound 2e-6 s | source PASS, bounded; CLI 0; full 1 | `CLOCK_ANCHOR_SPAN_UNRESOLVED`; bounded 0 |
| R05 | reference with duplicate `protocol` key | CLI 0; full 1 | `TRUTH_DUPLICATE_JSON_KEY` |

The six original RES-525 P1 witnesses (F01/F02/F03/F04/F05/F08) still refuse
with their RES-527 codes; the CMJ and squat controls still qualify (CLI 0,
full 1). The F08 traversal still has zero outside-SOURCE reads. The new test
file run against a6614bd fails (60 failures, 14 errors), so it falsifies the
pre-repair code; it passes on ea0b26d.

## Repair contract

* **R01 BSMB-PAIRED-MECHANICS-BINDING-1.0.** Before the independent I2
  evaluator runs, the submission must carry the protocol's exercise identity
  (power-clean pull is the clean-pull builder's `CLEAN_PULL`), the reference
  grid (one frame per interval, exact t0/t1), the same body set, total mass and
  gravity, initial and per-interval states equal to the reference states,
  contacts from the reference specs with their declared support group and
  friction, a frame contact ledger equal to `support_intervals.csv`, and the
  reference impulses. Reference momentum must equal momentum recomputed from
  reference states. Both artifacts serialize one trajectory, so equality is
  exact. A coherent ledger+submission forgery binds and is then rejected by the
  evaluator (tested); binding is identity, not physical acceptance. A donor with
  byte-identical physics (e.g. same parameters, other sensor seed) is
  indistinguishable and physically equivalent; a complete forgery of every
  authority remains outside unkeyed-SHA claims.
* **R02 BSMB-PAIRED-PATH-ADMISSION-1.0.** The bundle root is pinned; the root,
  each SOURCE/TRUTH/DIAGNOSTICS namespace and each case directory must pass
  lstat/reparse-point checks before any listing or open, and the canonical path
  must equal the pinned canonical root plus the same components. Lexical
  traversal, separators, drive/UNC forms and empty names refuse. Internal links
  remain refused. A hooked subprocess monitor asserts zero outside opens and
  directory listings for root, namespace and case links on Windows (junctions)
  and Linux (symlinks), and for the clean control. Ancestors above the bundle
  root are trusted; hard links, concurrent mutation and OS sandboxing are out
  of scope.
* **R03 BSMB-PAIRED-TRUTH-ADMISSION-1.0.** Every required TRUTH file is parsed
  under a closed schema before science: finite strictly increasing grid;
  aligned finite attachment/momentum arrays; positive mass and gravity;
  in-domain nondecreasing true sample times; for modeled protocols one state
  per grid time, identical body sets, finite planar body states, modes in
  {OPEN, STICK, SLIDE} that agree with the mode series, typed specs, in-domain
  impulses. Support rows need the exact header with N units and must each be
  exactly one positive-duration physics interval, strictly ordered, with full
  left/right coverage for the guide. Model events have a closed envelope with
  the grid's time resolution. The context ledger must parse, match the
  termination and keep the three claim-ceiling refusals; its other protocol
  findings are reported as `ARCHIVAL_MODEL_OUTPUT_NOT_RECOMPUTED`. The guide
  keeps only its endpoint impulse/sign claim; thresholds are unchanged.
* **R04 BSMB-CLOCK-ADMISSION-1.0.** With capture bound e on every edge, both
  anchor spans must exceed 2e; every scale in [(R-2e)/(N+2e), (R+2e)/(N-2e)]
  must be within 1000 ppm of unit rate; offset within 3600 s; the held-out
  residual bound uses the worst-case scale; recorded samples may not
  extrapolate beyond the anchors' capture bound; the worst-case mapped-time
  error over the samples must be below half a native sample period (about
  2e-5 s on every generated case). These are verifier-owned constants, not
  device metrology. Gaussian jitter still yields the typed
  REFUSED_UNBOUNDED_SAMPLE_JITTER outcome with SOURCE PASS.
* **R05.** One duplicate-free JSON object per authority (`object_pairs_hook`),
  applied uniformly to SOURCE manifest/notes, capture authority, run receipt and
  TRUTH JSON. All generated fixtures remain valid.
* **F07** remains a documented non-qualifying P2: no persistence redesign.

## Verification on ea0b26d

| Gate | Result |
|---|---|
| Windows suite (Python 3.12.10) | 396 tests OK; 2 skips are the optional SciPy cross-checks |
| Linux suite (WSL Ubuntu, Python 3.12.3, supplementary) | 396 tests OK; same 2 skips |
| Mandatory SciPy 1.17.1 / NumPy 2.4.4 | Windows (3.11.9) 2/2 unskipped; Linux (3.12.3) 2/2 unskipped |
| 16-case two-process replay (hash seeds 0/1) | Windows PASS 188 files; Linux PASS 188 files |
| Generated bytes vs pre-repair | Windows 188/188 identical to the re-audit Windows replay; Linux 188/188 identical to the re-audit Linux replay |
| Denominators (both platforms) | source 16; model-reference 16; bounded sync 15; jitter refusal 1; full case 15; geometry PASS 12; guide N/A 4; evidence refused 0; missing 0 |
| Paired Windows/Linux comparison | PASS; 16 cases, 30 event times, max difference 4.8e-13 s |
| I1 challenge suite | 69/69 adjudicated instances agree; refuse-all and accept-all controls fail as designed |
| I0 regression, I1 replicate, I2/I3 fresh-process reproduction (12 workers), platform probe | PASS (Windows) |
| PowerShell controller controls | not run locally (pwsh 7 absent); no PowerShell file changed; hosted Windows job runs it |

The I1 challenge suite rewrites the tracked `runs/i2/res489/benchmark-suite.json`;
the fresh output was preserved locally and the tracked blob restored. Hosted
eight-job CI on the exact pushed head is recorded in the PR and Linear.

## Remaining limitations (unchanged by this repair)

Coarse 73.286 J accumulated energy remainder; 0.005833 N m s angular quadrature
residual; 2/10 stress-task completions; 35 rejected recovery pairs; unmodeled
catches, hand stabilization and 3D motion; no empirical human, device or paired
validity. Manufactured capture receipts and unkeyed hashes provide no
cryptographic authentication. F07 persistent storage failure can still leave no
run receipt (never a qualification). Linux evidence is local Python 3.12.3
(hosted CI pins 3.12.10). This repair has not been independently re-audited;
it does not self-approve.
