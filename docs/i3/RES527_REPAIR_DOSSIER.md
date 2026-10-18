# RES-527 fail-closed paired qualification repair — independent re-audit handoff

Authority: RES-527 (owner-selected executor Opus 5.5), repairing the six P1
findings and F06 of the sealed RES-525 audit (local commit
0fa92457d349eeea8596259201ad433b59c58ab4, not modified). Audited base: PR #2
`codex/res-524-dual-1khz` at 295f652018f36070481202e2e3ab1c3ae458f5c2, stacked
on PR #1 `codex/i3-public-rc` at 650c797b87bbd8bcb9a5739c40ecc2552f487bb3.
Repair branch: `work/RES-527-fail-closed-qualification` (the issue text names
`work/RES-526-...`; RES-526 is a different issue, so the branch carries this
issue's identifier). Code-qualified commit: 2e5b969eb58892933d3e250570d83a0f5ed48c39;
this dossier is a documentation-only commit on top of it.

Claim ceilings are unchanged: NUMERICAL_VERIFICATION_ONLY,
SIMULATED_MEASUREMENT_ONLY, BIOLOGICAL_VALIDATION=NOT_ESTABLISHED,
REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED. No merge, tag, release or
Done transition is requested or authorized. INDEPENDENT_REAUDIT=PENDING.

## Commits (atomic, on top of 295f652)

| Commit | Scope |
|---|---|
| dbf0498 | `paired_source.py`: F08 path containment, F04 finite versioned capture, F02 source-side taut stage |
| 57c3273 | `verify_paired_corpus.py`, comparator, replay: F01, F02, F03, F05 admission; F06 report 2.0 |
| 3ee6557 | `generate_paired_corpus.py`: F07 OS write failure receipted as REFUSED |
| 776df34 | `tests/test_res527_fail_closed.py`: 25 adversarial/positive regressions |
| b460953 | contract documentation |
| 2e5b969 | OS write failures excluded from expected-refusal counter |

Changed files only: `biosyngen_benchmark/paired_source.py`,
`scripts/{verify_paired_corpus,compare_paired_platforms,reproduce_paired_corpus,generate_paired_corpus}.py`,
`tests/{test_paired_corpus,test_res527_fail_closed}.py`, `docs/i3/*`. No
physics, controller, sensor forward model, calibration, sampling-rate, legacy
500 Hz constructor, Plate2bar, rights or corpus-byte change.

## RED (unmodified 295f652) and GREEN (2e5b969) witnesses

The sealed RES-525 reproducers were copied byte-identically into a fresh
project-local root; a fresh control corpus was generated from 295f652 first.

| Finding | Original witness | RED on 295f652 | GREEN on 2e5b969 |
|---|---|---|---|
| F01 | CMJ and squat `missing-events`, coherently resealed | direct passed=true, CLI exit 0, qualified_count=1 | direct `TRUTH_INVENTORY_INCOMPLETE`; CLI exit 1, model-reference 0, missing-evidence 1 |
| F02 | `missing-derived` (path/refusal null, re-endorsed) | direct passed=true, source PASS, CLI exit 0 | source and reference each `TAUT_DERIVED_OBSERVATION_MISSING`; CLI exit 1 |
| F02 | `empty-derived` direct helper | geometry PASS, samples=0 | reference `DERIVED_SAMPLES_INSUFFICIENT`; source `INSUFFICIENT_SAMPLES` |
| F03 | balanced tensile support, no mechanics (min fz -99799.40943181817 N) | direct passed=true, CLI exit 0 | `TRUTH_INVENTORY_INCOMPLETE` direct and CLI exit 1 (same -99799.409 N ledger) |
| F04 | capture bound +Infinity, LPT edge +0.1 s | source PASS, BOUNDED, CLI exit 0 | source `NONFINITE_JSON_NUMBER`; CLI exit 1 |
| F05 | 95 kg guide SOURCE + 120 kg guide TRUTH | direct passed=true, CLI exit 0 | `TRUTH_SOURCE_BINDING_MISMATCH` direct and CLI exit 1; source still PASS |
| F08 | derived path `../../TRUTH/<case>/audit-unsealed-vertical.csv` | source PASS with out-of-SOURCE TRUTH read | `DERIVED_PATH_UNSUPPORTED`; zero out-of-SOURCE read attempts |

Controls (CMJ, squat) still qualify: exit 0, model-reference 1, full case 1.
The original harnesses for F03/F05/F08 assume no exception and now terminate on
the typed refusal; their unmodified outputs are retained, and a companion
harness replays the same mutations verbatim with typed-decision capture. For
the F04 witness the reference-only helper still reports passed=true because
its TRUTH is valid for the re-endorsed SOURCE; source admission refuses and the
full verifier never qualifies the case.

## Repair contract

* **F01** BSMB-PAIRED-TRUTH-INVENTORY-1.0 (closed, checksummed, link-free,
  flat): the guide is exactly `reference.json`, `support_intervals.csv`;
  modeled protocols add `model_events.json`, `mechanics_submission.json`,
  `context_qualification.json`. Each modeled protocol needs its full event set
  (jumps takeoff/touchdown[/COM reversal], floor lift-off/return, per-rep
  descent/bottom/threshold completion). The guide exception requires the
  P2B open profile, `guided_vertical` and the guide world, and finite ordered
  analytical events.
* **F02** Declared stage selects the inventory. Taut cable recordings must
  seal `derived/vertical.csv` with role DERIVED_OBSERVATION and >= 3 samples
  covering every LPT sample; only slack carries the null/SLACK refusal,
  reported as NOT_EVALUABLE_SLACK_CABLE_REFUSED (never full case). Source and
  reference checks enforce this independently. No COM promotion.
* **F03** Modeled cases require evaluator `LINEAR_MOMENTUM_CLOSURE` and
  `CONTACT_ADMISSIBILITY` (missing finding refuses). The endpoint ledger is
  guide-only and also checks unilateral support sign at the evaluator's 1e-6 W
  tolerance.
* **F04** BSMB-FOUR-EDGE-CAPTURE-1.0: four finite strictly increasing edges
  per clock and reference, fit [0,3], held-out [1,2], bound in (0, 2e-6] s (the
  contract's modeled envelope, not device metrology), finite positive fitted
  scale/offset, finite declared mappings, valid jitter policy; strict JSON.
  Manufactured receipts remain non-cryptographic.
* **F05** `reference.source_manifest_sha256` must equal the current SOURCE
  manifest digest; trial/acquisition/bundle IDs must equal the TRUTH case;
  schema and protocol must match; the run receipt's manifest and TRUTH
  inventory digests must match. Unkeyed SHA does not resist simultaneous
  forgery of every authority.
* **F08** Derived path exactly `derived/vertical.csv` or null, checked as a
  string before filesystem use; checksum names are plain POSIX relative names;
  every opened SOURCE file is resolved inside SOURCE with no symlink/reparse
  component; directory walk refuses links; run-receipt case IDs are validated.
  Not an OS sandbox.
* **F06** BSMB-PAIRED-QUALIFICATION-REPORT-2.0 per-case
  `source_integrity`, `model_reference_status`, `synchronization`,
  `geometry.status`, `full_case_status`, `expected_refusals`, typed
  `evidence_refusal`; counts as listed below. `qualified_count` is removed. The
  comparator refuses pre-2.0 receipts rather than reinterpreting them; old
  receipts are untouched. CLI exit 0 means every requested case is
  model-reference qualified, not full-case readiness.
* **F07** (P2) Narrow remedy: generator OS write failures become a REFUSED case
  (`OS_WRITE_FAILURE:...`) with retained partial files listed in `run.json`,
  exit 1; the verifier never qualifies it and does not count it as expected.
  No staging/persistence redesign; recovery remains a fresh output directory.

## Verification on 2e5b969

| Gate | Result |
|---|---|
| Windows suite (Python 3.12.10) | 373 tests OK; 2 skips are the SciPy-only cross-checks |
| Linux suite (WSL Ubuntu, Python 3.12.3) | 373 tests OK; same 2 SciPy-only skips |
| Mandatory SciPy 1.17.1 / NumPy 2.4.4 | Windows (3.11.9) 2/2; Linux 2/2 |
| 16-case SOURCE/TRUTH replay, hash seeds 0/1 | Windows PASS 188 files; Linux PASS 188 files |
| Generated bytes vs pre-repair 295f652 | Windows 188/188 SHA-identical; Linux 188/188 identical to the audit's Linux replay |
| Denominators (both platforms) | requested 16; source integrity 16; model-reference 16; bounded sync 15; sync refused 1; geometry PASS 12; guide N/A 4; not evaluable 0; full case 15; expected refusal 1; evidence refused 0; missing evidence 0 |
| Paired Windows/Linux comparison | PASS; max event-time difference 4.8e-13 s |
| Source-only truth-denial guard | PASS; no TRUTH read, no generator import, 4200 geometry samples, max cosine error 0 |
| I1 challenge suite, I0 regression, I1 replicate, I3 fresh-process reproduction, platform probe | PASS (Windows) |
| PowerShell controller controls | not run locally: the test requires pwsh 7, which is absent; no PowerShell file changed; hosted Windows job runs it |

Hosted Windows/Linux CI on the exact pushed HEAD is recorded in the PR and Linear.

## Remaining limitations (unchanged by this repair)

Coarse 73.286 J accumulated energy remainder; 0.005833 N m s angular quadrature
residual; 2/10 recovery completions; 35 rejected recovery pairs; unmodeled
catches, hand stabilization and 3D motion; no empirical human, device or
paired validity. Manufactured capture receipts and unkeyed hashes provide no
cryptographic authentication. Local Linux used Python 3.12.3 (hosted CI uses
3.12.10). Pre-existing local ref `codex/i3-public-rc` (633a14a) differs from
the remote PR #1 head 650c797; neither was touched.
