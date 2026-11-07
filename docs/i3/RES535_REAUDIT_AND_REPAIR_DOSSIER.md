# RES-535 independent re-audit of PR #4 and same-session repair

Authority: RES-535 (owner-authorized fresh Opus 5.5 re-audit of the RES-532
repair with bounded same-session repairs), parent RES-525. Audited candidate:
PR #4 `work/RES-532-evidence-clock-admission` at
4fb0a754f24fdfa24d4cfcaac60a9235885a1530, stacked on PR #3 (a6614bd), PR #2
(295f652) and PR #1 (650c797) over main (6115d70). Repair branch:
`work/RES-535-adversarial-admission-repair` (the issue text names
`work/RES-533-...`; RES-533 is an unrelated showcase issue, so the branch carries
the executing issue's identifier, as RES-527 did).

Claim ceilings are unchanged: NUMERICAL_VERIFICATION_ONLY,
SIMULATED_MEASUREMENT_ONLY, BIOLOGICAL_VALIDATION=NOT_ESTABLISHED,
REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED. No merge, tag, release or Done
transition is requested or authorized. The audit of the unchanged PR #4 is an
independent result of this fresh session; every GREEN below is implementer
verification of this session's own repair and requires an independent
re-audit (INDEPENDENT_REAUDIT_OF_OWN_REPAIR=PENDING).

## Preflight

Live heads matched the authorization (main 6115d70, PR #1 650c797, PR #2
295f652, PR #3 a6614bd, PR #4 4fb0a75; local `codex/i3-public-rc` is a stale
ancestor of the remote and was not used). All 478 hashes in the sealed RES-525
re-audit `EVIDENCE_HASHES.json` verified; the sealed report commit 57eaaf4 is
clean. Audit, witness and repair worktrees are separate and ignored; no sealed
evidence or PR head was modified.

## Historical witnesses (sealed reproducers copied byte-identically)

| Witness set | 295f652 (PR #2) | a6614bd (PR #3) | 4fb0a75 (PR #4) |
|---|---|---|---|
| F01 missing events, F02 missing derived, F03 rebalanced/tensile fallback, F04 nonfinite capture, F05 TRUTH swap, F08 derived escape | all false-pass (CLI 0) | refuse | refuse |
| R01 SJ mechanics into CMJ, R02 SOURCE/DIAGNOSTICS junctions, R03 duplicate time / reversed interval / empty context / empty ledger / malformed context / truncated states, R04 1e-300 edges, R05 duplicate key | n/a | false-pass (full 1) | refuse with typed codes |
| CMJ and squat controls | qualify | qualify | qualify |

F06 denominators at PR #4: 16 source, 16 model reference, 15 bounded sync,
1 jitter refusal, 15 full, 12 geometry, 4 guide. F07 persistent write failure
still leaves no run receipt and the verifier refuses fail-closed (never a
qualification): retained documented P2.

## New findings at 4fb0a75 (auditor-derived attacks, coherently resealed)

| ID | Severity | Source (4fb0a75) | Independent oracle | Observed at 4fb0a75 | Disposition |
|---|---|---|---|---|---|
| RA-01 | P1 | `scripts/verify_paired_corpus.py:289-293` (`bind_mechanics` checks contact x/z only for finiteness), `:299-302` (impulse points copied from an unchecked reference), `:203-206` (contact specs not geometrically admitted) | rigid transform p = r_A + R(theta_A) local_point_a of the reference body state at interval start (impulses: state at the impulse time); 0.0 m error on all 15 genuine modeled cases | foot contact moved 0.75 m in x or 1.5 m in z (CMJ, SJ, squat, deadlift), one frame moved to x=1000 m, impulse point moved 2 m (CMJ, SJ, deadlift), spec local point moved 0.5 m, floor normal flipped to [0,-1]: all CLI 0, full 1 | FIXED: mechanics binding 1.1 |
| RA-02 | P1 | `biosyngen_benchmark/paired_source.py:305-309` (sync outcome taken from the declared jitter policy); `verify_case` never compared the mapping with TRUTH | index-aligned protected TRUTH sample times; genuine max mapped-minus-true <= 2.0e-6 s, jittered recording ~2e-5 s | jittered guide re-declared with jitter bound 0 or 1 ns, and a capture-consistent LPT map shifted 0.4 ms: BOUNDED_CAPTURE_MAPPING, full 1 (would turn the 1 jitter refusal into a 16th full case) | FIXED: BSMB-SYNC-TRUTH-BINDING-1.0 |
| RA-03 | P2 | `scripts/verify_paired_corpus.py:266-273` (inertia only checked > 0; masses only by total and vertical momentum) | none in TRUTH (reference states carry no per-body mass or inertia) | inertia x10 / x0.01 and 3 kg moved left_thigh -> right_thigh accepted; 5 kg moved between non-mirrored bodies refused (REFERENCE_MOMENTUM_INCONSISTENT) | RETAINED_LIMITATION_OUTSIDE_CLAIM, now reported explicitly; no qualified claim uses these values |
| RA-04 | P2 | `scripts/verify_paired_corpus.py:339-342`, `:249-251` | generator's fixed claim-scope constants | `event_authority` elevated to an independent production sensor detector, `anatomical_events_from_sensors` set to ESTABLISHED/true, elevated `supported_context`: full 1 (the report's own scope strings were not promoted) | FIXED: TRUTH admission 1.1 |
| RA-05 | P2 | reference `physical_events` never admitted | simulator's closed event vocabulary | cleared or malformed simulator event log: full 1 | FIXED structurally; log reported archival (clearing remains accepted and is labelled) |

Checked without a defect: offsetting non-mirrored mass transfers refuse;
coordinated zero-force row omission is vacuous (generated ledgers contain no
zero-force rows) and ledger rows have no exact one-to-one rule with contact
modes, nor do simulator events with mode transitions, so no closed rule was
invented; R02 namespace, case and bundle-root junctions (Windows) and symlinks
(Linux) for SOURCE/TRUTH/DIAGNOSTICS, internal SOURCE/TRUTH directory links and
trailing-dot identities refuse with zero outside opens or listings under an
audit hook; R05 nested duplicate keys in the capture authority and operator
notes refuse; a 50 ms clock shift refuses (CLOCK_SAMPLES_OUTSIDE_FIT_WINDOW).
Observations (P3, no change): an internal DIAGNOSTICS directory link is ignored
with zero outside reads (DIAGNOSTICS is not a closed inventory); a missing
run.json exits through an uncaught typed SourceRefused; PR #1 adds two
trailing-whitespace lines in `biosyngen_benchmark/i1.py`.

## Repair commits (on 4fb0a75)

| Commit | Scope |
|---|---|
| de70b1d | BSMB-SYNC-TRUTH-BINDING-1.0 (`paired_source.py` exposes admitted maps; `verify_case` checks TRUTH sample times) |
| 00daad6 | mechanics binding 1.1: contact-spec schema, application and impulse points, unbound inertia/mirrored-mass labels |
| 4d173cf | TRUTH admission 1.1: closed event/context authority, physical event log structure |
| 40f5636 | declared-normal check moved after the independent evaluator; RES-532 contract assertion follows the 1.1 bump |
| 4ebd5e3 | `tests/test_res535_adversarial_admission.py` (12 methods) |
| 1da3e48 | contract documentation |

No file under `biosyngen/` changed. No dependency, workflow, license or data
asset was added. The new test file fails on 4fb0a75 (27 failures, 6 errors)
and passes on the repair.

## Qualification of the code head 4ebd5e3 (implementer verification)

| Gate | Result |
|---|---|
| Windows suite (Python 3.12.10) | 408 tests OK; 2 skips are the optional SciPy cross-checks (PR #4 baseline: 396 OK) |
| Linux suite (WSL Ubuntu, Python 3.12.3, supplementary) | 408 tests OK; same 2 skips |
| Mandatory SciPy 1.17.1 / NumPy 2.4.4 | Windows (3.11.9) 2/2 unskipped; Linux (3.12.3) 2/2 unskipped |
| I1 challenge suite | 69/69 adjudicated instances agree; refuse-all (16) and accept-all (20) controls fail as designed |
| I0 regression, I1 replicate, I2/I3 fresh-process reproduction (12 workers), platform probe | PASS (Windows); I0 differences are the two documented historical ones |
| 16-case two-process replay (hash seeds 0/1) | Windows PASS 188 files; Linux PASS 188 files |
| Generated bytes vs pre-repair (same OS) | Windows 188/188 and Linux 188/188 identical to the PR #4 replay on that OS |
| Denominators (both platforms) | source 16; model reference 16; bounded sync 15; jitter refusal 1; full 15; geometry 12; guide 4; evidence refused 0; missing 0 |
| Paired Windows/Linux comparison | PASS; 16 cases, 30 event times, max difference 4.8e-13 s |
| Auditor harnesses on the repair (Windows and Linux) | every RA-01/RA-02/RA-04/RA-05 attack refuses; accepted mutations are only the disclosed RA-03 inertia/mirrored-mass values, the labelled cleared event log, the vacuous zero-row omission and the ignored DIAGNOSTICS internal link; all positive controls qualify and the jitter control keeps its typed refusal |
| PowerShell controller controls | not run locally (pwsh 7 absent); no PowerShell file changed; the hosted Windows job runs it |

The Linux run was interrupted once by a full host disk (WSL I/O errors); only
auditor-generated corpora were removed, sealed RES-525 evidence re-verified
478/478, and the two affected harness runs were repeated to completion.

## Disposable merge rehearsal

Detached worktree from main 6115d70, `git merge --no-ff --no-edit` of
650c797, 295f652, a6614bd, 4fb0a75 and the repair head in order: no conflicts,
and every merge result's tree equals the merged PR head's tree (repair code head
merge tree ea1f22f equals the repair tree), so the integrated result is exactly
the qualified source. No protected or review ref was written. New merge commit
identities are disposable and are not a merge authorization.

## Remaining limitations (unchanged)

Coarse 73.286 J accumulated energy remainder; 0.005833 N m s angular quadrature
residual; 2/10 stress-task completions; 35 rejected recovery-controller pairs;
unmodeled catches, hand stabilization and 3D motion; model reference events are
sampled physical references, not production sensor-detected events; no
empirical human, device or paired validity; manufactured capture receipts and
unkeyed hashes are not cryptographic authentication; per-body inertia and
mirrored mass splits are not independently bound; F07 remains a documented
non-qualifying P2. A generated corpus does not qualify Plate2bar's ingest or
coupling pipeline.
