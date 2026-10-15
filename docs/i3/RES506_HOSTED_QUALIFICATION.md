# RES-506 actual hosted qualification

The public draft [PR #1](https://github.com/Litju/BioSynGenBenchmark/pull/1) preserves the cleared I2/I3 history on `codex/i3-public-rc`. All five jobs in [run 38506388914](https://github.com/Litju/BioSynGenBenchmark/actions/runs/38506388914) completed successfully at exact source `57484146cd12859dad34a004d21606cfb67bcd5f`. Main remains `6115d702ba3ab814982353945b0753f4846aae88`. There is no merge, release tag or scientific Done transition.

| Actual hosted gate | Result |
| --- | --- |
| Windows CPython 3.12.10 full suite | 289 tests, 287 pass, 2 optional SciPy skips; 401.941 s, true command success |
| Linux CPython 3.12.10 full suite | 289 tests, 287 pass, 2 optional SciPy skips; 275.914 s, true command success |
| Windows controller controls | 23 assertions pass; fake workers only |
| Required SciPy on Windows/Linux | Each: exactly 2 passing, unskipped DOP853 checks; CPython 3.11.9, SciPy 1.17.1, NumPy 2.4.4 |
| I1 challenge and two fresh processes, each platform | 69/69 adjudicated claim agreements; complete replica file bytes identical; reject-all/accept-all controls fail their gates |
| I2 reproduction, each platform | 12 fresh worker processes, all exits 0; 6 complete artifact inventory pairs identical; both benchmark replicas meet 10/10 positives and 76/76 faults |
| Exact-reference Windows/Linux comparison | 8/8 fixed analytical budgets pass; actual cross-platform float differences are zero in these fixtures |

Windows used image `windows-2025-vs2026`, version `20260925.250.1`; Linux used `ubuntu-24.04`, version `20261004.327.1`. Complete job logs record operating systems, runner/toolchain provenance, commands and results. `latest` runner images remain mutable; GitHub reports a forthcoming Ubuntu-label migration and pinned older actions running under Node 24. These warnings are disclosed, not presented as a fully immutable operating-system environment.

## Independent inspection of uploaded evidence

Downloaded hosted outputs are retained under `runs/i3/res506/hosted-38506388914`. `downloaded-evidence-inspection.json` records independently rehashed complete output inventories, all worker exits, both 10/76 denominator gates per platform, I1 decisions and recomputed exact-rational errors. Candidate-reported tolerances/errors are not trusted. The locally recomputed eight rows agree with the hosted comparator artifact. Triangle bounds apply only to the authored support/capture/restitution fixtures; no universal exercise-integration error bound follows.

I0 comparison was inspected beyond its CLI exit: numerical pilot data, evaluator input/output, figure content and mechanics/observation values remain unchanged. There are three logical source-hash changes and one previously documented finding-reason change. Windows additionally converts checkout line endings: frozen artifact byte hashes differ, and `observations.py` adds a fourth manifest source-hash path solely through LF/CRLF conversion. Exact Git EOL variants explain these hashes; regenerated nonmanifest pilot content matches canonical Git LF bytes. This correction is independently documented in `runs/i3/res508/FINAL_TIP_CI_INSPECTION.json`. Every numerical criterion remains unchanged; historical byte identity is not asserted across checkout conventions.

The Linux jump-family SVG was rasterized with the bundled Sharp runtime and visually inspected. Its SHA-256 is `c3ea40d8bb60dbff496b21a4db41990b927f279583810fa8dac2c277a2df8c14`; the local ignored preview is `30f704dc29c09ed5f5ccc79b29538a32817410ad2910ecd91643a403344fefa9`. Full plotted ranges and the rigid/filter peak refusal are visible. The committed SVG and earlier raw Matplotlib figures provide reviewable source; the optional local preview is not a new runtime dependency.

New console captures contain CRLF/trailing display whitespace. Only presentation whitespace was normalized for Git checks, with original/intermediate/final hashes and nonwhitespace payload equivalence recorded in `log-line-endings.json`. Generated hosted artifacts retain their original bytes and match their process receipts. Previously committed historical failure logs are unchanged.

## Preserved failure, diagnostic and repair history

The first PR run `38502977541` at `5434294a09d27b8e4ff1732be0748ca18a0d7082` had a genuine Linux suite failure (287 tests, one failure, two skips): only 6/7 alleged positive I1 work-ledger cases passed. Both SciPy jobs passed; Windows had not completed all gates when the diagnostic push cancelled that run. The overall run's actual conclusion is **cancelled**, with its Linux job **failure**, not success.

Run `38504694099` at `e7904453fd0b574c0b53e100df6e20ff350ea4f0` completed the scoped Linux diagnostic but was cancelled by the repair push; it is not a qualified run. Duplicate push runs were cancelled by the shared push/PR concurrency group. Exact IDs, events, SHAs and conclusions are in `hosted-run-history.json`.

One bounded repair iteration exposed and fixed the sampled-energy endpoint blind spot. Every admissible 21-frame start is now checked without changing tolerances or event exclusions. A one-sample analytical 1 J hidden storage change now rejects. CMJ/loaded CMJ/SJ generated records remain present and require REJECTED for WORK_ENERGY_LEDGER; four independent analytical positives and four negatives all match adjudication. The corrected 4/4-positive, 4/4-negative counts are not silently reported as the old 7/7 claim. See `RES506_LINUX_ENERGY_FINDING.md`. I2's separately scoped 10/76 decisions are unaffected.

Local repaired full suite: 289 tests in 312.877 s, 287 pass, two optional SciPy skips, exit 0. Required installed SciPy checks also pass; the no-dependency interpreter fails closed as intended. All red controls and original diagnostic logs remain inspectable.

## Publication boundary

The owner-selected Apache-2.0 and personal-research declaration are recorded verbatim in `RES505_OWNER_DECISION.json`; NOTICE identifies Julio Rodriguez and owned distribution scope. External source/data, private research, participant records, proprietary exports, holdouts, sibling evidence and local app/tool environments are excluded. The license legal text is the standard Apache document, not newly claimed project authorship.

The full repaired-source inventory gate passes: 531 files, 45 commits, 477 distinct blobs; 40 canonical/generic machine-path findings and 17 manually reviewed manufactured PNGs. No restricted path, credential-pattern finding, unknown binary or blob above 5 MB appears in that audit. Five unauthenticated downloads from the immutable repaired source match local hashes. New evidence is separately inspected/scanned before staging and publication. These are bounded content checks and owner-declared rights, not a legal title certificate or universal privacy detector.

This qualifies a scoped numerical research prototype for owner review. It does not establish ordinary hand-stabilized jump squat, whole-record stable bench technique, compliant landing force/RFD, robust general Olympic control, human or device fidelity. All original catch/drop/rebound/hang/block refusals persist. NUMERICAL_VERIFICATION_ONLY; SIMULATED_MEASUREMENT_ONLY; BIOLOGICAL_VALIDATION=NOT_ESTABLISHED; REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.
