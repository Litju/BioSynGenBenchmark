# Roadmap convergence status (I3 current; I2 historical checkpoint)

Current authority: GPT-6.1 Solo, RES-502, [binding I3 contract](https://linear.app/alignerr-cmj/document/i3-gpt-61-solo-open-source-and-independent-vandv-execution-contract-adf82b988204). I3 independently qualified preserved local I2 HEAD `3322d89499050eea0004f9e8882666e2a40d5a9a`; main remains `6115d702ba3ab814982353945b0753f4846aae88`. The Apache-2.0 public candidate is `codex/i3-public-rc`, [draft PR #1](https://github.com/Litju/BioSynGenBenchmark/pull/1). [Actual hosted run 38506388914](https://github.com/Litju/BioSynGenBenchmark/actions/runs/38506388914) passes all five jobs at numerical source `57484146cd12859dad34a004d21606cfb67bcd5f`. The exact audit, commits, evidence and per-I2 limitations are in [RES508_OWNER_DOSSIER.md](i3/RES508_OWNER_DOSSIER.md).

I3 roadmap: RES-503 source/history inventory → RES-504 independent V&V → RES-507 artifact qualification; RES-505 governance follows RES-503; RES-506 publication/hosted CI follows RES-505 and RES-507; RES-508 consolidates qualification after RES-506. RES-503/504/505/506/507 are In Review; RES-508 prepares the final owner handoff. All RES-489–498 remain In Review. Parent RES-502 awaits owner acceptance. No historical issue is moved Done, and main merge/release/tag remain owner-only. Full audit status and evidence are in `docs/i3/` and `runs/i3/`.

The material below is the preserved Opus checkpoint, including its reported counts and historical deviations. Its old "Next executable tasks" section is superseded by the I3 roadmap above.

The single durable checkpoint for the owner-authorized Opus 5.5 full-roadmap campaign. Linear is authoritative for issue state. This file records local Git and verification facts so a fresh session can resume without repeating work.

Last update: 2026-10-13, Opus 5.5 session 2. The I2 implementation chain RES-491 to RES-498 is in In Review.

**CAMPAIGN_RESULT=I2_IMPLEMENTATION_IN_REVIEW_WITH_SCOPED_REFUSALS**

## Git state

| Item | Value |
|---|---|
| Working branch | `claude/res-488-roadmap-r1`. Local only: the license decision is pending, so nothing has been pushed and no PR exists. |
| Base | `eade435` (RES-490) |
| Public `main` | `6115d702…` (I1). Nothing has been merged. |
| Full suite (Windows, CPython 3.12.10) | 243 run, OK, 2 optional SciPy skips (`python -m unittest discover -s tests`, exit 0) |
| `research/` | Git-ignored by project policy (rights). R1 deliverables are identified by SHA-256 in the Linear receipts. |

## Issue matrix

| Issue | State | Commits | Verification |
|---|---|---|---|
| RES-489, RES-490 | In Review (earlier sessions) | `895c8e6`, `913f01e`, `4cf4f6a`; `eade435` | earlier sessions |
| RES-491 | In Review | `b5b4c29` | engine 35 + extensions; contact-law iterations used: 3 of 4 (`19497a7`) |
| RES-492 | In Review | `2e71f21` `a80b35f` `f17c7b1` `e8565d9` `19497a7` `bad7a74` `427c68f` | test_i2d_jumps 15; runs/i2/res492 |
| RES-493 | In Review | `2afd084` | test_i2e_back_squat 11; runs/i2/res493 |
| RES-494 | In Review | `f0c7ee7` `245501d` | test_i2f_* 13; runs/i2/res494 |
| RES-495 | In Review | `cae12d8` `6fe4c2e` `1e6c04b` `d156eec` | test_i2g_bench_press 10; runs/i2/res495 |
| RES-496 | In Review | `9c8dfd5` | test_i2h_olympic_pulls 9; runs/i2/res496 |
| RES-497 | In Review (scoped subset) | `981248d` `229a279` | test_i2i_power_clean 5. Catch NOT_IMPLEMENTED (computed). `981248d` was committed with a failing test; fixed by `229a279`. |
| RES-498 | In Review | `dc11f0c` `eb720e8` | test_i2j_benchmark 6; runs/i2/res498; CI step added but not run remotely |
| RES-455..463 | In Review (R1) | research artifacts | owner |
| RES-464 | Backlog | Awaiting owner approval of the Deep Research brief | not launched |
| RES-465/466/467 | Backlog | Provisional drafts | owner, after RES-464 |

## Next executable tasks

There is no autonomous implementation item left in the authorized queue. The open work items:

1. Owner review of RES-491 to RES-498, then a license decision so the branch can be pushed and CI run remotely.
2. Optional follow-ups (would need new owner scope):
   - a wrist/hand segment for a real power-clean catch;
   - compliant foot contact for drop/rebound jumps and evaluable landing peaks;
   - more robust pull strategies;
   - removing the lockout-hold PD ringing.
3. RES-464 Deep Research needs owner approval of its exact brief.

## Reproduce

    python -m unittest discover -s tests
    python -m scripts.check_i2_reproducibility
    python -m scripts.run_i2_benchmark --out runs/i2/res498

## Claim ceilings

NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY, BIOLOGICAL_VALIDATION=NOT_ESTABLISHED, REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED. PLATE2BAR_MUTATIONS=0. RES-464 not launched.
