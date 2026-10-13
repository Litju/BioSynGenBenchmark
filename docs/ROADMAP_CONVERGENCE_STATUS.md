# Roadmap convergence status (RES-488 one-run campaign)

The single durable checkpoint for the owner-authorized Opus 5.5 full-roadmap campaign. Update it at each reviewable work item. Linear is authoritative for issue state; this file records local Git and verification facts so a fresh session can resume without repeating work.

Last update: 2026-10-13 (Opus 5.5 session 1)

## Git state

| Item | Value |
|---|---|
| Base for this campaign | `eade435c427e7f12349ad370168c0984b6836979` (RES-490) |
| RES-491 working branch | `codex/res-491-bilateral-multibody-foundation`. RES-491 files are untracked in the worktree and never staged on other branches. |
| R1 research and checkpoint branch | `claude/res-488-roadmap-r1` (local only; the repository license is pending, so there is no push or PR) |
| Public `main` | `6115d702…` (I1); nothing in this campaign merges to `main` |

## Issue matrix

| Issue | State | Commit(s) | Verification | Next action |
|---|---|---|---|---|
| RES-489 | In Review (prior) | `895c8e6`, `913f01e`, `4cf4f6a` | prior session | owner review |
| RES-490 | In Review (prior) | `eade435` | prior session | owner review |
| RES-491 | **In Progress — BLOCKED_AFTER_ONE_EXCEPTION** | none (3 untracked files) | focused 26: 25 pass / 1 fail; full 147: 144 pass / 1 fail / 2 skip | owner: authorize or decline one stick-capture repair |
| RES-492..498 | Todo — dependency-blocked on RES-491 | — | — | start only after RES-491 is qualified |
| RES-455..462 | in progress this session | see below | research | one issue-scoped commit each |
| RES-463 | Todo | — | — | brief for owner approval after RES-455..462 |
| RES-464 | Backlog | — | — | `WAITING_OWNER_APPROVAL_FOR_ONE_DEEP_RESEARCH` |
| RES-465/466/467 | Backlog | — | — | provisional drafts only |

## RES-491 blocker (exact)

The authorized inertia-contract repair is done: a typed `ParameterDomainError` with `field`/`reason` codes, and the full domain is tested. Reviewer falsifiers then exposed a missing slide→stick capture: a sliding contact that should come to rest chatters in SLIDE with |v_t| ≈ μ·g·dt. The falsifier is `tests/test_i2c_multibody.py::IndependentReviewerFalsifierTests::test_sliding_contact_captures_stick_after_slip_reverses`. Reproduce with:

    git switch codex/res-491-bilateral-multibody-foundation
    python -m unittest discover -s tests -p test_i2c_multibody.py -v

## Claim ceilings

NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY, BIOLOGICAL_VALIDATION=NOT_ESTABLISHED, REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED. PLATE2BAR_MUTATIONS=0.
