# Roadmap convergence status (RES-488 one-run campaign)

The single durable checkpoint for the owner-authorized Opus 5.5 full-roadmap campaign. Linear is authoritative for issue state; this file records local Git and verification facts so a fresh session can resume without repeating work.

Last update: 2026-10-13, Opus 5.5 session 2 (RES-491 resolved, RES-492 In Review, RES-493 next).

**CAMPAIGN_RESULT=IN_PROGRESS**

The RES-491 stick–slip blocker is resolved (In Review). RES-492 jump family is In Review. Next executable task: RES-493 (back squat and multi-repetition). R1 research is complete to In Review. R2, R3 and R4 are owner-gated.

## Git state

| Item | Value |
|---|---|
| Base | `eade435c427e7f12349ad370168c0984b6836979` (RES-490) |
| Working branch | `claude/res-488-roadmap-r1` (local only). RES-491 `b5b4c29`; RES-492 `2e71f21`…`427c68f` (7 commits). |
| Checkpoint branch | `claude/res-488-roadmap-r1` (local only; license pending, so no push or PR) |
| Public `main` | `6115d702…` (I1); nothing merged |
| `research/` | git-ignored by project policy (rights). R1 deliverables live there and are identified by SHA-256 in each Linear receipt. |

## Issue matrix

| Issue | State | Commit / identity | Verification | Next action |
|---|---|---|---|---|
| RES-489 | In Review (prior) | `895c8e6`, `913f01e`, `4cf4f6a` | prior session | owner review |
| RES-490 | In Review (prior) | `eade435` | prior session | owner review |
| RES-491 | In Review | `b5b4c29` (+ contact-law iterations 2–3 in `19497a7`, `bad7a74`) | focused 35/35; engine extensions 18; full 185 OK (2 skips) | owner review; contact budget 3 of 4 used |
| RES-492 | In Review | `2e71f21`, `a80b35f`, `f17c7b1`, `e8565d9`, `19497a7`, `bad7a74`, `427c68f` | `tests.test_i2d_jumps` 15/15; full 185 OK (2 skips); runs/i2/res492 byte-reproducible | owner review |
| RES-493 | Todo — **next** | — | — | back squat + multi-repetition on `biosyngen/i2` |
| RES-494..498 | Todo | — | — | dependency order |
| RES-455 | In Review | `research/RES-455-…` `1a03e328…` | research | owner |
| RES-456 | In Review | `ff4f7496…` | research | owner |
| RES-457 | In Review | `69884a5a…` | research | owner |
| RES-458 | In Review | `231b054d…` | research | owner |
| RES-459 | In Review | `97f8b00c…` | research | owner |
| RES-460 | In Review | `6423385b…` | research | owner |
| RES-461 | In Review | `7b499a94…` | research | owner |
| RES-462 | In Review | `57f314e8…` | research | owner |
| RES-463 | In Review — brief posted as a Linear document | `9df753a3…` | — | **owner:** approve, edit or reject the exact prompt |
| RES-464 | Backlog — `WAITING_OWNER_APPROVAL_FOR_ONE_DEEP_RESEARCH` | — | not launched | owner approval |
| RES-465/466/467 | Backlog — `PROVISIONAL_NOT_SCIENTIFICALLY_ACCEPTED` drafts | `1bcae139…`, `016b1760…`, `f412a90e…` | — | owner after RES-464 |
| RES-454 | In Review — reviewer addendum D1–D3 appended | `ae2c19ee…` (was `dcdc614a…`) | — | owner |
| RES-453, RES-468 | In Review — no unanswered reviewer requests found | — | — | owner |

## RES-491 resolution (summary)

Velocity-level Signorini–Coulomb contact law with coupled impulses, static-cone stick feasibility, slip-consistent sliding, unilateral projections and exhaustive normal-set fallback. See RES-491 / RES-492 Linear receipts and `docs/i2/RES491_PLANAR_MULTIBODY_MECHANICS.md`.

Reproduce:

    python -m unittest discover -s tests
    python -m scripts.run_i2_jumps --out runs/i2/res492

## Owner decisions pending

1. RES-491 and RES-492: review In Review evidence (no decision blocks RES-493).
2. RES-463: approve the exact Deep Research prompt, tool, budget and storage.
3. RES-467: go/no-go (provisional recommendation: NEED EXTERNAL DATA plus a verification-only track).
4. Author data requests (Carroll 2017; Li 2025) and the download location for the CC BY datasets (Zenodo 21840566; Horst 2018).

## Claim ceilings

NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY, BIOLOGICAL_VALIDATION=NOT_ESTABLISHED, REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED. PLATE2BAR_MUTATIONS=0. RES-464 not launched.
