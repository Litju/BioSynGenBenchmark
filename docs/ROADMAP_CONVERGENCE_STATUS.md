# Roadmap convergence status (RES-488 one-run campaign)

The single durable checkpoint for the owner-authorized Opus 5.5 full-roadmap campaign. Linear is authoritative for issue state; this file records local Git and verification facts so a fresh session can resume without repeating work.

Last update: 2026-10-13, end of Opus 5.5 session 1.

**CAMPAIGN_RESULT=PARTIAL_WITH_BLOCKERS**

The I2 physics chain is blocked at RES-491. R1 research is complete to In Review. R2, R3 and R4 are owner-gated.

## Git state

| Item | Value |
|---|---|
| Base | `eade435c427e7f12349ad370168c0984b6836979` (RES-490) |
| RES-491 working branch | `codex/res-491-bilateral-multibody-foundation` (HEAD `eade435`). The 3 RES-491 files are **untracked** in the shared worktree and have never been staged. |
| Checkpoint branch | `claude/res-488-roadmap-r1` (local only; license pending, so no push or PR) |
| Public `main` | `6115d702…` (I1); nothing merged |
| `research/` | git-ignored by project policy (rights). R1 deliverables live there and are identified by SHA-256 in each Linear receipt. |

## Issue matrix

| Issue | State | Commit / identity | Verification | Next action |
|---|---|---|---|---|
| RES-489 | In Review (prior) | `895c8e6`, `913f01e`, `4cf4f6a` | prior session | owner review |
| RES-490 | In Review (prior) | `eade435` | prior session | owner review |
| RES-491 | **In Progress — BLOCKED_AFTER_ONE_EXCEPTION** | uncommitted (3 files) | focused 26: 25 pass / 1 fail; full 147: 144 pass / 1 fail / 2 skip | **owner:** authorize or decline one slide→stick capture repair |
| RES-492..498 | Todo — dependency-blocked | — | — | start only after RES-491 is qualified |
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

## RES-491 blocker (exact)

- **Repaired (the authorized exception):** a typed `ParameterDomainError` with `field`/`reason` (NONNUMERIC/NONFINITE/NONPOSITIVE) for mass and inertia. The full data domain is tested.
- **Verified physics** (six reviewer falsifiers added; five pass):
  - central-difference rotating-frame rates and accelerations (BODY_A and BODY_B)
  - sliding power = −μN|v_t|
  - redundant stick supports fail closed
  - mirror equivariance
  - time-evolving support transfer releasing at the analytic time
- **Blocking defect:** no slide→stick capture in `advance`. The slip direction chatters with |v_t| ≈ μ·g·dt. Falsifier: `tests/test_i2c_multibody.py::IndependentReviewerFalsifierTests::test_sliding_contact_captures_stick_after_slip_reverses`.
- **Literature-backed repair direction (RES-458):** velocity-level complementarity time-stepping (Stewart & Trinkle 1996), or a stick trial on slip-sign reversal in `_prepare_contact_states`.

Reproduce:

    python -m unittest discover -s tests -p test_i2c_multibody.py -v
    python -m unittest discover -s tests

## Owner decisions pending

1. RES-491: one stick-capture repair (yes/no).
2. RES-463: approve the exact Deep Research prompt, tool, budget and storage.
3. RES-467: go/no-go (provisional recommendation: NEED EXTERNAL DATA plus a verification-only track).
4. Author data requests (Carroll 2017; Li 2025) and the download location for the CC BY datasets (Zenodo 21840566; Horst 2018).

## Claim ceilings

NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY, BIOLOGICAL_VALIDATION=NOT_ESTABLISHED, REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED. PLATE2BAR_MUTATIONS=0. RES-464 not launched.
