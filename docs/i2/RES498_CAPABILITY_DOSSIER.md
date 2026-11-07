# RES-498 I2 capability dossier: protocol matrix, independent benchmark and CI

_Every row of the RES-490 inventory has an explicit status: a modeled generator with independent tests, a scoped subset, or a refusal with a reason. Every implemented family is checked by an evaluator that imports nothing from the generator._

---

## 🗺️ Capability matrix

Built by [i2_coverage.py](../../biosyngen_benchmark/i2_coverage.py). A missing row or variant raises `CoverageGap`.

| Inventory exercise | Status | Generator / reason | Issue | Tests |
| --- | --- | --- | --- | --- |
| COUNTERMOVEMENT_JUMP (CMJ_UNLOADED, _AKIMBO, _FORCE_ONLY, _LPT_ONLY) | MODELED_NUMERICAL_VERIFICATION | `jumps.build_jump('CMJ')` | RES-492 | test_i2d_jumps |
| SQUAT_JUMP | MODELED_NUMERICAL_VERIFICATION | `build_jump('SJ')` | RES-492 | test_i2d_jumps |
| LOADED_CMJ (CMJ_LOADED, _AKIMBO) | MODELED_NUMERICAL_VERIFICATION | `build_jump('LOADED_CMJ')` (vest) | RES-492 | test_i2d_jumps |
| LOADED_JUMP_SQUAT | MODELED_NUMERICAL_VERIFICATION | `build_jump('LOADED_JUMP_SQUAT')` (back bar) | RES-492 | test_i2d_jumps |
| BACK_SQUAT | MODELED_NUMERICAL_VERIFICATION | `squat.build_back_squat` | RES-493 | test_i2e_back_squat |
| DEADLIFT | MODELED_NUMERICAL_VERIFICATION | `deadlift.build_deadlift` | RES-494 | test_i2f_deadlift |
| BENCH_PRESS | MODELED_NUMERICAL_VERIFICATION | `bench.build_bench_press` | RES-495 | test_i2g_bench_press |
| OLYMPIC_PULL_DERIVATIVE, CLEAN_PULL, SNATCH_PULL | MODELED_NUMERICAL_VERIFICATION | `olympic.build_pull` | RES-496 | test_i2h_olympic_pulls |
| POWER_CLEAN | SCOPED_SUBSET_NOT_IMPLEMENTED_REMAINDER | Pull simulated; catch infeasible in the planar lumped arm | RES-497 | test_i2i_power_clean |
| FULL_CLEAN, FULL_SNATCH | NOT_IMPLEMENTED | Front-rack / overhead receipt not modeled | — | test_i2j_benchmark |
| HANG_CLEAN, HANG_SNATCH, HANG_POWER_CLEAN, HANG_POWER_SNATCH | NOT_IMPLEMENTED | Hang starts deferred in RES-490 | — | test_i2j_benchmark |
| DROP_JUMP, REPEATED_REBOUND_JUMP | NOT_IMPLEMENTED | Rigid point-contact feet lack qualified compliance | RES-492 | test_i2d_jumps |

Every row is also marked **EXTERNAL_REFERENCE_NOT_EVALUABLE**: no qualified paired human or device reference is admitted.

## 🧪 Independent benchmark

The evaluator is [i2_evaluator.py](../../biosyngen_benchmark/i2_evaluator.py). It reads the strict `BIOSYNGEN_I2_SUBMISSION_V1` schema; unknown fields such as a candidate-supplied tolerance or latent labels make the submission INVALID.

**Claims and outcomes:**

| Claim | What it checks |
| --- | --- |
| LINEAR_MOMENTUM_CLOSURE | Integral of the declared external supports + impulses + gravity = momentum change. An omitted support therefore appears as an imbalance. |
| CONTACT_ADMISSIBILITY | No tension; patch friction cones hold |
| BILATERAL_FORCE | Always NOT_EVALUABLE: no admitted calibration reference |
| ANATOMICAL_PHASES | Always NOT_EVALUABLE: motion capture is required |

**Results** (`python -m scripts.run_i2_benchmark --out runs/i2/res498`, byte-identical across processes):

| Control | Outcome | Wilson 95 % |
| --- | --- | --- |
| Positive families (CMJ, SJ, loaded CMJ, jump squat, back squat, deadlift, bench, clean pull, snatch pull, power-clean pull) | 10/10 pass momentum and admissibility | 0.72–1.00 |
| Force scaled by 1.02 | 10/10 rejected | 0.72–1.00 |
| NaN force | 10/10 invalid | 0.72–1.00 |
| Tensile foot force | 10/10 rejected | 0.72–1.00 |
| Candidate tolerance field | 10/10 invalid | 0.72–1.00 |
| Anatomical label field | 10/10 invalid | 0.72–1.00 |
| Omitted external support group (feet, bench, bar–floor, rack) | 26/26 rejected | 0.87–1.00 |

The largest positive momentum residual is about 1e-10 N·s, against a 1e-6 × M·g·T tolerance fixed by the evaluator. With n = 10, the Wilson lower bound of 0.72 reflects the small number of families, not detection quality.

## 🖼️ Rendered-figure inspection

The figures for RES-492 to RES-496 were opened as rendered images (local static server) and inspected visually, not only parsed as SVG.

| Figure | Observed artifact |
| --- | --- |
| Back squat, deadlift | High-frequency force ringing in lockout holds. It is a declared PD/activation settling artifact, not tremor. |
| Bench press | Bench reaction fluctuates during pre-tension and unrack; there is a spike at rerack (hook impact). |
| Olympic pulls | Foot-force spikes from brief flight landings, and settling oscillation after the pull |
| All | Legends overlap part of the traces in a few panels (cosmetic) |

## ⚙️ CI

[verification.yml](../../.github/workflows/verification.yml) runs on Windows and Linux:

- the full suite;
- the I1 reproductions;
- `python -m scripts.check_i2_reproducibility`. It runs every I2 evidence script twice and fails on any nondeterminism or invalid SVG structure. Agreement with the committed bytes is reported but does not fail the run, because platform floating-point libraries may differ.

CI has **not been executed remotely**: the branch is local-only while the license decision is pending. Local results on Windows / CPython 3.12.10 are recorded in the Linear receipts.

## 🚫 What is not claimed

- No biological, device or paired FORCE/LPT validation. All parameters are ASSUMED_UNCALIBRATED.
- No global realism score or badge.
- No catch, overhead receipt, hang start, drop or rebound jump.
- No per-foot attribution claim without bilateral calibration.

Claim ceilings: NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY, BIOLOGICAL_VALIDATION=NOT_ESTABLISHED, REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.
