# RES-496 Olympic pulling derivatives: clean pull and snatch pull

_Floor-start clean and snatch pulls on the gripped-bar topology. Phases come from bar, contact and joint kinematics; joint phases are refused for FORCE/LPT-only claims._

---

## 🏋️ Variants (RES-490 inventory)

| Variant | Inventory identity | Grip (declared sagittal projection) | Setup |
| --- | --- | --- | --- |
| CLEAN_PULL | OLYMPIC_PULL_DERIVATIVE_DUAL_V1 / CLEAN_PULL_DUAL_V1 | 10° abduction, factor 0.985 | Ankle 0.45 rad, arms −0.10 rad |
| SNATCH_PULL | OLYMPIC_PULL_DERIVATIVE_DUAL_V1 / SNATCH_PULL_DUAL_V1 | 30° abduction, factor 0.866 | Ankle 0.65 rad (deeper) |
| HANG_* / BLOCK_* | Deferred or absent in RES-490 | — | Refused (`NOT_IMPLEMENTED`) |

Setups are solved like the deadlift (straight arms to a floor bar). They are refused if any joint lies outside its declared passive range.

**Sequence**, as a declared strategy:

1. setup;
2. pre-tension (load share);
3. a 1.0 s minimum-jerk pull with saturating gains to a flat-foot extension posture;
4. finish;
5. lowering;
6. bar return.

Heel rise and brief foot separation are not commanded. They emerge from the dynamics.

## 📏 Latent events

| Event | Definition |
| --- | --- |
| Lift-off | Bar–floor contact opens |
| First-pull end | Bar passes knee height |
| Transition (double knee bend) | Knee re-flexion after the first-pull end; reported only if the knee actually re-flexes |
| Second-pull end | Peak bar vertical velocity |
| Heel release | Both heels open |
| Flight segments | Contiguous all-feet-open intervals |
| Bar peak height | Highest bar position |
| Bar return | Bar–floor impact or touchdown |

The grip force is computed independently of the solver, from bar Newton balance: `m Δv/h + m g − F_floor`.

## ✅ Verification (`tests/test_i2h_olympic_pulls.py`, 9 tests)

| Check | Clean | Snatch |
| --- | --- | --- |
| Run completes, no refusals | Yes | Yes |
| Event order lift-off < first-pull end < second-pull end < bar return | Yes | Yes |
| Peak bar velocity (m/s) | 1.59 | 1.61 |
| Bar peak height (m) | 0.57 | 0.66 (shorter projected reach) |
| Momentum closure (feet + bar–floor) | 10⁻⁷ | 10⁻⁷ |
| No penetration, no tensile contact | Yes | Yes |
| Double knee bend | Not observed (no knee re-flexion); the predicate agrees with the knee rate | Same |

Further checks:

| Check | Result |
| --- | --- |
| Grip force while the bar rests on the floor | ≈ 0 |
| Grip force during the first pull | Positive (tension), > 1.2 × bar weight |
| LPT span | Equals bar-collar cable geometry (2 mm) |
| Hang and block variants | Refused |
| Unknown variant | Refused |
| Out-of-range snatch setup | Refused |
| Joint-phase claims from sensors (double knee bend, triple extension, technique proficiency) | Refused (`NOT_EVALUABLE`) |

Evidence: [summary.json](../../runs/i2/res496/summary.json) and [olympic_pulls.svg](../../runs/i2/res496/olympic_pulls.svg), byte-identical across processes. Strategy search: [strategy_search.md](../../runs/i2/res496/strategy_search.md).

~~~powershell
python -m unittest tests.test_i2h_olympic_pulls -v
python -m scripts.run_i2_olympic_pulls --out runs/i2/res496
~~~

## 🚧 Limitations and claim ceiling

- **Fragile strategies.** Plantarflexed, hip-hyperextended and slow pulls all fell. The declared flat-foot strategy completes, but neighbouring settings fail (see the search record).
- **No commanded features.** No double knee bend, shrug or scapular elevation is commanded or observed.
- **Simplified grip and bar.** Grip width is a planar projection. The bar is rigid, with no whip.
- **Not modelled.** Catch and turnover are out of scope here (RES-497), and technique proficiency cannot be inferred from bar-only LPT.
- **Uncalibrated.** Empirical comparison is NOT_EVALUABLE.

Claim ceilings: NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY, BIOLOGICAL_VALIDATION=NOT_ESTABLISHED, REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.
