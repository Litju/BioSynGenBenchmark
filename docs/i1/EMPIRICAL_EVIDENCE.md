# I1 empirical anchors, parameter provenance and uncertainty (RES-486)

Reproduce:

```
python -m scripts.i1_provenance_sensitivity
```

This writes:
* `docs/i1/parameter_provenance.json`;
* `runs/i1/res486/{sensitivity.json, sensitivity-table.md, sensor_noise_spread.json, counterexamples.json}`.

No dataset was read or downloaded.

## 1. Context of use (narrow, declared)

* **Subject:** one *hypothetical* adult (75 kg, 1.78 m), with segment fractions recalled from Winter (2009, not inspected). Not a measured person, sex, cohort or population.
* **Task:** maximal unloaded CMJ from quiet stance on level, rigid, frictional ground, plus a squat jump and a 20 kg gripped-bar CMJ. Sagittal plane, legs lumped, no arm swing.
* **Instruments:** one simulated force plate under both feet, and one simulated cable LPT (hip belt, or bar collar). Both are SIMULATED_DEVICE_HYPOTHESIS.
* **Compared quantities:** latent states, contact forces, phase events, and device exports.
* **Risk:** any use of these outputs as human norms, device specifications or validation evidence would be a misuse.

## 2. Parameter provenance

`parameter_provenance.json` lists every value.

| Group | Status | Source / note |
|---|---|---|
| gravity | PHYSICAL_CONSTANT_OR_CONVENTION | standard gravity |
| segment mass/length/COM/gyration fractions | SOURCE_IDENTIFIED_VALUE_NOT_INSPECTED | Winter (2009) table, recalled |
| activation τ_act 15 ms, τ_deact 50 ms, (0.5+1.5a) form | SOURCE_IDENTIFIED_VALUE_NOT_INSPECTED | Thelen (2003), doi:10.1115/1.1531112, recalled |
| Hill curvature 0.25 | SOURCE_IDENTIFIED_VALUE_NOT_INSPECTED | Hill (1938), recalled |
| eccentric ratio 1.5 | ASSUMED_UNCALIBRATED | Katz (1939) form |
| joint-level T0, ω_max, torque–angle, ROM limits | ASSUMED_UNCALIBRATED | scaled for plausibility (MODEL_SPEC §4); not fitted to anyone |
| foot geometry, friction μ = 0.8 | ASSUMED_UNCALIBRATED | declared |
| bar coupling k = 2e5 N/m, c = 1.2e3 N·s/m | ASSUMED_UNCALIBRATED | lumps grip and soft tissue |
| plate gain/cross-talk/offset/noise/filter/ADC; LPT encoder, reel, clocks | ASSUMED_UNCALIBRATED | device hypotheses, not vendor specifications |

**No parameter is SOURCE_VERIFIED.** Upgrading any of these requires opening the cited primary table or obtaining a calibration certificate.

## 3. Inspected literature anchors (PubMed abstracts only; qualitative use)

| Source | Inspected content | Use here | Comparison with the model |
|---|---|---|---|
| Bobbert et al. 1996, doi:10.1097/00005768-199611000-00009 | 6 male volleyball players: CMJ 3.4 cm higher than SJ from the same start posture. Simulation ruled out elastic energy; active-state build-up explained the gain | direction and mechanism of the CMJ–SJ difference | Model apex rise: CMJ 0.292 m vs SJ 0.231 m (+6.1 cm). Same direction; the only mechanism available here is activation build-up (no tendon). **Start postures differ**, so the magnitude is not comparable. Status: **MECHANISTIC_DIRECTION_CONSISTENT, MAGNITUDE_NOT_EVALUABLE** |
| Bobbert & Casius 2005, doi:10.1249/01.mss.0000155389.34538.97 | 4-segment, 6-muscle simulation: CMJ − SJ difference of 0.4–2.5 cm depending on stimulation rise rate | model-to-model mechanism comparison | Consistent in kind; this is not human evidence |
| Ferreira et al. 2023, doi:10.70252/LBRW9042 | phase-duration metrics are less consistent between days than force/impulse metrics (n = 42) | do not treat phase durations as fixed norms | Agrees with §4: durations here are dominated by the declared command timing |

## 4. Sensitivity and identifiability (one at a time)

Control timing is held at the reference, so these runs show how the *declared strategy* interacts with parameter error. The strategy is not re-optimized per parameter set. Full table: `runs/i1/res486/sensitivity-table.md`.

* **Dominant parameters (takeoff velocity):**
  * extensor Hill curvature 0.15 → 0.40: 1.96 → 2.74 m/s;
  * deactivation τ 40 → 60 ms: 2.62 → 2.22;
  * extensor ω_max ±15%: 2.19–2.57;
  * body mass 65/85 kg with strength fixed: 2.36 / 1.83;
  * strength ×0.85 / ×1.15: 1.70 / 2.37. The second is *below* the reference 2.395 because the fixed timing is no longer optimal.
* **Weak influence:**
  * activation τ 10–20 ms: 2.35–2.42 m/s;
  * friction μ 0.5–1.0: no change (no sliding until the last 3 ms before takeoff).
* **Strategy fragility (important):** shifting the push time by −20 ms or +20 ms *breaks* the jump. The first gives a toe tap before the apex (`TAKEOFF_NOT_SUSTAINED`); the second gives an unqualified heel impact. The optimized open-loop control is a knife edge, so it should not be read as a robust human coordination strategy.
* **Phase durations are control-dominated:** the unweighting duration is 0.340–0.342 s in every parameter variant, because its end is set by the declared knee-command time. In this model, the phase *existence and order* emerge from the dynamics, while their *durations* largely reflect the control hypothesis.
* **Identifiability:**
  * Latent excitation/activation states are **not identifiable** from FORCE/LPT. Different (τ_act, timing) combinations shift takeoff velocity by about the same amount as plate-noise and calibration effects (§5).
  * Muscle forces, neural drive and EMG timing are **not represented** as measured quantities and are refused.

## 5. Separated uncertainty sources

| Source | How quantified | Result (reference CMJ) |
|---|---|---|
| Numerical (solver) | refinement rtol 1e-6 → 1e-8; SciPy DOP853 cross-check | takeoff time and velocity change < 1e-4; states agree to < 1e-5 |
| Model parameters (epistemic) | OAT over declared ranges | takeoff velocity 1.70–2.74 m/s |
| Strategy (control hypothesis) | timing and release perturbations | jump fails at ±20 ms; release 0.4 → 1.83 m/s |
| Sensor noise (stochastic) | 10 plate noise seeds | threshold onset error 7.1 ± 1.2 ms; reversal 0.92 ± 0.14 ms; takeoff 0.32 ± 0.03 ms; takeoff-velocity estimate 2.4012 ± 0.0013 m/s |
| Sensor calibration (systematic) | fixed gain/offset error | +0.28% takeoff-velocity bias; −0.44% weight bias |
| Inter-individual biology | **NOT_EVALUABLE** | no qualified human data; no Gaussian "subject noise" was invented |

## 6. Counterexamples to mistaken equivalences (quantified)

* **Bar velocity is not system-COM velocity** (loaded CMJ): differences up to 0.28 m/s (0.36 m/s vs body COM).
* **Oblique cable displacement is not vertical displacement:** up to 3.1 cm.
* **Provider (derived) LPT velocity is not the true cable rate:** up to 0.033 m/s.
* **Hip-belt cable reversal is not COM reversal:** 27 ms apart (PHASE_OBSERVABILITY §4).
* **One plate under lumped feet cannot yield bilateral forces:** refused by the generator and the evaluator.
* **RES-454 negative evidence is preserved and not replayed.** The inherited real FORCE and WinLaborat/LPT corpora are unpaired. AMTI licensing is unresolved. Consumed holdouts cannot qualify this work. None was accessed.

## 7. External empirical comparison: NOT_EVALUABLE

**Missing reference.** There is no synchronized, rights-cleared, protocol-matched set of:
* force-plate data (≥ 1 kHz, calibration certificate, per-foot plates for bilateral claims);
* 3-D motion capture of segments and bar (≥ 200 Hz) under a shared hardware trigger;
* an LPT export with documented anchor geometry, encoder resolution and velocity algorithm;
* participant anthropometrics, for the same CMJ/SJ/loaded-CMJ protocol.

**Minimum acquisition experiment to make the claims evaluable:**
* *Participants:* n ≥ 12 adults, grouped by participant for holdout.
* *Trials:* 5 CMJ, 5 SJ and 5 loaded (20 kg) CMJ each.
* *Instruments and setup:*
  * two plates (left/right), with calibration verified by known weights;
  * motion capture, synchronized with the plates by a shared trigger;
  * an LPT with anchor surveyed in the capture frame.
* *Analysis:*
  * fit subject-specific segment and capacity parameters on one half of the trials only;
  * score latent-event timing, GRF profiles and LPT predictions on the held-out participants, with measurement uncertainty propagated.
* *Rights:* data must be collected under a protocol permitting this use.

**Until such data exist:** BIOLOGICAL_VALIDATION=NOT_ESTABLISHED; REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED; SENSOR_METROLOGY=NOT_EVALUABLE.
