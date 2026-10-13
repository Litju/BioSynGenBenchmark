# I0 integrated mechanics and measurement pilot

Evidence class: manufactured latent mechanics plus simulated FORCE/LPT observations.

## Context and assumptions

- Scenario: manufactured_vertical_controlled_stance_then_flight; body mass 78 kg; optional rigidly coupled bar mass 22 kg.
- Prescribed stance force: 1250 N for 0.37 s; ideal flight follows for 0.41 s with zero latent ground reaction.
- FORCE clock: period 0.04 s, offset 0.003 s, drift 50 ppm.
- LPT clock: period 0.025 s, offset -0.007 s, drift -35 ppm.
- Force and LPT observations retain independent native time vectors. The figure maps each vector using its declared affine clock only for display.
- FORCE perturbations: offset -1 N, Gaussian noise SD 15 N, quantization 5 N, saturation ±2000 N, missing sample indices [3].
- LPT perturbations: length offset 0.001 m and noise SD 0.003 m; rate offset -0.001 m/s and noise SD 0.015 m/s; angle noise SD 0.004 rad; missing sample indices [4].
- Quantization and saturation settings are in pilot-data.json. All noise, offsets, quantization, saturation, and clock drift are illustrative assumptions; none are calibrated to a device.

## Independent numerical report

The evaluator received public clean simulated observation arrays and explicit independent known-answer references. It did not receive the latent trajectory object.

Known-answer counts: evaluated=5, eligible=3, admitted=3, rejected=0, not-evaluable=2.
- force_impulse_balance: PASS (KNOWN_ANSWER_IMPULSE_PASS)
- ballistic_flight: PASS (BALLISTIC_FLIGHT_FORCE_PASS)
- cable_geometry: PASS (CABLE_GEOMETRY_PASS)
- biological_validity: NOT_EVALUABLE (BIOLOGICAL_REFERENCE_UNAVAILABLE)
- real_paired_force_lpt: NOT_EVALUABLE (PAIRED_EMPIRICAL_REFERENCE_MISSING)

Adversarial controls:
- INCONSISTENT_IMPULSE: REJECTED — Integrated covered external impulse disagrees with the independent momentum change.
- IMPOSSIBLE_FLIGHT_GRF: REJECTED — Public FORCE conflicts with the independent ideal-flight reference.
- LPT_PROJECTION_MISMATCH: REJECTED — Observed cable rate conflicts with the independent directional projection.

Missing-evidence counts: evaluated=5, eligible=0, admitted=0, rejected=0, not-evaluable=5.
- force_impulse_balance: NOT_EVALUABLE (SUPPORT_COVERAGE_INCOMPLETE)
- cable_geometry: NOT_EVALUABLE (CABLE_REFERENCE_MISSING)
- bilateral_force: NOT_EVALUABLE (COMBINED_FORCE_NOT_BILATERAL)
- biological_validity: NOT_EVALUABLE (BIOLOGICAL_REFERENCE_UNAVAILABLE)
- real_paired_force_lpt: NOT_EVALUABLE (PAIRED_EMPIRICAL_REFERENCE_MISSING)

## Claim limits

NUMERICAL_VERIFICATION_ONLY; SIMULATED_MEASUREMENT_ONLY; BIOLOGICAL_VALIDATION=NOT_ESTABLISHED; REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.

This run does not validate human biomechanics, instrument fidelity, vendor exports, real paired FORCE/LPT, or a universal credibility score. The vertical model has no segmented body, muscle control, horizontal dynamics, foot geometry, or landing impact.
