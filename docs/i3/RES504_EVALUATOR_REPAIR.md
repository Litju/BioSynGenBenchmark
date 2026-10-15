# RES-504 independently demonstrated evaluator defects and repair

The original suite reproduced (RES-503: 243 original tests plus 3 audit controls; 2 optional SciPy skips). The original curated benchmark also reproduced 10/10 positives and 76/76 prescribed faults. Nevertheless, independent analytical submissions exposed false acceptance and crashes in `biosyngen_benchmark/i2_evaluator.py`.

The first new test run had 15 methods with 21 failing subtest assertions and 23 subtest errors (exit 1). These are test outcomes, not 44 independently sampled defect families. The failing evidence is `runs/i3/res504/evaluator-before.txt`; only line endings and three trailing presentation spaces were normalized for Git checks. No assertion, traceback, count or result was removed. Demonstrated gaps include intermediate momentum corruption that cancels at the final endpoint, equal/opposite force corruptions, unchecked tensile/frictional impulses and their times, missing/nonfinite friction declarations, empty or duplicate body records, negative inertia, unknown support/claim identities, and malformed nested JSON types that throw rather than return INVALID.

The repair validates the declared JSON contracts before calculating, refuses derived numeric overflow, checks prefix momentum at each distinct recorded end time, and tests external impulses for tension and grouped friction. It retains the original momentum tolerance, `1e-6 * M * g * max(record_duration, 1e-12)` N s, and original force tolerance `1e-6 * M * g` N. Prefix balances use the same record-scale tolerance; this is a declared numerical policy, not a proved quadrature bound or a calibrated real-data acceptance criterion. Friction is checked only against the finite declared model hypothesis, not an admitted empirical friction authority.

A genuinely valid instantaneous-impact fixture then exposed a boundary-accounting error in the first repair. That failing run is `evaluator-boundary-before.txt`: 16 methods, 1 failure, exit 1. The schema provides event times without per-frame impulse associations, so only the last state at a shared end timestamp is checked against all impulses at that time. This prevents double counting or rejecting legitimate zero-duration impact frames. Intermediate states within one co-timed batch cannot be independently adjudicated with this schema; the limitation is explicit.

Final targeted command:

    python -m unittest discover -s tests -p test_i3_evaluator.py -v

16 methods pass, exit 0 (`evaluator-final.txt`). One method exercises 10 independently authored support/free-fall cases, not generator exports. Hostile tests also cover multiple corruptions, candidate tolerance/calibration/source-stage spoofing and existing empirical abstentions. Legacy `test_i2j_benchmark.py`: 6 pass, exit 0. Original benchmark reproduction and first post-repair reproduction both give 10/10 positives and 76/76 faults; the final source is additionally exercised in two fresh benchmark processes by the I3 runner.

The baseline benchmark's largest reported endpoint residual is `5.725317497873306e-09` N s on this Windows environment, rather than the dossier's approximate 1e-10 wording. Full per-family endpoint residuals and decisions are preserved in `baseline-benchmark/benchmark.json`; no historical RES-498 file is overwritten. Finite curated fault tallies and their Wilson intervals do not estimate power against a randomly sampled real-world error distribution.

The evaluator still checks declared whole-system linear momentum and vertical world-support admissibility. It does not verify every joint, orientation, angular momentum or anatomical limit from submissions, distinguish a coherent fabricated latent story from admitted measurement evidence, verify between-sample behavior, or certify all body/control physics. BILATERAL_FORCE and ANATOMICAL_PHASES remain NOT_EVALUABLE for empirical claims. The independently authored sensor checks verify numerical bilateral sum/geometry identities separately.

No generator dynamics, model parameters or scientific tolerances were changed to get a PASS. Existing unsupported catch/drop/rebound/hang/block mechanics remain refused. Before/after negative evidence is retained. Two bounded evaluator repair iterations were used; no additional repair loop was launched.

Claim ceilings: NUMERICAL_VERIFICATION_ONLY; SIMULATED_MEASUREMENT_ONLY; BIOLOGICAL_VALIDATION=NOT_ESTABLISHED; REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.
