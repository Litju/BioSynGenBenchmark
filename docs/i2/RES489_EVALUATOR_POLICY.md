# RES-489 evaluator hardening and acceptance record

**Status:** implementation for owner review (RES-489 stays In Review).
**Baseline:** public `main` and clean local checkout at `6115d702ba3ab814982353945b0753f4846aae88`.
**Scope:** evaluator false acceptance, input validation, calibration authority, quadrature qualification, suite accounting, and CI.

## Source audit and reproduced defects

The linked Linear source audit is `BioSynGenBenchmark — Independent I1 Source Audit & Full Protocol Coverage Gap` (Linear document `2718d3dfb375`). It pinned the audited source to the baseline SHA above and identified:

1. A candidate could accept one positive and abstain on every negative; the suite gate still passed.
2. NaN/infinite inertial inputs could flow through comparisons and produce a spurious pass.
3. Candidate-owned FORCE gain/zero and LPT encoder/timing uncertainty could inflate tolerances until corrupted measurements passed.
4. Zero or negative periods, duplicate kinematic times, nonfinite contact data, and bad vector dimensions were not screened consistently.
5. Same-sample quadrature differences were described as bounds without a justified regularity/bandwidth premise.
6. Existing repository instructions still described the completed I0 queue as the only authorized work; no CI workflow existed.

Fresh adversarial tests were authored in `tests/test_i2a_evaluator_adversarial.py` before changing the evaluator. `runs/i2/res489/falsifiers-before-repair.txt` records that baseline run: 9 tests, 12 failures and 1 error. It records the selective-abstention false suite PASS, NaN/∞ mass/inertia false PASS, zero/negative clock false PASS, duplicate-time false PASS, nonfinite contact false PASS, and candidate-inflated plate/LPT tolerance false PASS. The 16-test post-repair file includes the same regressions plus nested-metadata, analytic two-body, aliasing and event-boundary controls; all pass.

## Evaluator changes

### Schema and numerical validation

`biosyngen_benchmark/i1.py` now validates body mass/inertia, gravity, friction, frame time, six-element body states, contact components, support endpoints, powers/energy, activation values, 3-D LPT attachment vectors, event ordering, device sample values, exact supported units/sign conventions, sample indices, finite offsets, and strictly positive sample periods. Kinematic and device clocks must be strictly increasing. Numeric booleans are not accepted as measurements. Arithmetic results are checked for nonfinite values before a finding can be returned as PASS.

Invalid physical or schema values are rejected before they enter Newton/Euler, contact, flight, work-energy, detector, or LPT calculations. Device periods of zero or less and repeated timestamps cannot PASS clock integrity.

### Independent metrology authority

FORCE/LPT calibration bounds are no longer accepted from the submission. `evaluate_i1` accepts them only through separate `force_authority` and `lpt_authority` arguments of the immutable `MetrologyAuthority` type. The record carries a reference identifier, evidence SHA-256, device class, admitted clock values, and the relevant instrument bounds. Mismatched device class/period/offset is rejected; missing or malformed authority returns NOT_EVALUABLE. Candidate uncertainty keys are removed by schema filtering and listed in `refused_fields`.

No real instrument calibration has been admitted to this repository. Therefore ordinary submissions return NOT_EVALUABLE for FORCE flight-zero/weight and LPT geometry checks, and `SENSOR_METROLOGY` remains NOT_EVALUABLE. The test-only authority fixture is manufactured to exercise the software path and is not calibration evidence. The Python API relies on the benchmark host to supply an admitted authority object; the evidence digest format is checked, but the library does not independently authenticate a source archive or signature.

### Event-aware quadrature and resolution

The evaluator compares trapezoid estimates on nested grids and records the finer/coarser refinement deltas. Those values are named and documented as estimates; `quadrature_error_is_bound` is always false. Candidate-declared events and independently visible contact-topology changes split windows. The exporter collapses same-time one-sided solver records to the right-continuous sample and records an event boundary rather than passing duplicate timestamps downstream.

An unstable window is excluded from the sampled numerical decision. If remaining stable windows show an imbalance, the claim is REJECTED; if they do not, an unresolved window makes the whole claim NOT_EVALUABLE. Metrics report how many quadratures were under-resolved. The tests include an alternating load that aliases on the fine samples and is explicitly refused. Stable sampled checks still cannot rule out an arbitrary hidden between-sample signal; no bandwidth limit or external continuous reference is admitted here.

### Suite accounting and independent controls

`run_suite` now fails if any adjudicated positive is not accepted, any negative is falsely accepted or abstained, or any abstention control is not refused. Missing evaluator findings, evaluator exceptions, unknown expected claims, and invalid adjudications fail closed. It reports positive and negative decision coverage, conditional accuracy, negative-abstention rate, abstention-control accuracy, their denominators, and Wilson 95% intervals. Confidence intervals describe only the finite authored cases; they do not estimate performance on an unobserved candidate population.

`biosyngen_benchmark.controls.two_body_free_fall_control` supplies an independent analytic two-body family with distinct masses, velocities and spins. It does not import generator code. The current challenge report is `runs/i2/res489/benchmark-suite.json`; it includes the unknown family, generator exports, adversarial mutations, and abstention controls. The report's finite gate passed with **69/69** adjudicated instances; refuse-all and accept-all both failed. This is numerical evaluator verification only.

## Historical preservation and reproducibility

The historical I0 and I1 run files remain in their original directories. The I2 challenge report and I0/I1 regression reruns use `runs/i2/res489/`, so they do not overwrite the RES-481/485/487 records. `scripts/i1_i0_regression.py` accepts an explicit `--output` path for this purpose. The existing I1 results remain historical records; this evaluator version separately classifies CMJ sampled momentum as unresolved, loaded-CMJ linear/angular momentum as rejected at the current tolerance, and SJ sampled momentum as passing. No old result was rewritten to make those classifications appear unchanged.

Commands:

```powershell
python -m unittest discover -s tests -p test_i2a_evaluator_adversarial.py -v
python -m unittest discover -s tests -v
pwsh -NoProfile -File tests/test_overnight_controller.ps1
py -3.11 -m unittest tests.test_i1_solver_crosscheck -v
python -m scripts.i1_benchmark_suite
python -m scripts.i1_i0_regression --output runs/i2/res489/i0-regression.json
python -m scripts.run_i1 --replicate --out runs/i2/res489/i1/run
```

The new GitHub Actions workflow defines Python 3.12 standard-library jobs on Windows and Ubuntu, runs the full test suite, challenge suite, frozen-I0 comparison and two-process I1 reproducibility check. A separate Python 3.11 SciPy job installs SciPy and runs the DOP853 cross-check; installation or test failure fails the job. The local branch has not been pushed, so GitHub-hosted workflow status is not yet available.

Local verification on Windows completed with Python 3.12: `python -m unittest discover -s tests -v` ran 112 tests (110 passed, 2 SciPy-only skips); `pwsh -NoProfile -File tests/test_overnight_controller.ps1` passed all 23 assertions; CPython 3.11.9 with SciPy 1.17.1 passed both DOP853 checks. Logs are under `runs/i2/res489/`.

The final two-process I1 output bundle has digest `dc5fead3d2056050c0e6fd4703749d907b7062a58fbf92380c66608f7c8f4572`; its six artifact files match byte-for-byte across processes. The I0 comparison preserved the frozen payloads; only the historical source hashes in its manifest and the already-documented RES-478 finding reason differ.

## Claim ceiling and remaining uncertainty

Use only `NUMERICAL_VERIFICATION_ONLY`, `SIMULATED_MEASUREMENT_ONLY`, `BIOLOGICAL_VALIDATION=NOT_ESTABLISHED`, and `REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED`. There is no admitted human dataset, real paired FORCE/LPT trial, or device calibration. A synthetic authority test does not change that. No result supports a physiological, vendor-equivalence, clinical, or empirical accuracy claim. Public repository visibility still does not grant an open-source license; this work is on a local review branch and has not been pushed or merged.
