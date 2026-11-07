# RES-597 / RES-598 continuation evidence

This is partial numerical evidence, not campaign completion or independent
final-SHA acceptance. No owner-only merge, Done or scientific acceptance occurred.

## Revisions and commands

Mechanical source on existing PR #9:
`00d278ee7a56676ee852099d44dea49d3f1fd241`.
New commits preserved in order:

* `7819b2e9cee1e9a7dcc23b467e2efdb94a07690e`: reproducible failed landing controls.
* `557b2d4afc21b7fe09df2c4f690e0ed263c72999`: integrated elevated support/release and contact energy.
* `00d278ee7a56676ee852099d44dea49d3f1fd241`: separate numerical projection and approximate rigid work.
* `0865edcbe3a8f7e1058ff45f67ed6b25754b8bc6`: non-rewriting integration into existing PR #10.
* `c0cdb9cd7c1dd2e7d9daf337f2b2999969536cfb`: independent angular and energy checks.

All new source/tests/probes are owned code and manufactured data. No external
recordings, participant information, private holdouts or third-party code are
included. Complete staged inventories/diffs and diff checks were inspected.

Executed commands (exit 0):

* `python -m unittest discover -s tests -q`: 431 tests, 940.779 s, two optional
  SciPy skips. Discovery began on the mechanical branch before the new RES-598
  checks were added; this is not a full-suite test of the final benchmark SHA.
* `python -m unittest discover -s tests -p test_res597_platform_release.py -v`:
  5 passed, including support, release, gravity-only descent, invalid times and
  energy refinement.
* `python -m unittest discover -s tests -p test_i2c_multibody.py -q`: 35 passed.
* `python -m unittest discover -s tests -p test_res598_*.py -v`: 16 passed.
* `python -m unittest discover -s tests -p test_i2j_benchmark.py -q`: 6 passed.
* `.venv/Scripts/python -m scripts.require_i3_scipy_checks`: 2 passed, no skips;
  Python 3.12.13, SciPy 1.17.1, NumPy 2.4.4.
* `python -m scripts.run_movement_credibility --out runs/i3/res598/continuation-20261107-v1`:
  239.453 s, 128922 bytes before figures. Numerical source SHA `c0cdb9c...`;
  `source_dirty=true` is retained: the concurrent full suite had temporary
  working output and the plotting script was not yet committed. The report
  includes the actual evaluator/runner hashes; this is not clean final-SHA acceptance.
* `.venv/Scripts/python -m scripts.plot_movement_residuals --report runs/i3/res598/continuation-20261107-v1/credibility.json --out runs/i3/res598/continuation-20261107-v1/residuals-inspected.png`:
  generated with Matplotlib 3.10.8 and visually inspected. The earlier image is
  retained with its legend-overlap finding; the inspected image corrects it.

## Measured outcomes

The report contains all 120 cells: 0 PASS, 3 FAIL, 18 NOT_IMPLEMENTED,
11 NOT_EVALUABLE, 88 EVIDENCE_PENDING. This is not 120 independently completed
domain tests. A bounded invariant cannot qualify a whole domain.

Historical linear/contact positives: 10/10; original negative controls: 76/76
detected. New angular spin negatives: 10/10 detected. Three bounded drop cases
retain 15/15 original fault detections. Maximum final linear residual across
the 10 baseline fixtures: 5.2305160e-11 N s.

Angular balance passes only 5/10 baseline fixtures. Maximum prefix residuals
and budgets (kg m2/s) include CMJ 0.0257631 / 0.000805938, SJ 0.0272460 /
0.000697595, and snatch pull 0.00280667 / 0.00151288. Loaded CMJ and loaded
jump squat also fail; full per-fixture numbers are in the JSON. These are
failures of the declared planar numerical budget, not empirical human results.

Drop energy residuals at 0.5 s are 63.3005153 / 28.2569551 / 12.9130961 J
at dt 0.0004 / 0.0002 / 0.0001 s. Numerical projection corrections are only
-0.000531872 / -0.0000643270 / -0.00000773148 J and cannot explain or erase
the raw residual. No independent energy acceptance budget exists. The default
full candidate still penetrates the knee-floor boundary at 1.2886 s at 0.2 ms.

## Movement status

| Movement | Actual implementation/qualification status |
| --- | --- |
| CMJ | Existing planar movement retained; new angular budget FAIL. |
| Back squat | Existing planar movement retained; bounded angular invariant PASS, full domain qualification pending. |
| Deadlift | Existing changing bar-floor/load-transfer model retained; bounded angular invariant PASS. |
| Bench | Existing feet/bench/chest/rack mechanics retained; bounded angular invariant PASS; unrack/rerack and full recovery not newly qualified. |
| Squat jump | Existing static-start movement and contamination refusal retained; new angular budget FAIL. |
| Drop jump | Elevated support/withdrawal, descent, compliant first impact, reversal, rebound and second flight exist; stable final recovery FAIL. No admitted RSI or completed protocol. |
| Power clean | Pull-only derivative retained; actual turnover/front-rack catch/recovery absent. |
| Clean | Full-depth rack receipt/recovery absent. |
| Power snatch | Spatial shallow overhead receipt/recovery absent. |
| Snatch | Spatial full-depth overhead receipt/recovery absent. |

The Olympic prerequisite remains explicit: the present articulated engine has
two position coordinates and one rotation per rigid body, one lumped arm chain
with shoulder/elbow joints, no independent wrists/hands or spatial shoulder
rotations, and no integrated bilateral spatial grip/receiving dynamics. The
existing planar rack reach refusal remains reproducible with
`front_rack_feasibility()` and `build_power_clean(scope="FULL")`. No valid full
catch can be supplied by relabeling its pull. A spatial dynamics/geometry and
contact model is still required; it was not implemented in this continuation.

## Remaining gates

Exact-head hosted full suites and the new scientific-report CI step, source-only
truth-denial/reproduction at the final SHA, completed movement-specific energy,
actuation and uncertainty studies, and a separate final-SHA assessor are still
required. Hosted jobs are not declared green until observed successful. Source
and hidden-TRUTH roles and the original admission negatives are preserved.
No licensed matched human/device evidence is admitted.

NUMERICAL_VERIFICATION_ONLY
SIMULATED_MEASUREMENT_ONLY
BIOLOGICAL_VALIDATION=NOT_ESTABLISHED
REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED
