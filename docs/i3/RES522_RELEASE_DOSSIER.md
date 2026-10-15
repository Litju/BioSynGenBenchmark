# RES-522 bounded numerical release closeout

Review: [PR #1](https://github.com/Litju/BioSynGenBenchmark/pull/1).
The [RES-522 final review receipt](https://linear.app/alignerr-cmj/issue/RES-522) records the exact published commit, five hosted job URLs/results, final tree/history clearance and owner decision. This external receipt avoids a self-referential commit hash. Read it before treating this candidate as ready; earlier CI never qualifies a later commit.

## Selected integration and publication manifest

The release starts at I3 `d758254e0b18b12f5d8b670cfd5c536eddfe2824`. Only the contact repair from RES-517 `d5e651c539a29c9b4eb01192e3febf0f6ac8ec4d` is integrated: coupled total-impulse Coulomb projection, transported frames, grazing joint-velocity reconciliation and explicit impact/projection work. Physical coefficients, tolerances, controllers and evaluator admission criteria are unchanged.

`runs/i3/res522/integration-provenance.json` maps each selected source/fixture to its original commit/path/hash. The new tests extract self-contained state restoration and joint impulse/work closure; no I4 controller dependency is imported. The two older regression assertions now test the combined physical impulse rather than signed correction alone, with unchanged bounds.

| Published addition/change | Rights and content disposition |
| --- | --- |
| `biosyngen/multibody.py` | Owned RES-517 repair, Apache-2.0; unchanged qualified source |
| `tests/test_i2d_engine_extensions.py` | Owned RES-517 regression correction, Apache-2.0 |
| `tests/test_i4_projection.py` | Owned independent manufactured controls; no generator truth used as evaluator truth |
| Three JSON files under `tests/fixtures/` | Owned synthetic first/second cone and grazing witnesses; no participant/device data |
| README, this dossier, `runs/i3/res522/` selected receipts/logs | Owned release provenance and synthetic numerical evidence; local bulk outputs excluded |

The owner-selected Apache-2.0 LICENSE, NOTICE attribution and dependency inventory remain applicable. No new dependency, vendored code, paper payload, external registry implementation, restricted source, participant record or holdout is added. Clearance relies on the recorded RES-505 personal-research authority plus bounded content/tree/history inspection; it is not an independent title certificate or universal secret detector.

## Verification and reproducibility

The final receipt records actual counts/exits and inspected artifacts for:

```text
python -m unittest tests.test_i4_projection -v
python -m unittest discover -s tests -v
py -3.11 -m scripts.require_i3_scipy_checks
pwsh -NoProfile -File tests/test_overnight_controller.ps1
python -m scripts.verify_i3_reproducibility --out <fresh-project-directory>
python -m scripts.i3_platform_probe --out <fresh-project-file>
python -m scripts.audit_i3_release --ref HEAD --out <fresh-project-file>
```

The reproduction command runs all six exercise/benchmark workers twice in separate processes, checks complete artifact inventories and independently requires 10/10 numerical positives and 76/76 authored faults in both benchmark replicas. The standard suite's two optional SciPy skips are distinct from the mandatory separate SciPy job's two unskipped checks. SciPy shares the I1 derivative and does not independently certify I2 equations.

The 14 focused controls cover both historical cone witnesses, grazing, joint impulse and midpoint kinetic-work closure, reduced-mass analytical closure, cone edge/stick/slide/release, two-foot patch order/mirror, rotating frames/internal contact, simultaneous restitution, singular-support refusal and smooth timestep refinement. The old source reproduces both original excesses; failed-before logs remain evidence rather than release failures. A before-repair observer test also raised an API/observer error; the separate direct second-state replay reproduces the physical excess exactly.

Local requalification also reruns the original 30 stress/contact audit conditions with immutable RES-517 audit tools loaded from their pinned Git objects. Bulk derived trajectories stay local; summary, tool hashes and trace hashes identify that audit. This checks physical bookkeeping while preserving task failures. The public self-contained controls and full hosted suites are independently rerunnable without the private I4 branch.

Local results: 305 tests collected, 303 passed, two optional SciPy skips; separate SciPy 2/2 and PowerShell 23 assertions passed. All 30 physical audits pass over 101,242 intervals, with worst cone margin -8.85e-17 N s, no negative combined member normal impulse, maximum linear impulse residual 5.96e-13 N s, projection equation residual 1.08e-13 and projection kinetic-work residual 6.40e-13 J. Only 2/10 stress tasks reach duration at each timestep; task failures remain failures. Angular application-point quadrature residual reaches 0.005833 N m s. The coarse 2 ms stress work ledger has unclosed accumulated energy up to 73.286 J; refinement generally reduces it but does not provide a universal monotone bound or exact continuous energy certificate. These last two quantities are retained integration/geometry limitations, distinct from the much smaller algebraic projection residuals.

Acceptance requires all five hosted jobs at the exact final PR commit: Windows and Linux standard suites, Windows and Linux mandatory SciPy comparisons, and the exact-reference cross-platform comparator. Independently downloaded logs/artifacts must confirm source identity, benchmark denominators, reproduction inventory equality and all eight manufactured analytical comparisons. See the final receipt for actual results, URLs and artifact hashes.

## Accepted scope and retained limitations

Accepted scope is numerical planar rigid-body/contact software and simulated FORCE/LPT observations within declared fixtures and refusal boundaries. It includes the tested total-impulse contact correction, strict independent evaluator controls, modeled squat/deadlift/bench/jump and floor-pull fixtures, and manufactured reproducibility checks. These finite controls do not establish arbitrary multi-contact admissibility or a universal nonsmooth error bound.

Stationary contact work, COM pseudo-work, vertical bar-work estimates, signed numerical projection work, interval forces and instantaneous impact impulses remain distinct. Position projection and within-event geometry introduce documented numerical/quadrature limitations. Parameters, instruments and controllers are assumed and uncalibrated. Unsmoothed settling, separable shoulder-shelf jump-squat load and bench support openings remain limitations.

The original I1 red work-ledger evidence and original 2/10 I4 stress completions remain intact. Corrected bookkeeping does not qualify a recovery policy. All I4 commits remain on `codex/i4-recovery`, frozen at `13a0a8fb3cff935116e0787d66516a23dc4716ba`; rejected RES-515 controller `07a15af553d43faf909f1ec158e1e0b350ea59b4` is excluded. Its 35 pairs produced zero improved dynamic recoveries and zero safe loaded returns. RES-521's restricted QP diagnosis is not a statement of human infeasibility. RES-516 remains frozen and unrun.

Unsupported: biological/human performance, robust recovery control, vendor device metrology and real paired FORCE+LPT validity. Catch/turnover/front rack, full cleans/snatches, hang/block starts, drop/rebound, ordinary hand-stabilized jump squat, anatomical phase inference and compliant physical peak/RFD claims remain refused or unestablished. No new research roadmap is authorized by this closeout.

NUMERICAL_VERIFICATION_ONLY; SIMULATED_MEASUREMENT_ONLY; BIOLOGICAL_VALIDATION=NOT_ESTABLISHED; REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.

Owner review alone may authorize main merge, tag/release, scientific acceptance or Done transitions. This task stops after the bounded handoff; attention returns to Plate2bar ML-593 → ML-594.
