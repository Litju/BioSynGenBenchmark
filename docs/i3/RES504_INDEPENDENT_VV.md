# RES-504 independent numerical qualification receipt

This is source-level verification of declared numerical models and simulated measurements. It does not establish human biomechanics, sensor calibration or real paired FORCE/LPT validity. The source audit, repaired evaluator, closed-form tests and fresh process receipts are inspectable; a green curated suite is not a universal credibility score.

## Fresh execution

| Command | Observed result |
| --- | --- |
| `python -m unittest discover -s tests -v` | 272 tests, 270 pass, 2 optional SciPy skips, 435.612 s, exit 0; `runs/i3/res504/final-suite.txt` |
| `python -m unittest discover -s tests -p test_i3_evaluator.py -v` | 16 methods pass, exit 0; before failures/crashes and boundary correction retained |
| `python -m unittest discover -s tests -p test_i3_mechanics.py -v` | 6 independent analytical methods pass, exit 0 |
| `python -m unittest discover -s tests -p test_i3_instruments.py -v` | 3 independent hand-authored wrench/cable controls pass, exit 0 |
| `python -m unittest discover -s tests -p test_i3_reproduction_gate.py -v` | 1 gate attack method passes, including missing/failing denominators, exit 0 |
| `pwsh -NoProfile -File tests/test_overnight_controller.ps1` | 23 assertions pass, exit 0; fake workers only, no nested Codex |
| `python -m scripts.verify_i3_reproducibility` | 12 fresh worker processes, 6 artifact families × 2 replicas; all true worker exits 0, complete file inventories identical; both benchmark gates pass |

Primary runtime: Windows CPython 3.12.10. The two independently executed Python 3.11.9 / SciPy 1.17.1 DOP853 tests are in the preceding RES-503 receipt, both pass with no skips. NumPy 2.4.4 is a qualification dependency, not a generator/evaluator requirement. The DOP853 check shares I1 equations; it does not independently derive I2 equations.

The exact fresh-process receipt is `runs/i3/res504/repro/fresh-af9ju1l0/receipt.json`; worker logs and all synthetic outputs remain under that directory. It records actual commands, interpreter, per-source Python SHA-256 and artifact digests. The original same-process checker is preserved as historical code, with its limitation disclosed. The new runner does not trust a benchmark script's unconditional exit zero: it separately inspects 10 positive and 76 fault tallies with their required denominators. No historical I0/I1/I2 output was overwritten.

## Independent mathematical and transduction references

* A rigid platform's two unequal reactions under a known applied couple match `N_left = Mg/2 + tau/(2a)` and `N_right = Mg/2 - tau/(2a)` to 11 decimal places. This independently challenges sign and support load sharing.
* A point-contact block captures or slides according to `v+ = sign(v0) max(0, |v0| - mu g h)` for both directions; normal/tangential impulses satisfy Newton balance and the Coulomb cone, and kinetic energy does not rise.
* Restitution 0, 0.3 and 1 matches rebound velocity, `J = M(1+e)|v-|`, and exact instantaneous impact energy loss, to 12 decimal places.
* Free fall exposes, rather than hides, the symplectic-Euler energy defect: `E0 - E(T) = M g^2 T h / 2`. Halving the step halves this first-order global error. Energy conservation is not claimed exactly from deterministic bytes or momentum closure.
* A rotating body-frame plane's reported normal velocity agrees with an independently differenced geometric gap to 1e-9 m/s. A fixed no-slip pivot has zero contact power while COM pseudo-power is nonzero, falsifying their interchangeability.
* Hand-authored independent plate wrenches verify Fz/Fx, moment-origin depth, COP reconstruction and the exact numerical left/right sum. They do not calibrate a real device or prove the arbitrary rotated-frame case; the admitted fixture uses world-aligned plate axes.
* A radial cable path verifies Euclidean geometry, 1000 ppm drift with an independent native offset, quantization and provider differentiation. A shortening path verifies that a slow reel exposes slack rather than pretending to track the attachment.

These controls supplement the original multibody/jump/support tests, including rank refusal, Jacobian/constraint checks, randomized rigid-foot drops, mirrored contact solutions, flight angular momentum, Newton–Euler balances and step refinement. Some old dynamic checks use broad authored tolerances or shared solver state; they are not independently derived error bounds. Contact-event, coupled angular-momentum and energy behavior outside tested regimes remains a research limitation.

## Per-protocol benchmark accounting

The two final fresh benchmark processes independently reproduce all 10 numerical positives: CMJ, SJ, vest-loaded CMJ, shoulder-shelf jump squat, back squat, deadlift, bench press, floor clean/snatch pulls and power-clean pull-only. Both momentum and admissibility claims pass (20 decisions); bilateral empirical calibration and anatomical sensor-phase claims abstain (20 decisions). The 76 prescribed faults are 10 each for force scaling, NaN force, tensile foot force, candidate tolerance and anatomical labels, plus 26 support omissions. Each gets its expected rejection/invalid decision.

The baseline per-family residuals and decisions are in `baseline-benchmark/benchmark.json`; two final outputs/digests are in the process receipt. Largest baseline endpoint residual: 5.725317497873306e-09 N s. The stricter prefix check adds coverage at distinct frame-end times without changing the existing record-scale tolerance. Counterexamples and numerical scope are detailed in `RES504_EVALUATOR_REPAIR.md`.

The 10 external analytical reference cases are support/free-fall families, not reproductions of human exercise technique. Curated counts/Wilson intervals apply only to the authored controls and their denominators. A coherent fabricated body/force story can still satisfy the admitted whole-system equations; provenance, empirical reference and anatomical/mechanical completeness cannot be inferred from that PASS.

## Source and rendered-artifact findings passed to RES-507

`i2_evaluator.py` now validates nested inputs and event domains before calculations, checks prefix impulse balance and checks impulse admissibility. The original total-endpoint false PASS and malformed-input crashes are repaired with retained negative evidence.

`i2/squat.py` still exposes force-times-COM-velocity under `system_com_work_j`; it needs explicit pseudo-work terminology and an estimator definition. `i2/bench.py` checks loss of bench support mainly inside repetitions and its plate-plus-bench helper omits rack support unless the caller separately establishes rack absence. `i2/jumps.py` models a freely separable bar on a shoulder shelf without hand stabilization. `i2/power_clean.py` clips an over-reach law-of-cosines argument; valid short rack offsets remain refused, but over-reach needs a domain refusal. These do not justify new human/device claims.

Five original SVGs were rasterized at their native dimensions with bundled Node Sharp and visually inspected. Source hashes, preview hashes and findings are in `figure-inspection.json`; previews are in `figure-previews/`. The plotted squat/deadlift oscillations, bench transition/rerack reactions and pull flight/impact spikes are visible. Several scripts clamp forces before drawing, so the rendered caps do not establish actual peak magnitude. RES-507 must inspect raw values and avoid cosmetic smoothing.

Unsupported full power-clean catch, full clean/snatch, hang/block starts, drop/rebound mechanics, rigid-impact landing peak/RFD, anatomical inference from FORCE/LPT and empirical paired/device validity remain refused or NOT_EVALUABLE. Controller/parameter feasibility and the ordinary hand-stabilized bar/bench context still need the RES-507 artifact qualification.

Hosted Windows/Linux CI has not run for I3; local Windows evidence is not a Linux qualification claim. Publication, capability-context review and release acceptance remain subsequent gates. The owner has now selected Apache-2.0, but final legal/content adoption and history disposition are tracked separately in RES-505/506.

Claim ceilings: NUMERICAL_VERIFICATION_ONLY; SIMULATED_MEASUREMENT_ONLY; BIOLOGICAL_VALIDATION=NOT_ESTABLISHED; REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.
