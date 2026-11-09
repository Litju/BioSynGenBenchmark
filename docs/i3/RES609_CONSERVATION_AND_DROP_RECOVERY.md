# RES-609 — angular/energy invariants and drop-jump recovery

Base: PR #10 head `db4c58f90ebc209786e34a45b8ccc9405215ad74` (stacked on PR #9).
Branch `work/RES-609-conservation-drop-recovery`. This is implementation and
numerical evidence. It is not independent scientific acceptance and not a
completed drop jump.

Commits: `75c45dcdb2722056e466f91fb86ce2aa8d18206d` (angular projection policy),
`3fc92c402dcb10a4c545c41a04cae4571c110bb1` (implicit compliant contact and independent
energy ledger). Clean receipts were produced at `3fc92c4` (`source_dirty=false`):
`runs/i3/res609/angular_repair.json`, `runs/i3/res609/energy_and_recovery.json`.

## 1. Angular momentum: root cause and repair

**Root cause.** The historical `MASS_METRIC_V1` drift correction projects body
COM positions with no force. That changes angular momentum by Σ m (δr × v) with
no recorded impulse. For the CMJ at 1 ms the unrecorded change totals
−0.0223 kg m²/s. The V1 prefix residual was 0.0258 kg m²/s, against a budget
of 0.000806.

**Two rejected formulations** (preserved here because they were tested):

* An extra position row Σ m δr × v = 0 conserved L about a fixed point. It did
  so by moving the whole-body COM perpendicular to the momentum, which is a
  kinematic inconsistency. The free-flight analytic test caught it: |ΔL_com|
  was 0.159.
* With COM-relative velocity, that row is linearly dependent for a rigidly
  rotating chain, so positions alone cannot carry the fix. The projection did
  not converge.

**Adopted: `MASS_METRIC_ANGULAR_MOMENTUM_RESTORING_V2`** (opt-in; V1 stays the
default). The V1 position projection is kept. Then the minimal mass-metric
velocity correction removes exactly that unrecorded ΔL, while keeping every
active joint and contact row and the linear momentum fixed. This is a
conserving projection onto the momentum level set. It is numerical, not
physiological. Its energy is part of the separately reported projection work.
Where no admissible direction exists (all supports stuck) the change is not
erased: it accumulates in `angular_restoration_ledger`.

**Independent results.** The unchanged evaluator `i2_evaluator._angular_balance`
was used, with an unchanged budget, at the original 1 ms step
(`python -m scripts.res609_angular_repair`):

| Fixture | V1 max prefix | V2 max prefix | Budget | V1 | V2 | Spin falsifier |
|---|---:|---:|---:|---|---|---|
| CMJ | 0.0257631 | 4.249e-05 | 8.059e-04 | REJECTED | PASS | REJECTED |
| SJ | 0.0272460 | 1.501e-05 | 6.976e-04 | REJECTED | PASS | REJECTED |
| LOADED_CMJ | 0.0213100 | 4.307e-05 | 9.376e-04 | REJECTED | PASS | REJECTED |
| LOADED_JUMP_SQUAT | 0.0231537 | 1.0445e-04 | 1.0626e-03 | REJECTED | PASS | REJECTED |
| BACK_SQUAT | 4.064e-04 | 1.34e-07 | 1.708e-03 | PASS | PASS | REJECTED |
| DEADLIFT | 7.888e-04 | 1.17e-07 | 2.497e-03 | PASS | PASS | REJECTED |
| BENCH_PRESS | 2.929e-03 | 9.33e-06 | 6.224e-03 | PASS | PASS | REJECTED |
| CLEAN_PULL | 9.980e-04 | 5.60e-05 | 1.746e-03 | PASS | PASS | REJECTED |
| SNATCH_PULL | 2.8067e-03 | 1.318e-04 | 1.513e-03 | REJECTED | PASS | REJECTED |
| POWER_CLEAN_PULL_ONLY | 1.070e-03 | 8.4e-07 | 1.653e-03 | PASS | PASS | REJECTED |

V1 passes 5/10 and V2 passes 10/10. The spin mutation is rejected in 10/10,
and all 10 V2 baselines are eligible. Linear momentum and contact
admissibility still pass. The largest whole-body COM change V1→V2 is 4.5 mm
(SJ). The bench has 178 steps with no admissible direction; its unrestored
remainder is −2.2e-9 kg m²/s, recorded. Analytic oracle
(`tests/test_res609_angular_projection.py`): a free-flying spinning 3-link
chain under gravity gives V1 |ΔL_com| > 1e-4 and V2 < 1e-10, and joint
closure holds to < 1e-8 m.

## 2. Drop-jump energy: root cause and repair

**Exact decomposition of the RED** (0.5 s drop, 0.4 ms, V1). The legacy
residual is 63.3005 J. It splits as follows. Contact force times
contact-point displacement is **+36.55 J** (a passive contact should do ≤ 0)
while the ledger booked 76.38 J of dissipation, a 112.93 J mismatch. The
tangential part is +51.15 J of friction work against 54.38 J booked. Then
symplectic-Euler numerical dissipation is −64.51 J, and joint power quadratured
at interval-start rates contributes +14.88 J. Cause: the regularized Coulomb
friction has an effective viscosity μN/ε ≈ 3×10⁴ N s/m on a ~5 kg effective
foot mass. Integrated explicitly, that sits beyond or near the stability bound.

**Repair: `IMPLICIT_DISCRETE_GRADIENT_V2`** (opt-in; the explicit scheme stays
the default). The contact law is solved at the interval-end velocity, with a
discrete-gradient elastic force −(U(g1)−U(g0))/(g1−g0), so elastic work equals
−ΔU exactly. The fixed point uses Newton on a finite-difference linearization
of one full articulated step, Broyden corrections and a true-step check. A
relative fixed-point residual ≤ 1e-7 is required, and nonconvergence raises
`ImplicitContactNotConverged`. Frames now carry the discrete joint work
τ·Δθ and exact per-contact F·Δp.

**Independent ledger** (`i2_evaluator.discrete_energy_ledger`). It recomputes
contact work from exported forces and next-frame contact points. It applies
the passive sign test, compares declared and recomputed dissipation, and forms
the symplectic ledger ΔE − W_joint − W_contact + D_num − W_proj, with
D_num = Σ ½(m|Δv|² + IΔω²) taken from exported states. The ledger is reported
as NOT_EVALUABLE (`NO_PREDECLARED_ENERGY_BUDGET`); it is not a PASS.

| Contact / projection | dt (ms) | Legacy residual (J) | W_contact (J) | D (J) | D_num (J) | Closure (J) | Passive | Declared D | Angular |
|---|---:|---:|---:|---:|---:|---:|---|---|---|
| V1 / V1 | 0.4 | 63.3005 | +36.548 | −36.548 | 64.439 | −0.0666 | REJECTED | REJECTED | REJECTED |
| V1 / V1 | 0.2 | 28.2570 | +6.133 | −6.133 | 28.785 | −0.0276 | REJECTED | REJECTED | REJECTED |
| V1 / V1 | 0.1 | 12.9131 | −7.357 | 7.357 | 13.173 | −0.0133 | PASS | REJECTED | REJECTED |
| V2 / V1 | 0.4 | 4.1156 | −18.218 | 18.218 | 6.931 | −0.0499 | PASS | PASS | REJECTED |
| V2 / V1 | 0.2 | 1.9958 | −18.488 | 18.488 | 3.422 | −0.0259 | PASS | PASS | REJECTED |
| V2 / V1 | 0.1 | 0.9809 | −18.643 | 18.643 | 1.700 | −0.0136 | PASS | PASS | REJECTED |
| V2 / V2 | 0.4 | 4.1050 | −18.218 | 18.218 | 6.931 | −0.0500 | PASS | PASS | PASS |
| V2 / V2 | 0.2 | 1.9906 | −18.488 | 18.488 | 3.422 | −0.0259 | PASS | PASS | PASS |
| V2 / V2 | 0.1 | 0.9783 | −18.643 | 18.643 | 1.700 | −0.0137 | PASS | PASS | PASS |

Linear momentum passes in every row. The closure residual is first order
(ratio ≈ 1.9 per halving). The remaining legacy residual equals the
interval-start joint-power quadrature error, which also falls at first order.
Elastic analytic oracle (undamped, frictionless bounce): under V2,
E + D_num = E0 to round-off, with D_num = 0.288/0.155/0.081 J at
1/0.5/0.25 ms. V2 is therefore accountable and never creates energy, but it is
not energy-exact; symplectic Euler itself dissipates. V1 keeps elastic energy
bounded, but its work channel misses the identity by 0.335 J. Sliding block
(μ 0.8): V2 stops within 2 % of v0²/(2μg). At a 4 ms step V1 does not settle.
Tamper controls are rejected: forces ×1.5 (declared mismatch), sign-flipped
forces (passive), a missing frame (INVALID), nonfinite work (INVALID).

## 3. Final drop recovery: FAIL, with an exact mechanical blocker

0/3 full 3 s trials completed (0.4 ms, V2 contact + V2 projection). Each ends
with `BODY_GROUND_CONTACT_NOT_MODELED: left_knee, right_knee`:

| Recovery control | Knee–floor time (s) | Takeoff centroidal L (kg m²/s) | Touchdown ankle − COM x (m) |
|---|---:|---:|---:|
| Historical stages | 1.2796 | −13.714 | −0.503 |
| Attempt 1: CMJ flight, absorb, stand + COM balance | 1.1684 | −13.714 | −0.508 |
| Attempt 2: + balanced braking/propulsion (CMJ ankle strategy) | 1.3596 | −10.255 | −0.375 |

The blocker is the reactive stance. It leaves the ground with centroidal angular
momentum of −10 to −14 kg m²/s (the CMJ: −0.1). Flight cannot remove that, so
the body pitches and the feet land 0.38–0.51 m behind the COM. Joint-space
landing control then cannot recover. The smallest missing prerequisite is a
stance controller that regulates centroidal angular momentum during the
~0.13 s rebound contact (for example whole-body momentum control or an optimal-
control rebound), followed by a fresh landing. Under AGENTS.md the two new
repair attempts end this line of work. These witnesses falsify these controls
only; they do not show that no controller can recover. No body reset, imposed
impulse, early termination or knee-contact suppression was used. RSI is not
exported, and the existing 1 kHz FORCE/LPT remains diagnostic, not admitted
DROP_JUMP source.

## Reproduction

```
python -m unittest tests.test_res609_angular_projection tests.test_res609_energy_ledger -v
python -m scripts.res609_angular_repair --out <fresh dir>
python -m scripts.res609_energy_and_recovery --out <fresh dir>
```

17 RES-609 tests pass. These regressions pass unchanged: `test_i2c_multibody` (35),
`test_res597_*` (13), `test_i2d_jumps` (15), `test_res598_*`, `test_i2j_benchmark`,
`test_i2a_evaluator_adversarial`, `test_i3_evaluator`, `test_i3_work_reporting`,
`test_i3_contact_reporting`. Hosted exact-head CI is recorded in Linear when observed.

All code, tests and outputs are owned and manufactured. No participant data,
holdouts, Plate2bar edits or third-party code are included.

NUMERICAL_VERIFICATION_ONLY
SIMULATED_MEASUREMENT_ONLY
BIOLOGICAL_VALIDATION=NOT_ESTABLISHED
REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED
