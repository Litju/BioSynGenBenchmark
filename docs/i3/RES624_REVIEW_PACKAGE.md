# RES-624 final scientific acceptance hardening — review package

Claim ceiling: **NUMERICAL_VERIFICATION_ONLY**, **SIMULATED_MEASUREMENT_ONLY**,
**BIOLOGICAL_VALIDATION=NOT_ESTABLISHED**, **REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED**.
This is a static, contact-only overhead receiving hold in the declared Human V2 computational model. It is not a
dynamically completed clean/snatch, not a biologically calibrated human, and not an empirical athlete/device result.
Dynamic Olympic catch/recovery remains RES-611; the broader independent movement benchmark remains RES-612.

Status: **implementer self-review and scoped evidence. INDEPENDENT_FINAL_SHA_REVIEW=PENDING.**
No merge, release, tag, Done transition or owner scientific acceptance is claimed.

## Source and evidence identity

| Item | Value |
|---|---|
| Repository | https://github.com/Litju/BioSynGenBenchmark |
| Branch | `work/RES-624-grasp-contact-recovery` |
| Base | `82ff59b121ed92bd4424c38a1c1bb1573d00b4aa` (`work/RES-618-human-v2-anatomy`, PR #13) |
| Code head qualified by the committed evidence | `7c6c20d85c8d3d3487e0d09e42b3c8d0103c60fa` |
| Evidence commit (this package + regenerated receipts) | `b71382439dbccc1bdd407e6aa7d0d7accbb5bbe1` |
| PR | https://github.com/Litju/BioSynGenBenchmark/pull/15 (draft, stacked on #13) |
| Runtime | Python 3.12.13, MuJoCo 3.3.7, SciPy 1.17.1, NumPy 2.4.4, Windows 11 (local qualification) |
| Solver | elliptic cone, impratio 10, Newton, tolerance 1e-10, 200 iterations, timestep 1 ms |
| Qualification receipts | `worktree_clean=true`, `head=7c6c20d` (code) |

The exact-head hosted Windows/Linux CI result for the final source SHA is recorded on PR #15. CI results at
`807d4f8` (run 42816090274, 11/11 green) are **historical and superseded** by the evaluator repairs below.

## What was repaired (Gates A–E)

### Gate A — grasp-contact identity and actual supporting force
- New versioned independent contract `biosyngen_benchmark/grasp_contract.py`
  (`RES624_GRASP_CONTACT_CONTRACT_V1`) declares the eight permitted palm/finger/thumb interfaces and binds each geom to
  its compiled body. A test binds the registry to the compiled Human V2 model; the evaluator never infers a grip from
  an `l_`/`r_` name prefix.
- Each contact is checked independently: barbell body, `bar_shaft` geom, registered human body, registered pad geom,
  contact-point binding to that pad body, finite force/unit normal, unilateral compression, declared friction cap and
  Coulomb cone.
- Support is the integrated hand-on-bar wrench against the bar's exact discrete balance
  (`m dv/dt = m g + F_hand + F_other`, `dH/dt = M_hand + M_other` about the bar COM), not a contact count. The hand
  impulse must carry ≥ 98 % of the bar weight impulse; each hand must carry ≥ 2 % (asymmetric sharing is allowed);
  force and moment wrench residuals must be ≤ 0.1 %.
- New outcomes: `GRASP_IDENTITY_UNADMISSIBLE`, `GRASP_GEOMETRY_INFEASIBLE`, `GRASP_SLIP`, `GRASP_UNSUPPORTED`,
  `GRASP_NOT_BILATERAL`.
- Ten fabricated false-support scenes keep the bar wrench, the system force balance, contact admissibility and the
  static verdict intact while changing *which body carries the load*: external 5 % hand load, unilateral/zero-load
  hand, forearm/thigh/torso support, no permitted grasp contact, forged pad body, forged contact point, swapped contact
  sites, reversed hand forces. All are refused for the declared reason.
- Evidence: `runs/i3/res624/qualification/receiving_qualification.json` (`adversarial`), tests
  `tests/test_res624_grasp_identity.py` (11 tests).

### Gate B — relative bar orientation and slip
- New `bar_frame` export (position, rotation, local-y shaft axis). The evaluator tracks, per hand, SO(3) shaft tilt,
  full relative geodesic, shaft-axis twist, perpendicular relative angular velocity, axial shaft slip, load-bearing
  contact-site migration and COM displacement. Angles use acos-on-trace wrapping.
- Declared bounds: tilt ≤ 2°, perpendicular relative rate ≤ 0.2 rad/s, axial slip ≤ 2 mm, load-site drift ≤ 5 mm,
  COM displacement < one shaft radius (14 mm); the shaft axis must be within 5° of the hand's declared local-y grip
  axis. Nominal 10 s maxima: tilt 0.43°, axial 0.56 mm, site drift 3.89 mm, relative rate 0.012 rad/s.
- Shaft-axis twist is reported but not gated for the declared axisymmetric unplated shaft/sleeve body; it is gated
  only when the exported inertia is not axisymmetric about the shaft axis. A pose-only control proves a 30° roll
  produces no relative-pose failure (`relative_pose_controls` in the report); a *forged* roll with no exported torque
  is additionally refused end-to-end by the angular attitude/momentum oracle.
- Negative controls: 10° tilt at nearly zero COM translation, 5 mm axial slip, a constant hand-frame offset, a
  progressive hand-frame rotation and a zero-load hand. Small legitimate motion remains inside the bounds and the
  nominal hold qualifies.
- Evidence: `relative_bar_pose` per case; tests `tests/test_res624_relative_pose.py` (8 tests).

### Gate C — nominal versus anthropometric scope
- New `biosyngen_benchmark/qualification_scope.py` derives three typed verdicts:
  `NOMINAL_RECEIVING_HOLD_QUALIFIED`, `ANTHROPOMETRIC_GENERALIZATION_QUALIFIED`,
  `OVERALL_DECLARED_SCOPE_QUALIFIED` (plus the nominal-only and not-qualified forms).
- Nominal qualification depends on the 0.3/2/10 s mandatory cases, the frozen RED refusal, all adversarial refusals,
  both pose-positive controls and every non-anthropometric case expectation. A required anthropometric failure does
  **not** corrupt the nominal claim; an explicitly exploratory subject cannot invalidate either claim.
- Generalization requires both preregistered subjects present, qualified, certified, evidence-consistent
  (support/contacts/displacement/penetration/saturation/identity/pose) and identity-consistent: declared
  mass/stature, exported human mass and distinct compiled parameter-set hashes.
- Falsifiers: 70 kg fail, 80 kg fail, missing subject, mass mismatch, stature mismatch, model-mass mismatch, shared
  parameter set, unqualified subject relabelled as qualified, exploratory failure, missing RED/adversarial/positive
  controls. All fail the appropriate gate without corrupting a qualified nominal claim.
- Evidence: `verdicts` in the qualification report; tests `tests/test_res624_scope.py` (11 tests).

### Gate D — Windows/Linux negative-case comparison
- New `biosyngen_benchmark/failure_signature.py` (producer) and
  `biosyngen_benchmark/negative_signature.py` (stdlib comparator) define a predeclared signature: expected/observed
  category and primary reason, first failure event (category/reason/time), the required contact-loss or
  actuator-saturation onset (50 ms tolerance), the longest qualified pre-failure interval with support ratio/shares
  and bar displacement (1 % / 1 mm), and model/solver/source identity.
- Secondary post-impact failures, trajectories and force chatter are reported but not gated. A different primary
  mechanism, a different required onset beyond 50 ms, different pre-failure support, a different source/solver/model
  identity or a missing signature fails the comparison. Frame-level adversaries compare expected/observed/primary
  reason, not merely a generic refusal.
- Evidence: `failure_signature` per negative case, `signature` per adversary; tests
  `tests/test_res624_platform_negative.py` (10 tests) and the existing comparator suite.

### Gate E — independent angular-momentum oracle
- `biosyngen_benchmark/rigid_body_check.py` now integrates the body-frame attitude equations with an independent
  fourth-order Runge-Kutta reference using the exported state, body inertia and world-frame loads, and compares the
  predicted world angular momentum and attitude with the export. The momentum tolerance is 1e-7 N m s (nominal window
  residual 1.73e-9, 58× margin); the attitude residual (engine first-order quaternion update, 2.4e-6–6.0e-6 on valid
  perturbed/10 s/subject holds) is a gross-inconsistency gate at 5e-5 rad, not the fault-sensitive criterion.
- Independent analytic references (SciPy DOP853, rtol 1e-12): torque-free rotation, constant world torque, contact
  moment with a known lever arm, time-varying world inertia orientation and global world-frame rotation invariance.
- Six injected faults are rejected with >10× margin: removed external torque, corrupted contact application point,
  changed inertia tensor, reversed contact moment, undeclared angular impulse, altered angular velocity. A uniform
  inertia scale or uniform ω scale in *torque-free* motion is a physical similarity and is not detectable from
  angular momentum alone; the mass/inertia identity check and the torque-present faults cover it.
- The older derived discretization bound remains as a supplementary receipt (`bar_angular_bound`), no longer the gate.
- Evidence: `gateF/numerical_verification.json`, `runs/i3/res624/qualification` per-case `bar_angular_reference`;
  tests `tests/test_res624_angular_reference.py` (11 tests).

## Final qualification outcomes

`runs/i3/res624/qualification/receiving_qualification.json` at code head `7c6c20d`:

- 12/12 preregistered case expectations; 22/22 adversarial refusals; 2/2 pose-positive controls qualified.
- Verdicts: `NOMINAL_RECEIVING_HOLD_QUALIFIED`, `ANTHROPOMETRIC_GENERALIZATION_QUALIFIED`,
  `OVERALL_DECLARED_SCOPE_QUALIFIED`.
- Nominal hold: support impulse ratio 1.00035 (0.3 s), 1.00008 (2 s), 1.00001 (10 s); per-hand shares
  0.4996/0.5008; force wrench residual 5.4e-16; moment residual 5.5e-9; active contacts 8–10; minimum per-frame
  support 0.9983; bar displacement in hand 0.28/1.22/1.35 mm.
- Loads: 15 kg and 25 kg qualified (bar mass, subtree mass and engine gravity bias agree). Friction 0.6 qualified.
  Bar push 40 N/50 ms and pelvis push 60 N/100 ms qualified.
- Subjects: 70 kg/1.70 m and 80 kg/1.80 m qualified with subject-own-hand relations; exported human mass equals the
  declared mass; distinct parameter-set hashes.
- Frozen RES-618 RED: reproduced and refused (bar drop 0.3760531196714312 m, 0 hand contacts, static
  `STATIC_GRASP_INFEASIBLE`). Original script and evidence untouched.
- Release: pre-release hold qualified for 0.997 s (support 1.0002, displacement 0.735 mm); first required failure
  `GRASP_SLIP:CONTACT_LOST` at 1.603 s.
- Insufficient arm capacity: refused `ACTUATOR_INFEASIBLE` with saturation from the first qualified-window frame.
- Incorrect bar inertia: refused by the independent angular oracle (residual 7.43e-6 N m s = 74× tolerance); the
  supplementary derived bound also rejects it (excess 7.32e-6 > 4.905e-8).

False-acceptance denominator: 22 fabricated false-support/false-physics controls + the frozen RES-618 RED + 2
mandatory physical negatives (release, capacity) = **25 refused invalids, 0 false acceptances**; 12/12 required case
expectations and 2/2 pose positives preserved. Frame-level fabrications are labelled as ledger attacks; they preserve
the bar wrench, system force balance, admissibility and static verdict and differ only in the claimed load path.

## Numerical, convergence and energy evidence

`runs/i3/res624/gateF/numerical_verification.json` at `7c6c20d`:

| Check | Result |
|---|---|
| Bar linear momentum | max step residual 1.70e-16 N s (bar has no joints; closes to machine precision) |
| Whole-system linear momentum | max step 6.64e-6 N s (bound 9.32e-5), cumulative 6.20e-8 N s (bound 2.80e-4) |
| Bar angular momentum (independent RK4) | 1.734e-9 N m s (1e-7 bound); attitude 3.822e-7 rad (5e-5 bound) |
| Supplementary derived bound | residual 4.779e-7, excess 0 (retained as historical/derived receipt) |
| Contact impulses (0.3 s) | hand-on-bar 58.852 N s vs weight 58.860; floor 279.571 vs system weight 279.585 |
| Mechanical energy | signed residual −7.167e-5 J on 0.8453 J gross flux (−0.00848 %); rotor KE and trapezoid work included |
| Energy refinement | 7.167e-5, 3.590e-5, 1.797e-5 J at 1/0.5/0.25 ms; observed order 0.997/0.998 |
| Bar trajectory refinement | 1.537e-5 m (1 vs 0.5 ms), 7.930e-6 m (0.5 vs 0.25 ms), decreasing |
| Angular momentum refinement | 1.734e-9, 6.615e-10, 1.780e-10 N m s at 1/0.5/0.25 ms |
| Deterministic replay | bitwise identical on one platform |
| Solver tolerance 1e-8 | qualified |
| Pyramidal cone | `NUMERICAL_NOT_QUALIFIED`; qualification is **elliptic-cone only** |
| Mass/inertia identity | bar mass 20 kg and declared principal inertia agree |

Support and contact plots: `runs/i3/res624/gateDE/receiving_hold.svg` (hand support ratio and bar displacement for
the nominal, push, 25 kg, friction and release holds); the qualification report carries the per-case support,
contact-count, pose and position evidence.

## Cross-platform comparison

The Windows/Linux comparator consumes the exact Git HEAD, MuJoCo/SciPy/NumPy versions, the categorical verdicts, the
positive numeric fields within declared tolerances, and the negative failure signatures above. Tolerances:
displacement 1 mm, support/floor ratio 1 %, energy relative 5e-3, trajectory 1 mm, failure onset 50 ms, pre-failure
support 1 %, pre-failure displacement 1 mm. Bitwise equality is not required. The former negative-case behaviour
(generic refusal equality only) is superseded.

## Reproduce

```text
python -m scripts.res624_receiving_qualification --out runs/i3/res624/qualification
python -m scripts.res624_numerical_verification --out runs/i3/res624/gateF
python -m scripts.res624_receiving_campaign --out runs/i3/res624/gateDE
python -m scripts.res624_red_canary --out runs/i3/res624/canary/red_canary.json
python -m scripts.res624_anthropometric_diagnosis --out runs/i3/res624/gateG
python -m unittest tests.test_res624_acceptance tests.test_res624_grasp_identity tests.test_res624_relative_pose \
  tests.test_res624_angular_reference tests.test_res624_scope tests.test_res624_platform_negative tests.test_res624_ci_separation
```

The hosted workflow (`.github/workflows/verification.yml`) additionally runs the complete stdlib suite on Windows and
Linux, MuJoCo/SciPy on both, the paired generator/platform jobs, the RES-624 qualification and the exact-reference
platform comparison, all at the exact PR head SHA (no branch fallback).

## Historical and superseded receipts

- `runs/i3/res624/gateA`, `gateB`, `gateC`: preserved pre-repair diagnosis, collision-policy and isolated-grasp
  evidence; not modified.
- `runs/i3/res618/fullbody_receiving_failure.json` and `scripts/res618_fullbody_qualification.py`: frozen RED witness,
  untouched.
- The `807d4f8` review package and CI run 42816090274: historical. Its evaluator treated any `l_`/`r_` body as a hand,
  did not gate support force, did not track relative orientation/axial slip, excluded anthropometric cases from the
  aggregate and compared negative cases only by generic refusal. All four P1/P2 gaps are repaired above.
- Superseded, never-committed pre-fix outputs recorded in the prior package (invalid): qualification receipt
  `646e1f41…`, numerical receipt `9c4a6b83…`.

## Explicit limitations and required independent review

- Single synthetic anthropometric family (70/80 kg, 1.70/1.80 m); no population generalization, no soft tissue, no
  muscle activation or real athlete/device validation. The typed verdict says so explicitly.
- Static overhead hold only; no lift, catch, clean, snatch, recovery or dynamic stability claim.
- Elliptic-cone contact model only (impratio 10, Newton, tol 1e-10). The same hold under a pyramidal cone is
  `NUMERICAL_NOT_QUALIFIED` and is not called contact-model independent.
- The whole-system angular balance is reported but, as in the pre-repair evaluator, is not a gating condition; the
  fabricated external-support scene deliberately relies on the gated checks (bar wrench, system force, admissibility,
  support polygon) and is refused by the hand-support gate. This is disclosed rather than treated as a physical
  trajectory.
- A reported shaft roll or pose offset that is not driven by an exported torque is refused by the independent angular
  oracle as an inconsistent export; this is stronger than the relative-pose criterion, not a relaxation of it.
- Energy residual is a first-order integrator/solver remainder, not eliminated.
- The pose solver reports `optimizer_success=False` (kinematic residual ~0.07–0.10); admissibility rests on the
  equilibrium certificate and contact checks, not optimizer convergence.
- **Independent final-SHA review is not performed here. The implementing agent's review is not independent
  acceptance. `INDEPENDENT_FINAL_SHA_REVIEW=PENDING`.** Owner merge, release, scientific acceptance and Done remain
  withheld.
