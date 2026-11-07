# RES-598 scientific credibility evidence and limitations

This campaign extends the existing independent evaluator. It does not replace
its schema or promote generator/controller labels into independent references.
The 10 x 12 matrix reports whole-domain readiness separately from bounded
numerical subclaims. Its current 90 pending, 20 not implemented and 10 not
evaluable inventory cells are not qualified passes. A fresh report replaces the
default candidate movement cell with measured FAIL when final recovery collapses;
successful integration duration alone remains EVIDENCE_PENDING.

## Continued mechanical invariant checks

The existing strict I2 submission schema now optionally accepts the claim
`ANGULAR_MOMENTUM_CLOSURE`. Its independent checker integrates moments of
external forces and impulses, including gravity, about the fixed initial
system COM. Angular momentum is computed as `sum(I*omega + r cross m*v)`.
The numerical budget is the existing relative momentum budget (1e-6) times
system weight, observation duration and initial system radius of gyration.
It is a bounded planar discretization check, not a spatial model or an
empirical error budget. Every interval-end prefix is tested so intermediate
errors cannot disappear through final cancellation. Coordinate translations
do not change the conditioning length or tolerance. Candidate tolerances
remain forbidden.

Analytical tests include static support, translated coordinates, an angular
impulse, wrong inertia, off-centre support with an otherwise passing linear
balance, intermediate spin cancellation, absent observations and a forged
tolerance. Run `python -m unittest discover -s tests -p test_res598_angular_credibility.py -v`.

The fresh movement runner requests this claim for every implemented baseline
and the bounded rebound candidates. A failed angular invariant makes the
corresponding implemented movement's MULTIBODY cell FAIL; a passing isolated
invariant does not qualify the entire domain. The runner also records actual
joint work, contact dissipation, stored energy, raw energy residual and
projection correction separately. The DROP_JUMP CONTACT_ENERGY cell remains
NOT_EVALUABLE without an independent error budget and final recovery. Hosted
Windows/Linux jobs now execute the scientific report, in addition to the
historical regression/falsifier suite. The report retains biological and
real-device abstentions and all 120 primary cells.

## Executable evidence

```
python -m scripts.run_movement_credibility --out <fresh-directory>
python -m unittest discover -s tests -p test_res598_contact_credibility.py -v
python -m unittest discover -s tests -p test_res598_scientific_matrix.py -v
```

The runner retains the original ten numerical families and 76 authored faults,
then evaluates the 0.5 s articulated compliant candidate at 0.4, 0.2 and 0.1 ms.
Every result includes context, population, oracle, input conditions, measured
findings/error, fixed tolerance basis, uncertainty, falsifiers and submission
hash. A bounded subclaim never qualifies complete movement execution.
The first development run took 197 s and wrote 92,766 bytes. It was a dirty-tree
development run at parent `4e3dea5c60caf527ff473cc1736d487f0bdfcd66`, not final-SHA
acceptance. Exact clean-head reruns must identify themselves in their manifests.

| Retained numerical family | Final momentum residual (N s) |
|---|---:|
| CMJ | 1.620498180088248e-11 |
| SJ | 5.428244659322345e-13 |
| Loaded CMJ | 1.964402251974917e-11 |
| Loaded jump squat | 3.6892459573389935e-12 |
| Back squat | 5.442563066893058e-12 |
| Deadlift | 5.230516020304776e-11 |
| Bench press | 2.036582014142141e-11 |
| Clean pull | 1.0667799976715742e-11 |
| Snatch pull | 1.3801793041778865e-11 |
| Power-clean pull only | 9.286460489477122e-12 |

For the bounded compliant candidate, maximum prefix momentum residuals at
0.4/0.2/0.1 ms were 4.8317e-12/9.3223e-12/4.4309e-11 N s.
The inherited, evaluator-owned tolerance was 0.000367749375 N s; all three
had zero tensile intervals/impulses and zero friction-cone violations.
Anatomical phases and bilateral calibration claims remained NOT_EVALUABLE.

The distinct RK4 test reference integrates a directly written scalar contact
ODE with a 10 microsecond step. It shares no production constitutive function,
linear solver or stepping method. Its own step refinement is checked, and a
wrong-mass force ledger must fail. These are independent calculations, not
independent acceptance by a separate author of this final revision.

## Capability and limitation register

| Primary movement | Current capability / qualification boundary |
|---|---|
| CMJ | Retained articulated numerical model; rigid landing peaks unqualified |
| Back squat | Retained loaded/multiple-repetition model; controller ringing remains |
| Deadlift | Retained floor/grip support transfer; feet exclude bar-floor support |
| Bench press | Retained feet/bench/chest/rack model; stable unrack/rerack and complete anatomical qualification pending |
| SJ | Retained static flexed start; countermovement refusal preserved |
| Drop jump | New experimental compliant rebound; pre-release platform support absent and final recovery fails |
| Power clean | Pull-only; spatial receiving/catch/recovery absent |
| Power snatch | Full spatial overhead receiving model absent |
| Clean | Full-depth front-rack receiving model absent |
| Snatch | Full-depth overhead receiving model absent |

All retained loaded, pull-only, guided, source-only and 500 Hz legacy variants
remain in the existing coverage/export contracts. Full-lift refusals are not
removed. No RSI, whole-system power from feet-only GRF, muscle/neural truth,
spatial shoulder loads or human reliability is established.

## Data rights and external-reference ledger

No real recordings were downloaded, accessed or redistributed. Public metadata
review is distinct from file-level data admission.

| Candidate / source | Rights and scientific disposition |
|---|---|
| [AddBiomechanics](https://addbiomechanics.org/download_data.html) | Public page declares CC BY 4.0; per-file cohort/protocol/calibration and paired LPT matching not established. NOT_ADMITTED for this campaign. |
| [OpenBiomechanics](https://www.openbiomechanics.org/) | Current metadata fetch failed (HTTP 502); previously noted restrictions not independently reverified. NOT_ADMITTED. |
| [Elite snatch/clean paper](https://pubmed.ncbi.nlm.nih.gov/41352184/) | Relevant literature context; no freely licensed matched raw paired FORCE/LPT corpus admitted. |
| [LPT study](https://pubmed.ncbi.nlm.nih.gov/29851551/) | Publication about cross-exercise measurement validity/reliability; no participant recordings or transferable calibration admitted. |
| Other candidates listed in RES-598 | File rights, cohort, instrumentation and validation splits remain unverified; NOT_ADMITTED. |

No aggregate literature mean/SD was converted into manufactured participants or
ground truth. Agreement, bias, limits of agreement, ICC/CV/SEM/MDC, calibration
coverage and human test-retest reliability are NOT_EVALUABLE without matched
licensed experimental observations and preregistered uncertainty budgets.

## Figures and interpretation

```
<isolated-python> -m pip install matplotlib==3.11.2
<isolated-python> -m scripts.plot_movement_credibility --out <fresh-directory>
```

Figures use native dual-1kHz clocks without same-index synchronization. FORCE
is feet-only; LPT shows the declared cable attachment, distinct from latent
whole-system COM and joint states. The existing exercise-specific cable
builders must be used for bench, deadlift, squat and pulls. An initial plotting
run incorrectly used the jump default on those families; it was retained as
failed plotting evidence and regenerated after correction, not used as device
evidence. All charts show actual termination, including candidate collapse.
Rigid-contact acquisition spikes are unqualified for peak-force/RFD inference.
The figure suite is descriptive and does not provide missing anatomical or
experimental validation.

The corrected eleven rendered PNGs were inspected, including labels, units,
legends, FORCE/LPT time axes and the terminal collapse trace. Bar-collar motion
now appears in the bench figure; hip-belt motion is retained for unloaded/vest
jumps. The legacy rigid-contact spikes, squat/deadlift ringing and failed
candidate landing are retained visible limitations, not human reference features.
The bounded figure set is about 2.6 MB; the initial incorrect figures remain
local failed plotting evidence and are not published.

## Claim authority

NUMERICAL_VERIFICATION_ONLY
SIMULATED_MEASUREMENT_ONLY
BIOLOGICAL_VALIDATION=NOT_ESTABLISHED
REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED

Exact-head Windows/Linux CI, final evidence review and owner acceptance remain
separate gates. Neither this report nor an implementer's tests authorize merge,
release or Done transitions.
