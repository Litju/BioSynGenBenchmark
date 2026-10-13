# RES-494 deadlift: pre-tension, bar-floor support transfer and return

_A floor-supported barbell is lifted through a revolute grip by a bilateral human with a lumped arm chain. The bar–floor contact is unilateral and outside the force plates._

---

## 🔗 Topology

- **Arm chain** (commit `f0c7ee7`). `build_human(arms=True)` adds one lumped upper arm and one forearm/hand segment (both arms in the sagittal plane). Their Winter mass fractions are moved out of the HAT, so total mass is unchanged. Shoulder and elbow get Hill actuators.
- **Bar.** `LoadKind.GRIP_BAR` holds the bar with a revolute GRIP joint at the hands. It optionally adds a frictional bar–floor contact under the bar axis at the plate radius.
- **Setup.** [deadlift.py](../../biosyngen/i2/deadlift.py) solves knee and hip so that straight arms (declared absolute arm angle −0.10 rad, hands slightly behind the shoulder) reach the bar over the midfoot. The setup is exactly static: the feet carry body weight and the floor carries the bar (verified to 10⁻⁶ N).

## ⚙️ Mechanics, not triggers

**Pre-tension.** A declared feedforward load share ramps from 0 to 1 while the posture reference stays fixed. The plate force rises by ≈ the bar weight (978 N for a 981 N bar) while the bar–floor reaction falls by the same amount. The bar does not move (< 10⁻⁶ m). Plate force rise therefore precedes bar motion as a physical consequence: lift-off comes 0.57 s after pre-tension onset.

**Lift-off** is the instant the bar–floor reaction reaches zero and the contact opens. No position or force threshold is used.

**Later events** come from the bar and knee state, then the bar–floor touchdown on return:

- the bar passing knee height;
- lockout (bar velocity ≈ 0 near its peak);
- descent onset;
- bar–floor return, then a reset ramp.

**Plates and system force.** Plates measure foot contact only. Before lift-off they read body weight, not body plus bar, so a constant loaded-mass assumption is false. `system_force_from_plates` refuses (`NOT_EVALUABLE`) unless the bar–floor reaction is supplied independently.

## ✅ Verification (`tests/test_i2f_deadlift.py` 10 tests, `tests/test_i2f_arm_chain.py` 3 tests)

| Check | Result |
| --- | --- |
| Exact static setup (feet = body weight, floor = bar weight) | Pass |
| Pre-tension: plate rise + floor drop | Within 3 % of bar weight; bar stationary |
| Plate rise before lift-off | ≈ bar weight (± 5 %) |
| Lift-off | Equals the contact opening; full phase order holds |
| Momentum closure with feet and bar–floor as external supports | Pass, 10⁻⁷ |
| No penetration (bar and body) | Pass |
| No tensile contact | Pass |
| Impact work on return | ≤ 0 |
| 320 kg bar | `LOAD_NOT_LIFTED` (the floor reaction never reaches zero) |
| Plate force at setup | Body weight |
| Plate force at lockout | System weight (± 2 %) |
| Plate-only system wrench | Refused |
| Bar-collar LPT onset | Never before lift-off; threshold lag 39 / 107 / 149 ms at 0.1 / 1 / 3 mm |
| Misrouted hip-belt LPT | False onset 0.52 s before lift-off at encoder resolution |
| Slow reel (0.2 m/s) | Slack flagged during the descent; none with the default reel |
| Arm chain | Conserves total mass |
| Gripped bar without arms | Refused |
| Held bar with load share 1 | Exactly static; not static with share 0 |

Evidence: [summary.json](../../runs/i2/res494/summary.json) and [deadlift.svg](../../runs/i2/res494/deadlift.svg) are byte-identical in two fresh processes.

~~~powershell
python -m unittest tests.test_i2f_deadlift tests.test_i2f_arm_chain -v
python -m scripts.run_i2_deadlift --out runs/i2/res494
~~~

## 🚧 Limitations and claim ceiling

- **Rigid bar, rigid grip.** Bar whip, plate rolling, grip slack and hook-grip compliance are NOT_IMPLEMENTED. The force rise before motion therefore comes from load transfer only, not from bar elasticity.
- **No shin–bar contact.** The bar path is not constrained by the shins, and the setup is deeper (thigh nearly horizontal) than a typical conventional deadlift because of the lumped arm and trunk geometry.
- **Arms lumped and planar.** No scapular motion.
- **Pre-tension is declared.** The pre-tension ramp is a declared control hypothesis.
- **Not evaluable empirically.** All values are uncalibrated, and empirical comparison is NOT_EVALUABLE.

Claim ceilings: NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY, BIOLOGICAL_VALIDATION=NOT_ESTABLISHED, REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.
