# RES-495 bench press: supine upper-body pressing with bench, foot, chest and rack supports

_A supine lifter presses a gripped bar through repetitions. Its weight is shared by a bench, two feet on separate plates and, when racked, J-hooks. The force-plate channels see the feet only._

---

## 🛏️ Topology

[bench.py](../../biosyngen/i2/bench.py) uses the RES-494 arm chain. Nothing is borrowed from the jump or squat phase engines.

| Element | Representation |
| --- | --- |
| Trunk | Supine (HAT angle π/2), on a bench plane through two posterior points (scapular at shoulder level, pelvis) that form one friction patch |
| Legs | Articulated; each foot flat on its own plate, so each leg is a closed zero-mobility chain between bench and floor |
| Arms | Lumped shoulder + elbow chain with a revolute grip. Pressing capacities are declared separately (both arms: shoulder flexion 240, elbow extension 180 N·m). The hanging-arm defaults cannot hold 60 kg once the forearm tilts. |
| Chest | Unilateral bar–trunk contact; its normal is the trunk's anterior axis (body frame) |
| Rack | J-hooks as a finite ledge (engine commit `cae12d8`) over the forehead, 0.15 m headward of lockout. Side entry below the ledge top is refused (`EDGE_ENTRY_NOT_MODELED`, `6fe4c2e`). |

**Setup.** The bar rests on the hooks and the posture is exactly static:

- feet + bench = body weight;
- hooks = bar weight.

**Repetition sequence**, as a declared control hypothesis:

1. pre-tension (load share);
2. unrack along a Cartesian path;
3. lowering along a straight Cartesian J-path to the lower chest (minimum-jerk, resolved by closed-form two-link IK; `1e6c04b` adds per-stage reference functions);
4. optional pause on the chest;
5. press back to lockout;
6. lockout hold;
7. after the last rep, rerack onto the hooks and release.

## 📏 Latent events

Events come from contact modes and bar mechanics:

| Event | Definition |
| --- | --- |
| Unrack / rerack | Hook contact opens / closes |
| Lowering onset | Bar v < −0.02 m/s |
| Chest touch | Chest contact closes |
| Bottom reversal | Bar velocity crosses 0 |
| Lockout | Bar ≈ stationary in the upper half of its travel |

Repetitions need at least 0.05 m of travel; smaller lockout wobbles are not reps.

**Per-repetition refusals:**
- no chest touch;
- chest impulse > 5 N·s (`BAR_BOUNCE`, declared);
- hooks loaded during the rep (`SUPPORT_INCOMPLETE`);
- a bench contact lost during the rep (`BENCH_CONTACT_LOST`);
- not completed.

Set-level bench contact releases and impacts are listed in `support_events`.

## ✅ Verification (`tests/test_i2g_bench_press.py`, 10 tests)

| Check | Result |
| --- | --- |
| Exact static setup with separate bench / foot / hook reactions | Pass; bench > 3× feet |
| 60 kg × 2 reps | Unrack → lower → chest touch → press → lockout → rerack; travel > 0.4 m, touch < 5 N·s, no bench loss, no hook contact |
| Momentum closure with feet, bench and hooks as external supports | Pass, 10⁻⁷ |
| No penetration (every contact) | Pass |
| No tensile contact | Pass |
| Feet-only plates | < 30 % of system weight; `system_kinetics_from_plates` refuses without the bench reaction; feet + bench = system weight ± 3 % |
| Bar-collar LPT span vs cable geometry | Equal within 2 mm |
| LPT span vs system-COM excursion | Clearly different |
| Wrist / shoulder internal force, frontal-plane bar tilt | Refused (`NOT_EVALUABLE`) |
| No bench | Refused at setup (`SUPPORT_INCOMPLETE`) |
| Hooks in the bar path | Rep or run refused; never a valid rep |
| 0.8 s pause | Bar stays on the chest > 0.6 s, rep valid |
| 150 N·m leg drive | Pelvis lifts off the bench; reported |

Evidence: [summary.json](../../runs/i2/res495/summary.json) and [bench_press.svg](../../runs/i2/res495/bench_press.svg) are byte-identical in two fresh processes.

~~~powershell
python -m unittest tests.test_i2g_bench_press -v
python -m scripts.run_i2_bench_press --out runs/i2/res495
~~~

## 🧪 Findings during development (kept, not hidden)

- **Joint-space interpolation sweeps the bar through the chest.** The first version used joint-space references, which drove the bar into the chest (26 N·s bounce). It was replaced with Cartesian bar-path references.
- **Hanging-arm elbow capacity is too low for pressing.** With 120 N·m both-arm elbow extension, the bar fell mid-press. A pressing-specific capacity hypothesis is declared instead.
- **A saturated shoulder flexor can lift the rigid trunk.** With stiff press gains, the saturated shoulder flexor's reaction lifted the upper back off the bench. The declared press tempo was slowed (1.5 s) and the gains moderated.
- **Leg drive lifts the pelvis.** Leg drive as hip-extension torque lifted the pelvis in this model, so it is a negative control, not a default.
- **Pelvis lift during unrack.** During the unrack the pelvis lifts off the bench briefly. This is reported as a support event; it occurs outside the repetitions.

## 🚧 Limitations and claim ceiling

- **Planar, lumped arms.** There is no abduction, no scapular retraction, no wrist and no spine arch; the trunk is rigid.
- **Chest model.** The chest is a plane through one point.
- **Bar path is declared.** The straight-line J-path is declared, not measured.
- **Calibration.** Capacities, gains and the bench height are ASSUMED_UNCALIBRATED. Empirical comparison is NOT_EVALUABLE.

Claim ceilings: NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY, BIOLOGICAL_VALIDATION=NOT_ESTABLISHED, REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.
