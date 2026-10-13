# RES-497 power clean: scoped subset with a computed catch refusal

_The power-clean pull is simulated. Turnover, catch and recovery are **NOT_IMPLEMENTED**, for a computed kinematic reason. No catch is synthesized._

---

## 🧮 Why the catch is refused

A front-rack catch places the bar on the anterior shoulders, about 0.08–0.15 m from the shoulder joint centre. The model's arm is a lumped upper arm (0.33 m) plus a rigid forearm + hand (0.34 m) with **no wrist joint**. [power_clean.py](../../biosyngen/i2/power_clean.py) computes the elbow flexion this geometry needs (law of cosines; verified against forward kinematics in the tests):

| Grip-to-shoulder offset (m) | Required elbow flexion (rad) | Within declared limit (2.6 rad)? |
| --- | --- | --- |
| 0.08 | 2.90 | No |
| 0.10 | 2.84 | No |
| 0.12 | 2.78 | No |
| 0.15 | 2.68 | No |
| 0.20 | 2.52 | Yes, but the bar is then held in the hands ahead of the shoulders, not racked |
| 0.25 | 2.36 | Yes (same) |

Real lifters reach the rack through wrist extension and 3-D arm abduction and external rotation. Those degrees of freedom are absent, together with scapular elevation and foot relocation. Inventing a non-anatomical rack shelf, or a force template, would be a cosmetic catch, which the issue forbids.

## ✅ What is delivered

| Scope | Content |
| --- | --- |
| Pull (simulated) | `scope="PULL_ONLY"` (default) runs the verified RES-496 clean-pull mechanics under the POWER_CLEAN_DUAL_V1 identity. It reports REP_START, LIFT_OFF, FIRST_PULL_END, SECOND_PULL_ONSET (transition end, or first-pull end if there is no knee re-flexion) and EXTENSION_END (peak bar velocity) from bar, contact and joint mechanics. |
| Turnover, catch, recovery (refused) | TURNOVER_ONSET, CATCH_CONTACT, RECOVERY_END and REP_END are returned as NOT_IMPLEMENTED with the computed reason. `scope="FULL"` raises `CATCH_NOT_IMPLEMENTED` and lists the missing degrees of freedom. |
| Force-template shortcut | `catch_impulse_from_force_template` always refuses (`FORBIDDEN`). |

Tests (`tests/test_i2i_power_clean.py`, 6 tests) check:

- the feasibility formula against independent forward kinematics;
- infeasibility at shoulder offsets and reachability at hand offsets;
- the full-scope refusal with its reason and the missing degrees of freedom;
- the force-template refusal;
- that the model has no CATCH contact;
- the pull event order;
- that no NOT_IMPLEMENTED event appears among the reported events.

~~~powershell
python -m unittest tests.test_i2i_power_clean -v
~~~

## 🔓 What would unlock the catch (future work, not claimed)

1. A hand segment with a wrist joint (or a rigid forearm–hand with wrist extension), so the grip can rest at the shoulders within anatomical ranges.
2. A front-rack bar–trunk contact (the engine already supports body-frame unilateral contacts), with catch impulses from `resolve_impact`.
3. A receiving-squat strategy and foot relocation (separate plate re-contact).
4. Qualified reference data for catch impulses and the receiving depth.

Claim ceilings: NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY, BIOLOGICAL_VALIDATION=NOT_ESTABLISHED, REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.
