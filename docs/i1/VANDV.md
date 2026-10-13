# I1 verification and validation memo (RES-487)

**Bottom line:**
* The mechanics, contact, actuation and instrument software is **VERIFIED_NUMERICALLY** against independent invariants and closed forms.
* The neuromechanical model is **MODELED_NOT_EMPIRICALLY_VALIDATED**.
* Empirical biological, device-metrology and real paired FORCE/LPT validation is **NOT_EVALUABLE**: there is no qualified reference.

## 1. Fresh test campaign (2026-10-12, Windows 11)

| Command | Interpreter | Result |
|---|---|---|
| `python -m unittest discover -s tests -v` | CPython 3.12.10, standard library only | **96 tests: 94 OK, 2 skipped** (SciPy absent), exit 0. Log: `runs/i1/res487/final-test-suite.txt` |
| `py -3.11 -m unittest tests.test_i1_solver_crosscheck -v` | CPython 3.11.9 + SciPy 1.17.1 | 2/2 OK. Log: `runs/i1/res487/scipy-crosscheck-py311.txt` |
| `pwsh -File tests\test_overnight_controller.ps1` | — | **NOT RUN.** PowerShell 7 is not installed; the I0 controller claim stays LOCAL_AGENT_REPORTED |

**Composition of the 95 tests:**

| File | Tests | Scope |
|---|---|---|
| I0 suites | 43 | unchanged in substance |
| `test_benchmark.py` addition | 1 | new two-plate positive control (one I0 expectation reversed) |
| `test_i1_p0_audit.py` | 9 | P0 falsifiers |
| `test_i1_mechanics.py` | 9 | mass matrix, Newton/Euler, statics, actuators, KKT |
| `test_i1_jump.py` | 11 | emergent contact, conservation, phases, refinement, scaling, refusal; plus the ground-penetration test |
| `test_i1_instruments.py` | 13 | filter, ADC, COP, LPT geometry, slack, detectors, observability |
| `test_i1_benchmark.py` | 8 | independence, canary, controls, accounting, full suite |
| `test_i1_solver_crosscheck.py` | 2 | SciPy DOP853 |

## 2. Numerical verification of the mechanics

| Check | Reference (independent of the solver path) | Result |
|---|---|---|
| Mass matrix | ½q̇ᵀMq̇ vs segment-velocity kinetic energy; symmetric positive definite | agreement < 1e-10 relative |
| Newton/Euler, free body | finite-difference momentum rates vs −Mg and −Mg·x_com, for random states and torques | < 1e-4 |
| KKT contact | constrained point accelerations = 0; dP/dt = GRF + weight | < 1e-8 / < 1e-3 N |
| Statics | GRF = W; COP = x_COM | exact to 1e-9 |
| Work–energy identity (whole jump) | KE + PE − ∫(joint + damping + contact power) | CMJ 8.1e-7 J; loaded 1.1e-6 J; SJ 5.6e-7 J |
| Constraint drift | heel/toe positions | ≤ 2.5e-13 m |
| Flight | v_z = v0 − gt; H about COM constant | < 1e-7 m/s; < 1e-6 |
| Stance impulse–momentum | trapezoid ∫(Fz − W) vs MΔv | < 0.1% |
| Refinement | rtol 1e-6 vs 1e-8 | takeoff time and velocity Δ < 1e-4 |
| Scaling | mass and strength ×1.2 | identical kinematics; GRF ×1.2 |
| Independent solver | SciPy DOP853, rtol 1e-11 | states agree within 1e-5 |
| Event location | bisection on contact-force roots | 1e-10 s |

## 3. Independent evaluator V&V (RES-485)

* 24 adjudicated cases from four families:
  * benchmark closed forms;
  * the I0 prescribed-force family;
  * I1 CMJ, loaded CMJ and SJ;
  * 18 mutations.
* **81/81 claim instances agree with adjudication; 0 false accepts; gate PASS.**
* The refuse-everything and accept-everything baselines **FAIL** the gate.
* Hidden-truth canary: 4 poisoned fields refused, outcomes unchanged.
* Report: `runs/i1/res485/benchmark-suite.json`.

## 4. Defects found by verification during this run (not hidden)

| # | Where | Found by | Effect | Fix |
|---|---|---|---|---|
| 1 | I0 mechanics and evaluator (P0-M1/M2/E1/E2) | `test_i1_p0_audit.py` | false accepts; physically inadmissible flight | repaired; see `AUDIT.md` |
| 2 | I1 bar coupling damper | Euler-balance test | non-central force pair, ~30 N·m spurious moment | damper applied at a single material point |
| 3 | I1 landing detection | grid search | foot never cleared, body fell through the floor | penetration-based touchdown |
| 4 | I1 "takeoff" | trace inspection | knee-lock artifact | passive-limit penalty in control search; toe-tap classified `TAKEOFF_NOT_SUSTAINED` |
| 5 | Detector | SJ challenge case | spurious reversal for a jump with no countermovement | noise-derived significance floor |
| 6 | Evaluator | positive control | false REJECT at GRF kinks (Richardson bound only) | curvature bound added |
| 7 | Evaluator | force-in-flight mutation | abstained instead of rejecting | cross-modal check over the declared flight interval |
| 8 | I1 contact | SJ flight duration 0.78 s vs 0.45 s ballistic | head and trunk passed through the floor in flight (only foot points were checked) | `BODY_GROUND_CONTACT_NOT_MODELED` event; SJ control re-searched (new SJ: takeoff 2.128 m/s, flight 0.434 s = 2v/g) |
| 9 | Figure (a) | visual inspection | stick figures stretched 2.3× horizontally | equal-aspect layout |
| 10 | V&V figure | visual inspection | clipped labels, overlaps | layout fixed |

## 5. Validation status per subsystem

| Subsystem | Status |
|---|---|
| Rigid-body mechanics, contact, event location | IMPLEMENTED_AND_NUMERICALLY_VERIFIED |
| Activation / torque actuation | IMPLEMENTED_AND_NUMERICALLY_VERIFIED (equations); MODELED_NOT_EMPIRICALLY_VALIDATED (physiology) |
| CMJ phases | emergent and verified within the model; durations are control-dominated (EMPIRICAL_EVIDENCE §4) |
| SJ, loaded CMJ | IMPLEMENTED; NOT_EMPIRICALLY_VALIDATED |
| Landing, squat, deadlift, bench, Olympic lifts, bilateral | NOT_IMPLEMENTED / NOT_EVALUABLE |
| Plate and LPT chains | IMPLEMENTED_AND_NUMERICALLY_VERIFIED as stated hypotheses; SENSOR_METROLOGY = NOT_EVALUABLE |
| Evaluator | VERIFIED_NUMERICALLY on adjudicated controls; it cannot validate realism |
| Human validity | BIOLOGICAL_VALIDATION=NOT_ESTABLISHED; REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED |
