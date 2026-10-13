# I1 movement phases, support topology and event observability (RES-483)

Code:
* latent definitions: `biosyngen/i1/phases.py`;
* sensor-only detectors: `biosyngen_benchmark/detectors.py` (no generator import);
* latent-vs-observed study: `biosyngen/i1/observability.py`;
* tests: `tests/test_i1_jump.py::PhaseTests`, `tests/test_i1_instruments.py`.

## 1. Three event layers, never mixed

| Layer | Source | Example | Evidence class |
|---|---|---|---|
| Latent control | declared excitation schedule | `control_command` = first excitation change | LATENT_CONTROL_HYPOTHESIS. **Not** a movement onset, and not neural truth |
| Latent physical | simulated state and contact | kinetic onset, peak downward velocity, COM reversal, heel-off, takeoff, touchdown | LATENT_PHYSICAL_MODEL_STATE |
| Detector-estimated | FORCE / LPT export only | threshold onset, integrated-velocity reversal, force-threshold takeoff, cable lowest point | DETECTOR_ESTIMATED, under declared conventions |

## 2. Latent predicates (CMJ)

W is the static support force. Takeoff is not a scheduled time anywhere.

| Event / phase | Predicate on model state | Time resolution |
|---|---|---|
| kinetic onset | first recorded sample with \|Fz − W\| > 1e-6 W | ≤ 1 recording step (≤ 1 ms) |
| unweighting | onset → Fz returns to W from below while v_COM < 0 | linear root of exact sample forces (≤ 1 step) |
| peak downward velocity | end of unweighting (dv/dt = 0) | as above |
| braking | → system-COM v_z = 0 | — |
| COM reversal | v_z crosses 0 upward (lowest point) | cubic-Hermite root (exact dv/dt at samples); Hermite–linear gap reported |
| propulsion | reversal → takeoff | — |
| heel-off | heel normal force → 0 (bisection) | 1e-10 s |
| takeoff | toe normal force → 0 (bisection) | 1e-10 s |
| flight | takeoff → foot reaches ground | 1e-10 s; landing **not modeled** |

**SJ:**
* static flexed hold (ankle 68°, knee 100°, hip 90°) → propulsion → takeoff → flight;
* refused if any downward COM velocity exceeds 1e-3 m/s before propulsion (`SJ_COUNTERMOVEMENT_PRESENT`).

**Loaded CMJ:**
* body, bar and system COM are reported separately;
* the bar is a separate coupled body, so bar events can differ from COM events (§5).

## 3. Reference results (hypothetical adult, declared control; SIMULATED ONLY)

**CMJ** (`reference_cmj()`):
* quiet stance 0–0.301 s;
* unweighting 0.301–0.642 s;
* braking 0.642–0.796 s;
* propulsion 0.796–1.045 s;
* flight 1.045–1.533 s (ends at foot–ground contact);
* countermovement depth 0.305 m, peak downward velocity −1.677 m/s, takeoff velocity 2.395 m/s (apex rise 0.292 m);
* heel-off 0.986 s;
* takeoff falls 0.166 s after the last control breakpoint, i.e. it is not scheduled.

**SJ** (`reference_sj()`):
* static hold to 0.329 s, propulsion 0.329–0.530 s, takeoff velocity 2.128 m/s (apex 0.231 m), flight 0.434 s (= 2v/g);
* peak downward velocity ≈ 0 (no countermovement).

**CMJ − SJ:** a takeoff-velocity advantage of 0.27 m/s (apex +6.1 cm) emerges without being imposed. In this tendon-free model the only available mechanism is activation build-up during the countermovement; elastic storage is not represented.

**Model discrepancies, reported and not hidden:**
* minimum CMJ unweighting force ≈ 0 BW (the optimizer released the extensors fully);
* ankle 46° at the lowest point, below the declared passive limit of 48.7°;
* backward COM drift −0.22 m/s at takeoff.

## 4. Observability: detector vs latent (reference CMJ, default simulated devices)

| Latent event | Detector (export only) | Error (ms) | Device sample period (ms) | Status |
|---|---|---|---|---|
| kinetic onset | FORCE \|F−BW\| > 5 SD | +8.0 | 1 | compared; threshold detection lags |
| kinetic onset | same, backdated 30 ms (convention) | −22.0 | 1 | compared; backdating overshoots this model |
| peak downward velocity | integrated FORCE | +1.39 | 1 | compared |
| COM reversal | integrated FORCE | +1.25 | 1 | compared |
| takeoff | FORCE < max(20 N, 5 SD) | +0.30 | 1 | compared |
| COM reversal | LPT (hip belt) provider-velocity reversal | −27.3 | 2 | compared. **Attachment point ≠ COM** |
| COM reversal | LPT lowest displacement | −28.3 | 2 | compared. **Attachment point ≠ COM** |
| takeoff | LPT | — | 2 | NOT_EVALUABLE: a cable cannot observe contact |
| touchdown | FORCE | — | 1 | NOT_EVALUABLE: landing not in export |

**SJ challenge case:**
* Before repair, the integrated-force detector reported a spurious "peak downward velocity" (−3e-5 m/s) and a "reversal" for a jump with no countermovement.
* It now refuses (`NO_SIGNIFICANT_COUNTERMOVEMENT`) unless the dip exceeds 5σ of the integrated-noise random walk, (SD/m)·√(dt·T).
* Regression test: `test_squat_jump_has_no_countermovement_and_detector_refuses_a_reversal`.

**Other refusals exercised by the tests:**
* no threshold crossing (held stance);
* a missing sample in the quiet window;
* a moving (non-quiet) baseline;
* an invalid series.

**Uncertainty sources in the errors:**
* sample period;
* electronic noise and threshold choice;
* clock offset handled by nominal mapping, with drift (35 / −60 ppm) and jitter (2 / 5 µs) left in the residuals;
* plate gain/offset calibration error, which biases weight −0.44% and takeoff velocity +0.14%.

These are SIMULATED_DEVICE_HYPOTHESIS magnitudes, not instrument specifications.

## 5. Exercise × support topology × instruments × events × refusal

| Protocol | Support topology | Model DOF here | FORCE observable | LPT observable | Latent events | Extra measurement needed for validation | I1 status |
|---|---|---|---|---|---|---|---|
| Unloaded CMJ | feet on one plate; lumped legs | 6 | Fz, Fx, My/COP (when loaded), events of §2 except landing | hip-belt cable length; not COM | §2 | synchronized motion capture + plates; per-foot plates for bilateral | IMPLEMENTED; NOT_EMPIRICALLY_VALIDATED |
| Squat jump | as CMJ, static flexed start | 6 | onset, propulsion, takeoff | cable | static hold, propulsion, takeoff | as above | IMPLEMENTED; NOT_EMPIRICALLY_VALIDATED |
| Loaded CMJ (gripped bar) | feet on plate; bar gripped (compliant coupling) | 8 | system Fz (body + bar) | bar-collar cable (3-D, oblique) | §2 + bar velocity reversal ≠ COM reversal | bar kinematics (3-D markers) + plate; grip force unknown | see §6 (control search) |
| Landing / drop jump | impact | — | — | — | — | impact-qualified contact model | NOT_IMPLEMENTED (refused at touchdown) |
| Back squat (reps) | continuous foot support, bar on back | — | — | — | — | squat-specific control and bar contact | NOT_IMPLEMENTED |
| Deadlift | bar–floor support, pre-tension, lift-off | — | — | — | — | bar–floor contact, hand/grip, spine | NOT_IMPLEMENTED |
| Bench press | bench + feet + rack/chest contacts | — | — | — | — | upper-limb model, multiple supports | NOT_IMPLEMENTED |
| Olympic pulls / power clean | multi-contact, catch impact | — | — | — | — | arms, bar rotation, catch impact | NOT_IMPLEMENTED |
| Bilateral asymmetry (any) | two feet / two plates | legs lumped | **not identifiable** from one plate | — | — | two plates, bilateral model | NOT_EVALUABLE |

No phase annotations are produced for NOT_IMPLEMENTED protocols, and no generic force template is used.
