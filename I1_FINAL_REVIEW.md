# I1 final review: Opus 5.5 scientific audit and neuromechanical rebuild (RES-480)

> Historical I1 record, written before the RES-489 evaluator re-audit. The source audit linked from RES-489 found additional false-acceptance and resolution gaps; do not cite this file's old PASS counts as the current evaluator acceptance. Current contract and claim ceilings: [`docs/i2/RES489_EVALUATOR_POLICY.md`](docs/i2/RES489_EVALUATOR_POLICY.md).

**Date:** 2026-10-12
**Root:** `D:\projects\BioSynGenBenchmark`
**Linear:** RES-481…RES-487 are In Review (not Done); the parent is In Review.

**Outcome:** a research-grade *method* and a numerically verified mechanistic candidate. It is **not** a validated model of human physiology or of any device.

**Claim ceiling:** NUMERICAL_VERIFICATION_ONLY · SIMULATED_MEASUREMENT_ONLY · BIOLOGICAL_VALIDATION=NOT_ESTABLISHED · REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.

Supporting detail: `docs/i1/AUDIT.md`, `MODEL_SPEC.md`, `PHASE_OBSERVABILITY.md`, `EMPIRICAL_EVIDENCE.md`, `VANDV.md`, `RESULTS.md`, `parameter_provenance.json`.

## WHAT_WAS_FOUND

I0 reran cleanly (43/43 tests; artifacts byte-identical). It contained five P0 defects:

1. **Downward takeoff accepted.** Takeoff was allowed with a downward COM velocity.
2. **Physically inadmissible flight.** The I0 demonstration's 0.41 s "flight" outlasts its 0.213 s ballistic return and ends 0.40 m below takeoff height. The evaluator still admitted it as "ballistic".
3. **Candidate-controlled tolerances.** A candidate could widen its own tolerance (or time tolerance) to turn a corrupted force into a PASS.
4. **Circular bilateral PASS.** Left/right channels made by an assumed split passed against a reference built from the same split.
5. **Misleading figure.** "liftoff" was drawn at a scheduled time, while the COM fell 0.4 m during "flight".

P1 gaps:
* prescribed constant force with no articulation, actuation or contact;
* not actually a CMJ;
* rigid bar;
* LPT rate noise independent of length;
* no plate transduction chain;
* self-declared clocks;
* code-path-only "independence";
* no suite-level accounting.

P2:
* `run-b` was a byte copy of the same in-memory build;
* the PowerShell-7 controller test could not be rerun.

## WHAT_WAS_FIXED

* All five P0 defects have regression falsifiers: 5/9 fail on I0 and 9/9 pass after repair (`runs/i1/res481/`).
* I0 outputs are preserved: the legacy worlds are explicitly labelled, and the frozen I0 run directory was not rewritten.
* One I0 test expectation (the circular bilateral PASS) was reversed and documented.

## WHAT_WAS_IMPLEMENTED

* **`biosyngen/i1`:**
  * actuated planar foot–shank–thigh–HAT model, plus a gripped bar on a compliant central/same-point coupling;
  * Thelen-form activation driving Hill-type joint torque generators;
  * rigid unilateral Coulomb contact through KKT, with heel-off, takeoff, slip, inelastic heel strike, and touchdown/body-contact refusals;
  * adaptive DOPRI5 with event bisection to 1e-10 s;
  * declared open-loop control strategies chosen by a declared max-jump search;
  * latent phase predicates;
  * staged plate (filter/ADC/clock/calibration) and 3-D cable-LPT (encoder/reel slack/provider velocity/clock) models;
  * public export.
* **`biosyngen_benchmark`:**
  * strict-schema evaluator: Newton/Euler impulse balances with data-derived tolerances, contact, ballistic flight, energy ledger, activation scope, cable geometry, clocks, plate checks, and empirical refusals;
  * sensor-only detectors;
  * benchmark-authored closed-form controls and 16 mutation operators;
  * adjudicated accounting with Wilson intervals and refuse-all/accept-all baselines.
* **Scripts:** `run_i1` (one command, two-process reproducibility), `i1_benchmark_suite`, `i1_provenance_sensitivity`, `i1_optimize_control`, `i1_i0_regression`.

## WHICH_REAL_WORLD_PHASES_EMERGE

From model state and contact force, never from scheduled labels:

* **CMJ:** quiet stance → unweighting (Fz < W) → braking → COM reversal → propulsion → heel-off → takeoff (toe force = 0) → flight. Takeoff falls 0.166 s after the last control change.
* **SJ:** static hold → propulsion → takeoff → flight, with no countermovement.
* **Loaded CMJ:** body, bar and system COM evolve separately.
* **Countermovement advantage:** CMJ − SJ is +0.27 m/s takeoff velocity (+6.1 cm apex) without being imposed. Its direction agrees with inspected abstracts (Bobbert 1996; Bobbert & Casius 2005); magnitudes are not comparable.

## WHICH_PHASES_ARE_ONLY_ASSUMED

* **Phase durations are control-dominated.** Unweighting stays at 0.340–0.342 s under every parameter variation, because the declared knee-command time ends it.
* **Movement onset coincides with the declared command.** The "control command" is a hypothesis, not neural truth.
* **Landing is not modeled.** Runs end at foot contact. Arms, bilateral asymmetry and tendon elasticity are absent.
* **The optimized open-loop control is fragile.** A ±20 ms timing shift breaks the jump.

## NUMERICAL_VV

96 tests: 94 OK, 2 SciPy tests skipped on 3.12 but passing on 3.11.

| Check | Result |
|---|---|
| Energy identity | ≤ 1.1e-6 J |
| Constraint drift | ≤ 2.5e-13 m |
| Ballistic flight / angular momentum about COM | exact to tolerance |
| Tolerance refinement | Δ < 1e-4 |
| Mass/strength scaling | kinematics invariant |
| SciPy DOP853 cross-check | < 1e-5 |
| Invalid worlds refused | COM outside foot; capacity exceeded; too weak to take off; toe tap; body penetrating ground |

Ten verification-found defects are listed in `VANDV.md` §4.

## INDEPENDENT_EVALUATOR_VV

* 24 cases across 4 families: 81/81 adjudicated instances agree, 0 false accepts, gate PASS.
* Refuse-all and accept-all baselines FAIL.
* Canary: 4 poisoned fields refused, outcomes unchanged.
* AST check: no generator import.
* Two evaluator defects (one false reject, one abstain-instead-of-reject) were found and fixed.

## EXTERNAL_EMPIRICAL_EVIDENCE

* Three PubMed abstracts were inspected and used only qualitatively.
* No qualified paired human FORCE/LPT/kinematics data exist here, so **NOT_EVALUABLE**.
* The minimum acquisition experiment is specified in `EMPIRICAL_EVIDENCE.md` §7.
* No parameter is SOURCE_VERIFIED. Winter and Thelen values were recalled, not inspected.

## NOT_ESTABLISHED

* human neuromuscular validity, EMG/drive/muscle forces;
* inter-individual variability;
* device fidelity (ForceDecks/AMTI/GymAware/WinLaborat);
* real paired FORCE/LPT;
* landing;
* other exercises;
* bilateral forces.

Known model discrepancies:
* unweighting force reaches about 0 BW;
* ankle reaches 46° at the bottom;
* backward drift (−0.22 m/s) and trunk rotation in flight;
* actuator capacity scaled for plausibility.

## REPRO_COMMANDS

```
python -m unittest discover -s tests -v
python -m scripts.run_i1 --replicate
python -m scripts.i1_benchmark_suite
python -m scripts.i1_provenance_sensitivity
python -m scripts.i1_i0_regression
py -3.11 -m unittest tests.test_i1_solver_crosscheck -v   # optional, needs SciPy
```

Bundle digest: `ddbd7282e3b1fbf1675921ecc3507b3f2c9c4a2ef9e089cc63333c7aebdec53d`. Per-file hashes are in `runs/i1/res487/run/manifest.json` and `runs/i1/res487/source-sha256.txt`.

## OPEN_BLOCKERS

1. **PUBLIC_REPO=BLOCKED_PENDING_OWNER_CONFIRMATION.** A public-GitHub-publication instruction was added to the RES-480/RES-487 issue text during the run. It was not part of the chat request that launched this session, and AGENTS.md says no public repository. Nothing was published. Publishing needs an explicit confirmation in the session, plus a licence decision (none chosen).
2. **Empirical validation** needs the acquisition experiment in `EMPIRICAL_EVIDENCE.md` §7.
3. **Controller tests** need PowerShell 7 to rerun.
4. **Scientific improvements:**
   * tendon/biarticular actuation, so capacity need not be scaled for plausibility;
   * robust (feedback) control;
   * a landing contact model;
   * bilateral legs with two plates.
5. **Owner acceptance** of RES-481…487 is pending (In Review).

**SOURCE_HASHES:** `runs/i1/res487/source-sha256.txt`.
**PLATE2BAR_MUTATIONS=0.** The evidence folder, DynamisBench and historical corpora were not accessed.
