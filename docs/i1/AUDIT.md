# I1 source-level audit of the I0 implementation (RES-481)

**Scope.** Every I0 file was read: `biosyngen/{mechanics,observations}.py`,
`biosyngen_benchmark/core.py`, `scripts/{run_pilot,generate_*}.py`,
`scripts/overnight.ps1`, `tests/*`, `docs/*.md`, I0 `SUMMARY.md`, manifests,
fixtures and the I0 SVG. Line numbers below refer to the hash-verified pristine
snapshot in `runs/i1/res481/i0-source-snapshot/`. Its 13 files were rebuilt by
reversing the I1 edits, and each matches the pre-edit SHA-256 recorded at the start
of this session.

**Claim ceiling.** NUMERICAL_VERIFICATION_ONLY; SIMULATED_MEASUREMENT_ONLY;
BIOLOGICAL_VALIDATION=NOT_ESTABLISHED; REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.

## 1. Baseline rerun, fresh, 2026-10-12, Windows 11, CPython 3.12.10

| Command | Exit | Result |
|---|---|---|
| `python -m unittest discover -s tests -v` (I0 sources unmodified) | 0 | 43 tests, 43 OK (matches the I0 report) |
| `powershell -File tests\test_overnight_controller.ps1` | 1 | **Not reproducible here.** PowerShell 7 (`pwsh`) is not installed on this host, and the script calls `Get-Command pwsh` at line 104. The I0 count of 23 assertions stays LOCAL_AGENT_REPORTED. |
| `python -m scripts.generate_mechanics_fixture` / `generate_observation_fixture` / `generate_benchmark_report` / `run_pilot` | 0 | Every I0 artifact was byte-identical to the frozen record. |

The live Codex controller (`scripts/overnight.ps1`) has only ever run in dry-run
mode, and its own records say CLI Linear access was "Not logged in". Its status is
**LIVE_AUTOMATION_UNQUALIFIED**. Following RES-481 task 5, no time was spent on
controller authentication.

## 2. Severity-ranked defect ledger

Severity key: **P0** means invalid mechanics, an oracle false-accept, or a misleading
scientific claim. **P1** means a scientifically meaningful mechanism is missing.
**P2** means a documentation, visual or reproducibility issue.

| ID | file:function/line | Claim made | Failure mode | Sev | Reproducible falsifier | Corrective action (status) | Acceptance test |
|---|---|---|---|---|---|---|---|
| P0-M1 | `mechanics.py:simulate` L140–210 | "contact-mode transition" | A stance→flight transition is accepted even when COM velocity is ≤ 0, so a point mass "takes off" while moving into its support. | P0 | A 0 N stance for 0.2 s gives takeoff at v = −1.96 m/s, and I0 accepts it. | Takeoff now requires v > 0 (**FIXED**). | `test_i1_p0_audit.test_takeoff_with_downward_com_velocity_is_refused` |
| P0-M2 | `mechanics.py:simulate` and `run_pilot.py:scenario_config` L66–78 | "ideal flight" and "ballistic flight PASS" | Flight duration is a free input. The I0 demonstration world takes off at 1.0453 m/s, so the COM returns to takeoff height after 0.2131 s. Yet "flight" runs 0.41 s and ends 0.396 m *below* takeoff height (COM 0.727 m, versus 0.92 m standing) with zero GRF. With no foot or landing geometry, that segment is physically inadmissible. The evaluator's flight reference at t = 0.6 s (`run_pilot.py` L229) lies after ballistic return, and the I0 evaluator admitted it as PASS. | P0 | The I0 demo scenario is now refused with "descends below takeoff COM height". | A flight longer than ballistic return is refused (**FIXED**). I0 worlds remain reproducible only through the explicit flag `legacy_i0_unchecked_flight=True`, which stamps `LEGACY_I0_UNCHECKED_FLIGHT_NOT_PHYSICALLY_ADMISSIBLE` on the trajectory. | `test_flight_beyond_ballistic_return_to_takeoff_height_is_refused`; `test_flight_within_ballistic_return_is_still_accepted` |
| P0-E1 | `core.py` L95–107, L345, L424, L505–548, L946–955 | "independent numerical reference" | The tolerances (`tolerance_n_s`, `tolerance_n`, cable tolerances, `time_tolerance_s`) travel inside the candidate's own `MeasurementSubmission`, and the evaluator applied them unconditionally. A candidate can therefore buy a PASS for corrupted force by inflating the tolerance. A wide time tolerance also lets `_find_time_index` snap the declared interval onto a different sample. | P0 | +20 N corruption with tolerance 1e6 N·s gives **PASS** in I0. Interval start 0.2 s with a 0.3 s time tolerance snaps to 0.0 s and gives **PASS** in I0. | Evaluator-owned tolerance authority: value tolerances ≤ 1e-9 × signal scale, and time tolerance ≤ 1e-3 sample period. Otherwise the result is `REFERENCE_TOLERANCE_EXCEEDS_EVALUATOR_POLICY` (NOT_EVALUABLE) (**FIXED**). | `test_candidate_cannot_buy_a_pass_by_widening_its_own_tolerance`; `test_candidate_cannot_snap_interval_with_a_wide_time_tolerance` |
| P0-E2 | `core.py:_evaluate_bilateral_force` L552–718 (L684) | "BILATERAL_KNOWN_ANSWER_PASS" | Left/right channels made by multiplying the combined force by an assumed fraction (`observations.py:partition_support_force` L198) passed whenever the "independent" reference used the same fraction. That is circular acceptance of a non-identifiable quantity: fractions 0.5 and 0.7 of the same trace both PASS. | P0 | Both fractions PASS in I0. | A declared partition now yields `BILATERAL_DERIVED_FROM_ASSUMED_PARTITION` (NOT_EVALUABLE). Only separately measured foot channels can reach PASS (**FIXED**). The I0 test expectation of PASS was reversed with an in-file note, and a two-plate positive control was added. | `test_bilateral_channels_manufactured_from_a_split_fraction_never_pass`; `test_benchmark.test_separately_measured_foot_channels_can_pass_a_known_answer` |
| P0-S1 | I0 SVG panel A/B, I0 `pilot-report.md` | "liftoff 0.37s" and "ballistic flight" | The figure labels a *declared schedule boundary* as "liftoff" and plots the COM falling to 0.66 m during "flight" without any warning. | P0 (misleading visual) | Visible in `pilot-figure.svg` panel A. | The frozen I0 artifact is preserved. The I1 report and figures label it LEGACY and use model-derived takeoff (RES-482/487). | I1 figure review |
| P1-1 | `mechanics.py` whole module | n/a | GRF is a prescribed constant per phase (1250 N), and takeoff is a scheduled timestamp. There is no articulation, actuation, activation, foot or contact geometry. | P1 | n/a | Successor: actuated 4-segment planar model with unilateral contact (RES-482). | RES-482 tests |
| P1-2 | `run_pilot.py:scenario_config` | "controlled stance then flight" | No quiet stance (v0 = +0.05 m/s at t = 0), no unweighting or braking, constant force. It is **not a CMJ** and has no emergent phases. | P1 | n/a | RES-483 phase predicates on the I1 model. | RES-483 tests |
| P1-3 | `mechanics.py:_make_sample` L229 | "body, bar, system COM distinct" | The bar is rigidly offset (v_bar ≡ v_body), there is no interaction force, and bar-vs-body event disagreement is impossible by construction. | P1 | n/a | Loaded variant with compliant gripped bar coupling and explicit interaction force (RES-482). | RES-482 tests |
| P1-4 | `observations.py:observe_cable_lpt` L351/L398 | "cable rate" | Rate gets an independent Gaussian stream. A real encoder LPT derives rate from quantized length, so I0's length and rate channels are mutually inconsistent beyond their noise. There is no encoder, no provider filter, no slack, and no 3-D anchor. | P1 | n/a | Encoder-count length, provider-derived velocity, 3-D anchor, slack flag (RES-484). | RES-484 tests |
| P1-5 | `observations.py:observe_force_plate` | "FORCE" | White noise is added straight to N. There is no transducer bandwidth, anti-alias filter, ADC counts, gain/cross-talk, Fx or COP, and no sample-clock jitter. | P1 | n/a | Staged plate chain from the modeled ground wrench (RES-484). | RES-484 tests |
| P1-6 | `core.py:ClockContract.qualified` L49 | "qualified clock" | The candidate self-declares the boolean, and the expected clock also comes from the submission. Clock authority is circular. | P1 | n/a | The I1 evaluator derives timing checks from device-native metadata and data, and treats cross-device sync as NOT_EVALUABLE without a shared trigger (RES-485). | RES-485 tests |
| P1-7 | `run_pilot.py:independent_state_at` L127 | "closed-form reference independent of the mechanics module" | Code-path independent only. It reads the generator's own `Scenario` (the prescribed force inputs) and copies the rigid-bar assumption, so it is a known-answer *implementation* check, not an independent physical oracle. | P1 (labeling) | n/a | The I1 evaluator uses a different formulation: Newton–Euler from exported kinematics, plus truth-free cross-modal invariants (RES-485). | RES-485 tests |
| P1-8 | `core.py:_evaluate_flight` L362–437 | "BALLISTIC_FLIGHT" | It only checks \|F\| ≈ 0 at declared times. It does not check flight-time/impulse consistency or COM trajectory, so the name overstates the evidence. | P1 | n/a | The I1 evaluator separates the zero-support check from the impulse-momentum takeoff estimate (RES-485). | RES-485 tests |
| P1-9 | `core.py:evaluate` | counts | There are per-claim counts but no suite-level discrimination or false-accept/false-reject accounting, and no rule that a refuse-everything policy fails. | P1 | n/a | Challenge-suite confusion matrix with Wilson bounds and a refuse-all baseline (RES-485). | RES-485 tests |
| P2-1 | `run_pilot.py:main` L647–660 | "Two same-seed run directories" | `end-to-end-run-b` is a byte copy of the *same* in-memory bundle `first`. The second build is compared only in memory, inside the same process. | P2 | Code inspection. | The I1 runner executes two separate OS processes and compares on-disk digests (RES-487). | RES-487 |
| P2-2 | `tests/test_overnight_controller.ps1` L104 | "23 assertions pass" | It depends on PowerShell 7, which is absent here, so it was not rerun. | P2 | Exit 1. | Recorded. Not repaired, per scope. | n/a |
| P2-3 | `docs/mechanics.md` | "exact" | True for arithmetic, but the docs don't say the I0 demo world is physically inadmissible after t = 0.583 s. | P2 | n/a | Covered by this audit. The I0 docs are not rewritten. | n/a |

## 3. Before/after evidence

* Before repair: `runs/i1/res481/falsifiers-before-repair.txt`. Of 9 falsifiers, **5 FAIL** on the unmodified I0 code (P0-M1, P0-M2, P0-E1 ×2, P0-E2). The 4 controls pass: corrupted force REJECTED, clean force PASS, wrong anchor REJECTED, correct anchor PASS. This shows the falsifiers can tell a defect from a correct result.
* After repair: `runs/i1/res481/falsifiers-after-repair.txt` (9/9 OK) and `full-suite-after-repair.txt` (53/53 OK; the I0 suite plus the 9 falsifiers plus 1 bilateral positive control).
* I0 preservation: `python -m scripts.i1_i0_regression` produces `runs/i1/res481/i0-regression.json`. Pilot data, figure, report, evaluator input, evaluation report and the res476/res477 fixtures are **byte-identical**. `manifest.json` differs only in `source_sha256`, because the sources changed. `res478` differs only in `findings[3].reason` text; outcome and code are unchanged. The frozen I0 run directory is not rewritten.
* Test-expectation changes: one I0 assertion was reversed (P0-E2, documented in the test). Six I0 test/script worlds gained `legacy_i0_unchecked_flight=True`, with their arithmetic assertions unchanged.

## 4. MODEL_GAPS matrix (I0 → I1 target)

| Capability | I0 | I1 target issue | I1 status at end of run |
|---|---|---|---|
| Articulated segments, joints | none | RES-482 | see `MODEL_SPEC.md` |
| Actuation / activation dynamics | none | RES-482 | see `MODEL_SPEC.md` |
| Emergent GRF and takeoff | prescribed | RES-482 | see `MODEL_SPEC.md` |
| Unilateral contact, COP, friction | none | RES-482/484 | see `MODEL_SPEC.md` |
| Landing / impact | refused | RES-482 | NOT_IMPLEMENTED (refused) |
| Physical CMJ phases | none | RES-483 | see `PHASE_OBSERVABILITY.md` |
| Bar–body coupling | rigid offset | RES-482 | see `MODEL_SPEC.md` |
| Plate transduction chain | additive noise | RES-484 | see `VANDV.md` |
| LPT encoder/geometry chain | 2-D, independent rate noise | RES-484 | see `VANDV.md` |
| Truth-free physical oracle | none | RES-485 | see `VANDV.md` |
| Parameter provenance and sensitivity | none | RES-486 | see `parameter_provenance.json` |
| Empirical human / device validation | none | RES-486 | NOT_EVALUABLE (no qualified paired reference) |

## 5. Follow-on priorities

1. RES-482: forward dynamics with KKT-constrained unilateral contact, Hill-type joint torque actuators driven by first-order activation, and model-derived takeoff.
2. RES-483: latent phase predicates on model states, plus observable and detector counterparts.
3. RES-484: staged plate chain driven by the modeled wrench, and an encoder LPT with 3-D geometry and independent clocks.
4. RES-485: Newton–Euler oracle from exported kinematics, truth-free impulse/flight invariants, mutation suite and accounting.
5. RES-486: provenance ledger, sensitivity sweeps, and an inspected-abstract plausibility comparison (not validation).
6. RES-487: two-process reproducibility, figures, final review.
