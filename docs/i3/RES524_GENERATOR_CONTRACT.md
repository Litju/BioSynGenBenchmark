# RES-524 generator contract and Plate2bar handoff

This contract freezes the supported **BSMB-GENERATOR-1.1** candidate. The exact
commit, hosted CI and final readiness receipt belong to RES-524 in Linear.
It supersedes the old *generator* three-case/metadata-only pilot limits, while
retaining both earlier blocker dossiers and every historical negative.
Plate2bar integration remains deferred. RES-524 is not Done.

## Scope and registry

The retained RES-490 registry snapshot has 11 exercise rows. Its governance
and application test inventory describes downstream software tests, not extra
biomechanical generators. The following crosswalk exhausts those rows and the
current I2 builders; `guided_vertical` is an additional analytical control.

| CLI protocol | Registry / modeled topology | Source generation | Physical reference and limits |
| --- | --- | --- | --- |
| `guided_vertical` | Ideal two-mass vertical guide | SUPPORTED | Exact mathematical start/bottom/end; no human anatomy |
| `cmj` | COUNTERMOVEMENT_JUMP, bilateral foot support, hip belt | SUPPORTED | Unweighting/braking/reversal/propulsion, release/flight/landing; rigid landing peak/RFD refused |
| `sj` | SQUAT_JUMP, held squat, hip belt | SUPPORTED | Static hold/propulsion/flight/landing; no invented countermovement or downstream registered SJ method |
| `loaded_cmj` | LOADED_CMJ, welded vest, hip belt | SUPPORTED | Carried vest is distinct from a bar-attached loaded trial |
| `loaded_jump_squat` | LOADED_JUMP_SQUAT, separable shoulder shelf, collar | PARTIAL | Shelf release/recapture retained; ordinary hand stabilization is unimplemented |
| `back_squat` | BACK_SQUAT_DUAL_V1, shoulder shelf, bilateral feet | SUPPORTED | Descent/reversal/ascent/threshold completion; no anatomical lockout claim |
| `deadlift` | DEADLIFT_DUAL_V1, revolute grip, bar-floor support | SUPPORTED | Pre-tension, STICK/SLIDE/OPEN, lift-off/return; foot Fz excludes bar-floor reaction |
| `bench_press` | BENCH_PRESS_DUAL_V1, grip/bench/hooks/chest/feet | PARTIAL | Rep and support transitions exported; stable supine support throughout unrack/rerack is unqualified |
| `clean_pull` | OLYMPIC_PULL_DERIVATIVE_DUAL_V1 / CLEAN_PULL_DUAL_V1 | SUPPORTED | Floor pull, modeled knee passage/velocity peak/return; joint phases unobservable with FORCE/LPT |
| `snatch_pull` | OLYMPIC_PULL_DERIVATIVE_DUAL_V1 / SNATCH_PULL_DUAL_V1 | SUPPORTED | Declared planar wide-grip projection; no spatial technique validation |
| `power_clean_pull` | POWER_CLEAN_DUAL_V1, PULL_ONLY | PARTIAL | Uses retained clean-pull mechanics with 60 kg default; turnover/catch/front rack absent |

Full clean/snatch/power-clean catch, hang/block starts, drop/rebound mechanics,
bar bending/whip, compliant landing, anatomical phase detection and general
spatial motion are UNSUPPORTED. Explicit CLI refusals include `power_clean`,
`snatch`, `hang_pull`, `block_pull`, `drop_jump`, `rebound_jump`. Other unknown
protocols/parameters fail closed. A supported source model is not empirical
human validity, a complete exercise-technique model, or downstream admission.

## Configurable acquisition

```powershell
python -m scripts.generate_paired_corpus --protocol deadlift --count 1 --seed 524 --out runs/i3/res524/working/my-deadlift
python -m scripts.generate_paired_corpus --config docs/i3/RES524_CORPUS_CONFIG.json --protocol all --count 16 --seed 52416 --max-seconds 1800 --max-bytes 1073741824 --out runs/i3/res524/working/my-matrix
python -m scripts.verify_paired_corpus --root runs/i3/res524/working/my-matrix --out runs/i3/res524/working/my-verification.json
python -m scripts.reproduce_paired_corpus --out runs/i3/res524/working/my-replay
```

Every destination must be fresh. No existing files are overwritten. `--count`
is 1..10000, seed is an unsigned 32-bit integer. The CLI is sequential, retains
no trajectory cache and counts every failed/refused request. Configuration is
`BSMB-CORPUS-CONFIG-1.0`: global `parameters`, then a cyclic explicit `cases`
list with optional `protocol` and `parameters` overrides. A case override wins
over global settings. `--protocol all` otherwise enumerates the supported
tuple. Seeds are `seed + case_index`; no stochastic population is implied.

`biosyngen/i2/corpus_config.py::DOMAINS` is the executable parameter-domain
table. Mass, stature and load alter the physical model. Baseline, tempo and
effort alter existing control hypotheses; friction alters supported native
contacts. Squat and bench recordings currently contain one repetition per
trial. The historical builders retain their multi-repetition interfaces.
Unsupported parameter/protocol combinations refuse instead of silently ignoring
inputs. These computational domains do not assert successful human technique
or controller robustness throughout the Cartesian product.

Sensor controls include force gain/noise, analog cutoff and ADC range, encoder
resolution/retraction speed, inclination error/resolution, independent clock
offset/drift, sample jitter, per-device recording windows and dropped indices.
All acquisition hypotheses are ASSUMED_UNCALIBRATED. Source metadata records
saturation indices and preserves dropped-sample gaps; it never repairs them
by row alignment or interpolation. Record-window settings use nominal elapsed
device acquisition time and retain the original observed encoder zero.

New corpus acquisition defaults to **BSMB-FORCE1000-LPT1000-1.0**, with 0.001 s
native nominal periods on both independent devices. Legacy constructors and
explicit `acquisition_profile: legacy_mixed` retain
BSMB-FORCE1000-LPT500-1.0. No 500-to-1000 upsampling occurs. The 10 kHz force
front end is separate from native 1 kHz recording. Analog filtering precedes
ADC sampling; "raw" means no post-acquisition software processing.

Time/disk limits gate starts of subsequent complete trials using elapsed time
and measured largest preceding case size. They are forecasts, not hard OS
quotas: the first case or a larger next case can exceed a budget. A started
physical trajectory is never truncated to manufacture success. Do not request
thousands of cases before inspecting the measured pilot and available storage.

## Frozen namespace and measurands

```text
SOURCE/<case>/manifest.json, checksums.sha256
SOURCE/<case>/streams/force.csv
SOURCE/<case>/streams/lpt.csv
SOURCE/<case>/streams/angle.csv          # free/oblique cases
SOURCE/<case>/derived/vertical.csv       # omitted, with reason, on slack
TRUTH/<case>/reference.json, support_intervals.csv, checksums.sha256
TRUTH/<case>/model_events.json, mechanics_submission.json, context_qualification.json
DIAGNOSTICS/<case>/capture_authority.json
inventory.json, run.json
```

The guide alone uses unchanged P2B-OPEN-DUAL-VFORCE-LPT 1.0. Other cases use
BSMB-PAIRED-CABLE-ANGLE-1.1, BSMB-CABLE-INCLINATION-1.0 and
BSMB-ENCODER-COS-INCLINATION-1.0. Raw CSV headers remain
`sample_index,time_s,force_left_N,force_right_N` and
`sample_index,time_s,displacement_m`. FORCE means bilateral **foot-contact**
reaction only. Encoder displacement is cable-length change relative to the
observed **first ACQUIRED** absolute encoder sample (index 0), which may precede
the first EXPORTED sample when the recording window is cropped. The manifest
declares `FIRST_ACQUIRED_ENCODER_SAMPLE`; operator notes retain that observed
reference's index, native timestamp, absolute encoder length and inclination.
Cropping never silently re-zeros the recording; a first exported displacement
may therefore be nonzero. This provenance clarification and the roles below
are the BSMB-PAIRED-CABLE-ANGLE-1.1 / BSMB-GENERATOR-1.1 source revision.

The SOURCE manifest classifies `force.csv` and `lpt.csv` as `FORCE_STREAM`
and `LPT_STREAM`, `angle.csv` as `OBSERVED_AUXILIARY_STREAM`, and optional
`derived/vertical.csv` as `DERIVED_OBSERVATION`. A derived reconstruction
must never bear `ORIGINAL_SOURCE`. Open V1's four-file package shape and
legacy 500 Hz models remain unchanged. Hip belt, bar collar and system COM
are distinct. The raw incline CSV carries absolute encoder length, measured
angle from vertical, common encoder/angle acquisition grid and slack.

Derived vertical change is computed solely as
`r_observed*cos(theta_observed) - r_zero_observed*cos(theta_zero_observed)`.
It remains DERIVED_GEOMETRY_RECONSTRUCTION_V1, with its own source lineage and
bounded interval uncertainty. Encoder floor quantization, independent bounded
angle error and angle resolution propagate through cosine extrema. Binary64
guards are separate from instrument uncertainty. Slack refuses reconstruction.
The domain is fixed-anchor planar motion; no arbitrary spatial or moving-anchor
interpretation is admitted. Inclination and encoder metrology are assumptions.
The cable is an ideally non-loading observation hypothesis; cable tension,
attachment compliance and sensor-induced changes to athlete motion are not modeled.

TRUTH is BSMB-PAIRED-TRUTH-1.0: states, body inertial parameters, interval
reactions (N), instantaneous impulses (N*s), contact topology/modes, physical
event references, both true acquisition grids, exact clock parameters and
numerical resolution. `attachment_vertical_m` names the actual reference
coordinate. Model event analyses are generator references; the independent
checker reconstructs physical brackets separately and never reads controller
stages. Root/contact event timestamps are numerical estimates, even where the
historical model analysis reports zero within-step uncertainty. Missing events
and unsupported anatomical phases are not converted into successful detections.
Missing core movement events, early termination and model/rep refusals exclude
a case from complete-trial reference qualification even if files were generated.

SOURCE contains no protected state arrays, latent event labels or true-time
arrays. `verify_source` reads SOURCE and an independently supplied manufactured
capture receipt only; it does not import the generator or read TRUTH. The
post-generation reference checker is the process explicitly authorized to read
TRUTH. Directory separation is a workflow/access boundary, not an OS sandbox;
operators must separately control access when supplying production inputs.

## Synchronization, integrity and verification

BSMB-FOUR-EDGE-CAPTURE-1.0 models an external pulse fan-out at four times.
Each device independently captures edges with an assumed 2 microsecond bounded
error. The first/last edges fit affine mappings; two interior edges are held-out
checks. SOURCE mappings are fitted from observed edges, never copied from
protected exact transforms. Clock IDs, independent sample counts and native
offsets remain distinct. Partial overlap and preserved gaps are represented.
No overlap, resets, malformed native grids and incorrect mappings refuse.
Gaussian sample jitter has no finite worst-case bound and is reported as
REFUSED_UNBOUNDED_SAMPLE_JITTER; finite capture error does not conceal it.

The out-of-band DIAGNOSTICS receipt binds both stream hashes and the complete
SOURCE manifest. A different-world substitution with a self-consistently
rehashed local package still fails that external receipt. Without independent
capture authority synchronization refuses. An adversary who can forge the
entire external authority cannot be authenticated by unkeyed hashes; no device
security or empirical timing certification is claimed.

RES-527 fail-closed admission (repairing RES-525 F01-F08). The capture receipt
must declare BSMB-FOUR-EDGE-CAPTURE-1.0 with exactly four finite increasing
edges per clock, fit edges [0, 3], held-out edges [1, 2] and a finite capture
bound in (0, 2 microseconds]; the 2 microsecond value is this contract's
modeled envelope, not device metrology. JSON NaN/Infinity literals refuse.
Manifest-controlled paths are untrusted: the derived path is exactly
`derived/vertical.csv` or null, checked as a string before any filesystem use,
and every SOURCE file opened must be a sealed, link-free member of SOURCE. A
taut angle-aware recording must declare and seal its derivation; only a slack
recording may carry the typed null/refusal, reported as
NOT_EVALUABLE_SLACK_CABLE_REFUSED and never as a full-case pass. The reference
checker applies BSMB-PAIRED-TRUTH-INVENTORY-1.0: the guide has exactly
`reference.json` and `support_intervals.csv`; every modeled protocol adds
`model_events.json`, `mechanics_submission.json` and
`context_qualification.json`. TRUTH must name the exact current SOURCE manifest
digest and trial identity. Modeled cases require independent
LINEAR_MOMENTUM_CLOSURE and CONTACT_ADMISSIBILITY plus the complete per-protocol
event set; only the positively identified guide keeps its endpoint impulse and
unilateral-sign analytical ledger and analytical events. Verifier reports
(BSMB-PAIRED-QUALIFICATION-REPORT-2.0) count source integrity, model-reference,
bounded synchronization, geometry, full-case, expected-refusal, evidence-refusal,
missing-evidence and not-evaluable outcomes separately; the 16-case matrix is
16 model-reference qualifications, 15 bounded manufactured clock maps and one
explicit unbounded-jitter synchronization refusal, i.e. 15 full cases. Earlier
receipts without this schema are preserved and refused by the comparator rather
than reinterpreted. A generator OS write failure is now receipted as a REFUSED
case with its partial files listed and retained; it can never qualify.

Fresh checks include analytical 3-4-5 geometry, bounded uncertainty containment,
native odd samples disproving 500 Hz interpolation, 1 ms provider differences,
force ADC saturation, independent clocks, held-out edges, partial/no overlap,
gaps/resets, missing/wrong angle and encoder reference, slack, wrong
attachment/frame/processing, corrupted bytes and self-rehashed cross-trial
swaps. Existing adversarial physics/evaluator controls remain intact. The
independent I2 evaluator checks prefix momentum with all external supports and
separate impulses, plus tensile/friction admissibility. Missing support channels
must not be inferred from foot Fz. BILATERAL_FORCE metrology and anatomical
phases continue to abstain without independently admitted empirical references.

Hosted qualification adds two fresh 16-case generators per Windows/Linux
platform and compares complete same-platform SOURCE/TRUTH/diagnostic SHA
inventories. Cross-platform checks require matching physical/source decisions,
event inventories and absence classes; event-time differences are bounded by
two recorded integration intervals, and geometry maximum-error differences by
50 micrometres (half a default encoder count). Each platform must independently
pass physical interval containment. Cross-platform byte identity is not claimed.
Mandatory pinned SciPy checks retain their separate jobs. CI uploads bounded
receipts and hashes, not bulk SOURCE or TRUTH.

## Handoff and claim ceiling

Two completion-pass defects were preserved and repaired without changing
physics. The first full SOURCE/TRUTH replay found process-dependent contact
row ordering in the clean-pull support CSV: SOURCE and all physical states
matched, and the two 21,091-line support files were identical as multisets.
Per-frame rows are now canonical and replay explicitly varies Python hash
seeds 0/1. A reversed-contact-order regression exercises the original defect.
The original mismatch is retained under the project-local ignored completion
evidence, with immutable pre-repair commit 95ac37f.

An operator-reendorsed malformed FORCE header exposed a source-checker false
accept at cdf8ee8. Exact native CSV columns, manifest columns, SI units and
profile rates are now checked; the same retained witness refuses with
SOURCE_COLUMNS_INVALID. Missing sensor streams also have explicit controls.
Neither repair relabels derived observations or changes a physical law. The
initial missing-stream test helper attempted to rehash a deleted file and
errored; its log is retained, and the corrected inventory-refusal control passes.

P2B_DERIVED_COORDINATE_ADMISSION=DEFERRED.
P2B_EVENT_METHOD_QUALIFICATION=DEFERRED. P2B_WHOLE_PRODUCT=DEFERRED.
The next Plate2bar task must preserve raw admission, add governed observed
length/angle processing lineage, qualify derived attachment coordinates and
event methods, then execute its own affected product matrix and cold reopen.
No truth injection, admission weakening or direct persistence insertion is
authorized by this generator contract. The earlier PROCESSING_STATE_UNSUPPORTED
witness remains the downstream boundary. New generator schema/semantics changes
require a new version and fresh qualification.

RES-522/PR #1 stays at its separate reviewed candidate. No merge, tag, release,
scientific acceptance or Done transition follows from this work. New source,
tests and documentation are project-authored under the existing Apache-2.0
rights scope; no third-party implementation, participant data, restricted corpus,
consumed holdout or bulk generated record is added to Git.

NUMERICAL_VERIFICATION_ONLY; SIMULATED_MEASUREMENT_ONLY;
BIOLOGICAL_VALIDATION=NOT_ESTABLISHED;
REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.
