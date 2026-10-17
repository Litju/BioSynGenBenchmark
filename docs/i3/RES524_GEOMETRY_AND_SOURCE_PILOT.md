# RES-524 resumed geometry and source pilot

Status: **BLOCKED_PRODUCT_DERIVED_ADMISSION**. This continues local checkpoint
`2bad5c8d093a040d3f004d46c275afe7d8fa2511` on `codex/res-524-dual-1khz`.
The original blocker dossier and failed evidence remain unchanged. Work is
internal Plate2bar research/testing; no commercialization or publication occurs.

## Geometry resolution

The free-weight source measurand remains **cable displacement**, with frame
`CABLE_LENGTH_COORDINATE`, positive away from the reel and bar-collar attachment.
It is not vertical displacement, bar COM or system COM. In this planar model
the projected collar shares the bar-body origin height; this numerical
coincidence does not confer COM identity or a general spatial interpretation.

`BSMB-CABLE-INCLINATION-1.0` adds a simulated integrated inclinometer. The
encoder and inclinometer sample physical attachment geometry at the same LPT
clock instants. Inclination is measured from the lab vertical. Angle errors
are an independent seeded uniform sequence bounded by 0.0005 rad, followed by
0.00001 rad quantization. Encoder resolution remains 0.0001 m. These are
**assumed instrument hypotheses**, not calibrated device specifications or
an acceptance tolerance fitted to the old 0.291933 mm discrepancy.

`BSMB-ENCODER-COS-INCLINATION-1.0` reconstructs attachment-height change as
`r_i*cos(theta_i) - r_0*cos(theta_0)` from **observed absolute encoder length
and observed inclination only**. It never receives body positions, exact
angles or reference events. The result is a separate
`DERIVED_GEOMETRY_RECONSTRUCTION_V1` observation. Raw cable samples are retained.
Slack refuses reconstruction. Length floor-quantization intervals and angular
error/quantization intervals propagate through cosine extrema and product
intervals, including the shared uncertain reference sample. A documented
binary64 rounding guard is separate from physical uncertainty.

Post-execution comparisons to protected sampled mechanical reference found:

| Development world | LPT samples compared | Interval violations | Max vertical error | Max interval width |
| --- | ---: | ---: | ---: | ---: |
| Bar-loaded back squat | 4,200 | 0 | 0.105187 mm | 0.225591 mm |
| Deadlift | 6,000 | 0 | 0.130362 mm | 0.275541 mm |

This establishes numerical containment for the declared bounded sensor model;
it does not authorize production vertical-coordinate admission.

The separate **ideal vertical-guide analytical apparatus** has 75 kg body and
20 kg bar lumped masses sharing a constrained vertical motion, with no vertical
guide reaction. It is explicitly not an ordinary free-weight squat or a human
anatomical model. A smooth `sin^4` down/up path has mathematical start 1.2 s,
bottom 2.2 s and end 3.2 s. Interval reactions use analytical velocity
differences, so `sum(Fz-Mg)*dt = delta(momentum)` closes independently. All
support reactions are positive. Initially vertical geometry alone is never
used as evidence for the free-weight path.

## Actual source and product execution

The existing native dual-1kHz profile and historical 500 Hz profile remain
intact. Each case is generated from one physical world; FORCE and LPT retain
distinct clock identities, offsets and constant drifts. This pilot explicitly
sets sample jitter to zero rather than claiming a finite bound on Gaussian
jitter. FORCE analog acquisition filtering, gain/noise and ADC artifacts remain;
"raw" denotes digitized acquisition output with no post-acquisition software
filtering. The sensor front-end hypotheses are declared in source metadata.

| Case | FORCE samples | LPT samples | SOURCE bytes | Actual result |
| --- | ---: | ---: | ---: | --- |
| Constrained guide | 4,401 | 4,400 | 314,428 | Open 1.0 + canonical raw import/admission/coupling/reopen |
| Back squat | 4,201 | 4,200 | 800,139 | Canonical cable CSV import/admission/coupling/reopen |
| Deadlift | 6,001 | 6,000 | 1,121,768 | Canonical cable CSV import/admission/coupling/reopen |

The guide emits exact `P2B-OPEN-DUAL-VFORCE-LPT-1.0`:

```text
SOURCE/trial-00000524-0000/
  manifest.json
  checksums.sha256
  streams/force.csv
  streams/lpt.csv
```

Free weights use local **BSMB-PAIRED-CABLE-ANGLE-1.0**, not the frozen vertical
open adapter. They retain the same two native CSV headers and add
`streams/angle.csv` (`sample_index,time_s,encoder_length_m,inclination_rad,slack`)
and `derived/vertical.csv` (`sample_index,time_s,displacement_m,lower_m,upper_m`).
Their manifest declares raw/derived frames, methods, sensor assumptions and
SHA-256 source records; `checksums.sha256` seals every source file. Raw cable
CSV is admitted through the **existing** SourceArtifact -> DelimitedTextAdapter
-> ImportProfile -> canonical import path. No alternate scientific importer or
direct SQLite record insertion was introduced.

Two simulated external hardware pulses are timestamped by both devices. Their
source-visible capture receipt binds both stream hashes and independent clock
IDs. Clock maps are fitted from those captured edges, not read from hidden
truth or inferred from rate equality. Actual Plate2bar admits/persists the
explicit mappings and coupled trial. Uncertainty is explicitly ideal simulated
capture plus binary64 guard, not empirically measured timing uncertainty.
An unqualified local mapping refuses as `CLOCK_MAPPING_UNQUALIFIED` in each case.

The real application persists raw imports, components, mappings, coupled trials
and native C++ segmentation candidates. Reopening in a separate process yields
identical coupled trial identities for all three cases. A product-read audit
guard additionally refuses Python opens of any TRUTH directory; its negative
control raises `PRODUCTION_REFERENCE_TRUTH_ACCESS_REFUSED`. This guards Python
file access, not arbitrary native OS calls. Product code receives SOURCE only;
the separate post-execution comparator alone opens TRUTH.

## New precisely demonstrated product-contract blocker

**`PROCESSING_STATE_UNSUPPORTED`**, field
`LPT_DISPLACEMENT.source_processing_state`, message:
"component admission accepts only an explicitly declared raw processing state".

Both free-weight derived vertical recordings reach the real canonical source
import but refuse at `admit_lpt_component`. The profile truthfully declares
derived processing and `VERTICAL_ATTACHMENT_DISPLACEMENT`; it is not relabeled
raw or vendor processed. The relevant source is
`measurement/component_pipeline.py::_check_signal` and
`measurement/lpt_component_pipeline.py::admit_lpt_component` in Plate2bar.
Existing ProcessingProfile families are FILTER, SMOOTHING and RESAMPLING;
there is no registered angle/length geometric reconstruction family. This is a
new contract-boundary finding, not a repetition of the original cable witness.

### Exact bounded compatibility change required

Preserve open 1.0 unchanged. A prospective **P2B-OPEN-DUAL-VFORCE-LPT 2.0** must:

1. Represent raw cable length/displacement with its actual cable coordinate,
   anchor, attachment, sign and absolute/reference encoder semantics; keep
   FORCE lab-vertical and foot-contact-only.
2. Admit a separately identified inclination source (rad, reference vertical,
   calibration/error bound, resolution, native clock/timebase, sample coverage
   and source hash), and the observed absolute encoder reference. Missing angle,
   slack, incompatible clocks and unqualified reference must refuse.
3. Register a versioned native geometric ProcessingProfile for observed
   `r*cos(theta)` with two-source lineage, explicit reference subtraction and
   interval uncertainty. Do not replace it with a scalar affine W2A mapping.
4. Add a **separate derived attachment-coordinate admission path** bound to that
   executed processing lineage. Preserve the raw-only LPT component guard and
   forbid silent promotion to bar/system COM. Unknown processing still refuses.
5. Qualify the new quantity/operator against independent full-path controls and
   run the governed affected ML-586 matrix before enabling protocol consumption.

This is an exact compatibility proposal, not a new canonical Plate2bar method
authority. Plate2bar is read-only under the workspace rules; no source contract
or threshold there was changed in this pass.

## Events, supports and honest refusals

Actual frozen production segmentation operates on admitted source observations,
without supplied events or controller-stage labels. For the analytical guide,
the reversal bracket contains exact bottom, but confirmed motion lags the
mathematical start by **0.2190851451 s**. These are different event definitions;
no onset PASS is claimed. Plate2bar's live registry marks these method versions
`NOT_QUALIFIED`, with `AUTOMATIC_METHOD_NOT_QUALIFIED`, and refuses promotion to
ML-413 authoritative events/phases. Candidate result state is not qualification.

Protected deadlift contact mechanics record bar-floor STICK -> SLIDE at 1.768 s,
SLIDE -> OPEN at 1.778 s, and OPEN -> STICK at 5.635194874 s. Contact modes and
impulses are retained in TRUTH separately from the sampled foot forces. The
current two observed Fz channels do not measure bar-floor reaction. Full support
transition, anatomical phase and rep-completion conformance remain unestablished;
these reference events were never injected into product persistence.

Four actual open-adapter/component adversaries pass exact refusal expectations:
corrupted bytes -> `CHECKSUM_MISMATCH`; different-trial force substitution ->
`CHECKSUM_MISMATCH`; honestly declared cable frame through vertical-only open
1.0 -> `UNKNOWN_FRAME`; a declared processed source -> `PROCESSING_STATE_UNSUPPORTED`.
Together with two derived-stage refusals and three unqualified-clock refusals,
nine expected product refusal instances are retained.
Hash substitution detects broken acquisition receipts; it does not prove
authenticity against an adversary forging an entirely self-consistent manifest.

## Reproduction, tests and scale limits

Fresh small CLI examples (replace each OUT with a fresh project-local directory):

```powershell
python -m scripts.generate_paired_corpus --protocol guided_vertical --count 1 --seed 524 --out OUT
python -m scripts.generate_paired_corpus --protocol back_squat --count 1 --seed 525 --out OUT
python -m scripts.generate_paired_corpus --protocol deadlift --count 1 --seed 526 --out OUT
```

`--config` accepts `BSMB-PILOT-CONFIG-1.0` metadata only; scientific parameter
overrides are deliberately not implemented. `--count` is capped at three.
This is a bounded development CLI, not the completed configurable population
corpus generator. Failures remain in the run denominator; no bulk scaling.

Run the source-only reader with Plate2bar's installed interpreter:

```powershell
& D:/projects/plate2bar-instrument/.venv/Scripts/python.exe scripts/qualify_paired_source.py --source SOURCE_CASE --state FRESH_STATE --out FRESH_RECEIPT --protocol BACK_SQUAT_DUAL_V1
python scripts/compare_paired_reference.py --source SOURCE_CASE --truth TRUTH_CASE --product RECEIPT --out FRESH_COMPARISON
```

Actual commands/counts/results retained in `runs/i3/res524/resume/`:

* Focused geometry/acquisition/instruments: **19/19**, exit 0; full oblique
  trajectories, 3-4-5 analytical control, misleading angle, slack/nonfinite,
  independent angle noise, guide momentum and source separation.
* Full Windows suite: **320 collected, 318 passed, two existing optional SciPy
  skips**, 372.616 s, exit 0. Discovery preceded the final added guide-impulse
  unit control; that control and corrected interval averaging passed in the
  final 19-test run. No other production source changed after suite start.
* Separate mandatory SciPy: **2/2**, exit 0, Python 3.11.9, SciPy 1.17.1,
  NumPy 2.4.4. Standard suite uses Python 3.12.10.
* Actual affected product boundary tests: **154 passed**, exit 0, one existing
  Starlette deprecation warning. Targets: reviewer/ml46 open bundle,
  measurement/test_ml410_ml411_component_pipeline.py,
  measurement/test_ml412_coupled_trial.py and test_automatic_segmentation.py;
  `-p no:cacheprovider` and explicit tests confcutdir.
* Fresh-process source/TRUTH reproducibility: **19/19 file hashes identical**
  across all three generated cases; no nondeterministic performance report in
  the compared inventory. Total final SOURCE size: **2,236,335 bytes**.
* An initial open-adapter metadata failure (`MANIFEST_SCHEMA_INVALID`) exposed
  missing nullable identity/calibration fields. One repair supplied those
  declarations; original failed package and exact failure replay are retained.
* Staged review exposed a reader defect: it replaced a source's declared stage
  with RAW. A second bounded repair now maps the declared stage/frame unchanged.
  The prior false-admission witness, corrected exact refusal and permanent
  processed-source challenge are retained. Valid raw + derived-source checks
  passed again. This was a harness defect, not a Plate2bar admission defect.
* New logs are LF-normalized only, with original and normalized SHA-256 receipts.

Traced CLI measurements, with other checks running on this host, were guide
6.2415 s / 22,756,766 Python traced peak bytes; squat 130.4020 s / 50,105,943 B;
deadlift 218.5070 s / 74,869,171 B. The free-weight metadata-only completion was
re-exported without tracing; the guide's exact interval-average correction was
also re-exported. These are exploratory Python allocation/wall-time measurements,
not isolated throughput, peak process RSS or native memory qualification.

Full hosted Windows/Linux CI and ML-586 matrix were not run: this is a blocked
internal local checkpoint with no Plate2bar code modification or public push.
No closure, In Review/Done transition or complete scientific pilot PASS follows.

## Provenance and governance

Plate2bar checkout: `9a60eab55b9c527ca65acb3070de77d840d555b0`; live remote main:
`74ec9520b005fb1096c1afe89e14043455725636`. The measurement contracts, relevant
ProcessingProfile and open schema are identical between them. Actual installed
native extension hashes and source hashes are in the resumed evidence receipt.
PR #1 and its `650c797b87bbd8bcb9a5739c40ecc2552f487bb3` head remain unchanged.

New code/tests and manufactured summaries are project-authored; no dependency,
participant/corpus/holdout data, third-party implementation or historical source
was added. Generated source, protected truth, negative packages and disposable
SQLite state stay ignored under `runs/i3/res524/working/`. An import-time default
empty product catalog was preserved there too; the harness now directs bootstrap
state there. No V1/V2/R6 evidence or Plate2bar source was modified. Atomic commit
SHA, exact staged inventory and command receipts are recorded in Linear.

NUMERICAL_VERIFICATION_ONLY; SIMULATED_MEASUREMENT_ONLY;
BIOLOGICAL_VALIDATION=NOT_ESTABLISHED;
REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.
