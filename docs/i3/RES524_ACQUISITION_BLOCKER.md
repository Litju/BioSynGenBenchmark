# RES-524 acquisition checkpoint and first blocker

This work extends manufactured data generation and its qualification coverage,
including exercise phases and contact/support transitions. It does not create
a clinical-data platform. RES-524 is incomplete and is not ready for In Review.

## Implemented checkpoint

`biosyngen/i2/acquisition.py` adds `BSMB-FORCE1000-LPT1000-1.0` with nominal
0.001 s periods for both observers. It retains distinct supplied clock offsets,
drifts, jitter, seeds and sensor stages. The explicit legacy
`BSMB-FORCE1000-LPT500-1.0` profile and existing instrument defaults retain
the original 500 Hz LPT behavior. No existing source, controller or historical
fixture changed. This is an opt-in profile; existing historical scripts do not
silently acquire new defaults.

The independent quadratic-path control distinguishes fresh native 1 kHz
sampling from interpolation of the 500 Hz observation. Other focused controls
cover sample counts/monotonic native grids, independently drifting clocks,
encoder counts, numerical velocity, saturation and existing radial-path,
bilateral-force and slack behavior. These are software checks, not bandwidth,
metrological fidelity or synchronization qualification.

## Exact first blocker

`UNQUALIFIED_CABLE_TO_VERTICAL_COORDINATE`

A fresh one-repetition default back squat with the reel initially directly
under the bar collar reached its full 3.5 s duration (3,500 physics intervals).
The two observers consumed that same trajectory and emitted **3,501 FORCE**
and **3,500 LPT** samples in memory, both at nominal 1 ms spacing. Independent
drift/jitter and native origins are preserved; samples are not paired by row.
Neither sensor reported saturation/slack in this probe.

The cable starts at 0 degrees to vertical, but the full path reaches
**1.3559348619 degrees** and **0.02467284028 m** horizontal offset. Independently
computed Euclidean cable travel differs from vertical attachment travel by
up to **0.0002919332769 m** (0.291933 mm), at state 1,733. This exceeds two
0.1 mm encoder counts. Encoder resolution is not an acceptance tolerance.
No angle/error acceptance authority has been established for this geometry.

Plate2bar's existing open profile declares `LAB_VERTICAL_Z_UP`. Initial
alignment alone cannot justify labeling these cable-length observations as
vertical displacement. This finding is an unqualified interpretation, not a
claim that the observed error exceeds a scientifically approved tolerance.
The preflight probe always reports an unqualified result; it is diagnostic
evidence and never an admission or automatic refusal authority in Plate2bar.
The independent vertical-path positive and 3-4-5 triangle negative validate
its geometry diagnostics.

The attached execution brief requires: "If an actual scientific P0 defect,
unqualified acquisition geometry, missing synchronization authority, or failed
production-ingestion contract blocks qualification, preserve the evidence and
report ONE exact first blocker." Feature/export work stopped at this gate.
No deadlift/jump pilot, production import, qualified synchronization, phase or
support-transition conformance is claimed. No source bundle was fabricated to
bypass the issue; no truth was supplied to production.

Next engineering action: establish an independently justified, source-declared
geometry/error authority or implement and qualify physical vertical routing in
the same simulated world, then resume export and production-path qualification.
Do not correct cable travel with hidden model coordinates or choose a tolerance
merely to admit this trial. The owner-only RES-522 integration/adoption gate
also remains intact.

## Frozen preflight authority

* Reviewed BioSynGen source and public draft PR #1 head:
  `650c797b87bbd8bcb9a5739c40ecc2552f487bb3`.
* BioSynGen remote main: `6115d702ba3ab814982353945b0753f4846aae88`.
* New local branch: `codex/res-524-dual-1khz`, derived from that reviewed source.
* Plate2bar read-only checkout: `9a60eab55b9c527ca65acb3070de77d840d555b0`,
  `claude/etl-data-plane-convergence`; remote main observed as
  `74ec9520b005fb1096c1afe89e14043455725636`. No product source was modified.
  Pre-existing untracked `CANONICAL_AUTHORITY_AUDIT.md` was neither read nor changed.
* Plate2bar source schema SHA-256:
  `17cb0fce7625200d11cf6a9f52e8c95362015705775643b2e9ab31584b0f726a`.
* Read live binding I3 contract, RES-524, RES-522, ML-593/594 and existing
  ML-419/520/551/586 authorities. No historical corpus or locked holdout accessed.

PR #1 remains open/draft/unmerged. Existing hosted run 38690985881 has five
successful jobs at the old reviewed source; the separate failed run 38690980439
remains historical evidence. Neither run qualifies this new local branch.

## Commands and retained evidence

All evidence below is synthetic-only and lives in `runs/i3/res524/`.

```powershell
python -m unittest tests.test_acquisition_profiles tests.test_i3_instruments -v
python -m scripts.probe_acquisition_geometry --out runs/i3/res524/geometry-preflight.json
python -m scripts.probe_acquisition_geometry --out runs/i3/res524/geometry-replay.json
python -m unittest discover -s tests -v
py -3.11 -m scripts.require_i3_scipy_checks
git diff --check
```

Fresh output files are opened exclusively; repeat probes require a fresh path.
The first focused run had 9 passes and one failed assertion at one encoder
count plus 3.6e-17 m floating error. `acquisition-before.txt` preserves it.
One repair added 1e-14 m floating allowance to that encoder-count assertion;
the 500-to-1000 falsifier and physical tolerances were unchanged.
`acquisition-after.txt`: 10/10 pass. `focused-final.txt`: 12/12 pass, including
two analytical geometry controls. No skips/xfails added.

`geometry-preflight.json` and `geometry-replay.json` retain clocks, sample
counts, coordinate errors and SHA-256 source hashes. Separate-process reports
are identical excluding elapsed time. The second command explicitly preserves
native Python exit 2 (`geometry-replay.txt`); the first shell wrapper surfaced
nonzero as 1. These are expected diagnostic failures, not production refusals.
The first probe took 12.9782 s including simulation, both observations and
geometry inspection. No corpus throughput, peak-memory or storage qualification
was performed. No scaling or bulk generation occurred.

`scipy.txt`: Python 3.11.9, SciPy 1.17.1, NumPy 2.4.4; 2/2 checks pass, exit 0.
The full Windows suite log is `windows-suite.txt`: 312 collected, 310 passed,
two existing optional SciPy skips, no failures, 364.888 s, exit 0 on Python
3.12.10. That run started before the two final geometry
unit controls were added; both controls passed separately in the final focused
run. Production source did not change after the full suite started.
`log-eol-provenance.json` records original and LF-normalized hashes of the
Windows-captured logs. Only CRLF was converted to LF for Git whitespace checks;
test text, including the initial failed assertion, is preserved.

No hosted Windows/Linux CI or ML-586 product matrix ran for this checkpoint.
No issue closure is inferred from focused or existing legacy tests.

## Rights, inventory and review disposition

New code, independent manufactured analytical controls, numerical probe and
this record are project-authored under the existing owner-selected Apache-2.0
scope. No external implementation/schema payload, participant data, proprietary
source, holdout or dependency was added. Outputs contain only manufactured
diagnostics and local software-run records. The original failure is retained.

Staged scope is exclusively acquisition.py, test_acquisition_profiles.py,
probe_acquisition_geometry.py, this document and the exact RES-524 run records.
The complete staged diff and inventory must be inspected before atomic commit.
The full commit SHA and exact inventory are recorded in Linear after commit,
avoiding a self-referential hash here. Branch stays local at the stop gate;
there is no RES-524 public push or draft PR. PR #1 and all historical evidence
remain unchanged. No merge, release/tag, scientific acceptance or Done action.

NUMERICAL_VERIFICATION_ONLY; SIMULATED_MEASUREMENT_ONLY;
BIOLOGICAL_VALIDATION=NOT_ESTABLISHED;
REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.
