# RES-618 implementation receipt — receiving FAIL after two repairs

## Current result (supersedes the historical checkpoint below)

An executable full-body anatomical candidate now exists as
`HUMAN_V2_TORQUE_FULLBODY`, version `2.1.0`: **32 bodies including the independent
barbell, 64 generalized velocity DoF, 48 bounded actuators**, and four declared
scapular rhythm constraints. The human mass remains 75 kg; the default V2
equipment is a 20 kg unplated shaft, giving 95 kg system mass. The old foot-only
candidate and V1 remain separately named and unchanged.

Commit `46b9667bba11da6e988f6e14f802519e90520716` adds an independent mass/COM/inertia
decomposition oracle and deliberate faults. Commit
`173d1452d8277373b40bc08272115f204eeb2b57` adds the typed parameter authority and
integrated full-body model. `HumanV2Plant` now exports physical contact,
bounded torque, saturation and work channels without interpreting scapular
joint constraints as hand/bar grip. No hand/bar weld/connect is used.

The topology includes lumbar, thorax, cervical and independent head bodies;
bilateral explicit clavicle and scapula; palm, grouped proximal/distal fingers
and thumb; and the previous articulated feet. Existing parent mass ledgers
are partitioned rather than adding human mass. New segment inertias are
manufactured box hypotheses; they are not subject-measured inertias or a
claim that V1's compound inertia is conserved. The full-body parameter
authority remains incomplete: foot dimensions do not yet drive all pads,
new-segment tensor overrides are not exposed, and physiological population
ROM/strength authority is not established. Reduced scapular coupling is
explicitly `up=.4*GH_abduction`, `tilt=.1*GH_flexion`; these numbers are original
uncalibrated hypotheses, not Seth's ellipsoidal model or fitted research data.
Human-human self-collision qualification remains incomplete. The unplated
shaft inherits a collision mask that omits shaft/floor contact; that limitation
must be corrected and independently tested in a subsequent authorized repair.

28 focused tests PASS (exit 0, 0.489 s), including actual full-body integration,
deterministic standing replay, three mass/stature profiles, bilateral inertia,
head-COM finite-difference Jacobian, scapular coupling/asymmetry, inward thumb
opposition, parameter consumption and bounded motor output. These passing
checks do **not** qualify gripping, load transfer or receiving.

The first overhead candidate failed with -7.5589 mm contact distance and a
collapsing 0.3 s hold. Repair 1 exposed midfoot/MTP coordinates and permitted
unloaded toes; kinematic convergence improved but the single palm-center
target permitted the bar through tilted palms (-10.9578 mm). Repair 2 added
shaft/palm orientation closure, finger coordinates and explicit hand-pad
nonpenetration penalties. Initial penetration reduced to -0.200119 mm, but
the actual bounded forward hold still lost the bar: **1.6615317915 m →
1.2854786718 m in 0.3 s**, a **0.3760531197 m drop**, with maximum generalized
speed **15.6952410936**. Rendered candidate/failed states confirm loss of support.
Optimizer convergence is deliberately not receiving success.

The independent whole-system balance reports maximum step residuals
0.0018378803 N s linear and 0.0002884837 N m s angular. No convergence-derived
acceptance budgets have qualified these numbers. The final contact cone check
passes, which does not establish a supported bar or feasible receiving.

`AGENTS.md` requires: "Stop at a blocker, after two repair attempts, or at six
elapsed hours." Two repairs of this receiving failure were executed and their
original witnesses preserved. Further mechanism repairs stop at this boundary.
The issue stays In Progress; it is not accepted, Done or ready for V2 adoption
by RES-611. No owner merge/release decision was taken.

Reproduction (project virtualenv, from the RES-618 worktree):

```powershell
python -m unittest tests.test_res618_anthropometry tests.test_res618_feet tests.test_res618_inertial_oracle tests.test_res618_fullbody tests.test_res618_v2_interface -v
python -m scripts.res618_fullbody_qualification
python -m scripts.res618_render_fullbody
```

The qualification command returns **exit 1 / FAIL**, preserving the physical
witness. The renderer returns exit 0. The neutral, overhead-candidate and
failed-after-300-ms PNGs were each opened and inspected; they are diagnostic
figures, not validated movement examples. The CI spatial jobs now execute the
same failing physical gate and retain JSON artifacts, so unit-test GREEN
cannot silently imply receiving qualification. Final-head CI and independent
final-SHA review must be reported separately from the older published head.

```text
ISSUE=RES-618
MODEL=HUMAN_V2_TORQUE_FULLBODY
BASE_SHA=ac837e463f1bf76c7b48ad58cc2075617dba3d26
BRANCH=work/RES-618-human-v2-anatomy
COMMIT_SHAS=see Git history and exact SHA receipts in RES-618
DRAFT_PR=https://github.com/Litju/BioSynGenBenchmark/pull/13
V1_MODEL_PRESERVED=YES
V2_MODEL_VERSION=2.1.0
BODY_COUNT=32 including barbell; 31 human
GENERALIZED_DOF=64
ACTUATOR_COUNT=48
TOTAL_MASS_RECONCILIATION=75 kg human + 20 kg default bar; numerical tests PASS
COM_AND_INERTIA_VERIFICATION=PARTIAL; manufactured tensors, independent decomposition tests
ARTICULATED_FEET_AND_MTP=EXECUTABLE; dynamics qualification incomplete
WEIGHTLIFTING_SHOE_GEOMETRY=EXECUTABLE; massless equipment assumption
SCAPULA_CLAVICLE_SHOULDER=EXECUTABLE reduced hypothesis; biomechanical qualification incomplete
FUNCTIONAL_FINGER_GRIP=ARTICULATED CONTACT GEOMETRY EXECUTABLE; load-bearing grasp FAIL
FRONT_RACK_LOAD_TRANSFER=NOT_EVALUABLE
HEAD_NECK_AND_THORAX=EXECUTABLE; focused FK/inertial checks PASS
JOINT_LIMITS_AND_ACTUATOR_BOUNDS=EXECUTABLE; uncalibrated hypotheses
FRONT_RACK_POWER=NOT_EVALUABLE
FRONT_RACK_FULL=NOT_EVALUABLE
OVERHEAD_POWER=FAIL; actual bar drop 0.3760531197 m in 0.3 s
OVERHEAD_FULL=NOT_EVALUABLE
DYNAMIC_RECEIVING_FEASIBILITY=FAIL
MOMENTUM_VERIFICATION=MEASURED; no qualified convergence budget
ANGULAR_MOMENTUM_VERIFICATION=MEASURED; no qualified convergence budget
CONTACT_ENERGY_VERIFICATION=NOT_EVALUABLE
TIMESTEP_CONVERGENCE=NOT_EVALUABLE
ADVERSARIAL_CONTROLS=PASS within focused tested scope; full campaign incomplete
ANTHROPOMETRIC_SENSITIVITY=3 mass/stature profiles; dynamic sensitivity incomplete
RENDERED_FIGURES_INSPECTED=3 full-body diagnostic PNGs plus earlier foot candidate
FULL_WINDOWS_TESTS=see subsequent exact-head run record
FULL_LINUX_TESTS=see hosted exact-head run record
MANDATORY_SCIPY=see hosted exact-head run record
MANDATORY_MUJOCO=focused tests PASS; receiving qualification FAIL
HOSTED_CI_FINAL_SHA=see Linear exact-head run record; expected receiving RED
INDEPENDENT_FINAL_SHA_REVIEW=NOT_PERFORMED
RES611_COMPATIBILITY=NOT_QUALIFIED; do not transfer V1 trajectories
RES612_BENCHMARK_HANDOFF=plain export interface available; acceptance pending
RETAINED_LIMITATIONS=failed grasp/control; incomplete contact/energy/ROM/parameter authority
NEXT_ACTION=resolve witnessed grasp/load-support failure in a new authorized repair campaign
```

## Historical foot-only checkpoint (retained)

This checkpoint does not deliver HUMAN_V2_TORQUE_FULLBODY. Gates A and B are
partial; C–H are not implemented or qualified. No scientific acceptance is
requested. RES-618 stays In Progress. All V1 source and tests remain unchanged.

Base: `ac837e463f1bf76c7b48ad58cc2075617dba3d26`, verified against remote and
PR #12. PR #11 remains a sibling, not an ancestor. Local RES-611 head
`351684f25eaedab9fe5fd1cd7c534c757fbe1d39`, modified `olympic.py`, untracked
`render.py` and `res611_olympic_trials.py` were inspected by status only and
preserved. No experiments were migrated.

Verified source commits:

- `8dc2de98c9638ad2a30d870c7ec0d9625aa91506`: typed inertial migration and faults.
- `5783eb0f682cc4ef868152f483073ac4db6dac5a`: articulated foot/shoe candidate.
- `f3949a63cf105fab4eee7ed96ec3ac29982ce4a2`: root clearance correction and mandatory candidate CI modules.

Commands, from the RES-618 worktree, using the project `.venv/Scripts/python.exe`:

```powershell
python -m unittest tests.test_res618_anthropometry tests.test_res618_feet -v
python -m unittest tests.test_res610_spatial -v
python -m scripts.res618_candidate_receipt
git diff --check
git diff --cached --check
git diff --cached
git diff --cached --name-only
```

Focused checks: 11 PASS, exit 0 (0.192 s). Original V1 spatial suite: 12 PASS,
exit 0 (129.369 s). Full Windows suite, Linux, hosted CI and independent
final-SHA review have not been completed. Author-authored analytical tests
are not an independent human/agent final-SHA review.

The candidate has **21 dynamic bodies including the barbell**, 49 velocity DoF
and 37 actuators. Human mass is 75 kg; system mass is 135 kg including the
60 kg barbell. V1's original 75.105 kg witness remains unchanged. Three
mass/stature fixtures reconcile human mass to 11 decimal places; bilateral
COM/tensor symmetry and a cylinder analytical inertia check pass. Full
aggregate COM/inertia, uncertainty and independent decomposition qualification
remain incomplete. No empirical segment calibration is implied.

New foot bodies split each normalized foot mass 55%/30%/15%, rather than
adding mass. Uniform box inertias replace the old foot tensor; compound
inertia is not asserted to preserve the V1 value. Effective midfoot range is
[-0.15,0.15] rad and MTP [-0.35,1.05] rad with 25/30 Nm motor bounds, all
manufactured assumptions. Massless shoe support pads permit 0–40 mm heel
elevation and finite friction. Equipment mass, toe-off, stance relocation,
loaded squats, friction/COP and foot contact work are not qualified.

The initial contact probe exposed -0.028 m floor penetration, corrected by
raising the initial root for actual sole/heel thickness. The regression now
checks 0, 20 and 40 mm heel configurations. A 1 s forward standing probe with
grip constraints disabled and bar positioned separately ended with pelvis
height 0.967954096835 m, maximum generalized speed 0.0004570251175 and minimum
contact distance -0.000051178087 m. This does not establish loaded standing,
balance robustness, receiving or a controller suitable for V2. The ballistic
test disables contact/grip to isolate constant-gravity COM displacement;
it is not contact momentum qualification.

`runs/i3/res618/candidate.json` reports its precise source SHA and dirty state.
The first renderer invocation failed because 1000×800 exceeded the default
640×480 framebuffer. Reducing to the actual framebuffer dimensions succeeded
(exit 0); `standing_candidate.png` was opened and inspected. Limbs and stance
are readable, but the default dark soles are poor for detailed contact
inspection. This single image does not satisfy the requested visual suite.

Remaining work includes the complete parameter registry/provenance, full foot
dynamics qualification, clavicle/scapula, functional contact-only grip,
segmented trunk/head, bounded integrated receiving, momentum/angular/contact
energy/refinement/replay qualification, multiple proportions, complete visual
inspection, RES-611 migration, RES-612 handoff and final-SHA independent review.
Receiving POWER/FULL results for both rack and overhead are NOT_EVALUABLE on
this candidate. V1 controller qualification cannot be transferred.

Rights inventory: new source/tests/script/docs are original project code;
generated output contains only manufactured fixture data and primitive
rendering. No external meshes, source code, participant data or datasets were
imported. Existing owned baseline history was not rewritten. Literature is
context only, not copied numerical authority: de Leva,
https://pubmed.ncbi.nlm.nih.gov/8872282/ ; MuJoCo contact modeling,
https://mujoco.readthedocs.io/en/latest/modeling.html .

NUMERICAL_VERIFICATION_ONLY
SIMULATED_MEASUREMENT_ONLY
BIOLOGICAL_VALIDATION=NOT_ESTABLISHED
REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED
