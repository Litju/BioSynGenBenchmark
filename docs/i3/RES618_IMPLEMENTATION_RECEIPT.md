# RES-618 implementation checkpoint — incomplete

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
