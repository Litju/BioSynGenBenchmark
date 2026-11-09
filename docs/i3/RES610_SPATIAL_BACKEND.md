# RES-610 — spatial athlete–barbell backend and receiving feasibility

Base: PR #10 head `db4c58f90ebc209786e34a45b8ccc9405215ad74`. Branch
`work/RES-610-spatial-backend`. Commits: `158e4348c1c00610ed55029e19009efe21f0c70d`
(backend comparison), `c2d1e9188999b2b628c9d22667de177e7515b7df` (spatial plant,
receiving equilibria, independent oracle). Receipts:
`runs/i3/res610/backend_comparison.json`, `runs/i3/res610/spatial_qualification.json`
(generated at `c2d1e91`; `source_dirty=true` only because the uncommitted CI workflow
edit was present, while all plant, receiving and oracle sources were committed).

This is a feasibility prototype. **A static receiving pose and a perturbed hold
are not a clean or a snatch.** No catch is simulated here.

## 1. Executed backend comparison

`python -m scripts.res610_backend_comparison` ran on Windows 11 with Python 3.12.13,
dt 1 ms. Each backend ran in an isolated child process against the same
closed-form fixtures.

| Fixture (truth) | MuJoCo 3.3.7 | OpenSim 4.6 | Drake |
|---|---:|---:|---|
| Torque-free spin: energy / momentum vector error | 2.4e-13 J / 6.1e-7 | 2.5e-9 (max invariant) | not installable |
| Double pendulum energy error | 2.4e-7 J | 2.6e-8 J | — |
| Resting box normal force (9.81 N) | 9.8100 N | 9.8100 N | — |
| Coulomb slide distance (0.40775 m) | 0.40702 m | 0.40755 m | — |
| Wall time / peak RSS (two runs) | 0.58–1.07 s / ≈50 MB | 2.83–3.82 s / ≈88 MB | — |
| Licence / installed size | Apache-2.0 / 22.8 MB | Apache 2.0 / 268 MB (Moco present) | `pip`: no Windows distribution |

**Selection: MuJoCo**, chosen on this executed evidence. It is installable on
both CI operating systems, small and the fastest here. It exposes contact and
equality-constraint forces (`efc_force`, `mj_contactForce`), Jacobians, implicit
actuator damping and bounded `general` actuators, which is what the static QP,
the servo holds and the independent oracle all need. Its soft constraints are a
documented limitation (millimetre penetration, a non-energy-exact contact model).
OpenSim/Moco is retained as the candidate for later musculoskeletal plausibility;
the athlete model would have to be re-authored for it. Drake stays a possible
Linux-only alternate oracle. Two errors in my own script were fixed rather than
counted against OpenSim: `upd_` accessors invalidated the model, and a
manager-owned `State` was used after the manager was freed (segfault).

## 2. Spatial plant (`biosyngen/spatial`)

The planar `biosyngen.multibody` engine and all I0–I3 scopes are untouched. The
new model is programmatic MJCF: 18 bodies, 45 DOF, 33 bounded joint servos. It has
an independent 6-DOF barbell (20 kg shaft plus 2×20 kg plates, closed-form
inertia), a free pelvis, a 3-DOF lumbar joint, bilateral 3-DOF hips, knees,
ankle and subtalar joints, 4-sphere feet, a 2-DOF shoulder girdle, a 3-DOF
glenohumeral joint, elbows, forearm rotation, and 2-DOF wrists and hands. All
parameters are ASSUMED_UNCALIBRATED (75 kg / 1.75 m manufactured athlete).
Planar declared limits are kept: hip 2.30, knee 2.50, ankle 0.75/−0.90,
elbow 2.60, shoulder flexion 3.00 rad.

Declared assumptions, versioned:
* Grip `BILATERAL_TWO_SITE_CONNECT_V1` (closed). Two ball constraints 0.07 m
  apart along the bar leave only hand roll about the bar free, and can transmit
  a hand moment, bounded at 15 N m.
* Grip `BILATERAL_SINGLE_SITE_CONNECT_V1` (open-hand finger support). One ball
  constraint per hand: it transmits force but no moment.
* The grip site is fixed in the hand. Movement of the bar within the hand during
  a lift is **not** modelled.
* Front-rack support is a separable bar–anterior-deltoid shelf contact.
  Bar–body clearance is ≥ 5 mm everywhere off declared supports.
* Servo: torque = clip(kp(u − q) − kd q̇, ±capacity), with damping integrated
  implicitly. This is a control hypothesis, not physiology.

Source-observable channels (`plate_force_world_n`, `bar_position_world_m`) are
kept separate from latent ones (joint angles and torques, grip and rack forces,
body states).

## 3. Receiving qualification (`scripts.res610_spatial_qualification`)

Each configuration goes through five steps:
1. A symmetric pose solve with closure of feet, grips, shelf and clearance.
2. A static QP: friction pyramid, bounded grip, actuator capacity, both free bodies.
3. Independent oracle checks of the QP solution.
4. A 1.5 s hold with a 6 N s bar push, checked frame by frame.
5. Replay and refinement.

The depth criterion is explicit. POWER means the hip is above the knee (thigh
inclination > +0.05 rad). FULL means below (< −0.05 rad). The band between is
refused as AMBIGUOUS.

| Configuration | Static | Torque ratio | Grip closure (m) | Hold (sat. steps, undeclared contacts) | Max NE residual / step (N s) |
|---|---|---:|---:|---|---:|
| FRONT_RACK / POWER / open finger | **FEASIBLE** | 0.686 | 1.2e-7 | stable (0, 0) | 8.2e-6 |
| FRONT_RACK / FULL / open finger | REFUSED: DEPTH_NOT_ATTAINED | 0.687 | 5.1e-8 | stable (0, 0) | 1.2e-5 |
| OVERHEAD / POWER / closed palm | **FEASIBLE** | 0.678 | 5.4e-8 | stable (0, 0) | 7.0e-6 |
| OVERHEAD / FULL / closed palm | **FEASIBLE** | 0.674 | 5.4e-8 | stable (0, 0) | 1.0e-5 |
| FRONT_RACK / POWER / closed palm (RED) | REFUSED: POSE_NOT_CLOSED | — | 2.8e-3 | — | — |

In every feasible case the independent oracle passes all five checks: static
balance of the whole system and of the bar alone (residual ≤ 3e-13 N),
nonadhesive contact inside the cone with vertical floor normals, a centre of
pressure at least 0.03 m inside the foot polygon (measured 0.12 m), and declared
masses and bar inertia. Every recorded hold frame is admissible. The bar-only
Newton–Euler residual is exact. The whole-system residual falls about 4× when the
step is halved (8.2e-6 → 2.2e-6 N s for front rack, 7.0e-6 → 1.8e-6 for
overhead), which is the O(h²) configuration term. Replay is bit-identical.

Binding limits are reported, not hidden. Wrist extension, girdle protraction and
ankle dorsiflexion sit at their declared bounds less a 0.03 rad margin. The full
front squat is ankle-limited and does not reach FULL depth; heel-raised footwear
is not modelled. Trunk inclination is 0.57–0.58 rad in the front rack and 0.25
overhead, against a refusal limit of 0.60. Overhead elbows are 0.31 rad short of
lockout because shoulder flexion reaches its 3.00 rad bound.

### Defects found by the gates and corrected

1. **Closed-grip front rack.** It is infeasible within declared ranges: elbow,
   wrist and girdle sit at their bounds and the pose stays open by 2.4–2.8 mm.
   Widening the grip (0.28–0.37 m) did not close it, nor did a distal finger
   site. Wrist deviation is binding because the two-site grip forces the hand
   to stay parallel to the bar. This is the 3-D counterpart of the planar
   2.77 rad elbow refusal. It is retained as RED, and an open-hand topology
   is declared instead.
2. **Head interpenetration.** The independent contact-power split found
   head–bar contact injecting **+111.7 J** over 1.5 s. I had misread this as
   servo overconstraint. Bar clearance and an `UNDECLARED_INTERPENETRATION`
   refusal now exist. Afterwards shelf contact dissipates (−0.1 J), and even
   uniform-stiffness arms hold stably.
3. **COM velocity frame.** `mj_objectVelocity` already reports body velocity at
   the COM. My extra shift produced 0.12 N s per step of false momentum
   residual. Fixed, and covered by a test against `mj_jac`.
4. **Overconstrained balance target.** I had forced the COM exactly over
   midfoot. Balance is now certified by the centre of pressure inside the
   support polygon, and the COM target is soft.
5. **QP singularity.** The bar spin row about its own axis is identically zero
   overhead. Zero rows are now removed, with a check that gravity on them is
   also zero.

## 4. Tests and CI

`tests/test_res610_spatial.py` holds 12 tests (106 s; skipped without MuJoCo). They
cover mass and inertia (a falsifier at 1.3× inertia), the COM-velocity frame,
compiled ranges and violation detection, parameter refusals, grip constraint
counts, deterministic replay, feasible closed equilibria and bar orientation,
the shelf load path, and tampered exports: a flipped normal, an adhesive force,
forces scaled ×1.1, an omitted shelf, heel-only support and a zeroed grip are
each REJECTED. Also covered: the closed-palm refusal, the actuator-capacity
refusal at 2×170 kg plates, and the perturbed hold, where dropping the recorded
push is detected. CI job `spatial-mujoco` (Ubuntu and Windows) installs
`mujoco==3.3.7 scipy==1.17.1 numpy==2.4.4` and runs
`python -m scripts.require_spatial_checks`, which fails on any skip.

## 5. Deferred scope

The following remain unmodelled: soft tissue, muscle, scapulothoracic
kinematics, finger joints, bar whip, sleeve friction, footwear and bar motion
within the hand. No front-rack contact-transfer dynamics exist, no catch, and
no clean or snatch. Nothing here is validated against human data.

NUMERICAL_VERIFICATION_ONLY
SIMULATED_MEASUREMENT_ONLY
BIOLOGICAL_VALIDATION=NOT_ESTABLISHED
REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED
