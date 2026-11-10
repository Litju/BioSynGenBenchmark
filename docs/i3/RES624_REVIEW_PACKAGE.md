# RES-624 scientific review package (contact-only grasp and nominal supported overhead hold)

Claim ceiling: NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY, BIOLOGICAL_VALIDATION=NOT_ESTABLISHED,
REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED. Dynamic clean-snatch catch: NOT_QUALIFIED.
This package is prepared by the implementing agent. It is **self-review, not independent external review**.

Code head qualified by the committed evidence: `2851de84bc21ddcf28b77d0b60562891d87eb622` (receipts record
`worktree_clean=true`). The evidence commit that follows adds only `runs/i3/res624/{qualification,gateF,gateG,canary}`.
`runs/i3/res624/gateDE` predates the Gate G repair and is retained as historical Gate E evidence (its 70/80 kg
refusals are explained by item 10).

| # | Review item | Where / result |
|---|---|---|
| 1 | Original RED and preserved failure | `runs/i3/res618/fullbody_receiving_failure.json` untouched; `scripts/res618_fullbody_qualification.py` untouched (hardcoded FAIL). `runs/i3/res624/canary/red_canary.json`: reproduced, bar drop 0.3761 m, 0 hand contacts, only hindfoot/floor contacts. |
| 2 | Contact-topology diagnosis | `runs/i3/res624/gateA` (unchanged). |
| 3 | Bar-floor collision policy | `runs/i3/res624/gateB` (unchanged). |
| 4 | Isolated grasp / force closure | `runs/i3/res624/gateC` (unchanged). |
| 5 | Whole-body static certificate | generalized LP, 12 hand contacts, max actuator utilisation 0.359 (nominal), COP inside hull. |
| 6 | Bounded forward hold | `qualification/receiving_qualification.json`: nominal 0.3 s, 2 s, 10 s qualified; bar displacement in hand 0.28/1.2/1.35 mm vs 14 mm bound; hand support 1.0000 of bar weight. |
| 7 | Momentum, angular momentum, energy | `gateF/numerical_verification.json`: bar linear step residual 1.7e-16 N s; system step residual 6.6e-6 N s (bound 1e-4 W dt), cumulative 6.2e-8 N s; bar angular residual 4.8e-7 N m s = derived rigid-body discretisation bound (excess 0). Impulses over 0.3 s: hand-on-bar 58.852 vs weight 58.860 N s; floor 279.571 vs system weight 279.585 N s. Energy ledger (signed): actuator +0.2303 J, passive -0.0074 J, contact/constraint -0.3957 J, external 0; residual -7.2e-5 J = -0.0085 % of 0.845 J gross flux. The earlier 0.49 % residual came from omitting rotor (armature) kinetic energy and start-of-step work; both are now explicit. |
| 8 | Timestep convergence | 1.0/0.5/0.25 ms all qualified. Bar trajectory difference 1.54e-5 m then 7.93e-6 m (ratio 1.94); energy residual 7.2e-5, 3.6e-5, 1.8e-5 J (observed order 0.997/0.998, first order). Bitwise deterministic replay on one platform. Tolerance 1e-8 insensitive. |
| 9 | Negatives | 6 adversarial manipulations (deleted contacts, friction violated, capacity exceeded, undeclared bar support, non-foot floor support, uncertified state) all refused (6/6) plus frozen RED refused; finger release: pre-release qualified, contact loss detected; 50 % arm-capacity threshold: refused as ACTUATOR_INFEASIBLE with contact lost. Post-impact segments of those two runs also trip the balance tolerance; that is expected (impact transient) and is not used to qualify anything. |
| 10 | Load and anthropometric sensitivity | 15/20/25 kg: model mass, subtree mass and engine gravity bias agree (`gateG`), all qualified. 70 kg/1.70 m and 80 kg/1.80 m: root cause was `qualified_relation()` measuring the shaft position on the *nominal* hand while the pose solver applied it to stature-scaled hands (2.6 mm hand penetration for 70 kg; only 4 hand contacts for 80 kg). The relation is now measured with the subject's own hands; no limit, strength or anthropometric parameter changed; the pre-repair behaviour is retained as a refused control (`tests/test_res624_anthropometry.py`). Both now certify and hold 2 s. |
| 11 | Windows/Linux | `scripts/res624_compare_platforms.py` in CI job `spatial-platforms`: identical categorical verdicts, numeric fields within declared tolerances, no bitwise requirement. Hosted result: see PR #15 checks. |
| 12 | Limitations | Qualification is for the declared **elliptic-cone** contact model (impratio 10, Newton, tol 1e-10). The same hold under a **pyramidal** cone is `NUMERICAL_NOT_QUALIFIED`; it must not be called contact-model independent. Single synthetic anthropometric family, 20 kg unplated bar, static hold only, no lift or catch, no soft tissue, no biological or device validation. Energy residual first-order in dt (integrator), not eliminated. The certificate's pose solver reports `optimizer_success=False` (kinematic residual ~0.07-0.10); admissibility rests on the equilibrium certificate and contact checks, not optimizer convergence. |

Reproduce (Python 3.12, mujoco 3.3.7, scipy 1.17.1, numpy 2.4.4):
`python -m scripts.res624_red_canary`, `python -m scripts.res624_receiving_qualification`,
`python -m scripts.res624_numerical_verification`, `python -m scripts.res624_anthropometric_diagnosis`.
Superseded, never-committed pre-fix outputs (invalid; predate the cached-mass correction):
`receiving_qualification.json` sha256 `646e1f4120b3daf2d15d27687f7662c09056e18a5383ba6a071c81ad0a675304`,
`numerical_verification.json` sha256 `9c4a6b83152b644793368b7c432d540986f0f222b109f23fe97c22c7e97e5ced`.
