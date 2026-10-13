# RES-491 planar multibody mechanics foundation

_A candidate-neutral mechanics API for articulated planar bodies, independent contacts, load links, and explicit torque hypotheses._

---

## 📐 Coordinates and state

The [PlanarMultibodySystem](../../biosyngen/multibody.py) uses SI units and an inertial sagittal x-z frame:

| Quantity | State | Unit and convention |
| --- | --- | --- |
| Body configuration | (x, z, θ) at each body COM | m, m, rad; +x right/anterior, +z up, positive θ rotates +x toward +z |
| Body velocity | (vx, vz, ω) | m/s, m/s, rad/s |
| Body acceleration | (ax, az, α) | m/s², m/s², rad/s² |
| Body parameters | mass and COM inertia | kg and kg·m²; both require separate provenance records |
| Wrench | force and moment | N and N·m; point of application is an explicit local coordinate |
| Contact | signed gap, normal and tangent force | m, N and dimensionless friction coefficient |
| Time | caller-selected model clock | seconds; no FORCE/LPT device clock is created here |

For N free rigid bodies, generalized coordinates are ordered as q = [x1,z1,θ1,…,xN,zN,θN]. Since each pose is located at the body COM, the mass matrix is block diagonal with blocks diag(mᵢ,mᵢ,Iᵢ). Rigid-body inertia and constraint Jacobians use declared local joint and contact points.

~~~mermaid
flowchart LR
    accTitle: Mechanics and observation boundary
    accDescr: Declared bodies and topology feed constrained dynamics, which returns contact wrenches to separate simulated instruments and the independent evaluator

    model["Body and load definitions"] --> topology["Joints and active contacts"]
    state["COM states and model clock"] --> dynamics["PlanarMultibodySystem"]
    topology --> dynamics
    drive["Declared torque hypotheses"] --> dynamics
    solver["Injectable linear solver"] --> dynamics
    dynamics --> wrench["Per-contact world wrenches"]
    wrench --> instruments["Separate simulated instruments"]
    instruments --> evaluator["Independent benchmark"]

    classDef mechanics fill:#dbeafe,stroke:#2563eb,stroke-width:2px,color:#1e3a5f
    classDef boundary fill:#dcfce7,stroke:#16a34a,stroke-width:2px,color:#14532d
    class model,topology,state,drive,solver,dynamics mechanics
    class wrench,instruments,evaluator boundary
~~~

A contact can bind to a ForcePlateFrame with its own plate ID, LEFT/RIGHT side, vertical channel ID, origin, and axes. Heel/toe contacts can share one plate frame while retaining separate contact IDs. The solver returns each contact wrench in world coordinates and in its declared plate frame. This is a coordinate transform of the latent contact wrench; it does not add acquisition, noise, calibration, or a sampled force trace.

## ⚖️ Constraints and contact transitions

The equations are assembled in maximal coordinates:

$$
M \ddot q = Q_g + Q_{\mathrm{external}} + Q_{\mathrm{actuator}} + C^\mathsf{T}\lambda
$$

Bilateral revolute joints constrain paired local anchor points. Weld joints add a relative-angle constraint. For each active equality row J, the acceleration satisfies J q̈ + J̇ q̇ = 0. Joint and contact multipliers are solved together in a saddle-point system. Position and velocity drift are corrected with mass-metric projections after each integration step.

Each active unilateral contact has signed gap g, normal force λn, and tangential force λt:

- g ≥ 0 prevents penetration
- λn ≥ 0 prevents adhesion
- g λn = 0 enforces complementarity
- sticking is retained only while |λt| ≤ μ λn
- sliding applies λt = −μ λn sign(vt)

Contact normals can be fixed in the world frame or attached to body A/body B. A body-attached normal rotates with that body; its Jacobian, normal rate, and convective acceleration include the frame motion. Body-to-body contact points and normal frame are explicit. This supports declared bar/chest, bar/rack, and catch contacts; the engine never infers a catch from a bar path. Restitution is an explicit contact parameter in [0,1].

The advance method takes a velocity-level contact step and stops at the first detected impact. For a step Δt it solves the post-step velocity v⁺ and the step impulses p of every joint and active contact in one saddle-point system:

$$
M v^+ = M v + \Delta t\,Q + C^\mathsf{T} p,\qquad J_{\mathrm{joint}} v^+ = 0,\qquad J_n v^+ = 0
$$

All supports therefore share the generalized effective mass W = J M⁻¹ Jᵀ, and no per-contact force split is assumed. Each contact mode is then tested for complementarity:

- p_n < 0 is adhesive, so the contact opens.
- A STICK trial adds J_t v⁺ = 0. It is accepted only if |p_t| ≤ μ p_n. Otherwise the contact slides opposite the required impulse.
- A SLIDE uses p_t = −s μ p_n. It is accepted only if the post-step slip J_t v⁺ has sign s. If the slip reverses within the step, STICK is tried.

Slide→stick capture therefore happens exactly when static friction is feasible. It needs no velocity threshold or sign heuristic. A repeated active set, or a singular or rank-deficient system such as two sticking points on one rigid body, raises ConstraintSolveError. These cases are never regularized. Positions are then advanced with v⁺ (symplectic Euler) and projected in the mass metric. StepResult reports each contact's step impulse (p_n, p_t). Its contact_work_j is the impulse work at the post-step relative velocity: zero for stick and −μ p_n |v_t⁺| for slip. The stepper emits CONTACT_RELEASE, SLIP_ONSET and STICK_CAPTURE events. Once the step reaches an impact, it linearly interpolates the closing gap within that step; reducing the step size must reduce event-time error. The resolve_impact method solves a simultaneous impulse problem with restitution and the same Coulomb active-set check. Its contact_work_j is the exact kinetic-energy difference across the velocity jump. An impact that creates kinetic energy or starts with penetration is refused.

The contact lifecycle is:

~~~mermaid
stateDiagram-v2
    accTitle: Contact state lifecycle
    accDescr: A contact is open until geometric closure, sticks while friction is feasible, slides at the Coulomb bound, and opens when support force is nonpositive

    [*] --> Open
    Open --> Impact: Closing surfaces
    Impact --> Stick: Friction feasible
    Impact --> Slide: Friction saturated
    Open --> Stick: Touching and stationary
    Stick --> Slide: Coulomb bound reached
    Slide --> Stick: Slip reverses and stick impulse inside cone
    Stick --> Open: Normal force nonpositive
    Slide --> Open: Normal force nonpositive
    Open --> [*]
~~~

SupportContract names the contacts allowed and required for a protocol phase. If the active-set solution releases a required support, evaluation fails with SupportIncompleteError. This keeps an incomplete support path from being reinterpreted as a valid exercise topology.

## 🦿 Loads and actuation

PlanarJoint describes point connections and records a connection kind such as GRIP, BACK_RACK, or LOAD_LINK. World or body-to-body ContactSpec records separately represent FOOT, HEEL, TOE, BAR_FLOOR, BENCH, CHEST, RACK, and CATCH interactions. Geometry attaches to actual local body points; no load is silently replaced by a mass at a guessed COM location.

ActuatorHypothesis provides a signed excitation in [−1,1], bounded joint torque capacity, activation/deactivation time constants, provenance, and a fractional uncertainty field. Activation follows the exact first-order update a(t+Δt)=u+(a(t)−u)e^(−Δt/τ). Torque is a T_max; capacity uncertainty bounds are reported but not propagated into state uncertainty. This is a declared torque-source hypothesis, not a muscle-tendon model, EMG estimate, or neural-drive claim.

ReferenceAngleController is a saturated PD reference controller for software fixtures. Gains and target are explicit and provenance-tagged. The controller reads only the declared joint angle and rate, not contact forces or event labels. The test fixture proves control feasibility, not human strategy.

## 🧪 Verification and reproduction

The fresh RES-491 tests are in [test_i2c_multibody.py](../../tests/test_i2c_multibody.py). They compare free-body Newton/Euler balance with an independently computed wrench sum; check the mass matrix against kinetic energy; build a manufactured HAT/thigh/shank/foot model with separate left/right plate frames and an additional rack support; exercise bar-floor release, support transfer, unilateral release, friction saturation, simultaneous two-foot impacts, rotating body-attached catch normals, impact energy, joint constraints, actuator uncertainty, controller response, and invalid topology/parameter inputs.

Run on Windows with CPython:

~~~powershell
python -m unittest discover -s tests -p test_i2c_multibody.py -v
~~~

The linear solver is injectable. The default is the existing standard-library dense Gaussian solver; a custom solver can be supplied with a solver identity, which is retained in each dynamics result. Callers may also use solve_accelerations and resolve_impact from a different integrator. The included advance method is a small reproducible default, not a permanent solver or integrator choice.

## 🚧 Scope and claim ceiling

The API has no default anthropometry, segment mass fractions, torque values, or neural strategy. Tests use manufactured rigid bodies explicitly labeled MANUFACTURED_TEST_FIXTURE. Any exercise-specific human parameters remain ASSUMED_UNCALIBRATED until a source and uncertainty are admitted.

The body model is planar and rigid. Contacts are point contacts with world-fixed or body-attached normals; it does not model 3D torsion, compliant tissue, muscle fibres/tendons, or anatomical multi-axis joint limits. The dense constraint solve fails closed on redundant/singular constraint sets. Contact event times are step-size-convergent estimates, not certified error bounds. No Plate2bar code or acceptance output is imported, and no device force/LPT series is synthesized here.

Permitted claim ceilings remain NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY, BIOLOGICAL_VALIDATION=NOT_ESTABLISHED, and REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.

## ✅ Stick–slip defect resolution (2026-10-13)

**History.** The owner-authorized third repair replaced the unstable mass/inertia error wording with a typed `ParameterDomainError` (`field` = `mass`/`inertia`, `reason` = `NONNUMERIC`/`NONFINITE`/`NONPOSITIVE`). Six reviewer falsifiers followed. The sixth exposed a P0 defect: the force-level explicit stepper never entered STICK after sliding to rest. It alternated slip direction every step, leaving |v_t| ≈ μ·g·Δt.

**Repair.** The superseding owner authorization allowed up to four corrective iterations. The first iteration replaced force-level explicit integration in `advance` with the coupled velocity-level Coulomb impulse step above. `solve_accelerations` (the instantaneous force-level wrench) is unchanged.

**Independent falsifiers** are in `VelocityLevelCoulombFalsifierTests`. Every expected value is a closed form or a symmetry, never a value read back from the solver:

| Check | Expected (independent) |
| --- | --- |
| Slide→stick for Δt = 2, 1, 0.5, 0.25 ms | Capture within one step of t_stop = v₀/(μg). The slip sign never changes. STICK and v = 0 hold for ≥ 70 steps after capture. Stopping-distance error to v₀²/(2μg) falls monotonically below 2·10⁻⁴ m. |
| Lateral load at 0, 0.6 and 0.999 of μmg | STICK for 150 steps with zero displacement and p_t = −FΔt |
| Lateral load at 1.001 and 1.5 of μmg | SLIP_ONSET in the first step, p_t = −μmgΔt, v = (F − μmg)t/m |
| Near-zero or reversing slip under loads beyond the cone | Never a false STICK. Kinetic friction opposes the post-step slip. |
| Incline at 20° with μ = 0.5 | Sticks with zero travel |
| Incline at 35° with μ = 0.5 and with μ = 0 | Travel equals ½g(sinθ − μcosθ)t² within O(Δt) |
| Two sticking points on one rigid body | ConstraintSolveError (rank-deficient) |
| Two articulated legs with sticking feet, plus a rack on the torso | Pushing into the rack: rack impulse 10Δt, each foot normal 3.5gΔt, foot tangential impulses 0, body at rest. Pulling away: rack releases; equal foot tangential impulses. |
| Coupled-solve momentum | Impulse–momentum closure exact to 10⁻¹². The subsequent projection drift is < 10⁻³ relative. |
| Mirror geometry and reversed contact declaration order | Identical normal impulses, mirrored tangential impulses |
| Bar on the floor gripped by a revolute hand | Below μ(m_bar + m_hand)g: STICK. Above it: SLIDE with p_t = −μp_n and exact two-axis momentum closure. |
| Block on a rotating, pin-supported platform (BODY_B normal) | Holds STICK with relative v_t < 10⁻⁹, then slips outward before the static friction angle. Mirrored runs give equal onset times and opposite angles. Total mechanical energy never increases. |

Every step of these falsifiers also checks p_n ≥ 0, |p_t| ≤ μp_n, contact work ≤ 0 and gap > −10⁻⁹ m.

**Mutation controls** were run ad hoc and are not stored in the suite. With the slip-reversal STICK trial disabled, 8 checks fail, including positive contact work (energy creation). With the static-cone test disabled, 9 checks fail.

**Results.** Focused tests: 35 run, all pass. Full suite: 156 run, OK, with 2 optional SciPy skips. All earlier RES-491 tests and reviewer falsifiers still pass unchanged.

**Limits.** These are rigid point contacts with one friction coefficient (static = kinetic). The Coulomb active set is solved by deterministic pivoting with cycle refusal, not a general LCP solver, so multi-contact problems without a unique solution fail closed instead of being resolved. Event times are step-size-convergent estimates. All fixtures are manufactured. No human or device validity is claimed.
