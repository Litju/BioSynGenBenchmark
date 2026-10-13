# I1 model specification: actuated planar multisegment jumper (RES-482)

**Status:** IMPLEMENTED_AND_NUMERICALLY_VERIFIED as mechanics.
MODELED_NOT_EMPIRICALLY_VALIDATED as a human model.
NEUROMUSCULAR_VALIDATED is **not** claimed.
Code: `biosyngen/i1/` (standard library only). The I0 1-D model (`biosyngen/mechanics.py`) is kept unchanged as a separate known-answer fixture.

## 1. Choice of plant (provisional, justified by the target observables)

The target observables are the vertical/horizontal ground reaction, COP, system/body/bar COM, a cable attachment trajectory, and contact-derived events of a jump in the sagittal plane. The smallest plant that can *generate* these from actuation rather than prescribe them is a planar chain with:

* foot, shank, thigh and HAT segments (head-arms-trunk), with **both legs lumped** into one chain;
* a floating base (the ankle position) plus four absolute segment angles, i.e. 6 DOF; a gripped bar adds 2 DOF;
* ankle, knee and hip driven by joint-level Hill-type torque generators (extensor + flexor per joint) with first-order activation;
* a rigid foot with heel and metatarsophalangeal (toe) contact points; unilateral, Coulomb-frictional rigid contact.

**Alternatives considered.**
* *OpenSim/MuJoCo:* not installed on this Windows host, and installing either is a global dependency change that is not authorized. Their contact models (compliant/soft) would add stiffness parameters with no source here.
* *Compliant contact:* rejected for the same reason. Rigid contact with complementarity events makes takeoff an exact zero of a computed force.
* *Custom ODE:* chosen. It is inspectable, testable against invariants, and cross-checked by an independent solver.

**Not represented, so claims about these are refused:** arms (no arm swing), bilateral asymmetry (left/right not identifiable), toes as a segment, muscle-tendon series elasticity and biarticular transfer, spinal/trunk articulation, out-of-plane motion, landing impact.

## 2. Coordinates, conventions and units

* Lab frame +x anterior, +z up; angles in radians, CCW from +x. SI units throughout.
* `q = [x_ankle, z_ankle, φ_foot, φ_shank, φ_thigh, φ_HAT, (x_bar, z_bar)]`.
* Extension-positive joint angles: ankle `θa = φs − φf` (plantarflexion increases it); knee `θk = π − (φt − φs)`; hip `θh = π − (φt − φh)`. Full knee/hip extension = π.
* Each body point is `(x, z) + Σ c_k (cos(φ_k + α_k), sin(φ_k + α_k))`, so positions, Jacobians `J` and the convective terms `J̇q̇ = −Σ c_k φ̇_k² (cos, sin)` come from one expression (`body.py`).

## 3. Equations of motion

`M(q) q̈ + h(q,q̇) = G(q) + Q_joint + Q_coupling + Cᵀλ`, where:

* `M = Σ_b m_b J_bᵀJ_b + I_b e_b e_bᵀ`;
* `h = Σ_b m_b J_bᵀ(J̇_b q̇)`;
* `G = Σ_b J_bᵀ(0, −m_b g)`;
* `Q_joint = Σ_j τ_j ∂θ_j/∂q` (virtual work).

**Contact (KKT).** In each contact state, the accelerations and multipliers come from solving

`[M −Cᵀ; J 0][q̈; λ] = [G − h + Q; −J̇q̇ − 2βJq̇ − β²Φ]`, with β = 40 s⁻¹ (Baumgarte).

`J` are the constrained point directions and `C` the directions the multipliers act in:
* sticking: `C = J`;
* sliding: `C = J_z + σμJ_x`, with σ the sign of the friction force.

| Contact state | Constraints | λ |
|---|---|---|
| FLAT_FOOT stick | heel x, heel z, toe z | heel Fx, heel Fz, toe Fz |
| FLAT_FOOT slide | heel z, toe z (friction μFz) | heel Fz, toe Fz |
| TOE_ONLY stick | toe x, toe z | toe Fx, toe Fz |
| TOE_ONLY slide | toe z | toe Fz |
| FLIGHT | none | none |

**Transitions are roots of physical event functions, located by bisection to 1e-10 s:**

* heel Fz → 0 → heel off;
* toe Fz → 0 → **takeoff**;
* μFz − |Fx| → 0 → slip onset;
* slip velocity → 0 → re-stick;
* heel height → 0 in toe contact → perfectly inelastic heel strike. This is applied as the exact rigid impulse `P = −(J M⁻¹Jᵀ)⁻¹ J q̇⁻`, with the kinetic-energy loss booked as contact work, and is refused above 0.5 m/s.
* In flight, a foot point reaching the ground ends the run:
  * `LANDING_CONTACT_NOT_MODELED` if this occurs after the ballistic apex time `v_to/g`;
  * `TAKEOFF_NOT_SUSTAINED` (a toe tap) if it occurs earlier.
* Toe lift with the heel loaded, impacts that need tangential impulse outside the friction cone, and collapse are refused with named terminations.

**Bar coupling (loaded variant).**
* A point-mass bar is gripped by a central spring between the shoulder point and the bar.
* A damper acts at a *single* point, the HAT material point coinciding with the bar. Both force pairs therefore have zero net moment.
* The coupling stiffness/damping lump grip and soft tissue (ASSUMED).
* *Defect found and fixed during verification:* the first implementation applied the damper between the shoulder and the bar on their relative velocity. That force pair is non-central and created a spurious moment, which the independent Euler-balance test `test_free_flight_obeys_newton_and_euler_for_any_internal_torques` caught (a 30 N·m error).

## 4. Actuation hypothesis (latent; not EMG, not measured drive)

* **Excitation:** `u ∈ [0.01, 1]` from a declared control strategy (`control.py`).
* **Activation (Thelen 2003 form):** `ȧ = (u − a)/τ`, with `τ = τ_act(0.5 + 1.5a)` if u > a, otherwise `τ_deact/(0.5 + 1.5a)`; τ_act = 15 ms, τ_deact = 50 ms.
* **Torque:** `τ = a T0 f_L(θ) f_V(ω/ω_max)`, with:
  * Gaussian `f_L`;
  * Hill hyperbola `f_V = (1−v)/(1+v/k)` for k = 0.25, with `f_V = 0` for v ≥ 1;
  * eccentric branch `E − (E−1)/(1 − cv)`, E = 1.5, c = (1+1/k)/(E−1), so value and slope are continuous at v = 0.
* **Passive end-range:** quadratic springs with damping beyond the declared ROM.
* **Capacity (both legs summed)**, as (T0 N·m, ω_max rad/s):

  | Joint | Extensor | Flexor |
  |---|---|---|
  | Ankle | 390, 36 | 130, 24 |
  | Knee | 585, 30 | 260, 20 |
  | Hip | 650, 27 | 325, 18 |

  **Selection, stated plainly:**
  * With the first-guess capacities (extensor torques 300/450/500 N·m at 24/20/18 rad/s), the best optimized timing took off at only **0.72 m/s**.
  * Before sliding contact was implemented, 120+ random timings gave no sustained flight; their "takeoffs" were knee-lock artifacts.
  * Four capacity scalings were optimized (`runs/i1/res482/capacity_search.txt`). Torque ×1.3 with extensor ω_max ×1.5 gave the highest takeoff velocity (2.39 m/s) and was adopted, because ~2.4 m/s lies in the commonly reported adult CMJ range.
  * This is **calibration to general plausibility, not to any dataset or person**. The joint-level ω_max also stands in for missing tendon decoupling.

## 5. Control hypotheses (declared; phases are not scheduled)

The controller sees only time and whether the foot is airborne. It never sees forces, COM states or event labels.

* **CMJ:** quiet-stance excitations come from exact inverse statics (antagonists at 0.02). At 0.30 s the extensor excitations are released to `release_fraction ×` quiet. Each extensor is then fully excited from its onset time: knee 0.6386 s, hip 0.8324 s, ankle 0.8792 s. Flexors drop to the minimum, and all excitations relax in flight. The timing is the best penalized Nelder–Mead solution maximizing takeoff COM velocity (`scripts/i1_optimize_control.py`). The optimizer chose `release_fraction = 0`.
* **SJ:** a static flexed hold (ankle 68°, knee 100°, hip 90°) followed by maximal extension. The timing comes from the same declared search (`runs/i1/res482/control-sj.json`).
* **Loaded CMJ:** the CMJ strategy re-optimized for the declared bar mass (`control-loaded_cmj-20kg.json`).

## 6. Solver and error control

* Adaptive Dormand–Prince 5(4), rtol 1e-8, atol 1e-10, maximum step 1 ms (also the recording resolution).
* Steps are clipped to land exactly on control breakpoints, and stages are evaluated at the left limit so excitation jumps do not pollute the error estimate.
* Event roots are bisected to 1e-10 s.
* The run stops at budgets with named terminations (`SOLVER_BUDGET_EXCEEDED`, `EVENT_CHATTER`).
* Independent cross-check: SciPy DOP853 (rtol 1e-11) on the same right-hand side, for one stance segment and one flight segment (`tests/test_i1_solver_crosscheck.py`, Python 3.11).

## 7. Parameter provenance

See `biosyngen/i1/params.py::PROVENANCE` and the machine-readable `docs/i1/parameter_provenance.json` (RES-486). **No value is SOURCE_VERIFIED.**
* Anthropometric fractions are Winter (2009) values recalled, not inspected.
* Activation values are Thelen (2003) values recalled, not inspected.
* All capacities, foot geometry, friction and bar coupling are ASSUMED_UNCALIBRATED.

## 8. Non-claims

The model does **not** establish human CMJ kinetics, muscle forces, neural drive, EMG timing, tendon behaviour, inter-individual variability, device fidelity or landing mechanics. The latent excitation/activation states are model hypotheses.

The outputs are **SIMULATED_MEASUREMENT_ONLY**, with **BIOLOGICAL_VALIDATION=NOT_ESTABLISHED** and **REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED**.
