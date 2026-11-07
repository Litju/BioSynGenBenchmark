# RES-597 movement expansion evidence

Base main: `efe3cdf53fc89aa5b54aa9a1cb4ceaeaf03ae851`.
This is partial implementation evidence, not scientific acceptance.

## Finite contact implementation

`biosyngen/compliant_contact.py` provides a nonadhesive Hunt–Crossley law
and a multibody adapter. Selected world contacts are removed from the rigid
contact solve and applied as finite point forces. Other rigid contacts and
articulated constraints remain in the existing solver. Body-body compliant
contact is explicitly refused. Contact response is evaluated at interval start;
the adapter is first order and requires refinement. Tangential creep is inherent
to its regularized Coulomb law. Compliant states use the existing SLIDE vocabulary
to denote finite-contact engagement, not a rigid sticking solution.

Equation reference: [Simbody HuntCrossleyForce documentation](https://simbody.github.io/simbody-3.6-doxygen/api/classSimTK_1_1HuntCrossleyForce.html).
The implementation was independently written from the stated equations; no
third-party source or data were copied. Parameters are ASSUMED_UNCALIBRATED.

Manufactured 1 kg vertical drop: height 0.1 m, gravity 9.81 m/s², linear
contact stiffness 10,000 N/m, damping coefficient 0.3 s/m, duration 0.3 s.
Initial mechanical energy is 0.981 J. No rigid contact impulse is applied.

| dt (s) | Integrated normal impulse (N s) | Final mechanical energy (J) | Dissipation (J) | Peak finite force (N) | Energy closure error (J) |
|---|---:|---:|---:|---:|---:|
| 0.0004 | 2.76910586970008 | 0.543570522806797 | 0.439273920697840 | 141.410316686470 | 0.001844443504637 |
| 0.0002 | 2.76970479207306 | 0.543801662840496 | 0.438089174599959 | 141.094523421595 | 0.000890837440455 |
| 0.0001 | 2.76998712986581 | 0.543902930369626 | 0.437526992178365 | 140.935206663717 | 0.000429922547991 |

Momentum closes against integrated force and gravity within 1e-10 N s.
Energy closure decreases under both refinements; this does not establish human
tissue or footwear parameters. The separate zero-damping case checks energy
against the analytic conserved value with a 0.003 J integration budget.

## Articulated drop/rebound candidate

`build_drop_jump_candidate` starts immediately after elevated support release
at rest, 0.15 m above the ground. It does not model the preceding platform
support interval. The existing articulated human, joint actuators and solver
are retained. Foot contact has default stiffness 500,000 N/m^1.5, exponent 1.5,
damping 0.5 s/m, friction 0.8, and slip transition 0.01 m/s.

At dt=0.0002 s: first contact 0.1750 s, COM reversal predicate 0.2128 s,
both feet released by 0.3060 s, second landing begins at 0.7752 s.
The 0.5 s candidate passes momentum closure within 1e-7 N s at both
0.0004 and 0.0002 s. Contact time differences remain visible in the tests.
The full 1.5 s request terminates at about 1.2886 s with
`BODY_GROUND_CONTACT_NOT_MODELED: left_knee, right_knee`.
Final recovery is therefore NOT_QUALIFIED; RSI is not exported.
The ordinary `DROP_JUMP` protocol refusal is preserved. This experimental
builder is not admitted into the paired-source registry.

## Retained blockers and claim boundaries

Olympic full catches remain unsupported: spatial shoulder rotation, wrist and
hand mechanics, feasible front-rack/overhead receiving, catch impulse transfer,
stabilization and standing recovery are absent. Existing pull-only models remain
available and are not renamed as completed lifts. Bench unrack/rerack stability
and whole-system observability remain bounded by the existing dossiers.

## Reproduction and failed evidence

```
python -m unittest discover -s tests -p test_res597_compliant_contact.py -v
python -m unittest discover -s tests -p test_res597_drop_candidate.py -v
python -m unittest discover -s tests -p test_res597_jump_events.py -v
python -m unittest discover -s tests -p test_i2c_multibody.py -q
python -m unittest discover -s tests -p test_i2d_jumps.py -q
```

Constitutive/integration tests: 6 pass. Candidate tests: 2 pass, including
the retained final-recovery failure witness. Existing multibody tests: 35 pass.
Initial integration attempts exposed missing fixture plate identity, missing
restored contact-state identities, and a missing signed slide direction; these
were corrected without relaxing physical assertions. The candidate momentum
check initially used 9.81 instead of the model's declared gravity; it was
corrected to read the declared parameter rather than loosen the tolerance.

Disk budget remains 1 GiB for generated campaign artifacts; no participant data
or external recordings are admitted. New source/tests/docs contain only owned
code, factual equations and manufactured worlds. Independent final-SHA review,
Linux and exact-head hosted CI remain required.

NUMERICAL_VERIFICATION_ONLY
SIMULATED_MEASUREMENT_ONLY
BIOLOGICAL_VALIDATION=NOT_ESTABLISHED
REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED
