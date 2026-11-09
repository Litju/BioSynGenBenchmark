"""RES-610 spatial (3-D) athlete-barbell feasibility backend.

A narrowly scoped adapter over MuJoCo, separate from the planar I0-I3
``biosyngen.multibody`` engine, whose qualified scopes and SOURCE/TRUTH
contracts are untouched. The barbell is an independent 6-DOF rigid body;
hands couple to it through declared connect constraints; front-rack support
is separable bar-shoulder contact. All segment and actuator parameters are
ASSUMED_UNCALIBRATED manufactured values, not measured anatomy.

NUMERICAL_VERIFICATION_ONLY. A static receiving pose is not an Olympic lift.
"""
CLAIM_CEILINGS = (
    "NUMERICAL_VERIFICATION_ONLY",
    "SIMULATED_MEASUREMENT_ONLY",
    "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED",
    "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED",
)
