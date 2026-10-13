"""I2 exercise-specific simulators built on the RES-491 planar multibody engine.

The generator owns latent truth. Simulated instruments live in
``biosyngen.i2.instruments`` and the independent evaluator never imports this
package. Claim ceilings: NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY,
BIOLOGICAL_VALIDATION=NOT_ESTABLISHED, REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.
"""

CLAIM_CEILINGS = (
    "NUMERICAL_VERIFICATION_ONLY",
    "SIMULATED_MEASUREMENT_ONLY",
    "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED",
    "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED",
)
