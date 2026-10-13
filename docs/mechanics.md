# I0 one-dimensional mechanics

## Context of use

This module is a manufactured numerical mechanics pilot. It verifies a declared one-dimensional vertical force law and piecewise contact schedule. It is not a human movement model, a CMJ simulator validated against athletes, or a force-plate observation model.

## Coordinates and units

- The inertial laboratory z-axis points upward.
- Position is in metres, time in seconds, mass in kilograms, force in newtons, and acceleration in metres per second squared.
- Gravity is a positive magnitude g; its acceleration contribution is -g.
- The body centre of mass is the moving reference coordinate. An optional bar/load has a declared constant vertical offset and is rigidly coupled to the body in this pilot.

## Governing equations

For body mass m_b and optional rigidly coupled bar/load mass m_l, the system mass is M = m_b + m_l. Each phase declares a constant total normal support force N. The same force acts on the declared constrained system:

M a_z = N - M g

Within stance, N is a prescribed nonnegative input. Within ideal flight, N = 0, so a_z = -g. For each constant-force step of duration h, the exact update is:

v_next = v + a_z h

z_next = z + v h + 0.5 a_z h^2

The sample interval controls output spacing only. The integrator shortens steps at every phase boundary and includes that boundary exactly. Position and velocity remain continuous at stance-to-flight transitions; this finite-force model does not create an impact impulse.

When a bar/load is declared, its attachment coordinate is z_l = z_b + d, where d is the fixed offset. The constrained system centre of mass is (m_b z_b + m_l z_l) / M. Body, attachment, and system coordinates are returned separately.

## Contact and event rules

The phase schedule is an explicit prescribed contact hypothesis, not contact inferred from hidden solver state. The first phase is stance. Stance force must be nonnegative; flight force must be exactly zero. Mode is right-continuous at a phase boundary, and each stance-to-flight boundary is returned as an event. Flight-to-stance is rejected because landing impact and impulse semantics are not implemented.

## Assumptions and refusals

The model has one vertical translational degree of freedom, a prescribed support force, one rigidly coupled optional load, constant gravity, and ideal force continuity between samples. It has no foot geometry or penetration state, active muscle/neuromotor control, segment articulation, horizontal or rotational motion, impact, real sensor model, biological variation, or empirical parameters. A prescribed force is not an inferred muscle force.

Exact-solution test tolerances use a conservative binary64 roundoff scale: 64 times machine epsilon, the number of accumulated samples or phase operations, and the largest compared state magnitude. The manufactured fixture reports its observed maximum absolute position and velocity errors against independent piecewise analytic equations. These are numerical checks, not empirical biomechanical acceptance thresholds. Outputs are NUMERICAL_VERIFICATION_ONLY and SIMULATED_MEASUREMENT_ONLY; biological and real paired FORCE/LPT validation remain unestablished.
