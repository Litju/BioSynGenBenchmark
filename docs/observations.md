# I0 simulated FORCE and cable-LPT observations

## Source and claim boundary

These forward models consume only the declared I01 latent trajectory and emit synthetic observations. Every series is labeled SIMULATED_OBSERVATION. It is not vendor ADC/raw export, real human data, calibrated paired FORCE/LPT data, or evidence of biological or device fidelity.

## Independent native clocks

Each sensor has its own native sample period, time offset, and linear rate drift in parts per million. For sample index k, native time is offset + k times the native sample period; the simulation query time is k times that period divided by the drift-adjusted rate. FORCE and LPT keep separate sample records and separate native time vectors. Callers validate each vector against its declared clock; array index never implies cross-device synchronization.

Clock offsets define the sensor-local time reading at the first simulated sample. This small model has no nonlinear clock wander, timestamp wrap, trigger jitter, or dropped-clock tick.

## FORCE

The primary observation is the total declared vertical support force, in newtons, in the inertial lab frame, positive upward. Its latent input is zero in I01 flight; offset or noise can make an observed flight sample signed. Each channel applies additive offset and seeded Gaussian noise, rounds to the configured quantum using Python's round-to-even rule, then clips to symmetric saturation limits. Missing samples retain their timestamp and carry no value.

Left/right channels are emitted only when two virtual foot channels and an explicit left-force fraction are declared. The split helper returns left and right values whose sum is the declared total before channel errors; the two plate channels then receive independent seeded noise streams. A combined force trace never implies a bilateral split.

## Cable LPT

The LPT target is explicitly selected as body COM or the declared bar attachment. The anchor is fixed in the x-z plane. Cable length is the Euclidean distance from anchor to attachment; cable rate is attachment velocity projected onto the cable unit vector; angle is measured from +x toward +z. Cable rate is not vertical velocity, and cable length is not vertical position.

The observation can apply a zero-length reference, length/rate/angle offsets, seeded Gaussian noise, quantization, saturation for length and rate, and missing samples. Each signal applies offset and noise, quantizes using round-to-even, then saturates. Missing samples retain native time and carry no length, rate, or angle value. A coincident anchor and attachment is refused because cable direction and rate projection are undefined there.

## Simplifications

Clocks are affine and sensor error is additive. Calibration is a declared synthetic assumption, not a metrological claim. There is no vendor protocol fidelity, cable slack model, cable elasticity, anchor movement, horizontal body dynamics, or real cross-device synchronization. Observation outputs do not include generator latent truth fields; I03 independently chooses which public measurements and assumptions are admissible for each benchmark claim.
