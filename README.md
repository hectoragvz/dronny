# Dronny

A 2D quadrotor physics simulation with PID control.

## Files

### `libs/math/include/math/vec2.hpp`
A 2D vector struct with `x` and `z` components (horizontal and vertical). Supports addition, subtraction, scalar multiplication, dot product, and normalization. Used throughout the simulation to represent position and velocity in the x-z plane.

### `sim/include/sim/quadrotor_model.hpp`
The physical model of the drone. State is four values: `position`, `velocity`, `pitch`, and `pitch_rate`. The `step(dt, T1, T2)` method advances the simulation by `dt` seconds given left motor thrust T1 and right motor thrust T2.

Internally it applies Newton's second law in both translational and rotational form:
- `ax = -T·sin(θ) / m` — horizontal acceleration from tilt
- `az = T·cos(θ) / m - g` — vertical acceleration from thrust minus gravity
- `α = τ / I` where `τ = (T2 - T1) · L` — angular acceleration from torque

State is integrated forward using forward Euler: acceleration → velocity → position, and angular acceleration → pitch rate → pitch.

### `sim/include/sim/sim_clock.hpp`
A minimal clock that tracks simulation time. Holds a fixed timestep `dt` and a running `current_time`. Call `tick()` each iteration to advance time.

### `control/include/control/pid.hpp`
A PID controller. `compute(error, dt)` returns a control signal using the standard formula:

`output = Kp·e + Ki·∫e dt + Kd·(de/dt)`

The return value is a dimensionless correction — its physical meaning (acceleration, force, thrust differential) is determined by how the caller uses it.

### `sim/include/sim/sensors.hpp`
A simulation of both a barometer and a gyroscope to add noise and randomness to sensor readings. The barometer adds Gaussian noise to altitude readings. The gyroscope adds Gaussian noise plus a small random drift to pitch readings.

### `estimation/include/estimation/ekf.hpp`
A scalar (1D) Kalman Filter that estimates altitude. Alternates between two steps:
- **Predict** — uses vertical acceleration and `dt` to project the state forward; uncertainty grows by `Q` (process noise)
- **Update** — incorporates a barometer measurement; computes Kalman gain `K = P / (P + R)` to blend prediction and measurement, then shrinks uncertainty

Key variables: `state_variable` (X) is the altitude estimate, `state_covariance` (P) is confidence in that estimate, `process_noise` (Q) is unmodeled physics uncertainty, and `measurement_noise` (R) is distrust of the barometer.

### `apps/sim_main.cpp`
Three simulation scenarios:
1. **Hover** — equal thrust on both motors, verifies the drone stays stationary.
2. **Unequal thrust** — fixed asymmetric thrust, drone pitches and drifts.
3. **PID + EKF control** — two PID controllers (altitude and pitch) compute T1 and T2 each timestep. A Kalman Filter fuses barometer readings with a vertical acceleration model to produce a smoothed altitude estimate, which feeds the altitude PID. Total thrust and thrust differential are solved as a system of two equations to simultaneously satisfy altitude and pitch targets.
