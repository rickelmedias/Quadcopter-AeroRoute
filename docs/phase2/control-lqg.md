# Phase 2 / control-lqg: tethered LQR + Kalman test mode

Branch `phase2/control-lqg`, created from `phase2/01-bench-hmi`. It adds a
tethered test mode to the bench firmware and a **Controle** tab to the bench HMI:
arm, take off, hold a height (default 30 cm), land, disarm, and a red STOP.

Status: verified on the host only (C closed-loop test against the model's own
plant, Python/simulator tests). **Not flashed, not flown.** See "Before the first
test with motors".

## What runs on the STM32

```text
BNO085 + MTF-01 -> lqg_meas -> y[9] -> Kalman (generated) -> x̂[12]
                                      Δω = Kfd·r − Kd·x̂ (generated)
                  lqg_ctrl: sequence, trim, limits, mixer -> 4 motors (M1..M4)
```

- `stm32/App/Src/control/lqg_model/`: the Simulink Coder output of
  `references/Cod_C_STM32_V2` (model `STM32` v2.18, Ts = 0.01 s), copied
  **unchanged** (SHA-256 in its README). Regenerate in MATLAB and copy again to
  change gains, `Q`/`R` or the model.
- `control/lqg_meas.c`: sensors to the model's measurements
  `y = [z φ θ ψ u v p q r]` (NED/FRD, SI). Axis map, signs, level zero and flow
  scales are the ones measured on the real drone in the submitted firmware
  (`flight_config.h` on `submission/2026-11-11`); see `control/lqg_config.h`.
- `control/lqg_ctrl.c`: pure state machine around the generated step.
- `app/control_task.c`: FreeRTOS task, 100 Hz, owns the controller.
- `actuators/motor_arbiter.c`: new owner `CONTROL` (direct output, no ramp).

State vector `[x y z φ θ ψ u v w p q r]`, z down. `Out1` is Δω in rad/s per model
input; each becomes `motor ‰ = hover + Δω / 1.755` (provisional curve
ω = 1755 rad/s at 100 %, from the submission). Model inputs map to physical
motors `u1→M4, u2→M2, u3→M1, u4→M3` (same as the submitted `mixer.c`; M4/M2 in
front, M3/M1 behind; M1 and M4 reversed in the ESC).

### Sequence

| State | What happens |
| --- | --- |
| armed | Motors at zero, the output keepalive is still refreshed, the ground distance is averaged (≥ 50 samples needed). |
| spool | Base throttle rises from 5 % (15 %/s up to hover − 10 %, then the "ramp to lift-off" rate) with the Kalman locked to the measurements; the motor differential is limited to ±3 % only during the fast ramp. Lift-off counts at > 3 cm for 50 ms (MTF lever arm compensated): the base drops to the estimated weight-balancing throttle and the height reference starts from the measured height. While armed on the ground, the mean φ/θ becomes the level zero of the flight (if within ±5°). |
| climb / hover | Kalman free. The height reference slews to the target at the climb rate. Hover = reference reached. |
| land | Reference descends, throttle is also biased down (up to 80 ‰) so a wrong hover still descends. At < 5 cm the throttle falls to zero and the output disarms. |

At lift-off the base throttle of that moment (already corrected for the ramp)
becomes the hover; the typed hover only decides where the fast ramp stops. See
"Climb fix" below.

### EKF, integrators and faster lift-off (second iteration)

> Superseded on branch `phase2/control-lqg-quaternion` by the quaternion EKF3
> below. `control/ekf.c` (12-state Euler EKF) still exists on `phase2/control-lqg`.

Motivation, from the first tethered session (`20261111_203159`, 4.4 s airborne):
the drone drifted sideways, never passed ~4 cm and took 11 s to lift off. The log
showed consistent sensor signs and restoring motor commands, but: (1) a yaw
offset of ~100° with a constant yaw differential of +46 ‰ (P-only yaw loop, and the
generated model is linearised at ψ = 0, so with that heading its x/y commands push
the wrong way); (2) optical flow with quality 28–64, 22–27 % zeros and spikes
equal to several m/s, which the fixed-gain Kalman trusts; (3) hover really near
47–50 % while the parameter was 55 %, so the ramp from 5 % at 4 %/s needed 11 s.

- `control/ekf.c`: 12-state EKF with the same states and the generated `A`, `B`.
  Differences from the generated Kalman: the gravity-tilt acceleration is rotated
  by ψ (nonlinear transition); the MTF distance is predicted as
  (ground + height) / (cos φ cos θ) and the flow is compared with the state
  velocity rotated into the body frame; covariance `P` is propagated, so a missing
  measurement lets `P` grow; every range/flow scalar goes through a chi-square
  innovation gate (9 for flow, 16 for range), the flow variance grows as
  (100/quality)² below quality 100 and flow below quality 20 is dropped. After 25
  consecutive rejections the measurement is accepted with 10× variance to
  re-anchor. Order per step: update → control → clamp → predict with the Δω
  actually applied. Noise values are constants at the top of `ekf.c`.
- Control: `Δω = Kfd·r − Kd·x̂` with the generated gains, but position and velocity
  are rotated into the heading frame (by ψ̂) before `Kd`.
- XY and yaw integrators: states `ix, iy` (m) and `ipsi` (rad), "equivalent offset"
  integrators that reuse the `Kd` columns of x, y and ψ. Only in climb/hover, not
  while a motor is clamped, limited to ±1 m and ±2 rad. Gains `ki_xy` (default
  0.05 /s) and `ki_yaw` (0.3 /s) are HMI-editable and live.
- Lift-off: the throttle ramps at 15 %/s until `hover − 10 %`, then at the
  configured slow rate. With hover set near the real value this takes a few seconds
  instead of 11.
- "Usar EKF" off selects the generated stationary Kalman (no heading rotation, no
  integrators) for comparison. It is latched at Arm.

Closed-loop host results against the model plant with the physically rotated
gravity (`tests/test_lqg_ctrl.c`, 60–120 s runs): with a yaw torque that leaves the
heading ~55° off, the generated Kalman loses the hover (cut by the tilt limit)
while the EKF stays within 0.4 m; with MTF spikes of ~4.5 m/s every 0.37 s the
generated Kalman drifts ~7 m and the EKF 0 m; the yaw integrator brings ψ back to
~0 and the XY integrator removes the velocity drift of a 1.7° level error
(position stays dead-reckoned, so an offset of ~0.2 m remains: XY position has no
absolute measurement). Not validated on the real drone.

### LQR gain multipliers (third iteration)

The tethered sessions showed an effective height gain of ~1.5 % throttle per metre
(LQR weights `R = 0.1` on Δω in rad/s make the design very soft). The **Ganhos do
LQR** card in the Controle tab scales the columns of the generated `Kd` in eight
groups (height P/D, roll-pitch P/D, yaw P/D, XY P/D), live, EKF only (`CTRL_GAINS`,
saved in `config.json` and re-sent each session). The control uses the error form
`Δω = −Kd·G·(x̂ − x_ref)`, identical to `Kfd·r − Kd·x̂` at G = 1 because the generated
`Kfd` and `Kd` z columns are equal. Host test: with the hover throttle 3 % wrong and
no trim, height error is 0.18 m at 1× and 0.04 m at height P 10× / D 5×. This is a
tuning aid, not a substitute for redesigning `Q`/`R` in MATLAB with identified
`Kt`/`Kq`.

### Yaw and pitch feed-forward (14 Nov 2026)

Yaw P/D at 10× and the yaw integrator at 1.0 /s were tested (`20261114_165040`,
`20261114_165114`) and **did not help**: the spin started the same way when the base passed
~600 ‰ (still 3 to 6 cm above the ground), the rate reached −216 and −322 °/s, the controller
produced one-step spikes of ±300 to 400 ‰ (M4 262 / M3 888 in one sample) and the second run was
cut by the tilt limit. From the acceleration after the spike the yaw authority is about 3 % of
the model (the earlier regression over eight windows gave 20 %), far too little for the
differential to cancel the counter-clockwise torque, which grows with throttle. Defaults are
back to 5× and 0.3 /s (`hmi/benchlink/protocol.py`, `hmi/data/config.json`).

The same logs show the drone tilting backwards while the throttle rises: θ grows to about
+7° (nose up) as the base goes from 560 to 610 ‰, with the rear motors 40 to 56 ‰ above the
front ones. That is a pitch-up torque growing with throttle, i.e. a CG **behind** the motor
centre (about 3 to 4 mm; 2.5 mm in the first logs), the opposite of the photo estimate.

Update after seven more sessions (`20261114_172046` to `174044`, trim `{-52, +18, -18, +52}`):
every flight with yaw ±35 ‰ spun **clockwise** (ψ +118 to +437°), while every flight with 0 or
±15 ‰ had spun counter-clockwise, so the right value lies between. Interpolating gives ±25 ‰.
The pitch part (±17 ‰) looks right: θ stayed near 0 (mean −3 to +0.3°) and the pitch mode of the
motors was +33 to +39 ‰, which is the trim itself. The roll mode varied from run to run
(−90, −31, −1, +8 ‰) and M3 minus M1 from −15 to +198 ‰, so that is not a fixed hardware
asymmetry; the tether and the lift-off conditions are the suspects. `174002` spent 13.5 s at 7 to
15 cm with φ −13 to −30° and M3 at the 95 % cap. The table prepared for the next flash is
`{-42, +8, -8, +42}` ‰ (yaw ±25, pitch ±17); the one below was in the board until then.

The table in `lqg_gains.h` was `{-52, +18, -18, +52}` ‰ for u1..u4 (M4, M2, M1, M3):
yaw ±35 ‰ (M2, M3 up) plus pitch ±17 ‰ (rear up, front down), generated with
`python3 tools/lqr_design.py emit --trim -52 18 -18 52`. Both are fits to flight logs, not
measurements; the physical cause of the yaw torque (suspect: the orange M2/M3 pair against the
gray M1/M4 pair) is not found. A scale test, one motor at a time at the same command, would
show it. `build/NoTrim` ignores the table for comparison.

### Full battery, trim ±25 (`20261114_183911`)

Battery at 12.6 V (3 × 4.200): the base at lift-off fell to 476 to 541 ‰ (650 to 750 ‰ at 3.88 V per
cell), so the earlier hover creep was mostly the battery. The yaw spin direction was now
different in every attempt (+142°, −222°, +242°), i.e. the systematic bias is gone and ±25 ‰ is
about right, but the yaw rate oscillates (±50 to ±125 °/s, period 1 to 2 s) and the drone
stayed below 20 cm (above 10 cm in 22 % of the airborne samples, target 30 cm). The yaw command
has the right sign (correlation +0.2 to +0.58 in eight windows) and the response lags by 40 to
80 ms, which `tools/loop_margin.py` puts near the limit of the 5× multipliers. Experiment (HMI
only): roll/pitch P/D and yaw P/D at 2× and hover at 55 %. Not flown yet.

### Height estimate divergence (`20261114_184336`) and the 2× experiment

Roll/pitch and yaw at 2× did not help: in the first attempt the drone tilted to θ +22° and spun
(ψ +217°) before lift-off, needed base 670 ‰, and was cut by the tilt limit 0.4 s after lift. The
HMI is back to 5×.

The second attempt exposed an estimator failure. At 21.33 s the BNO linear acceleration read
−10 to −25 m/s² for 0.45 s while the motors had just collapsed and the MTF (valid, strength 255)
read 5 to 17 cm. `ekf3.c` integrated it: `w` −7 m/s and `h_est` up to 2.18 m against a measured
0 to 10 cm. The range innovation gate (σ ≈ 1 cm) rejected the valid readings, and 25 rejected
frames at ~20 Hz meant 1.25 s before the filter re-anchored. The height loop (Altura D 10×) read
the false velocity as a fast climb and cut all four motors to 30 to 200 ‰ for about a second;
after the re-anchor it commanded 800 to 900 ‰ and the tilt limit cut at θ +35°.

Fix in `ekf3.c`: the prediction clamps each acceleration axis to ±3 m/s² (`ACCEL_MAX`) and, above
it, treats the IMU as unreliable (`SIG_ACCEL_LOST`), so `P` grows and the range corrects the
height instead of being rejected; the range channel re-anchors after 4 rejected frames
(`REJECT_LIMIT_RANGE`) instead of 25. New host test `test_false_vertical_acceleration` (−20 m/s²
for 0.45 s, range valid): height error 2.5 cm and |w| 0.39 m/s, against 0.78 m and 2.45 m/s with
the old code. Not flown. Altura D 10× (set from the HMI during that session) makes the loop
sensitive to velocity errors: 0.5 m/s is about 150 ‰ of collective throttle.

### Pi flights (`stage1_hover`, `hover`) on 14 Nov 2026

Four free flights (USB cable removed): the drone drifted to one side right after lift-off and
the XY position was never held. Causes seen in `~/drone/ensaios`:

1. It stayed at 6 to 9 cm for seconds, below the 10 cm where the optical flow becomes valid and the
   XY loop enters, so only the proportional attitude loop acted.
2. θ was +2 to +4° (nose up) after leaving the ground: a constant backward acceleration of about
   0.7 m/s² (u −0.28 / v −0.20 m/s at ψ +33° is the same body-backward motion rotated by the yaw).
   The same rear-heavy signature as the HMI logs (rear motors above the front ones by +77 ‰ at
   base 690): the pitch trim was too small.
3. Base at lift-off had crept up with the number of flights (48 % with a full battery, 58 to 62 %
   later): the battery, not measured (no divider on PA1). Charge before comparing.

Changes: trim `{-50, 0, 0, +50}` ‰ for u1..u4 (yaw ±25, pitch ±25), scaled by
`((base + mean collective Δω)/550)^4` (`lqg_cg_trim_scale`, fixed reference
`LQG_CG_TRIM_REF_PERMILLE`, ratio capped at 1.25; using the base alone left the host plants 25 % off
during the climb); the hover trim integrates 2× faster below 12 cm; boot `trim_ki` 100 ‰/(m·s). An attitude
integrator was tried and dropped: it integrates the sensed tilt to zero, so a biased level
reference becomes a real tilt (host drift 2.8 m against 0.5 m without it). Not flown.

### Margin of the multipliers with delay and motor lag

`stm32/tools/loop_margin.py` closes the loop with `Kd × multipliers` on the linear plant
(inertia tensor from `drone_params.json`, CG at the motor centre) adding `delay` samples of
10 ms and a first-order motor (`tau`). Results (largest closed-loop pole, < 1 is stable;
delay and `tau` were **not measured**, so the table sweeps them):

| att = rate multiplier (z 10, w 5, yaw 5 as saved) | nominal plant, tau 20 to 60 ms, delay 1 to 3 | worst case (kt ±20 %, inertia ±25 %) |
| --- | --- | --- |
| 1× (design) | stable everywhere | stable (0.993) |
| 2× | stable everywhere | unstable at 1.014 |
| 3× | unstable for delay ≥ 2 with tau ≥ 40 ms | 1.039 |
| 5× (saved in `config.json`) | unstable for delay ≥ 2, and for tau ≥ 60 ms with delay 1 | 1.099 |

So 5× is stable only if the real chain is faster than 10 ms of delay and 20 ms of motor lag,
which nobody measured. The old flights (11 and 12 Nov, 5×) show no sustained oscillation at
3 to 10 Hz (gyro peaks at 15 to 21 Hz with 0.2 to 0.45 rad/s rms, which looks like motor
vibration), but they were short (1.7 s to 12.6 s in the air besides the 56 s session), so that
is weak evidence. Recommended until delay and lag are measured: att = rate = 2×, then raise
one step at a time watching `q` and `p` in `imu.csv`. The generated `Kd` and the weights are
unchanged: the physics `Kd` rests on kt and kq that were never validated.

### Quaternion EKF3 (branch `phase2/control-lqg-quaternion`)

Replaces the 12-state Euler EKF. **It is an EKF3-style error-state filter, not
ArduPilot's EKF3**: no GPS, barometer, magnetometer or multiple lanes. XY position
still has no absolute reference. Host-tested only; **not flashed, not flown**.

| | Euler EKF (before) | `control/ekf3.c` (now) |
| --- | --- | --- |
| State | `[x y z φ θ ψ u v w p q r]` | nominal `q, v, p, bg, ba`; error state 15 (`δθ δv δp δbg δba`) |
| Prediction | generated `A`, `B` with the applied Δω | strapdown: calibrated gyro and BNO linear acceleration (gravity removed, `bno_sample_t::lin_accel`) |
| Attitude | φ θ ψ measured, ψ unwrapped | quaternion `q` (body FRD to local NED), rotation-vector residual; no Euler in the filter |
| Biases | none | gyro `bg` (±0.05 rad/s) and linear-acceleration `ba` (±1 m/s²), clamped |
| Range | `(ground + h + lever) / (cos φ cos θ)` | `(ground − p_z − (R b)_z) / R₃₃` from the rotation matrix |
| Flow | velocity rotated by ψ only | `Rᵀ v` (full attitude) |
| Gate, quality R, reanchor | same | same |

Units and frames: SI, NED/FRD, origin and heading of the arm instant; `q` rotates
body vectors into local; `bg` rad/s and `ba` m/s² in body axes.

Control (`lqg_ctrl.c`, `ekf_control`): `Kd` is the same generated matrix and the
same multipliers, but the tilt error is the rotation vector of `q̂` relative to its
own heading (`ekf3_attitude`), so the reference for φ, θ is level without Euler
angles; yaw error is `ψ̂ − ψ_ref` with ψ̂ from the quaternion, unwrapped in
`lqg_ctrl_t::psi_unw` so `ψ_ref`, `goto` and the telemetry keep the same meaning.
`p q r` are `gyro − bg`. The Δω fed back to the old prediction is no longer used
by the filter (the model plant is only used by the generated Kalman, "Usar EKF" off).

Sensors: `lqg_sensors_t` gains `accel[3]` / `accel_valid` (BNO `lin_accel`, sensor
axes), the BNO game rotation vector `q[4]` / `q_valid` and its accuracy status
`att_accuracy` (0..3, SH-2 status bits 1:0). `lqg_meas` maps the acceleration with the
**same signs and axis swap as the gyro** (`LQG_SGN_P/Q/R`, a proper rotation, det = +1).
While armed on the ground the mean gyro and acceleration become the initial `bg`, `ba`
(`LQG_REST_*`, ignored if implausible); on the ground the filter is re-seeded every
step, as before. `FLT_SIM` and the host test plant also produce the linear acceleration
and the quaternion.

### BNO quaternion used directly, accuracy, bias and variance guards

Second pass on the same branch, after comparing against ArduPilot `AP_NavEKF3` and the
SH-2 Reference Manual v1.2:

- **No Euler round trip.** The BNO quaternion used to go quaternion → Euler
  (`bno08x.c`) → mapped Euler → rebuilt quaternion. Now `lqg_meas` computes
  `q_body = WORLD ⊗ q_bno ⊗ MOUNT ⊗ conj(zero)` (`LQG_Q_WORLD`, `LQG_Q_MOUNT`,
  `lqg_config.h`). WORLD is ENU (BNO world, z up) → NED; MOUNT is the same −90° about z
  as the verified gyro map. Both were found by exhaustive search over the 24
  axis-permutation rotations against the verified Euler map: the pair is unique and
  the next best is 100× worse. `zero` is the tilt the pair gives at the old constant
  pose (`LQG_ZERO_ROLL_DEG`, `LQG_ZERO_PITCH_DEG`). Yaw reference = heading at the
  first sample after arming; the level of the flight is the mean tilt rotation vector
  while armed (±5° like before), applied in the body frame (`level_rv_sum`).
- **It is not identical to the old Euler map**, and the difference is deliberate: the
  Euler map adds tilt offsets and composes roll and pitch in the wrong order, so it is
  only first-order. Against it, yaw-only is exact, and combined tilts differ by about
  (|φ|+2.9°)(|θ|+5.5°): 0.4° for 8.6° of single-axis tilt and 4.3° at 14° + 14°
  (`tests/test_lqg_meas_q.c`). At hover tilts (< 5°) it is under ~1°. The mapping was
  verified on the real drone one axis at a time; the combined case never was.
- **Fallback.** Without `q_valid` (or a quaternion off the unit sphere) the EKF3 uses
  the quaternion built from the Euler `y[φ θ ψ]` as before. The generated Kalman
  ("Usar EKF" off) and the telemetry still use the Euler `y[]`.
- **Accuracy scales R.** `LQG_ATT_R_SCALE` = {8, 4, 2, 1} for status 0..3 multiplies the
  attitude noise. It never drops the measurement: without gravity in the acceleration,
  the BNO attitude is the only tilt reference.
- **Accel bias x, y frozen without flow** (as ArduPilot does without aiding): variance
  zeroed until a flow sample arrived in the last 0.5 s, then it learns again from its
  initial variance. Host test: bias 0.3 m/s² learned as 0.299 with flow, stays at 0
  without.
- **Variance guards** (EKF3 `FinishFusion`/`ConstrainVariances`): a scalar update that
  would drive any variance negative is skipped and counted (`variance_skips`); caps on
  the attitude, gyro-bias and accel-bias variances.

Not done: the report `Delay` field and delay compensation (ArduPilot fuses at a delayed
horizon, `FLOW_DELAY` 10 ms, `HGT_DELAY` 60 ms; we fuse on arrival); the BNO
Gyro-Integrated Rotation Vector (changes report rates on the real hardware); a
magnetometer or any absolute yaw; the BNO accelerometer with gravity. The SH-2 manual
used is v1.2 (2017); the current CEVA datasheet was not reachable, so accuracy, latency
and rate figures of the BNO085 are not verified.

Host tests (`tests/test_ekf3.c`, 30 s 3D trajectory with exact geometry, noisy
sensors): attitude error 0.001 rad, velocity 0.005 m/s, height 0.3 mm; gyro bias
0.0276 estimated for 0.0300 injected; 4.5 m/s flow spikes rejected (velocity error
0.009 m/s); stable with the acceleration missing. All existing closed-loop
scenarios still pass (yaw 55° off, MTF spikes, level offset, battery).

Tuning note found by the tests: the filter first followed a sudden BNO yaw jump
too slowly (gyro noise 0.02 rad/s) and then let `bg_z` absorb the jump (−0.25 rad/s,
a false yaw rate). `SIG_GYRO` is 0.1 rad/s (same as the old `Q_ANG`) and the biases are clamped.

Before the first flight with this filter (nothing below was done):

1. `FLT_SIM=1` on the board, motors at zero: the Controle tab must behave as in the
   host test.
1b. Disarmed on the bench, tilt the drone by hand and compare the Euler `Medidas`
   with the EKF estimate (φ, θ, ψ in the Kalman card): they must agree within about a
   degree for single-axis tilts; a sign or axis swap means `LQG_Q_MOUNT` or
   `LQG_Q_WORLD` does not match the real mounting (the Euler fallback is one edit away:
   `q_valid = false` in `control_task.c`).
2. Disarmed on the bench, check that the BNO `lin_accel` axes follow the gyro map:
   push the drone forward (+x), right (+y) and up, watch the acceleration in the body
   axes. A wrong sign there makes the velocity estimate fight the flow.
3. Expected failure modes: a flipped acceleration axis gives a velocity that
   accelerates the wrong way (flow pulls it back, XY oscillates); vibration or the
   BNO's internal filter lag the acceleration (`SIG_ACCEL`, `ekf3.c`, is inflated
   for this, tune it from a log); a persistent gyro bias at the clamp (±0.05 rad/s)
   means a wrong bias, not a real one. The tilt limit (35°), the 250 ms range cut and
   the estimator check (`fail`, non-finite or |h| > 5 m) still stop the motors.

### Controller design in Python (`stm32/tools/lqr_design.py`)

The LQR no longer needs MATLAB. `lqr_design.py` rebuilds the plant from physical
parameters, discretises it (ZOH, Ts = 0.01 s), solves the discrete Riccati equation and
writes `control/lqg_gains.h` (`LQG_KD`, `LQG_KFD`), the only file the EKF3 controller
reads for its gains. The generated Simulink model is still used by the stationary Kalman
("Usar EKF" off) and by `FLT_SIM`.

Checks (`tests/test_lqr_design.py`, 12 tests): the physics with body 455 g, motor +
propeller 24 g, total 551 g, xy leak 1e-3 reproduces the generated `A`, `B` to 1e-12 and
the Riccati reproduces `Kd`, `Kfd` to 4e-11 with the submission `Q = diag(5 5 500 500 500 50
5 5 5 10 10 5)` and `R = 0.1 I`. So the submitted model was built with those masses, and the
flow is fully reproducible without MATLAB. The generated model has an artificial 1e-3 leak
on x and y that the review script (`projetar_drone.m`) removed.

What `A` and `B` alone determine: only `w0`, `l·kt/Ixx`, `l·kt/Iyy` and `kq/Izz`. Mass, `kt`,
inertias and `kq` separately need measurements (scale, CAD, bench).

Real mass distribution: `tools/inertia_calc.py` composes the components (parallel axis
theorem, box / point / ring) into the CG and the full inertia tensor, refuses to run with
any `null` field, and `lqr_design.py emit --physics --params p.json` uses the tensor, the CG
offset (arms relative to the CG) and the per-motor hover speeds. Fill
`tools/inertia_components.json` (the unconfirmed 551 g table; positions and CAD plate tensors
from the 14 Nov 2026 estimate, see "CG trim" below). `tools/inertia_components.example.json`
is the empty template. A part can be a CAD solid (`"shape": "tensor"`, `inertia_kgm2`).

**CG offset is probably the biggest unmodelled effect.** With the CG 10 mm ahead of the
motor centre the hover needs ±46 ‰ per motor (front vs rear), about 4.6 ‰ per mm. The
controller has one common `base` for all four motors (`mix`), no attitude integral, and the
XY integrator is limited to ±1 m, worth only ~3.3 rad/s ≈ 1.9 ‰. It can absorb roughly
0.4 mm of CG offset; the rest becomes a steady tilt and drift. Not validated on the
drone: it follows from the model and needs the measured CG. The per-motor hover trim
below is the fix; an attitude integral is still not implemented.

### CG trim (per-motor hover trim)

Mechanism, in firmware and host-tested: each motor gets `base + trim_i · min(base/hover, 1) +
Δω_i` in `lqg_ctrl.c` (`mix`); the `applied` Δω fed back to the Kalman prediction excludes the
trim. `LQG_HOVER_TRIM_PERMILLE[4]` (u1..u4, ‰) in `control/lqg_gains.h` is the default;
`lqg_ctrl_set_cg_trim` changes it (finite, |value| ≤ 120 ‰) and it survives arming.
`LQG_CG_TRIM_ENABLE` (`lqg_config.h`, default 1) turns the mechanism off at build time
(`python3 tools/build.py --no-cg-trim`, variant `NoTrim`). `lqg_sim.c` and the host plant
apply the same table so FLT_SIM keeps hovering level. Host result: a plant that needs
+30/+30/−30/−30 ‰ tilts to the 35° limit without the trim and hovers level (θ = 0, 5 cm drift
in 30 s) with it.

**Yaw feed-forward (14 Nov 2026, provisional).** The five lift-offs of `20261114_160221` and
`20261114_160848` all spun counter-clockwise right after leaving the ground (gyro z down to
−3.6 rad/s, ψ to −188° in 2 s) and the yaw mode of the motors (M2+M3 minus M1+M4, halved) was
+28, +49 and +87 ‰ in the hover windows (part of that is the controller reacting to the yaw
error). The table now holds `{-15, +15, -15, +15}` ‰ for u1..u4 (M4, M2, M1, M3): M2 and M3
up, M1 and M4 down, a guess at the constant torque, generated with
`python3 tools/lqr_design.py emit --trim -15 15 -15 15`. Not a CG estimate and not validated:
compare against the `NoTrim` variant (`tools/build.py --no-cg-trim`, table ignored) and read
ψ and the yaw mode after lift-off. Propeller photos (M1 to M4) show the same 3-blade model,
orange on M2/M3 and gray on M1/M4, no visible damage; the handedness cannot be told from them.

**Before that, the shipped table was zero.** A photo/CAD estimate (`tools/inertia_components.json`,
`tools/drone_params.json`: CG x = +8.2 mm forward, trim ≈ +35/+40/−35/−40 ‰) was **refuted by
the drone's own logs** before any flight: with the old common-base firmware the controller's
steady front−rear motor difference at hover was +1, 0, −23 and −13 ‰ (sessions
`20261111_203159`, `20261111_205711`, `20261112_174934`, `20261112_180218`), i.e. a CG between
the motor centre and about 2 mm behind it (≈9 ‰ front−rear per mm), not 8 mm forward. That
estimate is wrong, including its sign (photo positions and the duct/plate masses were guessed).
Flashing it would have added a ~75 ‰ front−rear differential: the drone would be cut by the
tilt limit or flip at lift-off. The inertia tensor in those files is still usable as an
order-of-magnitude figure, its CG is not.

How to fill the trim from data instead: fly (tethered) with zero trim and read the mean motor
outputs in hover from `motor.csv` (rows with state ≥ 3, height > 2 cm). The front−rear and
right−left differences are the trim (u1, u2 front; u3, u4 rear), minus the mean. Or hang the
drone by a thread at two points and measure the CG, then
`python3 tools/lqr_design.py emit --params <json with cg_m>`. Not validated on the drone.

### Deviations and additions (not in the generated model)

1. **Slow hover trim.** The LQR has no integral action. Its height gain is about
   20 ‰ of throttle per metre, so a 1 % error in the configured hover becomes
   ~0.5 m of height error. A slow integrator on the filtered height error adjusts
   the hover (limit ±25 %, gain default 1.2 %/(m·s), HMI-editable, can be
   switched off). The gain is capped at 1.5 %/(m·s): the height loop has a period
   of about 7 s and the integrator is unstable above ~2.6 %/(m·s). It does not enter the
   Kalman. The trim is shown in the HMI; add it to the hover value to make the
   next flight start from the right point.
2. **Saturation feedback.** When a motor is clamped, the Kalman prediction is
   corrected with the Δω actually applied (the generated step predicts with the
   unclamped value).
3. **Ground lock.** Before lift-off the Kalman state is reset to the measurements
   every step so nothing integrates on the ground.

### Protections (firmware)

| Condition | Action |
| --- | --- |
| STOP (button, Space, Esc), closed port, corrupt frame | all motors to zero (existing bench rules) |
| no HEARTBEAT for 300 ms | arbiter disarms (browser tab hidden, page frozen) |
| no controller update for 100 ms | arbiter disarms |
| BNO invalid for 100 ms | cut |
| MTF distance invalid for 250 ms (armed, not on the ground) | cut |
| |φ| or |θ| > 35° | cut |
| no lift-off before the throttle reaches the limit or hover + 25 % | cut |
| height > target + 30 cm | land |
| optical flow invalid for 2 s in the air | land |

Cut means the output is disarmed (DShot 0) and the reason is logged. The MTF-01
distance is also the height reference: it does not know about a cable pulling
the drone.

### Climb fix (fourth iteration, 2026-11-12)

Symptom: the drone left the ground at ~55 % but never reached 30 cm. Causes found
in the code and the fixes (all host-tested, not flown):

| Cause | Fix |
| --- | --- |
| "Lift-off" at 3 cm for 50 ms, reachable on the ground by tilt and MTF noise; the base was then set to that throttle minus 0.45 s of ramp, below the real hover, overriding the typed hover | base = estimated weight-balancing throttle (ramp lag from the model), MTF lever arm compensated (`LQG_RANGE_POS_X_M`, sign to be checked with a ruler). A first version confirmed lift-off at 8 cm for 0.2 s: on the real drone it bounced at 2–7 cm for 4 s with no height control and drifted into a wall (12/11 17:36), so it went back to 3 cm / 50 ms |
| BNO heading (rotation vector, magnetometer) jumped 29° in 0.4 s when the motors started, with no real rotation; the controller turned the drone 28° to "fix" it (12/11 17:49) | game rotation vector (accelerometer + gyroscope); the flight heading is relative to arming anyway |
| MTF read 2–4 cm higher with the motors spinning: "lift-off" declared at 3 cm and 47 % throttle, base set far below the real hover, 12 s on the ground with the controller "in the air" (12/11 18:02) | lift-off is counted 3 cm above a slow (1 s) moving floor of the height reading, not above zero |
| 170° yaw error asked for the maximum motor differential and used up the thrust margin | yaw error fed to the controller (and its integrator) limited to ±0.5 rad |
| Attitude differential limited to ±3 % until lift-off | only during the fast ramp: the drone flipped twice while bouncing light near the ground with the limit active (12/11 17:23) |
| Fixed level zero from the submission (θ read −1.3 to −1.7° on the ground) | level zero measured at arming (mean φ/θ on the ground, ±5° max) |
| Hover trim frozen when the height error exceeded 30 cm, and capped at 1.5 %/(m·s) | no error window; directional anti-windup; parameter up to 20 %/(m·s) (default 6), effective gain limited to 10 % of the stability limit a·Kz·Kw (≈0.26 %/(m·s) at 1×, ≈13 at height P 10×/D 5×) |
| Motor limit 80 % with hover near 70 % on a used battery | 95 %; when a motor hits the limit all four drop together (attitude keeps priority over height) |
| Thrust changes with battery voltage (open-loop ESC) | output × (cells · 3.9 V)/V filtered (1 s), 0.88–1.25, when the PA1 divider (10 kΩ/1 kΩ, ×11) is calibrated; land after 2 s below 3.4 V/cell or with base > 85 % |
| Flow used below its focus distance (quality 25–28 on the ground) | flow only above 10 cm (off below 7 cm); XY position/velocity enter the control only then, with the XY origin reset at the first valid flow; no "flow lost" landing below 10 cm |
| Flow gyro compensation used the instantaneous gyro and fused repeated MTF frames | gyro averaged over 2 steps; range and flow fused once per MTF frame |
| Raising roll/pitch P to 5× made XY 5× weaker (tilt per metre = Kx/Kθ) | XY P/D multipliers are now relative to roll/pitch P |

New HMI/telemetry fields (bench-link 1.4): x/y/yaw reference, battery factor,
flags "XY in the controller" and "battery compensated".

## HMI: Controle tab

`ARM → TAKEOFF → LAND`, `Desarmar` (STOP when on the ground) and a large red
**PARAR** (Space or Esc also stop, from any tab). Arming needs the safety
checkbox. Editable at any time (applied immediately, saved in `hmi/data/config.json`
and re-sent at every USB session):

| Parameter | Default | Range |
| --- | --- | --- |
| hover height | 30 cm | 10–80 cm |
| hover throttle | 55 % | 10–90 % |
| motor limit (control only) | 95 % | 20–100 % |
| ramp to lift-off | 4 %/s | 0.5–20 %/s |
| climb / descent rate | 10 / 10 cm/s | 2–50 cm/s |
| hover trim gain / on-off | 6 %/(m·s) / on (effective gain limited, see above) | 0–20 |
| XY integrator / yaw integrator | 0.05 / 1.0 per s (yaw raised from 0.3 on 14 Nov 2026) | 0–0.5 / 0–2 |
| use EKF | on | latched at Arm |

The 40 % bench cap on the Painel tab is separate and untouched; the control limit
only applies to the control owner. Motor values above 40 % are therefore possible
in this mode by design (hover is expected above 40 %).

Tab contents: state, estimated / reference / measured height, base throttle and
trim, the four motors in their physical positions, the measurements the model
receives (with the sign check below), the Kalman estimate, and recent messages.
Everything is also recorded: `ctrl.csv` in each session (units in the column
names), plus the usual motor/IMU/range files.

## Before the first test with motors

Nothing below was done yet.

1. Flash (ask first, DFU) and open the HMI on the board. Disarmed, check that
   `Controle → Medidas` follows the drone: right side down → φ positive, nose up →
   θ positive, clockwise seen from above → ψ positive; level drone → φ, θ ≈ 0.
   The level zero (−174.5°, −2.9°) is a constant from the submitted firmware; if
   the mount changed, φ/θ will not read ≈ 0 level and the constants in
   `lqg_config.h` must be recalibrated. A wrong sign makes the controller push the
   tilt the wrong way.
2. Confirm motor numbers and directions with the Painel tab (beep, one motor at a
   time, no propellers): M4/M2 front, M3/M1 rear, M1/M4 reversed. A wrong mixer is
   as bad as a wrong sign.
3. Cable and area: the cable must allow ~50 cm of height and not pull the drone.
   Keep a hand on Space.
4. First spin-up without propellers only checks wiring and that the output goes
   above 40 %. The hover throttle (default 55 %) is a guess: the drone with its
   current battery and propellers needs an actual measurement. Note the "hover
   trim" after each attempt and update the hover value.
5. Battery: no low-voltage failsafe exists yet (ADC calibration optional).
6. CG trim: the shipped table is zero (see "CG trim"). Fly the first test with it zero;
   a steady tilt toward one side is the CG, and the hover differential in `motor.csv`
   gives the table to apply next.

## Known limits

- The LQR height loop is slow (period ≈ 7 s; in the closed-loop test a 30 cm step overshoots by
  ~8 cm and settles in ~10 s) and has no integral action beyond the trim above.
  Horizontal position (x, y) only integrates optical flow and drifts; the
  reference is zero at arming and there is no XY integral.
- The thrust curve (1.755 rad/s per ‰) and the hover throttle are provisional.
- The host test uses the model's own linear plant: it proves the sequence, mixer,
  limits and failsafes, not the real drone's behaviour.
- The `Q` used by the generated Kalman gains was set in MATLAB; this branch does
  not change it. The EKF3 has its own noise constants (`ekf3.c`).
- If the height still does not rise with throttle clearly above the lift-off value
  (first session: motors ~28 ‰ above the lift-off throttle and height stuck at
  ~3 cm), look for a physical limit (cable length or yaw wind-up of the cable);
  the controller cannot see it.
