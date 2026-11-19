# Phase 2 control review: why the drone climbs and loses XY

Review of 2026-11-14 over the thesis (`latex/`), the firmware on `phase2/control-lqg`,
`references/DroneLQG_revisao`, the 2026-11-08 free-flight logs (submission firmware)
and the tethered sessions of 2026-11-11/12 (`hmi/data/sessions`). Log analysis was
done by subagents; the simulations are linear (generated `A`, `B`, `Kd`, Ts = 0.01 s)
and indicative, not flight validation. Nothing here was flashed or flown.

Units are SI. Frames: NED/FRD as in `control/lqg_config.h`.

## Verdict

Attitude is good because it is the fastest and most local loop. XY and height rely on
slow loops with no useful integral action, designed on a model that does not match the
drone. At take-off three things stack up: the drone spends seconds where XY is blind, a
torque or level bias appears, and the real hover throttle is above the estimate. There
is no single bug: a full-state LQR without integral action plus a fixed Kalman cannot
hold XY at lift-off even with perfect attitude.

## Findings

### A. Confirmed by model, code or logs

**A1. The XY loop is slow and very sensitive to bias.**

| Item | Value |
| --- | --- |
| XY natural frequency | about 0.8 rad/s (period about 8 s) |
| Tilt demanded per metre of error | 0.066 rad/m (3.8 deg/m) |
| Level error of 1 deg | about 0.27 m steady offset |
| Level error of 1.5 deg | about 0.40 m |
| Pitch torque bias of 10 rad/s^2 (about 3-4 mm of CoG or about 4 % thrust mismatch), gains 1x | about 3.1 m |
| Same bias with attitude gains x3 | about 0.9 m |

- Mechanism: attitude has only P, so a torque bias becomes a position offset. This is
  "drifts to one side without stopping" while attitude looks fine.
- Logs: mean phi bias in flight +0.2 to +3.3 deg, theta +0.4 to +5.3 deg. The
  submission carried hand-tuned feedforward of -50 rad/s (theta) and -40 rad/s (phi)
  (`DEFAULT_CTL_FF_*`).
- Current XY integrator: `ki_xy = 0.05` gives tau of about 20 s, limit +-1 m
  (`LQG_INT_XY_LIMIT_M`). Even `ki_xy = 0.6` peaks at 2.2 m in simulation, because the
  bias appears at once when leaving the ground.
- Structure that fixes it: an integral in the attitude loop whose error is measured
  against the tilt demanded by the outer loop (cascade form). In the linear simulation,
  `ki = 60` brings the offset to zero in about 10 s with a 0.77 m peak. An integral on
  the raw angle does not help: it only pins theta, which is already about 0.

**A2. The drone lives in the XY blind zone.**
- Optical flow is used only above 10 cm height (`LQG_FLOW_MIN_HEIGHT_M`).
- In the tethered sessions the maximum height was 4-5.5 cm in 4 of 5 flights; 180218
  reached 19 cm but stayed >= 10 cm for about 1 % of the window. `XY_ACTIVE` almost
  never turned on.
- Below 10 cm XY is attitude plus integrated gravity, so tilt-vs-drift agreement in
  those logs is circular.
- The 10 cm gate is a controller choice. MTF-01 range on the ground is 62-89 mm and the
  sensor already flags flow valid from about 80 mm. Retailer data quotes 8 cm as the
  minimum flow distance (secondary source).

**A3. Height is a soft loop with no useful integral.**
- Gain 20 permille of throttle per metre, wn about 0.93 rad/s, zeta about 0.7. A 1 %
  hover thrust error becomes about 11 cm of height error.
- The trim integrator is capped at about 0.26 permille/(m s) at 1x.
- In 180218 the base rose from 468 to 564 permille (+100) in 12 s while height stayed at
  3-5 cm against a 20 cm reference. Consistent with a false "lift-off" plus an
  underestimated hover; also consistent with tether or ground contact. The logs do not
  separate them.

**A4. The physical model does not match the drone.**
- The model is linearised at omega0 = 804 rad/s (45.8 % throttle). The submission used
  `W0 = 1050` (60 %); measured hover was 47-55 %.
- Kt = 2.09e-6 came from a 12-14 V bench supply, with the curve above 1200 rad/s
  extrapolated; the drone flies on a 3S LiPo with voltage sag. Thrust and torque
  columns of `B` are plausibly off by 25-45 %.
- Logs: control effectiveness vs model about 0.95 (roll), about 0.67 (pitch), about 0.5
  (yaw, low confidence). Command-to-angular-acceleration lag 20-40 ms (roll, pitch),
  >= 80 ms (yaw). R^2 is only 0.2-0.5 on 3-5 s windows: indicative.
- Submission: M4 asked for 6-11 % less throttle than the other motors (thrust or CoG
  asymmetry, or a mapping error).
- Thesis/repo inconsistencies: `Kq` table 2.61e-5 vs about 6e-8 in generated code and
  9.5e-8 in the MATLAB script; `l = 0.142 m` is the motor-to-motor diagonal while the
  code uses 0.071 m; `Ixx` 5.1e-4 (thesis) vs 7.35e-4 (script); Ts = 100 ms in the text
  vs 10 ms in code; omega0 about 146 rad/s in the effort figure vs 804 in the equation.
  Inertia was never measured.

**A5. The attitude loop margin is thin.**
- With boot gains (attitude and rate x5) the loop is stable with 10 ms delay but
  unstable with 20 ms delay plus a 30 ms motor lag (pole at +1.6 1/s).
- Submission: attitude oscillated +-19 deg and the 20 deg failsafe tripped in 4 of 4
  flights.
- Control runs at 100 Hz on the BNO085 (about 315 Hz effective, opaque internal
  fusion). Gyro spectrum is broadband at 45-146 Hz with aliasing.

**A6. The reference frame rotates with yaw.**
- With the model linearised at psi = 0 and no rotation, the XY loop loses stability at
  about 52 deg heading error (1x gains). At 60 deg the slowest pole is +0.15 1/s, at
  90 deg +0.67, at 180 deg +1.34.
- Yaw ended at -62 to -222 deg in all logged Phase 2 flights (+80 to -60 deg in the
  submission). The EKF path rotates the frame; the "generated Kalman" path does not.
- The regression shows motor-to-r signs match the model, so the yaw loop is not positive
  feedback. The yaw problem is low effectiveness and lag; the cause is not proven.

### B. Plausible, needs a test

- **B1.** The BNO underestimates tilt under sustained horizontal acceleration: a
  quadrotor accelerometer reads thrust along the body z axis, so a gravity-referenced
  correction pulls the estimated tilt toward zero. The BNO's internal gain is unknown;
  the tethered logs cannot test it because the tether blocks horizontal motion.
- **B2.** Flow quality varies by flight. First three flights: median quality 44, 5-16 %
  of samples above 200 cm/s at 1 m, 7-16 % zeros. Last two: quality about 95 but
  77-100 % of samples above 200 cm/s at 1 m (surface, light or sensor?). In 180218 the
  estimator diverged (x 8 m, y 28 m).
- **B3.** Ground effect: literature for multirotors is not consistent on the direction
  or size of the thrust change near the ground; no direction is assumed here.

## Reference architecture

The PX4/ArduPilot/Crazyflie pattern is a cascade:

```text
position P -> velocity PI(D) -> acceleration -> tilt + collective thrust
-> attitude P (quaternion error) -> rate PID (>= 500 Hz, with I) -> mixer with desaturation
```

- PX4 tunes in the order rate, attitude, velocity/position, and says the velocity loop
  matters most and I reduces steady-state error.
- Crazyflie fuses the same sensor pair (PMW3901 + ToF) in a scalar-update EKF, inflates
  the flow noise, and notes the drone drifts if it yaws too much.
- Doyle (1978): LQG has no guaranteed margins, unlike LQR. The separation principle is
  not a robustness guarantee; margins must be checked per design.

## Plan (about one month, by risk)

Any motor test is a command for the person at the bench, never run by the agent. Each
firmware step goes host tests, then `FLT_SIM=1`, then motors. Before any propeller test,
list what changed and the expected failure mode.

### Phase 0: no motors (1-2 days)
1. Re-check axis signs in the HMI with the drone still (phi, theta, psi, gyros).
2. Fix thesis/repo values (`Kq`, `l`, `Ixx`, Ts, omega0) or record an erratum.
3. Measure inertia (bifilar pendulum) and CoG (balance).
4. Log control at 100 Hz (today `ctrl.csv` is decimated to 20 Hz); add battery voltage
   and raw flow.

### Phase 1: identify the real plant (bench)
5. Static thrust per motor on a LiPo under load (not a bench supply): real Kt, hover,
   asymmetries (M4).
6. Hover throttle from a load cell holding the drone: replaces the guessed 55 %.
7. Step or PRBS identification on a restrained rig for gain, sign and lag of roll,
   pitch, yaw.
8. Separate "tether/ground" from "gain" for the height stall: repeat with a verified
   slack tether or a known load.

### Phase 2: hardware and sampling
9. Add a 6-axis SPI IMU (ICM-42688-P, BMI270 or MPU-6000), gyro >= 1 kHz. PB12-PB15
   are free in the `.ioc` (SPI2, AF5).
10. Rate loop >= 500 Hz, attitude >= 250 Hz. Keep the BNO as redundancy or for yaw.
11. Mechanics: raise the flow sensor to at least 10 cm from the floor (taller legs) so
    flow is valid from take-off.
12. Later: bidirectional DShot with RPM filter.

### Phase 3: new controller (host, then SIM)
13. Cascade as above: rate PID (D on filtered measurement, anti-windup), quaternion
    attitude error with tilt priority over yaw, velocity PI in acceleration domain,
    altitude P to velocity PID with hover from calibration, 1/(cos phi cos theta) and
    battery compensation, mixer with desaturation.
14. Keep the current LQR as a comparable baseline for the thesis.

### Phase 4: estimator
15. EKF/ESKF (Sola 2017): 1 kHz gyro, gyro and accelerometer bias, quaternion, flow as
    predicted velocity with gyro compensation, ToF with tilt compensation.
16. Keep the chi-square gate already in `ekf.c`.
17. If the BNO stays, test B1 before trusting its tilt.

### Phase 5: take-off
18. Detect lift-off from estimated vertical velocity, not a 3 cm height.
19. Climb decisively to a height where XY is active, then hold the point.
20. For the blind window: learn level/trim on the ground and start the velocity
    integrator as soon as flow is valid.

### Acceptance criteria (suggested)
- Hover at 30 cm for 10 s with XY RMS < 20 cm, no tether, 5 repetitions.
- Recover position after a 0.5 m push.
- Only then, the point sequence.

## What is not known

- Whether the 3-5 cm stall is tether, underestimated hover, ground effect or a mix.
- The internal accelerometer gain of the BNO (B1).
- Why flow quality jumped to about 95 in the last two flights.
- All simulations use a linear plant, and the thesis model already proved imprecise.
- Effectiveness numbers come from 3-5 s windows with R^2 of 0.2-0.5.

## Sources

Read during this review:
- MicoAir MTF-01: <https://ardupilot.org/copter/docs/common-mtf-01.html>;
  retailer sheet with 8 cm minimum flow distance:
  <https://reach.dog/shop/robofusion/products/mtf-01-integrated-laser-rangefinder-optical-flow-module-for-unmanned-aerial-vehicles-uavs>
- ArduPilot optical flow setup: <https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html>
- PX4 basic PID guide: <https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter_basic.html>;
  advanced position tuning: <https://docs.px4.io/v1.9.0/en/config_mc/advanced_mc_position_tuning.html>
- Crazyflie state estimation:
  <https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/state_estimators/>;
  Greiff, Lund 2017: <https://lup.lub.lu.se/student-papers/record/8905295>
- Doyle 1978, Guaranteed Margins for LQG Regulators:
  <https://doyle.caltech.edu/images/doyle/6/62/GuaranteedMarginsLQGRegs.pdf>
- Bouabdallah et al., PID vs LQ, IROS 2004:
  <https://www.research-collection.ethz.ch/entities/publication/f1b0a55e-579d-4653-b86a-386f25c75121>
- Quadrotors and accelerometers (drag model):
  <https://www.bohrium.com/paper-details/quadrotors-and-accelerometers-state-estimation-with-an-improved-dynamic-model/813166445880934401-99233>
- Sanchez-Cuevas et al. 2017, ground effect: <https://idus.us.es/handle/11441/94910>
- BNO08X family: <https://docs.luxonis.com/hardware/platform/sensors/imu/bno08x>
- Mahony, Hamel, Pflimlin 2008, IEEE TAC, DOI 10.1109/TAC.2008.923738; Mahony, Kumar,
  Corke 2012, IEEE RAM, DOI 10.1109/MRA.2012.2206474
- Smeur et al. 2016, DOI 10.2514/1.G001490; 2018, DOI 10.1016/j.conengprac.2018.01.003

Cited from memory, not verified here (check before using in the thesis): Mellinger and
Kumar 2011; Lee, Leok, McClamroch 2010; Sola 2017 (arXiv:1711.02508); Brescianini et al.
2013; Pannocchia and Rawlings 2003.
