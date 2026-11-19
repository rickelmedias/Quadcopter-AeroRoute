# Phase 2: DroneLink between the Raspberry Pi and the STM32

DroneLink 1.2 is the MAVLink 2 dialect of the thesis (`protocol/drone_controller.xml`),
ported from `submission/2026-11-11` onto the phase 2 controller. The Pi decides
**what** to do (route order, next objective); the STM32 decides **how** and
**whether** (validation, LQR + EKF, failsafes).

Status: host tests only (`stm32/tests/test_dronelink_protocol.c`,
`test_flight_command.c`, `rpi/tests`). **Not flashed, not flown.**

## Wiring and build

| Item | Value |
| --- | --- |
| Link | USART1, 115200 8N1, 3.3 V: Pi GPIO14/TXD (pin 8) → PA10, PA9 → Pi GPIO15/RXD (pin 10), common GND |
| Pi port | `/dev/serial0` (`enable_uart=1`, `dtoverlay=disable-bt`, serial console off) |
| IDs | STM32 system 1 component 1; Pi system 1 component 191 |
| Boot identity | counter in the last sector of the W25Q64 on SPI1 (CS on PA4). Without it, arming and legs are refused (`NOT_IMPLEMENTED`) |
| ESC telemetry | USART1 is taken, so `ESC_TELEM_UART` is 0 (KISS requests disabled). Bring ESC data back with bidirectional DShot |
| Headers | `cd stm32 && conda run -n aeroroute python tools/gen_mavlink.py` → `Middlewares/Third_Party/mavlink` (generated, do not edit) |

The USB bench HMI keeps working in parallel (bench-link). Parameters and gain
multipliers are set there; DroneLink 1.2 dropped `PARAM_*` and `DRONE_GAIN_*`
(capability `TUNING` is not advertised).

## Messages

As in the thesis: HEARTBEAT (Pi 4 Hz, STM32 2 Hz), DRONE_PROTOCOL_INFO 1 Hz,
COMMAND_LONG arm/disarm → COMMAND_ACK, DRONE_CONTROL_LEASE 5 Hz (valid 1 s),
DRONE_LEG_COMMAND / ACK / CANCEL (32-entry idempotent history), TIMESYNC,
ATTITUDE and LOCAL_POSITION_NED 5 Hz (z relative to the arming ground),
SYS_STATUS 2 Hz (battery when the PA1 divider is calibrated), DRONE_CTRL_DEBUG
10 Hz (y, x̂, applied Δω, reference x/y/z NED/ψ, motors M1..M4, flags; ctrl flag
bit0 = FLT_SIM), STATUSTEXT for controller notes (lift-off, landing, cuts).

## Legs on the new controller (`stm32/App/Src/dronelink/flight_command.c`)

| Action | Needs | Becomes |
| --- | --- | --- |
| TAKEOFF_RELATIVE (z = −height, 0.10–0.80 m) | armed on the ground | `control_takeoff_to(h)`, retried until the ground is measured (≤ 2 s) |
| ROTATE_RELATIVE (yaw rad) | HOLD | yaw goal = yaw reference + yaw |
| TRANSLATE_RELATIVE (BODY_FRD or LOCAL_FRD, m) | HOLD; XY in the controller for horizontal moves | goal = reference + rotated offset; height − z |
| GOTO_LOCAL (x, y, z, yaw absolute) | same | goal x, y, −z (0 keeps the height), shortest turn to yaw |
| HOLD / LAND | in the air | `control_hold()` / `control_land()` (LAND on the ground armed disarms) |

A leg ends ACCEPTED after 1 s at the goal: reference stopped, position within
tolerance (default 5 cm, 0.05 rad), speed < 5 cm/s, |r| < 0.1 rad/s. It fails on
timeout (the drone holds), cut, or landing in the middle. Horizontal limits:
|x|, |y| ≤ 3 m; XY reference speed 0.10 m/s by default (≤ 0.5).

The XY origin is where the optical flow became valid in flight (above 10 cm),
not exactly where the drone was armed.

## Failsafes of a Pi flight

The motor arbiter needs a keepalive every 300 ms; in a Pi flight the DroneLink
task renews it while the output is armed, so losing the Pi never cuts the motors
in the air. Instead (only for flights armed by the Pi): no Pi HEARTBEAT for 1 s →
HOLD and new legs refused; 2 s with degraded navigation or 5 s in any case →
LAND; lease withdrawn → HOLD, and LAND after 5 s. The controller's own cuts
(tilt, BNO, MTF, estimator) and its landings (flow lost, low battery, no thrust
margin) apply as in the tethered mode.

## Pi side (`rpi/`)

See `rpi/README.md`: `dronectl status | monitor | hover | mission | land | disarm | routes`.

## Before the first test on hardware

1. Flash the `--sim` build (ask first). With the Pi wired, run
   the stages of `rpi/README.md` in order (`stage1_hover` ... `stage4_square`):
   each must arm, fly and land on the simulated plant while the motors stay at zero
   (`[FLT_SIM]` in `dronectl status`).
2. Check the W25Q64 is fitted: `dronectl status` must show a non-zero boot id,
   and it must increase on every reset.
3. Only then flash the Release build and repeat with the tether, props on, a
   person at STOP.
