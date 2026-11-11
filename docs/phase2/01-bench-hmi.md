# Phase 2 / 01: Bench HMI

Branch `phase2/01-bench-hmi`, created from `main` with the STM32 and Pi code of `exploration/ekf-2026-11-10` (EKF, range filter, latest control). Status: design, nothing implemented yet.

## Goal

A bench interface that tests everything from the STM32 onward, over USB, without the Raspberry Pi:

- see every sensor and estimator value live;
- arm and disarm, and drive each motor or all four with capped throttle;
- **freeze** a sample window and compare it with live values (for example, before and after spinning the motors);
- record every session with units for later analysis (`log-analyst`).

The first experiment it serves is the sensor-noise study in `.opencode/knowledge/project-history.md`: BNO085 and MTF-01 with motors stopped, ESC powered, motors without props and with props.

## Architecture

```text
STM32 (firmware)  --USB CDC, DroneLink/MAVLink 2-->  hmi/server (Python, PC)  --WebSocket-->  hmi/web (browser)
```

- **Link: binary DroneLink over USB CDC**, not the text console. The same messages later travel over the Pi UART, so the HMI and the Pi share one protocol and one set of tests. The text console stays available for debugging.
- **Prefer standard `common.xml` messages** already included by the dialect, and add custom messages only for project-specific data:

| Data | Message |
| --- | --- |
| Link, armed state | `HEARTBEAT` |
| BNO085 accel, gyro (raw and scaled) | `SCALED_IMU`, `RAW_IMU` |
| BNO085 orientation | `ATTITUDE_QUATERNION` |
| MTF-01 range | `DISTANCE_SENSOR` |
| MTF-01 flow and quality | `OPTICAL_FLOW` |
| EKF state | `LOCAL_POSITION_NED`, `ATTITUDE` |
| ESC telemetry (RPM, voltage, current per motor) | `ESC_STATUS` |
| ESC temperature, errors | `ESC_INFO` |
| Battery (divider, CURR) | `BATTERY_STATUS` |
| Arm/disarm | `COMMAND_LONG` `MAV_CMD_COMPONENT_ARM_DISARM` + `COMMAND_ACK` |
| Motor test | `COMMAND_LONG` `MAV_CMD_DO_MOTOR_TEST` + `COMMAND_ACK` |
| DShot value actually sent, EKF internals | new `DRONE_MOTOR_OUT`, existing `DRONE_CTRL_DEBUG` |

- **Server (`hmi/server`)**: Python in the `aeroroute` conda env. pyserial plus pymavlink parse the link, a WebSocket pushes decoded data, and every message is written to a session file with timestamps.
- **Web UI (`hmi/web`)**: plain HTML, CSS and JavaScript ES modules with no build step (`// @ts-check` and JSDoc types for checking), served by the Python server, with uPlot for high-rate charts. A web UI also lets us use **Impeccable** (`/impeccable critique`, `audit`, `polish`, `distill`) on the first versions. Install it when `hmi/web` exists: `npx impeccable install`, then `/impeccable init`.

## Motor safety (firmware, not the UI)

The UI only requests; the STM32 decides.

- Disarmed means DShot value 0 on all four channels, sent continuously. This is the boot state.
- Arming requires: DTR on, HMI heartbeat present, all motor commands at 0, sensors valid, battery above a minimum when measured.
- Link loss (no HMI heartbeat for 300 ms), DTR drop, USB disconnect or a fault forces DShot 0 and disarms.
- Throttle is capped in firmware at 40 % (hard limit, compile-time). The UI cap slider can only lower it. Commands above the active cap are clamped, not rejected.
- Direction changes are accepted only while disarmed, with all motors stopped.
- The UI has a permanent **STOP** control (also the space bar) that sends disarm. The firmware disarms on its own if the UI freezes.
- Agents never arm or spin motors. A person at the bench uses the HMI.

## Screen

- **Top: live panel.** Groups for IMU, orientation, MTF-01, EKF, motors and ESC, and power. Each value shows its unit and a validity marker. Rolling charts are optional per value.
- **Freeze.** One click saves the current values instantly, with no waiting. The saved row stays below until deleted. Alongside each value it also stores mean and σ of the last second already buffered, so noise can be compared. A freeze has an editable label ("ESC on, motors stopped").
- **Bottom: frozen samples.** List of saved rows, each with the difference from live values (Δ), highlighted when Δ exceeds 3σ of that row. Two saved rows can be compared with each other.
- **Motor panel.**
  - Arm/disarm and STOP.
  - **Cap slider** 0–40 %: maximum any motor slider can reach.
  - **Four motor sliders** 0 to cap, each showing DShot value sent and RPM received.
  - **Group:** checkboxes select which motors (any subset or all) follow one shared slider.
  - **Direction per motor:** normal/reversed toggle, applied with AM32 DShot commands 7/8 while disarmed. By default not saved to the ESC (resets on power-up); a separate "save to ESC" button sends command 12. The current bench configuration (M1 and M4 reversed, `stm32/docs/guia_teste_motores.md`) is shown as the reference.
  - Step test: selected motors 0 → x % in steps, hold, back to 0.
- **Session.** Start/stop recording, file name, notes; export a CSV per message type plus a JSON with freezes and notes.
- **Analysis view.** FFT of gyro and accel with motor RPM marked, so vibration peaks can be matched to rotor speed.

## ESC telemetry and power wiring

See the ESC section of `.opencode/knowledge/project-history.md`. Summary:

1. **ESC TX → PA10 (USART1_RX)** with the Pi TX disconnected. One wire gives temperature, voltage, current, mAh and eRPM per motor (KISS, 10 bytes at 115200 baud). The firmware requests motors in turn with the DShot telemetry bit.
2. **ESC CURR → PB0** (already in `power.c`). Analog total current at ADC rate, calibrated against the KISS current.
3. **Battery divider → PA1** (already in `power.c`): optional with TX wired, useful as a check independent of the ESC.
4. Later, for flight with the Pi: bidirectional DShot with EDT on PB6–PB9, no extra wiring.

## Code standards for new code

- **C (new modules under `stm32/App`)**: MISRA C:2012 as a guideline, checked by `cppcheck --addon=misra` from the conda env. Documented deviations are allowed. HAL, CubeMX and Simulink code are excluded. Also: `-Wall -Wextra -Wconversion` clean, fixed-width types, no dynamic allocation, no recursion, SI units in names or comments (`_m`, `_rad_s`), one owner task per peripheral.
- **Python**: ruff format and lint, type hints, unit tests for parsers and statistics.
- Each module gets a host test in `stm32/tests` or `hmi/server/tests`.

## Proposed layout

CubeIDE expects `stm32/App/{Inc,Src}`. Inside it, new code is grouped by responsibility, and old files move only when they are touched:

```text
stm32/App/Src/
  drivers/     bno08x, mtf01, dshot, esc_telem, power
  estimation/  nav_ekf, range_filter, meas
  control/     lqg, mixer, trim, flight_ctrl
  link/        dronelink, usb_link
  app/         tasks, arming state machine, configuration
hmi/
  server/      Python link, recorder, statistics, tests
  web/         UI
```

## Plan

1. Firmware: arming state machine and DShot 0 guarantee, with host tests.
2. Firmware: DroneLink over USB CDC with heartbeat and the standard messages above.
3. Server: link, recorder and replay of recorded sessions (allows UI work without hardware).
4. Web UI: live panel, freeze and comparison, then the motor panel.
5. Firmware: KISS telemetry on USART1 and `power.c` enabled, after wiring.
6. Noise study session (four conditions) and report.

## Open decisions

- Which values get charts by default.

Decided: freeze saves instantly and persists until deleted; firmware cap 40 % with a UI cap slider; per-motor sliders, any-subset grouping and direction toggles; no front-end build step.
