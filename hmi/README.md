# Bench HMI

Python server plus browser interface for the Phase 2 bench. It talks
[bench-link v1](../docs/phase2/bench-link.md) to the STM32 over USB, without the
Raspberry Pi.

```sh
cd hmi
conda run --no-capture-output -n aeroroute python -m benchlink serve --simulate           # no hardware
conda run --no-capture-output -n aeroroute python -m benchlink serve --port /dev/ttyACM0  # real board
conda run --no-capture-output -n aeroroute python -m benchlink serve --replay <session>   # recorded session
```

Open <http://127.0.0.1:8765>. The source can also be changed in the
**Fonte e sessões** tab. Read-only terminal monitor (never sends motor
commands): `python -m benchlink monitor --port /dev/ttyACM0`.

## What it does

- **Live panel**: every BNO085, MTF-01, motor, ESC, power and system value with
  its unit, a validity/freshness dot and the 1 s standard deviation.
- **Freeze** ("congelar"): click a value to save it below with its 1 s mean, σ
  and sample count, or freeze a whole group or everything (key F). Rows persist
  in `data/freezes.json` until deleted. Each row shows the live value, Δ and
  |Δ|/σ, and is highlighted when |Δ| > 3σ. Select two rows to compare them
  against each other. Charts draw frozen values as dashed reference lines.
- **Motors**: arm/disarm, STOP (button, Space or Esc), cap slider (40 % after
  every boot; going above 40 % needs an explicit unlock), ramp, four motor
  sliders, a group slider for the checked motors, direction per motor (ESC RAM
  or saved to EEPROM after a second click), beep, step test and ESC telemetry.
  Arming requires ticking the "no propellers or fixed stand" confirmation.
- **Controle tab** (branch `phase2/control-lqg`): arm, take off, hover, land, a red
  PARAR button and Space/Esc, editable height/hover/limits, and the measurements,
  estimate and trim of the LQR + Kalman running on the STM32
  ([control-lqg.md](../docs/phase2/control-lqg.md)). The simulator only plays the
  sequence (first-order height), it does not run the LQR.
- **Charts** for any value (📈), **spectrum** (FFT, Hann window) of gyro and
  acceleration with measured motor rotation (from ESC telemetry) and its second
  harmonic marked.
- **Recording**: raw bytes for exact replay, one CSV per message type with units
  in the column names, events (commands with ACK, markers, freezes, notes),
  bench conditions and the firmware identity from HELLO. Key M adds a marker.
- **Calibration**: battery divider and CURR scale/offset with helpers based on a
  multimeter or a known current, minimum voltage to arm, re-sent to the STM32 at
  every new session. Host-side estimate settings (below).
- **Simulator**: a Python STM32 that speaks the same protocol and applies the
  same arming/keepalive/cap rules, for UI work and tests. It is not a model of
  the drone.

## Host-side estimate

`benchlink/estimator.py`, read-only and labeled "Estimativa no PC": height =
distance·cos φ·cos θ (filtered), vertical speed, and horizontal speed from flow ×
height with an optional gyro compensation gain that must be measured on the
bench. It is a bench validation aid, not the future flight estimator.

## Safety model

The STM32 decides; the HMI only requests (see `stm32/App/Inc/app/bench_link.h`).

- One browser tab holds control; others are view-only (STOP works from any tab).
- The server forwards HEARTBEAT only while the controlling page sends its own
  heartbeat every 100 ms. If the page freezes, closes, or the tab goes to the
  background (browsers throttle timers), the server sends STOP and the firmware
  also disarms on its own 300 ms after the last HEARTBEAT. This automatic STOP
  only applies to an output armed from this HMI: in a flight armed by the
  Raspberry Pi (DroneLink) the HMI just records, and its STOP button still works.
- No blocking browser dialogs are used: they would stop the heartbeat.
- Replay never sends commands. The server listens on 127.0.0.1 only.

## Layout

```text
benchlink/protocol.py   framing and message codecs (mirror of the C headers)
benchlink/hub.py        decoding, live values, control, recording, freezes
benchlink/sources.py    serial port, simulator, replay
benchlink/simulator.py  simulated STM32
benchlink/recorder.py   session files
benchlink/freezes.py    persistent frozen samples
benchlink/signals.py    signal names, units and groups
benchlink/stats.py      rolling windows, σ, spectrum
benchlink/estimator.py  host-side estimate
benchlink/server.py     HTTP + Server-Sent Events + JSON API (stdlib)
web/                    index.html, app.js, chart.js, style.css (no build step)
tests/                  unittest suite (protocol contract, hub safety, server, analysis)
data/                   sessions, freezes and config (ignored by Git)
```

Tests: `cd hmi && conda run -n aeroroute python -m unittest discover -s tests -t .`
