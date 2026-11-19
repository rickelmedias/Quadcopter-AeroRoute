# Phase 2 / 01: Bench HMI

This branch first validates the new firmware platform in a clean environment.
It reuses lessons from the legacy code but does not carry legacy control,
protocol, Pi tools, model exports or flight ensaios.

## Bench objective

Validate, with truthful measurements and repeatable sessions:

- BNO085 raw reports and orientation;
- MTF-01 range, optical flow and quality;
- power measurements when hardware is connected;
- DShot output, motor mapping, direction and safety stops;
- timestamped sensor, motor and system logs.

The first planned study is sensor noise and motor interference under controlled
bench conditions. No flight claim is made by this branch.

## Implemented

- Firmware: bench-link v1 over USB CDC (`bench-link.md`), HEARTBEAT-based
  keepalive, ESC KISS telemetry parser and scheduler, power ADC with runtime
  calibration and optional minimum voltage to arm.
- HMI (`hmi/`): live panel with units and validity, per-value and group
  freezing with 1 s mean/σ, live-versus-frozen Δ and two-row comparison, motor
  panel (cap, ramp, sliders, group, direction, beep, step test), charts, FFT with
  measured rotation markers, recording and replay, calibration, host-side
  estimate, simulated STM32.
- Tests: C host suites (framing and golden vectors shared with Python, command
  ranges, KISS, power, arbiter) and Python suites (contract, hub safety, server,
  analysis).

## Not yet validated

Everything above is verified on the host and the simulator only. Remaining:
flash and check USB/sensors with motors disarmed, then DShot zero and STOP
behavior on a logic analyzer, KISS reception once the ESC TX is wired to PA10,
ADC calibration, and the sensor-noise study sessions. The UI was not visually
reviewed in a browser (headless Firefox hung in this environment).

## Safety boundary

Automated tests never arm or spin motors. Physical tests are performed by a
person at the bench, without propellers unless a protected thrust stand and a
written test procedure are in place.
