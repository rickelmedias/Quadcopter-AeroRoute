# Architecture / Structure

Phase 2 is currently a clean STM32 bench project. It deliberately contains no
flight controller, Raspberry Pi runtime, DroneLink/MAVLink protocol or old
Simulink model.

```text
stm32/
  App/                  manual bench firmware
    Inc,Src/            sensors, power, DShot, motors, logging, USB
    sh2/                BNO085 vendor library
  Core/                 CubeMX/FreeRTOS startup
  Drivers/              STM32 HAL/CMSIS
  Middlewares/          FreeRTOS and USB CDC
  USB_DEVICE/           generated USB CDC device
  tests/                host-only bench tests
  tools/                build and checks
  docs/                 bench notes
hmi/                    bench HMI (Python server, browser UI, simulator)
rpi/                    empty future-control boundary
references/             ignored local legacy/upstream material
```

The historical Simulink export was moved locally to
`references/legacy-simulink/`. It is not compiled, generated from, or treated
as the new model. The old code and experiments remain available by switching
to their historical Git branches/tags.

## Current flow

```text
BNO085 ─┐
MTF-01 ─┼─ telemetry queue ── bench-link v1 ── USB CDC ── hmi (Python) ── browser
ADC    ─┤                                          ▲
ESC TX ─┘ (KISS, PA10)       HEARTBEAT, commands ───┘
                                │
                  motor safety arbiter ── DShot ── ESCs
```

The firmware decides safety; the HMI only requests. See
[`bench-link.md`](../phase2/bench-link.md) and [`hmi/README.md`](../../hmi/README.md).

## Build and tests

```sh
conda run -n aeroroute python stm32/tools/build.py
conda run -n aeroroute python stm32/tools/build.py --debug
cd stm32 && sh tests/run.sh
```

These commands compile and test the bench firmware only. They do not flash the
board, open a serial port, arm motors or qualify physical behavior.

## Historical branches

Use `submission/2026-11-11`, `milestone/*`, `archive/*` and
`exploration/ekf-2026-11-10` to inspect the legacy control, protocol, Pi tools,
models and ensaios. This branch is not required to preserve those files in its
working tree.
