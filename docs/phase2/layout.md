# Phase 2 Bench: current layout

This branch is intentionally a clean bench foundation. It is not a reduced
flight branch and it does not contain the old controller, DroneLink, MAVLink,
Raspberry Pi mission software or the old Simulink model as build inputs.

```text
stm32/
  App/{Inc,Src}/       sensors, power, DShot, motors, logging and bench USB
  Core/                CubeMX/FreeRTOS startup
  Drivers/             STM32 HAL/CMSIS
  Middlewares/         FreeRTOS and USB CDC only
  tests/               host tests for bench modules
  tools/               reproducible build and code checks
  docs/                bench documentation
rpi/                   reserved boundary for the future control branch
references/            ignored local material, including legacy-simulink/
hmi/                   bench HMI: Python server, browser UI, simulator, recorder
```

The old generated Simulink export was copied locally to
`references/legacy-simulink/` and removed from the firmware tree. It is
historical material only and must not be used as the new controller model.
The ignored `references/` directory also contains upstream research clones.

## Bench firmware

The only application is `bench`: sensors (BNO085, MTF-01), power ADC (PA1/PB0),
KISS ESC telemetry (USART1 RX on PA10), DShot motors with the safety arbiter,
and the binary bench-link v1 session over USB CDC. Protocol, safety rules and
message tables: [`bench-link.md`](bench-link.md). The arbiter enforces zero at
boot, a 40 % cap after every boot (0–100 % adjustable), ramping, the 300 ms
keepalive renewed only by `HEARTBEAT`, direction restrictions and STOP.

```text
stm32/App/Src/
  link/        bench_frame (COBS+CRC16), bench_proto (messages, command validation)
  actuators/   motor_arbiter, esc_kiss (pure), esc_telem (USART1 DMA)
  app/         startup, bench_link (session, rules, periodic status)
  telemetry.c  queue and the single USB transmit task
  power*.c     ADC task and pure calibration
```

The earlier text console was removed; there is no second USB protocol.

## Build and tests

```sh
conda run -n aeroroute python tools/build.py
conda run -n aeroroute python tools/build.py --debug
sh tests/run.sh
(cd ../hmi && conda run -n aeroroute python -m unittest discover -s tests -t .)
```

Artifacts are written to `stm32/build/Release/` or `stm32/build/Debug/` and
include a manifest identifying `phase2-bench`; its `firmware_id` is also sent in `HELLO`
and stored in every recorded session. The build never flashes hardware.

## Deliberately absent

Flight control/LQG/EKF, protocol XML/MAVLink/DroneLink, Pi deployment and
mission tools, old experiments and logs. Not yet qualified on hardware: USB
timing, DShot zero delivery, KISS reception on the real ESC wiring, ADC
calibration, and the host-side estimate gains.

The next clean control branch will be built from validated bench contracts and
measurements, with a new model and modules following the software standards.
