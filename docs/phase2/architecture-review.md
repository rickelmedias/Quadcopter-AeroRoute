# Phase 2 Bench architecture decision

## Decision

`phase2/01-bench-hmi` is a new bench-learning environment. Legacy flight code
is not a runtime profile here. It is removed from this branch, not hidden behind
preprocessor switches.

The branch keeps only the platform and bench contracts needed to answer the
first questions: do the sensors acquire valid data, do the logs preserve it,
and does the actuator path fail safe under manual testing?

## Removed from this branch

- `App/{Inc,Src}/control` and `App/{Inc,Src}/dronelink`;
- generated MAVLink middleware and the project protocol XML;
- old LQG/EKF/simulation tests and generation tools;
- Raspberry Pi mission code, deployed ensaios and old profiles;
- duplicate model/reference files under `rpi/` and `stm32/`;
- tracked `provenance/` archives and old route/flight evidence.

The old Simulink export is retained only as local historical material under
`references/legacy-simulink/`. It is explicitly not the new control oracle.
The Git branches/tags remain the way to inspect legacy code and evidence.

## Kept as the bench base

- CubeMX/STM32 HAL, CMSIS, FreeRTOS and USB CDC platform files;
- BNO085 and MTF-01 acquisition;
- power inputs, DShot codec/driver and motor task;
- pure motor arbiter with cap, ramp, direction, timeout and stop rules;
- bench-link v1 binary USB protocol with framing and golden-vector tests;
- build and static-check tools.

## Next work

The HMI (`hmi/`) and the versioned USB bench transport now exist. They consume truthful raw sensor and actuator data and never own safety
decisions. The new controller is a later branch built only after bench values
and behavior are validated.
