# Thesis and deployed software alignment

Reviewed sources: thesis introduction and conclusions; communication methodology, results and conclusions; route methodology, results and conclusions; deployed Pi software; source branch history and protocol definitions.

| Thesis claim | Import evidence and qualification |
| --- | --- |
| STM32 flight control and Pi mission planning are separate | Firmware under `stm32/`; deployed mission software under `rpi/` |
| DroneLink uses MAVLink 2 and a shared XML definition | Baseline STM32 and deployed `drone` XML files are byte-identical (SHA-256 `cfa487b4aa1625b5acdd3bcd95db662ba1a74456f991a5027540f20ba107b3d5`) |
| UART at 115200 bit/s through `/dev/serial0` | CLI defaults agree; Pi symlink resolves to `ttyAMA0`; boot configuration enables UART, disables Bluetooth and has no serial console |
| Pi software versions | Read-only inspection confirms Python 3.13.5, NumPy 2.2.4, pymavlink 2.4.49 and pyserial 3.5 on Raspbian 13 |
| Mission command names in English | `takeoff`, `yaw`, `goto`, `route`, `land`, etc. are implemented; numeric protocol reason identifiers use English names |
| Genetic algorithm minimizes route distance | `dronectl/rotas.py` sums Euclidean 3D distances around a closed circuit; time, energy and obstacles are not cost terms |
| Same seeded GA route on PC and Pi | Archived source contains the signature reproduction script and historical results; local rerun is recorded separately in validation evidence |
| Integrated communication tests use simulated flight on real STM32 | Preserved thesis and communication result files describe this setup; do not relabel them as autonomous physical flight |
| Linear Kalman/LQG baseline; EKF is future work | Baseline uses committed `hover-lqg-pi`; local EKF changes belong to the exploration branch |

## Deployed differences

The deployed `~/drone/README.md` still names `comunicacao-pi-stm32`, but its files include later hover-test changes. That README alone cannot identify the deployment commit. Relative to the integrated baseline's shared files, the Pi differs in README files, CLI, hover tests and hover-test implementation. Changes include `~/real_log_kit` recording, a pre-arm parameter snapshot and low-altitude abort handling. These deployed changes were retained. See `provenance/pi-vs-firmware-files.json`.

`~/drone-better` has a different protocol hash (`13972c42414242aeef37fef1f8d900671d1af4673edc96c6e7a4f749715c6975`) and is preserved separately. It must not silently replace the baseline Pi software.

## Language audit

The thesis says the mission commands use conventional aeronautical nomenclature and the protocol identifiers are English. It does not establish that every program message is English. The deployed CLI help, diagnostics, comments and several historical module/file names are Portuguese. The repository adaptation translates the main CLI help and adds English top-level command aliases without changing protocol fields, algorithms or flight behavior. Full localization of inherited runtime diagnostics, source comments and historical documentation remains unfinished and is listed in the backlog. The thesis and raw evidence remain unchanged in their original language.

## Evidence boundaries

This is an import and offline verification, not a new hardware qualification. No UART commands, arming, flashing, deployment or flight tests were performed. The flashed STM32 image was not read back. The original MATLAB/Simulink project is unavailable; generated C references do not replace it. The thesis reports limitations in horizontal position and yaw stabilization and does not demonstrate robust autonomous free flight.
