# Quadcopter AeroRoute

Experimental quadcopter developed for the UniFacens Computer Engineering thesis by Rickelme Gabriel Dias and Thalison Rosa (2026).

The STM32F411 runs FreeRTOS, sensor acquisition, LQG control (LQR plus linear Kalman estimation), motor mixing, DShot and failsafes. The Raspberry Pi Zero W runs the Python mission computer and a genetic algorithm that minimizes the **total Euclidean distance of a closed 3D route**. DroneLink is the project application protocol over MAVLink 2 framing and UART. Minimum distance is the objective; minimum travel time, obstacle avoidance and energy optimization are not implemented objectives.

## Repository layout

| Directory | Contents |
| --- | --- |
| `stm32/` | Firmware, CubeIDE configuration, generated drivers, protocol, host tests and original desktop tools |
| `rpi/` | Software retrieved directly from the deployed Raspberry Pi `~/drone`, including its required model and protocol files |
| `latex/` | Original Portuguese thesis source and figures, preserved without translation |
| `docs/` | English repository guide, thesis alignment notes and documentation backlog |
| `matlab/` | Placeholder for the original MATLAB/Simulink project, not yet available |
| `provenance/` | Source manifests, complete original Git history, Raspberry Pi snapshots and experimental changes |

## Versions

Use branch `submission/2026-11-11` or annotated tag `tcc-2026-11-11` for the thesis baseline. It was captured on **2026-11-10** for the planned November 11 submission; this is not a claim that a new capture occurred on November 11. `main` starts from this baseline and can evolve. Do not move the submission tag as development continues.

Branch `exploration/ekf-2026-11-10` preserves the uncommitted experimental STM32 work and the Pi `drone-better` snapshot. It is not evidence for the thesis results. See [versions](docs/versions.md) and [thesis alignment](docs/thesis-alignment.md).

The flight results reported in the thesis do **not** establish reliable autonomous free flight: horizontal position and yaw drift remained limitations. Communication mission results include a simulated plant running on the real STM32.

## Working with the source

Open `stm32/` as the existing STM32CubeIDE project. Preserve its internal directory layout; scripts and generated references depend on it. The original motor GUI is `stm32/tools/motores_gui.py` on the motor-test milestone.

For offline Python tests:

```sh
python3 -m venv .venv
.venv/bin/pip install -r rpi/tools/pi/requirements.txt
(cd rpi/tools/pi && ../../../.venv/bin/python -m unittest discover -s tests -v)
```

Run firmware host tests with dependencies available on PATH: `cd stm32 && sh tests/run.sh`. These are host checks, not flight qualification. Use `rpi/dronectl --help` for the local entry point; `DRONE_PYTHON` can select a Python interpreter. Commands that open UART are intended for a deliberately configured test setup.

Build the thesis from `latex/` using XeLaTeX, BibTeX, then XeLaTeX twice, with abnTeX2 and its dependencies installed. The original Arial font requirement and fallback behavior are defined in `main.tex`.

New repository documentation is in English. Original historical sources, logs, identifiers and the submitted Portuguese thesis retain their original language for traceability. The deployed program is not fully localized; see the language audit before describing it as an English-only interface.

## Test milestones

- `milestone/03-usb-control`: original `controle` firmware and Python GUI over USB CDC, testing attitude/height without horizontal position control.
- `milestone/03-position-lqg`: later LQG position-control stage.
- `milestone/04-dronelink`: isolated Pi–STM32 communication/protocol test using the real UART and a simulated plant on STM32; also preserves route reproducibility work.

The position-control and communication branches are combined in the submission baseline. See [version provenance](docs/versions.md) for the distinction and historical source commits.

- `milestone/05-integrated-pi-hover`: integrated position-control and Pi communication baseline, identical to the submission snapshot.

See [import review](docs/import-review.md) for verified preservation and outstanding requirements.
