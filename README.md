# Quadcopter AeroRoute

Experimental quadcopter developed for the UniFacens Computer Engineering thesis by Rickelme Gabriel Dias and Thalison Rosa (2026).

The Phase 2 branch is a clean STM32 bench platform: FreeRTOS, sensor acquisition,
logging, DShot and safe manual motor tests. The old flight controller, Raspberry
Pi mission software, protocol and Simulink model belong to historical branches.

## Getting Started

- [Phase 2 bench architecture](docs/phase2/architecture-review.md): what was deliberately retained and removed from this clean foundation.

- [Architecture / Structure](docs/getting-started/architecture-structure.md): system responsibilities, directory guide, branches and experimental milestones.
- [Getting Started index](docs/getting-started/README.md): documentation entry point.

## Repository layout

| Directory | Contents |
| --- | --- |
| `stm32/` | Bench firmware, CubeIDE configuration, platform drivers, host tests and build tools |
| `hmi/` | Bench HMI: Python server and browser UI, simulator, recording and replay |
| `rpi/` | Empty boundary reserved for the future control branch |
| `latex/` | Original Portuguese thesis source and figures, preserved without translation |
| `docs/` | English repository guide, thesis alignment notes and documentation backlog |
| `matlab/` | Placeholder for the original MATLAB/Simulink project, not yet available |
| `references/` | Local ignored upstream material and copied legacy model |

## Versions

Use branch `submission/2026-11-11` or annotated tag `tcc-2026-11-11` for the thesis baseline. It was captured on **2026-11-10** for the planned November 11 submission; this is not a claim that a new capture occurred on November 11. `main` starts from this baseline and can evolve. Do not move the submission tag as development continues.

Legacy control, protocol and Pi work remains available in the historical
branches, especially `exploration/ekf-2026-11-10`, `milestone/*`, `archive/*`
and the submission tag. It is not part of this bench working tree.

The flight results reported in the thesis do **not** establish reliable autonomous free flight: horizontal position and yaw drift remained limitations. Communication mission results include a simulated plant running on the real STM32.

## Working with the source

Open `stm32/` as the existing STM32CubeIDE project. Preserve its internal directory layout; scripts and generated references depend on it. The original motor GUI is `stm32/tools/motores_gui.py` on the motor-test milestone.

For firmware host tests:

```sh
cd stm32 && sh tests/run.sh
```

These are host checks, not hardware qualification. The bench HMI (`hmi/`, see
[`hmi/README.md`](hmi/README.md)) runs against the simulated STM32 with
`conda run -n aeroroute python -m benchlink serve --simulate` from `hmi/`.

Build the thesis from `latex/` using XeLaTeX, BibTeX, then XeLaTeX twice, with abnTeX2 and its dependencies installed. The original Arial font requirement and fallback behavior are defined in `main.tex`.

New repository documentation is in English. Original historical sources, logs, identifiers and the submitted Portuguese thesis retain their original language for traceability. The deployed program is not fully localized; see the language audit before describing it as an English-only interface.

## Historical work

The old control, protocol, Pi and flight experiments are intentionally accessed
by switching to their historical branches. They are not dependencies of Phase 2
Bench and are not used by its build or tests.
