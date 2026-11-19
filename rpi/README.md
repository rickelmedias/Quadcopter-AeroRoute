# Raspberry Pi: dronectl

Mission computer side of DroneLink (`docs/phase2/dronelink.md`). The controller
runs on the STM32; the Pi asks for legs, orders route points and records tests.

## Install (Pi Zero W, Raspberry Pi OS)

```sh
cd rpi
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python -m dronectl status            # needs the STM32 on /dev/serial0
```

The MAVLink module is generated from `../protocol/drone_controller.xml` into
`~/.cache/dronectl` the first time and whenever the XML changes.

## Commands

| Command | What it does |
| --- | --- |
| `status` | boot id, DroneLink version, state, pose, battery, `[FLT_SIM]` |
| `monitor [--time s]` | state and pose every second |
| `hover --height 30 --time 20` | arm, take off, hold, land; writes `~/drone/ensaios/hover_<date>/` (`telemetry.csv`, `texts.txt`, `summary.json`) and prints height error, XY deviation, yaw error and saturation in HOLD |
| `mission [file]` (alias `missao`) | step-by-step commands from a file or an interactive shell: `takeoff`, `yaw`, `forward/back/right/left/climb/descend`, `goto`, `waypoints`, `route [alg]`, `rtl`, `hold`, `wait`, `land`, `status` (cm, degrees, s) |
| `land`, `disarm` | land and disarm / disarm now |
| `routes [alg] x,y[,z] ...` | only computes the visit order (no drone) |

**Stopping.** In `mission` and `hover`, **Ctrl+C is the emergency stop**: the
motors are disarmed at once (< 0.1 s; in the air the drone falls) and nothing
tries to land. Use the `land` command (or let the mission finish) for a soft
landing. If the Pi program dies without Ctrl+C (SSH dropped, power), the STM32
holds after 1 s and lands after 5 s on its own. `dronectl disarm` from another
terminal also cuts, but takes 2–4 s on the Pi Zero (Python start-up).

Limits follow the firmware: height 10–80 cm, |x|, |y| ≤ 300 cm from the take-off
origin.

## Test stages (one at a time, next only after the previous one works)

| File | Checks |
| --- | --- |
| `missions/stage1_hover.txt` | take off to 30 cm, settle within 5 cm of (0, 0), hold 15 s, land |
| `missions/stage2_a_b_a.txt` | 40 cm forward to B and back to A, no turning |
| `missions/stage3_yaw.txt` | 90° right and back, staying on the spot |
| `missions/stage4_square.txt` | the 40 cm square of `missions/points.txt` in genetic-algorithm order (turn + straight leg per point) |

```sh
~/drone/dronectl mission ~/drone/rpi/missions/stage1_hover.txt --record
```

`--record` writes `~/drone/ensaios/mission_<stage>_<date>/telemetry.csv` and prints
the HOLD summary (height error, XY deviation from the reference, yaw error, saturation).

`dronectl/routes.py` is the thesis `rotas.py` with English names; host tests
check that every algorithm returns the same order and cost (seed 2026).

## Tests (PC, no hardware)

```sh
cd rpi && conda run -n aeroroute python -m unittest discover -s tests -t .
```

`tests/fake_fc.py` is a fake STM32 on a pseudo-terminal (no dynamics: each leg
"arrives" after 0.2 s).

Never arm with propellers from here without a person at the bench; test first
against the `--sim` firmware build (motors at zero).
