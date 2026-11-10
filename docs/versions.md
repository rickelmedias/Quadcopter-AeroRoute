# Version provenance

The source repository was read without switching branches, committing its changes, flashing hardware or deploying software. Its active branch was `hover-lqg-pi-better`, at `af6034e`, with tracked and untracked changes.

| Stage | Original reference | Repository branch | Purpose |
| --- | --- | --- | --- |
| Sensors | `mtf-bno-test` (`3166a23`) | `milestone/01-sensors` | MTF-01 and BNO08x testing |
| Motors | `motores` (`a9ce0b4`) | `milestone/02-motors` | Includes `tools/motores_gui.py`; `origin/motors` is an older alternative |
| Isolated USB control | `controle` (`2c638cb`) | `milestone/03-usb-control` | Attitude/height control through the USB CDC console and `tools/controle_gui.py`; horizontal position control disabled |
| Position control | `controle-lqg-posicao` (`fdd7106`) | `milestone/03-position-lqg` | LQG position-control tests |
| Isolated communication/protocol | `comunicacao-pi-stm32` (`f7349fe`) | `milestone/04-dronelink` | Real Pi/UART/DroneLink with a simulated plant on STM32 (`FLT_SIM=1` for the documented test), plus route reproducibility |
| Integrated Pi hover milestone | `hover-lqg-pi` (`af6034e`) | `milestone/05-integrated-pi-hover` | Explicit stage 5 reference; same snapshot as the submission branch |
| Integrated baseline | `hover-lqg-pi` (`af6034e`) | `submission/2026-11-11` | Merge of position control and Pi communication, before local EKF changes |
| Exploration | Active working tree on `hover-lqg-pi-better` | `exploration/ekf-2026-11-10` | Local EKF/filter/control changes, not thesis baseline |

Milestone branches replace `stm32/` with the corresponding original tracked tree. Other directories remain the submission capture, not historical deployments from those earlier dates. See each branch's `provenance/active-version.json`.

`provenance/firmware-history.bundle` preserves all original refs, commits, tags and the stash (including unused branches). Restore it separately with `git clone provenance/firmware-history.bundle restored-firmware`. `source-branches.txt` maps original references. The experimental patch is relative to `af6034e`; the accompanying untracked archive preserves files outside that commit. These archives are historical evidence, not the active firmware.

The Pi snapshot contains `drone`, `drone-better`, `real_log_kit` and `backup`; bytecode caches were excluded. Runtime recordings are evidence, not reproducible build inputs. No virtual environment, SSH credentials or home-directory shell history were copied. No source branch was checked out or changed. No push was performed.

The exact firmware binary currently flashed on the STM32 was not read back, so its identity cannot be certified from the source branch alone.

## Isolated tests versus integrated flight control

`milestone/03-usb-control` preserves the earlier `controle` branch. Its desktop GUI talks directly to the STM32 USB CDC console (typically `/dev/ttyACM0`). It tests attitude and height with horizontal control disabled; it is distinct from the later `milestone/03-position-lqg` stage.

`milestone/04-dronelink` already represents the isolated communication/protocol test. The Pi and UART are real; the documented test explicitly enables `FLT_SIM=1` so the STM32 controls a simulated plant without spinning the motors. Simulation is a runtime test setting, not the default firmware setting. This original branch does not contain the later position-control branch. Their merge is commit `5ff1699`, retained in the integrated submission firmware at `af6034e`.

The two `03-*` names distinguish control stages without renaming existing branches. These are source snapshots, not newly repeated hardware tests. The USB milestone's own README explains its scope; its `rpi/` directory remains the submission capture and is not needed for the USB test.

The stage 5 milestone is an additional reference to the frozen submission commit; no firmware or submission tag was changed. Subsequent cleanup on `main` removes unrelated shopping automation scripts while originals remain in submission, milestone and archive history.
