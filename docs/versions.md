# Version provenance

The source repository was read without switching branches, committing its changes, flashing hardware or deploying software. Its active branch was `hover-lqg-pi-better`, at `af6034e`, with tracked and untracked changes.

| Stage | Original reference | Repository branch | Purpose |
| --- | --- | --- | --- |
| Sensors | `mtf-bno-test` (`3166a23`) | `milestone/01-sensors` | MTF-01 and BNO08x testing |
| Motors | `motores` (`a9ce0b4`) | `milestone/02-motors` | Includes `tools/motores_gui.py`; `origin/motors` is an older alternative |
| Position control | `controle-lqg-posicao` (`fdd7106`) | `milestone/03-position-lqg` | LQG position-control tests |
| Communication | `comunicacao-pi-stm32` (`f7349fe`) | `milestone/04-dronelink` | DroneLink integration and route reproducibility |
| Integrated baseline | `hover-lqg-pi` (`af6034e`) | `submission/2026-11-11` | Merge of position control and Pi communication, before local EKF changes |
| Exploration | Active working tree on `hover-lqg-pi-better` | `exploration/ekf-2026-11-10` | Local EKF/filter/control changes, not thesis baseline |

Milestone branches replace `stm32/` with the corresponding original tracked tree. Other directories remain the submission capture, not historical deployments from those earlier dates. See each branch's `provenance/active-version.json`.

`provenance/firmware-history.bundle` preserves all original refs, commits, tags and the stash (including unused branches). Restore it separately with `git clone provenance/firmware-history.bundle restored-firmware`. `source-branches.txt` maps original references. The experimental patch is relative to `af6034e`; the accompanying untracked archive preserves files outside that commit. These archives are historical evidence, not the active firmware.

The Pi snapshot contains `drone`, `drone-better`, `real_log_kit` and `backup`; bytecode caches were excluded. Runtime recordings are evidence, not reproducible build inputs. No virtual environment, SSH credentials or home-directory shell history were copied. No source branch was checked out or changed. No push was performed.

The exact firmware binary currently flashed on the STM32 was not read back, so its identity cannot be certified from the source branch alone.
