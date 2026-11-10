# Raspberry Pi mission computer

Imported directly from the deployed `~/drone` on 2026-11-10. The original byte-for-byte snapshot is archived in `provenance/raspberry-pi-snapshot.tar.gz`; hashes are in `provenance/pi-deployed-sha256.json`.

The local launcher accepts `DRONE_PYTHON=/path/to/venv/bin/python`. Install `tools/pi/requirements.txt` in that environment. Run `./dronectl --help` for commands. `mission`, `routes-bench` and `routes-limit` are English aliases; original command names remain compatible. Main CLI help is translated into English. Internal historical identifiers, diagnostics and comments retain their original language.

The mission shell supports `takeoff`, `yaw`, `forward`, `back`, `right`, `left`, `climb`, `descend`, `goto`, `route`, `waypoints`, `rtl`, `hold`, `wait`, `land`, `status`, `help` and `exit`. `route ga` selects the genetic algorithm. Distances use centimeters in the mission shell; the separate `hover` CLI uses meters. Read each command's help before use.

The XML and generated-gain references remain in this subtree because Python resolves them relative to its root. Its `protocol/drone_controller.xml` is byte-identical to the baseline STM32 definition. The installed program on the Pi was not modified.

Tests: from `tools/pi`, run `python -m unittest discover -s tests -v`. They use offline computation and a fake flight controller; no real serial connection is needed.
