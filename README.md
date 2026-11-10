# Isolated USB control milestone

This branch preserves the original `controle` firmware at `2c638cb`. The Python desktop GUI in `stm32/tools/controle_gui.py` communicates directly with the STM32 USB CDC console, typically `/dev/ttyACM0`. It tests attitude and height with horizontal position control disabled. The Pi is not required for this test interface.

The firmware tree is preserved byte-for-byte. Original procedures and recorded limitations are in `stm32/docs/teste_controle.md`; no new hardware qualification was performed during this import.

Related versions:

- `milestone/03-position-lqg`: later position-control stage.
- `milestone/04-dronelink`: isolated communication/protocol tests with real Pi/UART and simulated flight on STM32.
- `submission/2026-11-11`: integrated thesis baseline.

Other directories, including `rpi/` and `latex/`, remain the submission capture, not deployments from the earlier USB-test date. See `docs/versions.md` and `provenance/active-version.json`.
