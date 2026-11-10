# Import validation

- All 97 thesis files match the source SHA-256 manifest; all active input/include paths resolve. PDF compilation was not run because the LaTeX toolchain is unavailable here.
- Original Git bundle verified successfully and contains complete source history.
- Baseline deployed Pi and STM32 protocol XML files match byte-for-byte.
- Experimental Pi and local experimental STM32 protocol XML files match byte-for-byte.
- Firmware host runner completed successfully: C tests with ASan/UBSan, generated LQG/reference checks, cross-language gains CRC `21F15F2A` and bundled Python tests.
- Deployed Pi copy passed 47 offline unit tests (91.386 seconds).
- Three local GA signature reruns match the thesis prefixes: `52d2b0c4`, `9ffc5830`, `11f260a1`.
- English CLI help and aliases were checked without opening a serial connection.

Evidence is under `provenance/validation/`. Local verification used Python 3.14.7 and NumPy 2.5.4, not the deployed Python 3.13.5 / NumPy 2.2.4 environment. pymavlink 2.4.49 and pyserial 3.5 match the Pi. Initial dependency setup was incomplete; the recorded final firmware run occurred after dependencies were installed.

No flight qualification, STM32 cross-build, firmware readback or hardware test was performed. Historical milestone branches are preserved snapshots, not newly tested versions. Experimental changes are retained separately and are not promoted to the thesis baseline.
