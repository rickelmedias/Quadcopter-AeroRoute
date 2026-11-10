# Import review against the original request

## Verified preservation

The firmware trees of all original milestone snapshots and the submission branch match their source Git trees exactly. The original Git bundle verifies as complete. All 97 thesis files match the capture manifest. The deployed Pi copy differs in exactly the three documented adapted files: README, launcher and main CLI. Both baseline and experimental STM32/Pi protocol pairs match. The submission branch and annotated tag still identify the same commit. Machine-readable results are in `provenance/validation/import-audit.json`.

## Corrections

- The isolated USB control milestone was added from `controle` after its omission was identified.
- Stage 5 now has an explicit `milestone/05-integrated-pi-hover` reference to the submission snapshot.
- Two unrelated shopping automation scripts were removed from the evolving `main` tree. Their originals remain preserved in historical snapshots and the Git bundle.
- The isolated communication branch does not already contain the later position-control branch. The combination occurred in merge `5ff1699`, before integrated baseline `af6034e`.

## Outstanding requirements and limitations

- Full English localization was requested but not completed. Main CLI help and English command aliases were adapted; mission errors, runtime diagnostics, comments, inherited documentation and desktop GUIs still contain Portuguese. Historical evidence and the Portuguese thesis should retain their original content; adapting maintained code remains separate work.
- Milestone branches change firmware only. Their `rpi/` directories retain the submission deployment, not reconstructed contemporaneous software. Historical companion sources are available in each milestone's `stm32/tools/pi/` where present.
- The frozen submission documentation predates later milestone clarifications. Current guidance is on `main`; the frozen tag has not been rewritten.
- No branches or tags have been published: the remote reference query returned an empty list during review. Local commits alone do not make the thesis version accessible to readers on GitHub.
- No STM32 firmware readback was performed, so source preservation cannot certify the flashed binary's identity. No cross-build or thesis PDF build was performed. Prior offline test results remain recorded; this audit checks import integrity, not renewed hardware qualification.
- MATLAB/Simulink originals remain unavailable as stated by the project owner; the placeholder is intentional.

This review does not claim the original request is fully complete while localization and publication remain outstanding. The deployed drone and original source directories were not modified.
