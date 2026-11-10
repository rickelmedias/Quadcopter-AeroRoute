# Capturas locais

Esta pasta recebe ensaios binários gravados com `tools/capture_binary.ps1` e
os CSVs produzidos por `tools/decode_capture.py`.

Os dados são ignorados pelo Git porque crescem rapidamente e são artefatos de
bancada. Quando uma captura precisar acompanhar uma versão pública, publique o
arquivo `.bin`, seus dois JSONs de metadados e, se necessário, os CSVs como
artefatos de uma GitHub Release. Registre no texto da Release o hardware, o
roteiro do ensaio e o commit usado para gerar o firmware.

Os ensaios presentes apenas nesta cópia local não são apagados pelo
`.gitignore`; eles simplesmente deixam de ser incluídos por padrão no Git.

