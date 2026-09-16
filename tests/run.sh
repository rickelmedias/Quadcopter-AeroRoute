#!/bin/sh
# Compila e roda todas as suítes de teste de host com ASan/UBSan.
#
# Só os fontes puros são ligados - os que não dependem de HAL, FreeRTOS ou
# USB. Esse conjunto é exatamente o código que os testes podem exercitar: o
# que não pode ser ligado aqui também não pode ser testado aqui, e o erro de
# ligação diz isso em vez de o teste cobrir em silêncio uma reimplementação.
# tests/stubs fornece o único header da HAL de que mtf01.h precisa para os
# tipos.
#
# Uso (no WSL, a partir de qualquer pasta):
#     sh tests/run.sh
#
# Sai com código 1 se alguma suíte falhar.
set -e
cd "$(dirname "$0")/.."

PURE="App/Src/attitude.c App/Src/disp_estimator.c App/Src/flow_ekf.c
      App/Src/flow_calibrator.c App/Src/mtf_parser.c"

CFLAGS="-std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined
        -fno-omit-frame-pointer -g -IApp/Inc -Itests/stubs"

OUT="${TMPDIR:-/tmp}/dronecontroller-tests"
mkdir -p "$OUT"

failures=0
for src in tests/test_*.c; do
  name=$(basename "$src" .c)
  if ! gcc $CFLAGS "$src" $PURE -lm -o "$OUT/$name" 2>"$OUT/$name.log"; then
    echo "BUILD FAIL  $name"
    cat "$OUT/$name.log"
    failures=$((failures + 1))
    continue
  fi
  if "$OUT/$name" >"$OUT/$name.out" 2>&1; then
    echo "PASS  $name"
  else
    echo "FAIL  $name"
    cat "$OUT/$name.out"
    failures=$((failures + 1))
  fi
done

echo
# As suítes C acima só provam que o escritor é coerente consigo mesmo. Esta
# etapa prova que ele concorda com quem o lê: o decoder em Python e o replay.
# Um formato que só o autor consegue ler não é um formato.
if python3 tools/verify_replay.py >"$OUT/replay.log" 2>&1; then
  echo "PASS  replay, decoder agreement and sabotage detection"
else
  echo "FAIL  replay, decoder agreement and sabotage detection"
  cat "$OUT/replay.log"
  failures=$((failures + 1))
fi

echo
if [ "$failures" -eq 0 ]; then
  echo "all suites passed"
else
  echo "$failures suite(s) failed"
  exit 1
fi
