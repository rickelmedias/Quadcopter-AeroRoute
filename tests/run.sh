#!/bin/sh
# Compila e roda todas as suítes de teste de host com ASan/UBSan.
#
# Só os fontes puros são ligados - os que não dependem de HAL, FreeRTOS ou
# USB. Esse conjunto é exatamente o código que os testes podem exercitar: o
# que não pode ser ligado aqui também não pode ser testado aqui, e o erro de
# ligação diz isso em vez de o teste cobrir em silêncio uma reimplementação.
# tests/stubs fornece o único header da HAL de que mtf01.h precisa para os
# tipos; CTRL_HOST_BUILD troca a seção crítica do FreeRTOS por nada.
#
# test_lqg_ert também liga o código gerado pelo Simulink
# (reference/STM32_ert_rtw) para comparar os dois, em double (bit a bit) e
# em float.
#
# Uso (Linux ou WSL, a partir de qualquer pasta):
#     sh tests/run.sh
#
# Sai com código 1 se alguma suíte falhar.
set -e
cd "$(dirname "$0")/.."

PURE="App/Src/attitude.c App/Src/mtf_parser.c
      App/Src/control/lqg_ert.c App/Src/control/lqg_ert_model.c
      App/Src/control/ctrl_gains.c App/Src/control/ctrl_params.c App/Src/control/meas.c
      App/Src/control/mixer.c App/Src/control/attitude_mode.c App/Src/control/trim.c"
REFERENCE="reference/STM32_ert_rtw/STM32.c reference/STM32_ert_rtw/STM32_data.c"

CFLAGS="-std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined
        -fno-omit-frame-pointer -g -DCTRL_HOST_BUILD -IApp/Inc -Itests/stubs
        -Ireference/STM32_ert_rtw"

OUT="${TMPDIR:-/tmp}/dronecontroller-tests"
mkdir -p "$OUT"

failures=0

# Compila e roda uma suíte. $1 = nome do binário, $2 = fonte, demais = flags.
run_suite() {
  name=$1
  src=$2
  shift 2
  extra=""
  case "$src" in
    *test_lqg_ert.c) extra="$REFERENCE" ;;
  esac
  if ! gcc $CFLAGS "$@" "$src" $PURE $extra -lm -o "$OUT/$name" 2>"$OUT/$name.log"; then
    echo "BUILD FAIL  $name"
    cat "$OUT/$name.log"
    failures=$((failures + 1))
    return
  fi
  if "$OUT/$name" >"$OUT/$name.out" 2>&1; then
    echo "PASS  $name"
  else
    echo "FAIL  $name"
    cat "$OUT/$name.out"
    failures=$((failures + 1))
  fi
}

for src in tests/test_*.c; do
  run_suite "$(basename "$src" .c)" "$src"
done
run_suite test_lqg_ert_float tests/test_lqg_ert.c -DLQG_ERT_SINGLE_PRECISION

echo
# As tabelas do firmware precisam ser as da exportação do Simulink.
if python3 tools/ert_import.py --check >"$OUT/ert_import.log" 2>&1; then
  echo "PASS  lqg_ert_model.c confere com reference/STM32_ert_rtw"
else
  echo "FAIL  lqg_ert_model.c desatualizado"
  cat "$OUT/ert_import.log"
  failures=$((failures + 1))
fi

# O firmware e a ferramenta da Pi precisam concordar sobre o formato do envio
# de ganhos: os dois calculam o CRC dos ganhos do Simulink.
c_crc=$(sed -n 's/^default_crc=//p' "$OUT/test_ctrl_gains.out" 2>/dev/null || true)
py_crc=$(cd tools/pi && python3 -c 'from dronectl.gains import default_gains; print("%08X" % default_gains().crc32())' 2>"$OUT/crc.log" || true)
if [ -n "$c_crc" ] && [ "$c_crc" = "$py_crc" ]; then
  echo "PASS  CRC dos ganhos igual no C e no Python ($c_crc)"
else
  echo "FAIL  CRC dos ganhos: C=$c_crc Python=$py_crc"
  cat "$OUT/crc.log"
  failures=$((failures + 1))
fi

# Projeto LQR/Kalman da Pi (precisa de numpy) e enlace contra um STM32 falso
# (precisa de pymavlink e pyserial). Sem as bibliotecas, só avisa.
if python3 -c 'import numpy' 2>/dev/null; then
  if (cd tools/pi && python3 -m unittest discover -s tests -q) >"$OUT/pi.log" 2>&1; then
    echo "PASS  tools/pi: projeto reproduz o Simulink e enlace com STM32 simulado"
  else
    echo "FAIL  tools/pi"
    cat "$OUT/pi.log"
    failures=$((failures + 1))
  fi
else
  echo "SKIP  tools/pi (instale numpy, pymavlink e pyserial para rodar)"
fi

echo
if [ "$failures" -eq 0 ]; then
  echo "all suites passed"
else
  echo "$failures suite(s) failed"
  exit 1
fi
