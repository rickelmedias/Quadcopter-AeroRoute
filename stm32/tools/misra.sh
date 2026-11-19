#!/bin/sh
# Verificação MISRA C:2012 (cppcheck + addon misra) dos fontes pedidos.
#
# Uso (da pasta stm32/):
#     sh tools/misra.sh App/Src/actuators/motor_arbiter.c [mais fontes...]
#
# Usa o ambiente conda "aeroroute" (environment.yml). AEROROUTE_ENV muda o
# caminho. O cppcheck do conda-forge não acha o addon por --addon=misra, por
# isso o script gera o .dump e roda o misra.py diretamente. Os textos das
# regras não vêm junto (são da MISRA); a saída mostra só o número da regra.
# Sai com código 1 se houver alguma violação.
set -e
cd "$(dirname "$0")/.."
ENV=${AEROROUTE_ENV:-$HOME/miniconda3/envs/aeroroute}
ADDON="$ENV/share/Cppcheck/addons/misra.py"
if [ "$#" -eq 0 ]; then
  echo "Uso: sh tools/misra.sh <fonte.c> [...]" >&2
  exit 2
fi
if [ ! -x "$ENV/bin/cppcheck" ] || [ ! -x "$ENV/bin/python" ] || [ ! -f "$ADDON" ]; then
  echo "Ferramentas MISRA ausentes no ambiente: $ENV" >&2
  exit 2
fi
mkdir -p /tmp/opencode
OUT=$(mktemp -d /tmp/opencode/aeroroute-misra.XXXXXX)
trap 'rm -f "$OUT"/*; rmdir "$OUT"' EXIT HUP INT TERM

status=0
for src in "$@"; do
  dump="$OUT/$(echo "$src" | tr '/' '_').dump"
  "$ENV/bin/cppcheck" --dump --quiet --std=c11 -IApp/Inc -Itests/stubs -DCTRL_HOST_BUILD \
    --suppress=missingIncludeSystem "$src"
  if [ ! -s "$src.dump" ]; then
    echo "cppcheck nao produziu dump: $src" >&2
    exit 2
  fi
  mv "$src.dump" "$dump"
  rm -f "$src.ctu-info"
  # Não usar pipeline: ele esconderia o status de falha do Python/addon.
  addon_status=0
  "$ENV/bin/python" "$ADDON" "$dump" >"$OUT/addon.log" 2>&1 || addon_status=$?
  cat "$OUT/addon.log"
  if grep -q 'Traceback\|Error:\|No such file' "$OUT/addon.log"; then
    exit 2
  fi
  if grep -q 'misra-c2012-' "$OUT/addon.log"; then
    status=1
  elif [ "$addon_status" -ne 0 ]; then
    exit 2
  fi
done
exit $status
