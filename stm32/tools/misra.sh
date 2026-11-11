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
OUT="${TMPDIR:-/tmp}/dronecontroller-misra"
mkdir -p "$OUT"

status=0
for src in "$@"; do
  dump="$OUT/$(echo "$src" | tr '/' '_').dump"
  "$ENV/bin/cppcheck" --dump --quiet --std=c11 -IApp/Inc -Itests/stubs -DCTRL_HOST_BUILD \
    --suppress=missingIncludeSystem "$src" >/dev/null 2>&1
  mv "$src.dump" "$dump"
  rm -f "$src.ctu-info"
  # O addon sai com 0 mesmo com violações: decide pela saída.
  if "$ENV/bin/python" "$ADDON" "$dump" 2>&1 | grep '^\[' ; then
    status=1
  fi
done
exit $status
