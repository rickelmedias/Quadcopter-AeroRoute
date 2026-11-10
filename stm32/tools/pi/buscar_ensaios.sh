#!/bin/sh
# Traz os ensaios de hover gravados na Pi (~/drone/ensaios) para o PC, em
# tools/pi/logs/hover/ (fora do Git), e desenha o gráfico dos que ainda não
# têm um (precisa de matplotlib no python3 do PC; sem ele, só copia).
#
# Uso (no PC, de qualquer pasta):
#     sh tools/pi/buscar_ensaios.sh [usuario@host]
# Padrão: r1ddax@r1ddax.local.
set -e
cd "$(dirname "$0")/../.."
HOST=${1:-r1ddax@r1ddax.local}
DEST=tools/pi/logs/hover
PY=${PYTHON:-python3}

mkdir -p "$DEST"
rsync -a "$HOST:drone/ensaios/" "$DEST/"
echo "ensaios em $DEST/"

if ! "$PY" -c 'import matplotlib' 2>/dev/null; then
  echo "sem matplotlib: gráficos não gerados (PYTHON=caminho/do/python para outro)"
  exit 0
fi
for folder in "$DEST"/hover_*; do
  [ -f "$folder/grafico.png" ] && continue
  (cd tools/pi && "$PY" -m dronectl hover-report "../../$folder" --plot >/dev/null) &&
    echo "gráfico: $folder/grafico.png"
done
