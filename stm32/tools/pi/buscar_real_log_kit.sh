#!/bin/sh
# Copia sem apagar os kits de voo da Raspberry Pi para real_log_kit/ no PC.
# Uso: sh tools/pi/buscar_real_log_kit.sh [usuario@host]
set -eu
cd "$(dirname "$0")/../.."
HOST=${1:-r1ddax@r1ddax.local}
DEST=real_log_kit
mkdir -p "$DEST"
rsync -a --ignore-existing "$HOST:real_log_kit/" "$DEST/"
echo "kits copiados para $DEST/"
