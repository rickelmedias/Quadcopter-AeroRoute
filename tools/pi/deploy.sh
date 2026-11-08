#!/bin/sh
# Copia para a Raspberry Pi só o que a ferramenta dronectl usa, na mesma
# estrutura de pastas do repositório (o dronectl acha o XML do protocolo e os
# ganhos do Simulink a partir da raiz).
#
# Uso (no PC, da raiz do repositório ou de qualquer pasta):
#     sh tools/pi/deploy.sh [usuario@host] [pasta]
# Padrão: r1ddax@r1ddax.local, pasta drone (= ~/drone na Pi).
#
# Na Pi, depois:
#     ~/drone/dronectl status
#     ~/drone/dronectl missao --sim ~/drone/tools/pi/missoes/quadrado_40cm.txt
set -e
cd "$(dirname "$0")/../.."
HOST=${1:-r1ddax@r1ddax.local}
DEST=${2:-drone}

rsync -aR --delete --exclude '__pycache__' --exclude '.venv' \
  protocol/drone_controller.xml \
  reference/STM32_ert_rtw/STM32_data.c \
  tools/ert_import.py \
  tools/pi/dronectl tools/pi/missoes tools/pi/tests tools/pi/ensaios/ensaio_dronelink.py \
  tools/pi/requirements.txt tools/pi/README.md \
  "$HOST:$DEST/"

# Atalho que roda o dronectl no venv da Pi (~/drone-venv).
ssh "$HOST" "cat > $DEST/dronectl <<'EOF'
#!/bin/sh
# Atalho: ~/drone/dronectl <subcomando> ... (roda tools/pi/dronectl no venv)
cd \"\$(dirname \"\$0\")/tools/pi\" && exec \"\$HOME/drone-venv/bin/python\" -m dronectl \"\$@\"
EOF
chmod +x $DEST/dronectl"
echo "ok: $HOST:$DEST"
