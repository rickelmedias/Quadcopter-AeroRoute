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
#     ~/drone/dronectl hover-test --sim
#     ~/drone/dronectl missao --sim ~/drone/tools/pi/missoes/quadrado_40cm.txt
#
# O perfil (~/drone/perfil_hover.json) e os ensaios (~/drone/ensaios/) ficam
# fora das pastas copiadas: o --delete não os apaga.
set -e
cd "$(dirname "$0")/../.."
HOST=${1:-r1ddax@r1ddax.local}
DEST=${2:-drone}

# lqg_ajustado.py e lqg_modelo.py projetam os ganhos do LQG ajustado a partir
# do modelo do firmware (lqg_ert_model.c), para o hover-test com pesos Q.
rsync -aR --delete --exclude '__pycache__' --exclude '.venv' \
  protocol/drone_controller.xml \
  reference/STM32_ert_rtw/STM32_data.c \
  App/Src/control/lqg_ert_model.c App/Src/control/lqg_ajustado_gains.c \
  tools/ert_import.py tools/lqg_ajustado.py tools/lqg_modelo.py \
  tools/pi/dronectl tools/pi/missoes tools/pi/tests tools/pi/ensaios/ensaio_dronelink.py tools/pi/ensaios/assinatura_ag.py \
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
