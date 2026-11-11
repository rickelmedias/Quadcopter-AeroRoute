# dronectl — comando e ajuste do drone pela Raspberry Pi

Ferramenta de linha de comando para usar por SSH na Raspberry Pi Zero W
ligada ao STM32 pela UART (DroneLink, MAVLink 2). Com ela dá para:

- ver a filtragem dos sensores ao vivo (medida × estimativa do Kalman e o
  ruído de cada sensor);
- fazer hover e pousar;
- ajustar, **em tempo real**, os pesos do LQR e do Kalman (a Pi recalcula os
  ganhos exatamente como o MATLAB e o STM32 troca todos de uma vez) e os
  parâmetros do firmware (rotação de hover, curva do motor, sinais, rampas);
- gravar a telemetria em CSV e salvar perfis de ajuste.

Detalhes do controle e do protocolo: [`docs/controle_lqg.md`](../../docs/controle_lqg.md).

## Instalação na Pi

1. Habilite a UART e desligue o console serial: `sudo raspi-config` →
   *Interface Options* → *Serial Port* → login shell **não**, hardware **sim**.
2. **Pi Zero W:** por padrão o Bluetooth fica com a UART boa (PL011) e os
   pinos 8/10 recebem a mini-UART, cujo baud muda com o clock da CPU. Para o
   enlace ficar estável, acrescente ao `/boot/firmware/config.txt` (ou
   `/boot/config.txt` nas versões antigas):

   ```text
   enable_uart=1
   dtoverlay=disable-bt
   ```

   e rode `sudo systemctl disable hciuart`. Reinicie. A porta fica em
   `/dev/serial0` (= `/dev/ttyAMA0`).
3. Ligações: TXD (GPIO14, pino 8) → PA10 (USART1_RX); RXD (GPIO15, pino 10)
   → PA9 (USART1_TX); GND comum. Os dois lados são de 3,3 V.
4. **Forma usada no projeto (sem git na Pi):** do PC, `sh tools/pi/deploy.sh
   r1ddax@r1ddax.local` copia só o necessário para `~/drone` (mesma estrutura
   do repositório) e cria o atalho `~/drone/dronectl`, que roda no venv
   `~/drone-venv` (pymavlink, pyserial e numpy já instalados). Depois:
   `~/drone/dronectl status`. Mudanças se fazem no repositório e se repete o
   deploy.

   Alternativa: clone o repositório (a ferramenta lê o dialeto MAVLink e as matrizes do
   modelo direto dele) e instale as dependências num venv. No Pi Zero W
   (ARMv6) o Raspberry Pi OS baixa o numpy já compilado do piwheels; o
   primeiro `pip install` ainda leva alguns minutos:

```bash
git clone git@github.com:rickelmedias/DroneSTM32.git
cd DroneSTM32/tools/pi
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

Na primeira execução o módulo Python do dialeto é gerado a partir de
`protocol/drone_controller.xml` e guardado em `~/.cache/dronectl` (no Zero W
isso é lento, mas acontece só uma vez); ele é refeito sozinho quando o XML
muda. No PC o recálculo dos ganhos leva ~7 ms; no Zero W deve levar bem
mais (ainda não foi medido), o que só atrasa o `apply`, nunca o controle,
que roda no STM32.

## Uso

Sempre a partir de `tools/pi`:

```bash
.venv/bin/python -m dronectl status                    # estado, ganhos, estimativa
.venv/bin/python -m dronectl monitor                   # filtragem ao vivo, sem armar
.venv/bin/python -m dronectl tune                      # shell de ajuste com o drone no chão
.venv/bin/python -m dronectl hover --height 0.5 --time 20
.venv/bin/python -m dronectl hover --dry --debug       # ensaio a seco com shell de ajuste
.venv/bin/python -m dronectl hover --debug --profile perfil.json
.venv/bin/python -m dronectl land                      # pousa
.venv/bin/python -m dronectl disarm                    # desarma na hora
.venv/bin/python -m dronectl param list
.venv/bin/python -m dronectl param set CTL_W0 690
.venv/bin/python -m dronectl gains query | reset | upload --profile perfil.json
.venv/bin/python -m dronectl design                    # confere o projeto contra o Simulink (offline)
```

Opções gerais: `-p/--port` (padrão `/dev/serial0`) e `-b/--baud` (115200).
`monitor`, `tune` e `hover` aceitam `--rate` (Hz do DRONE_CTRL_DEBUG, padrão
25), `--window` (amostras da estatística), `--csv arquivo.csv` e `--profile`.

`hover` arma, decola até `--height`, segura por `--time` segundos e pousa.
**Ctrl+C pousa.** Se a Pi cair, o STM32 percebe a perda da autorização, segura
a posição e pousa sozinho depois de 5 s.

## Ensaio de hover (`hover-test`)

Um voo de ajuste por comando, para rodar por SSH (pelo atalho `~/drone/dronectl`)
e repetir mudando um valor por vez. Cada ensaio grava tudo e termina com a
análise e a sugestão para o próximo voo.

```bash
~/drone/dronectl hover-test --sim                    # drone simulado no STM32 (confere a cadeia toda)
~/drone/dronectl hover-test                          # a seco: sensores reais, motores parados
~/drone/dronectl hover-test --motors                 # VOO: 30 cm por 20 s (valores do perfil)
~/drone/dronectl hover-test --motors --set CTL_W0=1080 --label "w0 1080"
~/drone/dronectl hover-test --motors --q x=80,y=80   # pesos Q do LQG ajustado só neste voo
~/drone/dronectl hover-test --motors --height 20 --time 30
~/drone/dronectl hover-report                        # reanalisa o último ensaio (--timeline)
~/drone/dronectl hover-profile                       # perfil, última sugestão e histórico
~/drone/dronectl hover-profile --apply-suggestion SNS_LVL_PHI SNS_LVL_TH
~/drone/dronectl hover-profile --set CTL_W0=1080 | --unset CTL_W0 | --q x=80 | --clear-q
```

**Valores em uso.** Todo ensaio parte dos **valores de fábrica do firmware +
o perfil (`~/drone/perfil_hover.json`) + os `--set`/`--q` daquele ensaio**.
Nada vaza de um ensaio para o outro: na primeira vez em cada boot do STM32 o
`hover-test` lê o valor de fábrica de cada parâmetro que toca e, nos ensaios
seguintes, volta a ele antes de aplicar o perfil. `--set` vale só para aquele
voo; para manter, use `--save` ou `hover-profile --set`. Com pesos Q, a Pi
projeta Kd e Kfd com o mesmo dlqr da tabela do firmware
(`tools/lqg_ajustado.py`) e envia com CRC; sem eles, volta aos ganhos de
fábrica.

**O voo.** Liga o `DRONE_CTRL_DEBUG` a 50 Hz, confere atitude, giro e altura
no chão, mede o chão (1 s), conta 3 s (só com `--motors`), arma, decola, segura
`--time` segundos **depois de chegar** à altura e pousa. Termina, no fim, com
FLT_DRY_RUN = 1, FLT_SIM = 0 e a telemetria rápida desligada.

| Proteção da Pi (além das do STM32) | Ação |
|---|---|
| Altura mais de 25 cm acima do alvo por 250 ms | pousa |
| Mais de 80 cm do ponto de decolagem por 250 ms (`--geofence`) | pousa |
| Telemetria parada por 1 s | pousa |
| Não chegou à altura no tempo da rampa + 23 s | pousa |
| Ctrl+C durante a decolagem, com altura medida < 10 cm e estimada < 12 cm | corta os motores |
| Ctrl+C acima disso, SIGTERM ou a sessão SSH caindo (SIGHUP) | pousa |
| **Ctrl+C de novo** durante o pouso | **corta os motores** (desarma) |
| Erro de comunicação | pousa (no ar) ou desarma (no chão) |

Se a Pi morrer de vez, vale a supervisão do STM32: sem autorização ele segura
e pousa sozinho em 5 s.

**O que fica gravado** em `~/real_log_kit/hover_<data>/`: `real_log.csv`
(separado por `;`, uma linha por amostra, com M1..M4 físicos, medidas,
estimativas, referências, saídas LQG, flags, todos os parâmetros, CRC/Q/R dos
ganhos repetidos), `gains.json` (Kd, Kfd, L e M completos), `telemetria.csv`
(formato compacto usado pelo relatório),
(cada passo do LQG: y, x̂, du, referência, motores, estado), `textos.txt`
(mensagens do STM32, como os relatos `dist`), `meta.json` (parâmetros em uso,
ganhos, chão, motivo do fim), `resumo.json` (métricas e sugestão) e
`linha_do_tempo.txt` (uma linha por segundo).

**A análise** (`dronectl/hover_report.py`) mede, na janela de hover (2 s
depois de chegar até o pouso): erro de altura, deslocamento XY médio, RMS,
máximo e deriva, φ e θ médios, quanto a guinada girou, motores M1..M4 e
saturação, oscilação dominante e o relato `dist` de antes do pouso. E sugere:

- `CTL_W0` = atual + d̂ de altura (o hover que o estimador achou);
- `CTL_FF_TH/PHI/PSI` = atual + d̂ do canal (o desequilíbrio inteiro fica fixo
  e o estimador começa do lugar certo);
- `SNS_LVL_PHI/TH` = atual − φ/θ médios **com o drone parado** (parado no ar,
  sem acelerar, ele está nivelado de verdade: o que o sensor mostra é o erro
  do zero de nível, que vira deslocamento — 0,5° ≈ 14 cm). No máximo 1,5° por
  voo e só com o XY ligado e pelo menos 5 s de janela.

A sugestão fica no perfil, mas **só entra quando alguém aplica**
(`hover-profile --apply-suggestion [NOMES]`).

**No PC:** `sh tools/pi/buscar_real_log_kit.sh` traz os kits para
`real_log_kit/` na raiz do repositório, sem apagar os já copiados. O comando
antigo `sh tools/pi/buscar_ensaios.sh` traz os ensaios legados para
`tools/pi/logs/hover/` (fora do Git) e, com matplotlib, desenha `grafico.png`
de cada um (altura, XY, inclinação, guinada e motores com as fases do voo);
`python3 -m dronectl hover-report <pasta> --plot` faz o mesmo para uma pasta.

## Missão passo a passo (`missao`)

```bash
.venv/bin/python -m dronectl missao --sim              # shell: takeoff 50, yaw 90, forward 100, goto 0 0, land
.venv/bin/python -m dronectl missao --sim missao.txt   # um comando por linha
.venv/bin/python -m dronectl missao --sim missoes/quadrado_40cm.txt   # quadrado de 40 cm
```

Medidas em centímetros, graus e segundos (inteiros). Cada comando espera o STM32 responder que o drone chegou e ficou estável (1 s
parado em HOLD) antes do próximo; `goto`/`waypoints`/`route` calculam o giro e a distância
até o ponto; `route` escolhe a ordem com o algoritmo genético e, sem pontos, lê
`missoes/pontos.txt`. `--sim` liga o drone simulado no STM32 (`FLT_SIM = 1`, motores
parados) para testar a comunicação na bancada. Protocolo, comandos e o
diagrama das mensagens: [`docs/teste_comunicacao.md`](../../docs/teste_comunicacao.md).

## Shell de ajuste (`hover --debug` e `tune`)

```text
filtragem:  stats | watch [s] | rn auto
pesos:      q <estado|all> <v> | r <motor|all> <v> | qn <estado|all> <v> | rn <medida|all> <v>
            pesos | apply (recalcula e envia) | reset (ganhos do Simulink)
parâmetros: set NOME VALOR | get NOME | params
voo:        ref z <m> | goto <x> <y> <z> | move <frente> <direita> [baixo] | yaw <graus>
            hold | land | disarm
perfil:     save <arquivo.json> | load <arquivo.json>
            sair (em voo, pousa antes)
```

Estados: `x y z phi theta psi vx vy vz p q r`. Medidas: `z phi theta psi vx vy p q r`.

- `q`/`r` são os pesos do LQR; `qn`/`rn` os do Kalman. Os padrões são os do
  TCC e reproduzem os ganhos do Simulink.
- Nada muda no drone até `apply`: só então os ganhos são recalculados,
  conferidos (polos dentro do círculo unitário) e enviados. O STM32 confirma
  com o CRC.
- `stats` e `watch` mostram, para cada medida: o valor medido e o estimado, o
  desvio padrão de cada um e de y − x̂. Com o drone parado, a variância de
  y − x̂ ("Rn sugerido") é o ruído real daquele sensor; `rn auto` usa esses
  valores no Kalman.

Exemplo de sessão:

```text
[HOLD] > watch 5
[HOLD] > q z 800
q[z]: 500 -> 800  (use "apply" para enviar)
[HOLD] > apply
maior polo: controle 0.992110, estimador 0.999990
ganhos novos em uso (crc 50A4A641)
[HOLD] > set CTL_W0 690
CTL_W0 = 690
[HOLD] > ref z 0.7
[HOLD] > save perfil.json
[HOLD] > sair
```

## Perfil de ajuste

JSON com os pesos e os parâmetros alterados; é aplicado antes de armar com
`--profile`:

```json
{
  "weights": {
    "q": [5, 5, 800, 500, 500, 50, 5, 5, 5, 10, 10, 5],
    "r": [0.1, 0.1, 0.1, 0.1],
    "qn": [1e-05, 1e-05, 1e-05, 1e-05, 1e-05, 1e-05, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001],
    "rn": [0.000115, 5e-08, 1.94e-07, 6.04e-08, 0.0128, 0.0022, 2.06e-06, 3.24e-07, 2.38e-07]
  },
  "params": {"CTL_W0": 690}
}
```

## Testes

```bash
python3 -m unittest discover -s tests
```

Conferem que o projeto em numpy reproduz os ganhos do Simulink e exercitam o
enlace inteiro (parâmetros, envio de ganhos com linha perdida, armar, decolar,
pousar e telemetria) contra um STM32 simulado num pseudo-terminal
(`tests/fake_fc.py`). Rodam também em `sh tests/run.sh`, na raiz.

`tests/test_hover.py` cobre o ensaio de hover: a análise sobre um voo
sintético (fases, métricas, sugestões e seus limites), o perfil, os ganhos do
ajustado (com o Q de fábrica a Pi reproduz a tabela do firmware) e ensaios
completos contra o STM32 falso com um modelo de voo 10× mais rápido (gravação,
sugestão que leva o drone ao ponto, `--set` que não vaza para o próximo voo,
pesos Q, cerca, telemetria perdida, Ctrl+C duplo e ensaio a seco).
