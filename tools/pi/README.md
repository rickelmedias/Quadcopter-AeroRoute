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
4. Clone o repositório (a ferramenta lê o dialeto MAVLink e as matrizes do
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
