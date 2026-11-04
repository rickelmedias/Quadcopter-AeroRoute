# Teste de controle (branch `controle`)

Testa só o controle do TCC (Kalman + LQR do `main`): estabiliza **rolagem e
arfagem**, mantém a **guinada no zero da energização** e **sobe 15 cm** em
rampa, lendo a altura no MTF-01. Ao desligar, pousa em rampa.

## O que é diferente do `main`

| Item | `main` | `controle` |
|---|---|---|
| `LQG_FLIGHT_ENABLED` | 0 | 1 (mas o firmware sobe com `FLT_DRY_RUN = 1`) |
| Altura máxima (`FLIGHT_MAX_HEIGHT_M`) | 1,5 m | **0,15 m** (qualquer pedido maior é negado) |
| Rampa de subida e de descida | 0,3 m/s e ≥ 0,2 m/s | **0,05 m/s** (subir 15 cm leva 3 s) |
| Corte por inclinação (`FLT_TILT_MAX`) | 45° | **20°** |
| Fim do pouso (`FLT_LAND_H`) | 8 cm | 3 cm |
| `MOT_MAX` | 100% | **60%** |
| Posição horizontal (`CTL_XY`) | segura x e y (fluxo) | **0: só atitude e altura** |

Com `CTL_XY = 0` (`control/attitude_mode.h`) o LQR perde os termos de x, y, vx,
vy e o fluxo óptico é ignorado. É o mesmo ganho do Simulink sem esses termos,
não o ganho validado no MATLAB. Em simulação (planta exportada, ruído nos
sensores, fluxo absurdo) a atitude volta a zero em ~1,5 s e a altura segue a
rampa com erro de ~1 mm (`tests/test_attitude_mode.c`).

**Consequência:** nada segura a posição horizontal. Qualquer erro de nível
vira aceleração (≈ g·θ: 1° de erro ≈ 0,17 m/s²), então o drone **desliza**.
Prenda-o com barbante e teste numa área livre. O zero de roll e pitch
(`SNS_ZERO_ROLL`/`SNS_ZERO_PITCH`) precisa estar certo.

Proteções do firmware: inclinação > 20° corta os motores; altura acima de
25 cm pousa; BNO sem dado por 50 ms corta; MTF sem dado por 300 ms pousa;
porta USB fechada corta; **PC calado por 1 s (watchdog) pousa em rampa**.

## Painel

```text
python3 tools/controle_gui.py [/dev/ttyACM0]
```

- **LIGAR / DESLIGAR:** LIGAR arma e sobe 15 cm em rampa; DESLIGAR pousa em
  rampa e desarma sozinho.
- **EMERGÊNCIA** (ou barra de espaço / Esc): corta os motores na hora.
- **Ensaio a seco** (marcado): o controle roda e a tela mostra o que cada motor
  receberia, mas os motores **não** giram. Desmarque só com o drone preso; o
  painel pede confirmação.
- A tela mostra o estado do voo, φ, θ, ψ, a altura do MTF e a rampa, e uma
  barra de `du` por motor (azul = o controle aumenta, vermelho = diminui).
- Ajustes ao vivo: `CTL_W0` (rotação de hover), `MOT_MAX` e a rampa.
- Cada ensaio grava as linhas EST/MOT em `tools/logs/controle_*.log`.

## Roteiro

1. **A seco, sem hélices ou motores desligados da bateria.** Conecte, deixe
   nivelado e confira φ ≈ 0°, θ ≈ 0°, ψ ≈ 0°, BNO e MTF "OK".
2. Com o drone na mão, **ainda a seco**, confira o sentido de cada correção:

   | Movimento | O controle deve… |
   |---|---|
   | baixar o lado direito (φ +) | **aumentar** M1 e M2, diminuir M3 e M4 |
   | levantar o nariz (θ +) | **aumentar** M1 e M3 (trás), diminuir M2 e M4 |
   | girar no sentido horário (ψ +) | diminuir M2 e M3, aumentar M1 e M4 |
   | subir o drone (altura +) | diminuir os quatro |

   Sinal trocado: corrija `SNS_SGN_*` (`param set` pelo console). O de ψ também
   depende de as hélices pretas (M2, M3) girarem no sentido anti-horário visto
   de cima.
3. LIGAR a seco: o estado vai a DECOLANDO e a referência sobe 15 cm. Como os
   motores estão parados, ele não chega em SEGURANDO.
4. **Com motores:** drone preso, barbante, mão na EMERGÊNCIA. Desmarque o
   ensaio a seco e LIGAR. Se não sai do chão, suba `CTL_W0`; se dispara, desça.
   Comece com `MOT_MAX` baixo e suba aos poucos.
5. DESLIGAR pousa em rampa e desarma.
