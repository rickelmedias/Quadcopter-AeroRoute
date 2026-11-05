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

## Compensação automática (`CTL_TRIM`)

O LQR do TCC não tem integrador. Com a rotação de hover (`CTL_W0`) um pouco
errada, ou com o peso mal distribuído (frente leve, traseira pesada), o
proporcional não fecha o erro: no ensaio real a frente chegou a 14° de arfagem
com a traseira no chão, e o drone não saía do chão a 40% de aceleração. Cada
10 rad/s de erro no hover valem ~30 cm de erro de altura.

`control/trim.h` soma à rotação de cada motor três termos lentos: **hover**
(igual nos quatro, integra o erro de altura), **arfagem** (frente +, trás −) e
**rolagem** (esquerda +, direita −). Detalhes que importam:

- enquanto o drone não se soltou do chão, o hover sobe numa rampa de 10 rad/s
  por segundo (sem ela levaria dezenas de segundos); depois integra devagar, e
  só com o drone assentado (|vz| < 0,05 m/s);
- congela com motor saturado, em ensaio a seco ou sem altura medida; zera ao
  armar e desarmar; limites de ±150 rad/s (hover) e ±60 rad/s (equilíbrio);
- o Kalman recebe só o `du`: o ajuste é o novo ponto de hover, não uma entrada;
- ao pousar, o log mostra `trim z=.. th=.. phi=..`: o hover real é
  `CTL_W0 + z`. Passe esse valor para `CTL_W0` e a próxima decolagem sai mais
  rápida; `th` e `phi` mostram o desequilíbrio (conserte o balanço do drone).

Em simulação (`tests/test_trim.c`), com hover errado em +40, +80 e +120 rad/s
e desequilíbrio de até 30 rad/s, o ajuste converge ao valor da perturbação,
φ e θ vão a menos de 0,6° e a altura fica em 15 cm ± 1 cm. A caixa
"Compensação automática" do painel vem marcada. Não faz parte do controle
validado no MATLAB: desmarque para voltar ao LQG puro.

**Limites e o ensaio de 04/11.** No primeiro ensaio com a compensação o hover
médio chegou a 48% (850 rad/s) e a traseira a 53%, contra 44% na frente, e o
drone ainda levantava só a frente (θ = +16°): os dois ajustes bateram no limite
(150 e 60 rad/s). Os limites agora são 250 rad/s (hover) e 120 rad/s
(equilíbrio), e o painel parte de `CTL_W0 = 850`. Uma diferença de ~170 rad/s
entre traseira e frente significa que o drone está muito pesado atrás (ou que
os motores de trás empurram menos): **mova a bateria/peso para a frente**, que
resolve melhor do que qualquer ajuste de software. O log mostra a cada 2 s
`trim z=.. th=.. phi=..` e avisa `NO LIMITE` quando um termo satura.

**Ensaio de 04/11, 2ª rodada.** Três problemas, todos corrigidos:

1. *"Com CTL_W0 = 800 ele ligava os motores de uma vez e desarmava."* Ao fim
   da marcha lenta a aceleração saltava de 5% para ~48%; a frente (leve)
   pulava a 23° com 2,7 rad/s e o corte de inclinação (20°) desarmava. Agora a
   aceleração entra em **rampa de 2 s** (`FLIGHT_TAKEOFF_RAMP_MS`) e a
   referência de altura só começa a subir depois dela.
2. *A traseira não segurava ao soltar.* Os motores traseiros ficavam no teto
   `MOT_MAX` = 60% (flag de saturação acesa) e cada motor era cortado sozinho,
   então a diferença traseira/frente que segura o nível sumia. Agora o teto é
   80% e a **dessaturação desloca os quatro juntos** (`control/throttle.h`):
   o nível tem prioridade sobre o empuxo total.
3. *Pouso pedido logo após armar* (clique duplo virou DESLIGAR): o estado ia a
   POUSANDO com o controle ativo e sem marcha lenta, saltando para 47%. Agora,
   se o drone ainda não subiu 3 cm, o pouso vai direto para a descida suave; o
   painel também ignora cliques por 1,5 s depois de LIGAR e grava as mensagens
   de texto (FAILSAFE, trim, pouso) no log.

O desequilíbrio medido continua grande: a frente se sustenta com ~45% e a
traseira não sobe com 60%. Como o empuxo cresce com o quadrado da rotação, a
traseira carrega mais de 1,8× o que a frente carrega. Os dois motores de trás
pedem o mesmo, então não é um motor fraco: é o centro de gravidade atrás.

**Pouso.** Ao encostar (altura abaixo de `FLT_LAND_H` por 0,5 s) a aceleração
desce em rampa linear até a marcha lenta em 1,5 s (`FLIGHT_LAND_SPINDOWN_MS`) e
só então o drone desarma; o ajuste lento congela durante a descida.

Proteções do firmware: inclinação > 20° corta os motores; altura acima de
35 cm pousa; BNO sem dado por 50 ms corta; MTF sem dado por 300 ms pousa;
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
