# Teste de controle (branch `controle`)

Testa só o controle do TCC (Kalman + LQR do `main`): estabiliza **rolagem e
arfagem**, mantém a **guinada no zero da energização** e **sobe até 25 cm** em
rampa, lendo a altura no MTF-01. Ao desligar, pousa em rampa.

## O que é diferente do `main`

| Item | `main` | `controle` |
|---|---|---|
| `LQG_FLIGHT_ENABLED` | 0 | 1 (mas o firmware sobe com `FLT_DRY_RUN = 1`) |
| Altura máxima (`FLIGHT_MAX_HEIGHT_M`) | 1,5 m | **0,25 m** (qualquer pedido maior é negado) |
| Rampa de subida e de descida | 0,3 m/s e ≥ 0,2 m/s | **0,05 m/s** (subir 25 cm leva 5 s) |
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

**Ensaio de 04/11, 3ª rodada.** A rampa de partida e o pouso suave
funcionaram, e entre 22 s e 33 s o drone ficou com as duas pontas no ar, quase
nivelado (θ entre −1° e +5°). A traseira voltou ao chão porque a compensação de
arfagem saturou em −120 rad/s (`NO LIMITE` no log) enquanto a de hover
continuava subindo nos quatro motores (de 135 para 183 rad/s): sem margem na
arfagem, o empuxo extra foi para a frente, que é leve, e θ subiu devagar até
16°. Correções:

- o hover não sobe enquanto a arfagem ou a rolagem estiverem no limite;
- o limite de equilíbrio virou o parâmetro `CTL_TRIM_ATT` (padrão 250 rad/s);
- o ganho de equilíbrio `CTL_KI_ATT` passou de 40 para 80 (corrige mais rápido
  quando algo empurra o drone);
- o painel mostra a compensação atual.

Em simulação, com o desequilíbrio medido (~125 rad/s por lado e hover +170
rad/s), o limite antigo nunca nivelava e o novo leva a 15 cm com θ ≈ 0°.

**Ensaio de 04/11, 4ª rodada.** Até 38 s ficou nivelado (θ ±2°, φ ±4°) com as
duas pontas no ar, mas a ~5 cm do chão: as cordinhas não deixavam chegar aos
15 cm. Depois, `CTL_W0` subiu de 772 para 832 em voo e a compensação de hover
somou mais ~40 rad/s; com +100 rad/s nos quatro motores a altura quase não mudou,
ou seja, o drone puxava contra as cordinhas. Aos 48,5 s a traseira caiu (θ +16°),
o equilíbrio acumulou durante o empurrão e, quando ela voltou, passou do ponto
(θ −20,5°): corte por inclinação. Correções:

- no ar, o erro de ângulo integrado pelo equilíbrio fica limitado a ±4,6°
  (`TRIM_ATT_ERROR_MAX_RAD`); em simulação o rebote depois de um empurrão caiu
  de ~57° para ~14°;
- painel: altura escolhível de 3 a 15 cm (use uma que as cordinhas permitam),
  `CTL_W0` travado durante o voo com a compensação ligada e, ao pousar, a
  sugestão `CTL_W0 = CTL_W0 + z` para o próximo voo.

**Ensaio de 04/11, 5ª rodada (melhor até aqui).** De 5 s a 41 s voou nivelado
(θ ±1°, φ 0 a +2°, guinada estável) subindo devagar até +5 cm; o equilíbrio
frente/trás precisou só de ~−23 rad/s (antes −120) e o hover nivelado ficou em
`CTL_W0 + z` ≈ 850 rad/s. **Correção:** nesse ensaio uma pessoa apoiava com a
mão o lado que subia, então os −23 rad/s não medem o desequilíbrio real; o
valor sem apoio é o dos ensaios anteriores (~−120 rad/s, traseira ~60% contra
frente ~45%). O equilíbrio lembrado entre voos também aprende com a mão
apoiando: reinicie a placa antes de um ensaio sem apoio. Depois a altura parou em +6 cm (teto das cordinhas)
com o alvo ainda em 15 cm: o hover seguiu somando contra elas (z 86 → 152), as
cordinhas forçaram θ entre +16° e −12° e um giro rápido de nariz para cima
(2,3 rad/s) passou do corte de 20°. O painel agora abre com 4 cm e
`CTL_W0 = 850`.

**Desligava sozinho logo depois de LIGAR.** A supervisão de enlace do DroneLink
(Raspberry Pi) pedia HOLD e pouso assim que o drone entrava em DECOLANDO, porque
a Pi não estava ligada; isso valia até para voo armado pelo console USB. Agora
ela só age em voo armado pela Pi (`flight_snapshot_t::commanded_by_pi`).

**Os FAILSAFE por inclinação** de 04/11 aconteceram todos com o alvo acima do
teto das cordinhas (a altura parava em +4 a +6 cm): o hover empurra contra elas
e elas puxam torto. Escolha a altura abaixo do teto ou afrouxe as cordinhas.

**Desequilíbrio fixo (`CTL_FF_TH`).** A traseira precisa de ~90 rad/s a mais
que a frente em cada motor. Esse valor entra nos motores desde a rampa de
decolagem, em todos os modos; os integradores do LQI começam do zero a cada voo
e corrigem só o resto (ver docs/controle_lqi.md, "Ajuste depois dos ensaios de
05/11").

**Equilíbrio lembrado entre voos (só no ajuste lento).** No modo "ajuste
lento", o firmware guarda a arfagem e a rolagem do trim do último instante em
que o drone estava em HOLD, com a altura a menos de 1,5 cm do alvo, nivelado
(|φ|, |θ| < 2°) e assentado, e o próximo LIGAR começa delas (`trim inicial
th=.. phi=..` no log). Vale até reiniciar a placa. No LQI isso foi retirado em
05/11: preso nas cordinhas, ele guardou θ = −128 e φ = −10 rad/s (o hover de
verdade estava em cerca de −85 e 0) e o voo seguinte saía inclinado.

**Subida presa.** Se, na decolagem, o alvo fica mais de 4 cm acima da altura
medida e o drone não sobe 2 cm em 3 s (já fora do chão), o voo para de subir:
segura na altura medida, devolve o empuxo que o integrador acumulou desde o
travamento e escreve `altura nao sobe (preso?): HOLD em .. cm`. Nos ensaios de
05/11, com alvos de 15 a 25 cm, a altura parou em 2–5 cm, o integrador de
altura passou de 120 rad/s, os motores de 57% e todos acabaram em FAILSAFE de
inclinação. A primeira versão do detector olhava a velocidade vertical do
Kalman e não disparou: preso, o filtro estimava 4–5 cm/s de subida (o modelo
prevê o que o LQR pede; a altura medida só corrige a posição). Agora ele e o
pouso usam a altura do MTF filtrada (0,3 s) e a velocidade derivada dela.
Reproduzido nos logs de 05/11, ele teria segurado os 14 voos presos de 7 a 23 s
antes do corte.

**Pouso.** O integrador de altura guarda o empuxo extra do hover e desfaz
devagar. Em 05/11, ao pousar, a referência chegava ao chão em menos de 1 s, mas
o drone ficava 3–5 s parado no alto e depois caía de uma vez. Agora, no pouso,
se o drone está mais de 1 cm acima da referência e desce a menos de 3 cm/s, a
correção de hover dos integradores cai 20 rad/s por segundo até ele descer.

**Rigidez da atitude (decisão pendente).** O ganho de arfagem/rolagem do LQR
do TCC é macio: 50,6 rad/s por radiano, ou seja, um empurrão constante de
10 rad/s (≈1% de aceleração entre frente e trás) inclina o drone 11° até o
integrador alcançar. Recalculando o `dlqr` com pesos maiores em φ, θ, p e q
(mesmo modelo A, B do Simulink):

| Pesos de φ/θ e p/q | Kd θ | Kd q | ωn / ζ da atitude | Inclinação por 10 rad/s |
|---|---|---|---|---|
| TCC (500 / 10) | 50,6 | 11,4 | 6,9 rad/s / 0,78 | 11,3° |
| ×4 | 82,1 | 16,1 | 8,8 / 0,86 | 7,0° |
| ×10 | 117,8 | 21,3 | 10,5 / 0,95 | 4,9° |
| ×40 | 205,4 | 33,6 | 13,9 / 1,14 | 2,8° |

Altura (Kd z = 35,1, Kd vz = 50,2) não muda. Ganhos maiores também amplificam
o ruído do giroscópio e o atraso dos ESCs, que o modelo não tem; ×10 é o passo
sugerido.

**Pouso.** Ao encostar (altura abaixo de `FLT_LAND_H` por 0,5 s) a aceleração
desce em rampa linear até a marcha lenta em 1,5 s (`FLIGHT_LAND_SPINDOWN_MS`) e
só então o drone desarma; o ajuste lento congela durante a descida.

Proteções do firmware: inclinação > 20° corta os motores; altura acima de
45 cm pousa; BNO sem dado por 50 ms corta; MTF sem dado por 300 ms pousa;
porta USB fechada corta; **PC calado por 1 s (watchdog) pousa em rampa**.

## Painel

```text
python3 tools/controle_gui.py [/dev/ttyACM0]
```

- **LIGAR / DESLIGAR:** LIGAR arma e sobe em rampa até a altura escolhida (3 a 25 cm); DESLIGAR pousa em
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
3. LIGAR a seco: o estado vai a DECOLANDO e a referência sobe até a altura escolhida. Como os
   motores estão parados, ele não chega em SEGURANDO.
4. **Com motores:** drone preso, barbante, mão na EMERGÊNCIA. Desmarque o
   ensaio a seco e LIGAR. Se não sai do chão, suba `CTL_W0`; se dispara, desça.
   Comece com `MOT_MAX` baixo e suba aos poucos.
5. DESLIGAR pousa em rampa e desarma.
