# QUALIFY: controle LQG qualificado (05/11/2026)

Este documento registra a versão do controle que a equipe aprovou nos ensaios
de 05/11/2026 (commit `e0606d1`, tag `lqg-melhorado-aprovado`, branch
`controle-lqi`), o que mudou do projeto do MATLAB/Simulink do TCC até ela, por
que cada mudança foi feita e como os testes foram conduzidos.

Resumo: o drone voa **estável em atitude e altura**, sem tombar, inclusive
fora da jaula e depois de puxões com a mão. Ainda **não segura a posição
horizontal**: o controle de x e y está desligado (`CTL_XY = 0`), então ele
deriva devagar para um lado e precisa ser trazido de volta.

---

## 1. O que foi qualificado

| Item | Valor |
|---|---|
| Modo | **LQG melhorado**: LQG ajustado + Kalman com estimação de perturbação |
| Parâmetros | `CTL_LQG_AJ = 1`, `CTL_DIST = 1`, `CTL_LQI = 0`, `CTL_TRIM = 0`, `CTL_XY = 0` |
| Ponto de operação | `CTL_W0 = 820` rad/s (bateria cheia), `CTL_FF_TH = −90` rad/s, `CTL_FF_PHI = 0` |
| Limites | `MOT_MAX = 80%`, corte por inclinação 20°, altura máxima 25 cm |
| Malha | 100 Hz (Ts = 10 ms), DShot600 a 1 kHz, STM32F411 + FreeRTOS |
| Sensores | BNO08x (atitude e giro, montado invertido), MTF-01 (altura; fluxo não usado) |

Resultado do ensaio de qualificação (9 voos, logs `tools/logs/controle_20261105_1643*`
a `_1647*`, só no PC):

| Medida | Resultado |
|---|---|
| θ (arfagem) | RMS de 1,4 a 4,0° por voo (picos −16° a +19°, com a mão) |
| φ (rolagem) | RMS de 1,2 a 2,6° por voo |
| Duração | até 66 s no ar (limitado pela bateria) |
| FAILSAFE | 1 em 9 voos (antes do ajuste: 7 em 13) |
| Perturbação estimada de arfagem | −28 a +25 rad/s em torno de 0, o que confirma `CTL_FF_TH = −90` |
| Puxão com a mão | volta à altura de referência ao soltar |
| Voo livre | solto da jaula, estável, mas derivando na horizontal |

---

## 2. Do MATLAB para o firmware: o que ficou igual

O projeto do TCC continua sendo a base, sem mudar o modelo:

- **Modelo**: A, B e C discretos (Ts = 10 ms), 12 estados
  `x y z φ θ ψ vx vy vz p q r`, 4 entradas (rotação de cada motor), 9 medidas
  `z φ θ ψ vx vy p q r`. Vêm da exportação do Simulink
  (`reference/STM32_ert_rtw`) por `tools/ert_import.py`; o `tests/run.sh`
  confere que a tabela do firmware é a do Simulink.
- **Kalman** (L, M): o de regime do TCC, sem mudança, com
  `Qn = diag(1e-5 ×6, 1e-3 ×6)` e o Rn das variâncias medidas dos sensores.
- **LQR**: `Kd = dlqr(A, B, Q, R)` com `R = 0,1 I`, e o pré-compensador
  `Kfd = inv(Cr (I − A + B Kd)^-1 B)`, como no TCC. O `dlqr` em C e em Python
  reproduz o Kd do Simulink com erro relativo de 1e-12.
- **Estrutura do passo**: a mesma do `STM32_step()` gerado (correção,
  controle `du = −Kd x̂ + Kfd r`, predição com o du aplicado). Em double, o
  código do firmware é idêntico bit a bit ao gerado.

## 2A. MATLAB (modelo não linear) × drone real

No MATLAB, com o modelo não linear do TCC, o LQG voou bem e se recuperou de
pequenos empurrões. No drone real, o mesmo LQG não decolava nivelado. O
controlador era o mesmo; o que mudou foi o que o modelo supõe e o drone não
cumpre. Cada item abaixo diz o que o modelo assume, o que os ensaios
mostraram e o que foi feito.

> **1. Centro de gravidade**
>
> **No MATLAB:** CG no centro geométrico; os quatro motores sustentam o mesmo peso.
>
> **No drone real:** traseira mais pesada. Antes do chumbo, a traseira pedia ~60%
> de aceleração e a frente ~45% (1,8×); depois, 1,25×, ~90 rad/s a mais por
> motor atrás (7.2).
>
> **O que fizemos:** chumbo para aproximar o CG do centro; `CTL_FF_TH = −90`
> (ponto de operação real); estimador de perturbação de θ e φ para o resto (3.3).

> **2. Motores e curva de rotação**
>
> **No MATLAB:** quatro motores idênticos; o comando é a rotação ω exata.
>
> **No drone real:** o comando é um valor DShot, convertido por uma curva
> provisória (0–100% = 0–1755 rad/s, nunca medida); motores diferentes giram
> diferente no mesmo DShot; o M4 tem um "teco" parado.
>
> **O que fizemos:** nada no modelo; as diferenças aparecem como perturbação
> constante e o estimador as absorve. Medir rpm × DShot por motor continua pendente.

> **3. Rotação de hover ω0**
>
> **No MATLAB:** ω0 = 702 rad/s, exato e fixo.
>
> **No drone real:** 790–840 rad/s perto do chão, e sobe à medida que a bateria
> descarrega (46% → 53% de aceleração para pairar, no mesmo ensaio).
>
> **O que fizemos:** `CTL_W0 = 820`; Q z de 500 para 5000 (o erro de altura cai
> 3×); estimador de perturbação de altura, que acha o ω0 que falta (o log mostra
> quanto em `dist z=`).

> **4. Chão, decolagem e efeito solo**
>
> **No MATLAB:** o drone começa no ar ou sai do chão sem atrito; não há efeito solo.
>
> **No drone real:** ele começa apoiado; um lado sai antes do outro; perto do chão
> (2–4 cm) o efeito solo dá empuxo extra, que some ao subir.
>
> **O que fizemos:** 1 s de marcha lenta + 2 s de rampa até o controle;
> `CTL_FF_TH` desde a rampa (sem ele, 7 de 13 decolagens viraram); estimador
> congelado até o drone subir 1 cm (no chão ele lia o apoio como empuxo
> sobrando); rampa fixa de hover enquanto não solta.

> **5. Dinâmica dos motores**
>
> **No MATLAB:** o empuxo segue o comando instantaneamente (ou com atraso pequeno).
>
> **No drone real:** ESC + motor + hélice levam algumas dezenas de ms; o atraso
> real não foi medido.
>
> **O que fizemos:** todos os ganhos foram escolhidos em simulação com 40, 80 e
> 120 ms de atraso; os pesos de atitude param em 8000 porque 16000 já diverge a
> 120 ms (7.4).

> **6. Sensores**
>
> **No MATLAB:** z, φ, θ, ψ, p, q, r e vx, vy medidos com ruído branco pequeno.
>
> **No drone real:** BNO montado de ponta-cabeça (zero em −174,5° de rolagem);
> o MTF lê a distância inclinada, parado varia ~2 cm, e o fluxo só mede em degraus
> de 0,2 rad/s; o zero da altura vem de uma leitura só.
>
> **O que fizemos:** zero de nível e troca de eixos do BNO; altura = distância ×
> cos φ × cos θ; fluxo (vx, vy) fora da malha por enquanto (`CTL_XY = 0`).

> **7. O drone sempre segue o modelo**
>
> **No MATLAB:** a planta é o próprio modelo, então a velocidade estimada pelo
> Kalman é boa.
>
> **No drone real:** preso pelas cordinhas, o Kalman estimava 4–5 cm/s de subida
> com a altura parada: o modelo previa o que o LQR pedia e só a posição era
> corrigida (7.7).
>
> **O que fizemos:** as proteções (subida presa, pouso) usam a altura medida
> filtrada, não a velocidade do Kalman.

> **8. Empurrões**
>
> **No MATLAB:** o LQG do TCC se recupera de pequenos empurrões.
>
> **No drone real:** com os ganhos do TCC (50,6 rad/s por rad), desvios lentos de
> até 18° no ar, e um desequilíbrio de ~1% inclinava 11°. Com os pesos ajustados,
> o drone volta à altura de referência quando puxado e solto, e θ ficou com
> RMS de 1,4–4,0°.
>
> **O que fizemos:** Q φ, θ de 500 para 8000 e p, q de 10 para 100 (7.4).

> **9. Restrições externas**
>
> **No MATLAB:** drone livre.
>
> **No drone real:** cordinhas (teto de ~4 cm), jaula, mão. Presos, os
> integradores somavam força contra as cordinhas até o corte por inclinação.
>
> **O que fizemos:** detector de subida presa (útil com cordinhas; atrapalha
> fora delas, ver 7.7).

> **10. Posição horizontal**
>
> **No MATLAB:** o LQG controla os 12 estados, inclusive x e y, com vx e vy medidos.
>
> **No drone real:** x e y ficaram fora da malha (`CTL_XY = 0`) até validar o
> fluxo do MTF no drone. Por isso ele deriva: é o próximo passo (branch
> `controle-lqg-posicao`).

> **11. Bateria**
>
> **No MATLAB:** tensão constante.
>
> **No drone real:** o mesmo comando dá menos empuxo à medida que a bateria
> descarrega; não há medida de tensão.
>
> **O que fizemos:** o estimador de altura compensa durante o voo; `CTL_W0`
> pensado para bateria cheia.

### Recomendação para o texto do TCC

Para uma comparação justa entre o MATLAB e o drone, vale reproduzir no modelo
não linear as condições medidas aqui e mostrar o LQG do TCC e o LQG melhorado
lado a lado:

| Condição | Valor para simular | Onde foi medido |
|---|---|---|
| CG fora do centro | ~90 rad/s a mais nos motores de trás (ou o deslocamento do CG equivalente) | 7.2 |
| ω0 errado | ±30 rad/s em torno do hover | 7.3 |
| Atraso dos motores | primeira ordem com 40, 80 e 120 ms | 7.4 |
| Ruído do MTF | σ ≈ 1 cm na altura | 3.4 / 6 |
| Empurrão | 30 rad/s em frente/trás por 1,5 s | 7.4 |
| Chão | drone começa apoiado; empuxo abaixo do hover na rampa | 7.6 |

O resultado esperado é o da simulação linear da seção 3.3: o LQG do TCC fica
inclinado (ou não decola) e o melhorado zera os erros. Esta seção foi escrita
sem os números do modelo não linear do MATLAB; quando vocês os tiverem com
essas condições, eles completam a comparação.

## 3. Do MATLAB para o firmware: o que mudou e por quê

### 3.1 Ponto de operação (hover)

| | TCC | Qualificado | Por quê |
|---|---|---|---|
| ω0 (hover) | 702 rad/s (deduzido do B) | `CTL_W0 = 820` | Nos hovers os motores ficaram em 45–48% de aceleração (790–840 rad/s perto do chão). A curva rotação→DShot é provisória (100% = 1755 rad/s, nunca medida no dinamômetro). |
| Equilíbrio frente/trás | motores iguais (CG no centro) | `CTL_FF_TH = −90` rad/s por motor | O drone real tem a traseira mais pesada: em todo hover o controle pedia de 82 a 105 rad/s a mais atrás. Sem esse valor desde a decolagem, a frente levantava sozinha no fim da rampa (θ de +2° a +18° em 0,2 s, FAILSAFE em 7 de 13 voos). É linearizar em torno do equilíbrio real em vez do ideal. |

### 3.2 Pesos Q (LQG ajustado)

Mesmo `dlqr`, mesmo R, só outro Q (`tools/lqg_ajustado.py`):

| Estado | Q do TCC | Q ajustado | Ganho do TCC | Ganho ajustado |
|---|---|---|---|---|
| x, y | 5 | 5 | 3,34 | 3,15 |
| **z** | 500 | **5000** | 35,1 | **110,4** |
| **φ, θ** | 500 | **8000** | 50,6 | **142,4** |
| ψ | 50 | 50 | 4,27 | 4,27 |
| vx, vy | 5 | 5 | 6,75 | 10,1 |
| **vz** | 5 | **50** | 50,3 | **89,6** |
| **p, q** | 10 | **100** | 11,4 | **22,4** |
| r | 5 | 5 | 1,40 | 1,40 |

Ganhos em rad/s por unidade do estado (m, rad, m/s, rad/s), motor u1.

Motivos:

- **Atitude (φ, θ, p, q)**: o ganho do TCC é macio (50,6 rad/s por radiano).
  Um desequilíbrio de só 10 rad/s, ~1% de aceleração, inclinava o drone 11°,
  e um empurrão levava ao corte. A escolha foi feita por simulação com o motor
  atrasando 40, 80 e 120 ms. Rebote de um empurrão:

  | Q φ,θ / p,q | 40 ms | 80 ms | 120 ms |
  |---|---|---|---|
  | 500 / 10 (TCC) | 16,0° | 22,3° | 28,5° |
  | 2000 / 30 | 11,3° | 16,5° | 26,2° |
  | 4000 / 60 | 8,7° | 13,2° | 18,4° |
  | **8000 / 100** | **6,7°** | **11,1°** | **10,7°** |
  | 16000 / 100 | 5,9° | 9,9° | **instável** |

  8000/100 deixa margem para um motor mais lento que o esperado.
- **Altura (z, vz)**: sem ação integral, o erro de altura é força / ganho. Com
  o ganho do TCC (35), 30 rad/s de hover errado davam até 1 m de erro em
  simulação; com 110, menos de 20 cm.

### 3.3 Estimador de perturbação (LQG melhorado)

O LQR é proporcional ao estado: com uma força constante que o modelo não tem
(hover errado, CG fora do centro, motores diferentes, bateria caindo), o drone
para onde o erro gera exatamente a correção que a cancela. Para zerar esse
erro, o controlador precisa de um modelo da perturbação (princípio do modelo
interno). No LQI esse modelo é um integrador; aqui ele fica **no estimador**,
mantendo o LQG (Kalman + LQR, princípio da separação).

Para cada canal (altura, φ, θ), um Kalman de regime com posição, velocidade e
uma perturbação constante d nos motores, no padrão do canal (os quatro iguais;
frente contra trás; esquerda contra direita), com o mesmo A e B do Simulink:

```
[pos; vel; d][k+1] = [A_pp  A_pv  -4 b_p ;  A_vp  A_vv  -4 b_v ;  0  0  1] [pos; vel; d][k] + [b_p; b_v; 0] U[k]
U = Σ_j padrão_j · u_j        medidas: z (MTF) | φ, p (BNO) | θ, q (BNO)
controle: u_j = du_j (LQR) + d̂ · padrão_j
```

- Qn e Rn são os do Kalman do TCC; só a variância de d é nova:
  **0,1 (rad/s)² por passo na altura** (acha um hover 30 rad/s errado em
  segundos) e **1e-3 na atitude** (com 1e-2 a malha oscila a 120 ms de atraso).
- A correção d̂ fica fora da predição do Kalman do TCC: ela compensa uma força
  que o modelo não tem, não é uma entrada.
- **No chão o estimador não aprende.** O apoio segura o drone e a correção
  leria isso como empuxo sobrando. Na primeira versão, d̂ de altura foi a −160
  rad/s na rampa de partida e o drone não decolava. Agora ele fica congelado
  até o drone subir 1 cm; com o alvo acima, o hover sobe 10 rad/s por segundo
  até lá.
- O log mostra `dist z=.. th=.. phi=..`: `z` é o quanto o `CTL_W0` está
  errado; `th` e `phi`, o quanto o `CTL_FF_TH`/`CTL_FF_PHI` estão errados.

Simulação (`tests/test_lqg_dist.c`: modelo do TCC, Kalman do TCC, ruído,
motor de primeira ordem, chão, subida a 20 cm, empurrão):

| Modo | Hover errado | Desequilíbrio | Atraso | Erro de altura | θ, φ finais | Rebote |
|---|---|---|---|---|---|---|
| Original (TCC) | +30 | 15 / 10 | 40 ms | não decola | −17°, −11° | 51° |
| Ajustado | +30 | 15 / 10 | 40 ms | não decola | −6°, −4° | 18° |
| **Melhorado** | +30 | 15 / 10 | 40 ms | **0** | **0, 0** | **8,3°** |
| Melhorado | −30 | −15 / −10 | 80 ms | 0 (pico 35,6 cm) | 0, 0 | 12,7° |
| Melhorado | +30 | 105 / 10 (sem FF) | 40 ms | 0 | 0, 0 | 8,3° |
| Melhorado | +30 | 15 / 10 | 120 ms | 0 | 0, 0 | 11,4° |
| LQI (comparação) | +30 | 15 / 10 | 40 ms | 0 | 0, 0 | 6,7° |

### 3.4 O que o firmware acrescenta em volta do LQG

Não muda o projeto de controle; faz a passagem do modelo para o drone real.

| Item | O que faz | Origem |
|---|---|---|
| Modo só atitude (`CTL_XY = 0`) | Zera as colunas x, y, vx, vy de Kd e Kfd; vx e vy do fluxo são trocados pela predição | O fluxo ainda não foi validado no drone |
| BNO invertido | Zero de nível em −174,5° de rolagem e −2,9° de arfagem (`ZERO_ROLL`/`ZERO_PITCH`, `flight_config.h`) e eixos trocados | Montagem física |
| Altura | distância do MTF × cos φ × cos θ, relativa ao chão no instante de armar | Telêmetro inclinado lê a mais |
| Decolagem | 1 s em marcha lenta + 2 s de rampa até o controle | Sem rampa, o salto ao hover fazia o lado leve pular |
| Saturação | Motor no teto desloca os quatro (mantém a diferença que nivela) | — |
| Pouso | Referência desce em rampa; se o drone não desce, o hover estimado cai 20 rad/s por s; ao encostar, os motores descem em 1,5 s | Antes ele ficava 3–5 s no alto e caía de uma vez |
| Subida presa | Se não sobe 2 cm em 3 s com o alvo 4 cm acima, segura onde está | Cordinhas da bancada (ver 5.3; hoje atrapalha, ver seção 7) |
| Altura filtrada | Detector e pouso usam a altura do MTF filtrada (0,3 s), não a velocidade do Kalman | Preso, o Kalman estimava 4–5 cm/s de subida falsa |
| Proteções | Corte por inclinação > 20°, BNO sem dado, MTF sem dado → pouso, watchdog do PC → pouso, porta USB fechada | — |

---

## 4. Metodologia de teste

### 4.1 Ferramentas

- **Painel** (`tools/controle_gui.py`): conecta pela USB, escolhe o modo
  (LQG original / ajustado / melhorado / LQI), altura, `CTL_W0`, `CTL_FF_TH`,
  `MOT_MAX`, rampa; mostra atitude, altura e o que cada motor recebe; botão de
  emergência (barra de espaço). Grava um log por sessão em `tools/logs/`
  (linhas `EST` a 20 Hz com medidas, estados, du e referência; `MOT` com a
  aceleração e o DShot de cada motor; `TXT` com eventos).
- **Ensaio a seco** (`FLT_DRY_RUN = 1`): o controle roda e o painel mostra o
  que cada motor receberia, sem girar. Serve para conferir sinais com o drone
  na mão.
- **Simulação no PC** (`tests/run.sh`): o código do firmware em malha fechada
  com o modelo do TCC, ruído, atraso de motor, chão e perturbações. Todo ajuste
  de ganho passou por ela antes do drone.
- **Análise de log**: scripts que resumem cada voo (altura, θ/φ RMS e picos,
  motores, saturação, estimador, motivo do fim) e mostram séries no tempo.

### 4.2 Sequência de bancada

1. **Sensores**: BNO e MTF no serial; zero do BNO invertido.
2. **Motores**: teste de bancada pela IHM (sem hélice e com hélice), DShot600.
3. **Atitude a seco**: drone na mão, conferir se o du corrige no sentido certo.
4. **Voo preso por cordinhas** (04/11 e manhã de 05/11): altura baixa,
   cordinhas laterais como proteção.
5. **Voo na jaula** (tarde de 05/11) e **solto da jaula**.
6. Cada mudança: simulação → ensaio a seco → voo baixo (3–10 cm) → mais alto.

### 4.3 Como cada voo foi avaliado

- Atitude: θ e φ (RMS e picos); FAILSAFE e o instante em que ocorreu.
- Altura: alcançou o alvo? onde parou? (altura medida contra a referência).
- Motores: média, máximo, diferença frente/trás, saturação. É isso que separa
  "falta força" de "algo segura o drone".
- Compensação: o valor final do integrador ou estimador (`lqi`/`dist`) diz
  quanto o ponto de operação estava errado.

---

## 5. Histórico dos ensaios até o ponto qualificado

### 5.1 04/11: primeiro voo com o LQG do TCC (preso por cordinhas)

- **Traseira 1,8× mais pesada**: sem ajuda da mão, traseira a ~60% e frente a
  ~45% de aceleração. Rolagem sempre boa; arfagem sempre o problema.
- **Hover real ≈ 850 rad/s** contra 702 do modelo.
- Todos os FAILSAFE por inclinação ocorreram com o alvo acima do teto das
  cordinhas (~4–6 cm): o drone empurrava contra elas até virar.
- Bug do DroneLink pedindo pouso sem a Pi ligada: corrigido.
- Daí saíram o "ajuste lento" (trim) e depois o LQI, para compensar o
  desequilíbrio.

### 5.2 05/11 manhã: chumbo no CG, comparação LQI × ajuste lento × TCC puro

- Com chumbo, a razão traseira/frente caiu de **1,8× para 1,25×**.
- **LQG do TCC puro**: não decolou; ficou 38 s com θ = +13° e um lado no chão.
  O ganho proporcional sozinho não vence o desequilíbrio.
- **Ajuste lento**: θ de 11–19° enquanto aprendia, 2 FAILSAFE.
- **LQI**: o melhor; a 3–4 cm, θ dentro de ±5°.
- Problemas encontrados e corrigidos:
  - pouso não descia (integrador de altura carregado): desconto do hover no pouso;
  - "equilíbrio lembrado" guardado com o drone preso (θ −128): só em HOLD, depois removido;
  - detector de subida presa não disparava: o Kalman estimava subida falsa com o
    drone preso, então trocado pela altura do MTF filtrada.

### 5.3 05/11 tarde (16:10): 13 voos, diagnóstico de "não sobe"

- **Motores ≤ 61%** com limite de 80%, e o integrador de altura negativo em
  HOLD. **Não faltava força: as cordinhas seguravam.**
- **7 de 13 voos em FAILSAFE no fim da rampa de partida** (θ +2° → +18° em
  0,2 s): resultou no `CTL_FF_TH = −90`.
- θ oscilando ±15–18° no ar: resultou nos pesos de atitude 8000/100.

### 5.4 05/11 tarde (16:34): os modos LQG

- **LQG ajustado**: alvo de 4 cm atingido (3,5 cm), θ entre −8° e +7°. Puxado
  para 6–7 cm e solto, voltou para 4,5–5 cm. Com alvo de 17 cm não subiu: é
  só proporcional e pedia ~1% a mais; fora do efeito solo falta empuxo se o
  `CTL_W0` não acompanhar.
- **LQG melhorado (primeira versão)**: não decolou; estimador aprendendo o
  apoio do chão como empuxo sobrando (d̂ = −160). Corrigido congelando-o até
  o drone se soltar.

### 5.5 05/11 tarde (16:43–16:51): qualificação do LQG melhorado

Ver a tabela da seção 1. Variações de `CTL_W0` (820 → 856 → 912 → 947) e
`MOT_MAX` (80 → 90 → 100%) nos últimos voos compensavam a **bateria acabando**:
para pairar a ~2 cm foram precisos 46% de motor no início e 52–53% no fim. Os
motores nunca passaram de 64% nem saturaram.

---

## 6. Limitações conhecidas

- **Sem controle de posição horizontal** (`CTL_XY = 0`): deriva para um lado.
  Os logs também não registram o fluxo do MTF, então a deriva ainda não foi
  medida.
- **Altura baixa nos ensaios** (mediana de 2–3 cm): o detector de subida presa
  cortava a subida aos 7–10 s; o efeito solo some acima de ~4 cm e o
  estimador precisa de tempo para achar o empuxo que falta; a bateria
  descarregava.
- **Curva rotação → DShot provisória** e motores diferentes entre si (nunca
  medidos por motor).
- **Sem medida de bateria** (`power.c` lê o pino, falta o divisor).
- **Motor M4** (frente esquerda) dá um "teco" parado só com DShot ativo, não
  em DFU: fiação ou ESC, não resolvido.
- Zero de altura vem de uma leitura só do MTF (parado, ele varia ~2 cm).

## 7. Decisões e evidências

Para cada parâmetro ou mudança: a decisão, o dado que a motivou (trechos
reais dos logs ou da simulação), o raciocínio e onde conferir. Os logs ficam só
no PC do ensaio (`tools/logs/`, fora do Git); os trechos abaixo foram copiados
deles.

Como ler os trechos: `h` é a altura (cm), `ref` a referência (cm), `th`/`phi`
os ângulos θ/φ (graus), `mot=[M1, M2, M3, M4]` a aceleração de cada motor em
milésimos (M1 trás-dir., M2 frente-dir., M3 trás-esq., M4 frente-esq.).
**Frente = M2 e M4; trás = M1 e M3.** `lqi th=` e `dist th=` são a correção
frente/trás que o integrador ou o estimador achou, em rad/s por motor
(negativo = traseira mais forte).

### 7.1 O LQG do TCC puro não sustenta o drone real

**Evidência** (05/11 manhã, `controle_20261105_115606.log`, voo no modo
"nenhuma", LQG do Simulink sem compensação; `h` absoluto do MTF, chão ≈ 8,6 cm):

```
t      h      ref     th      mot
303,0  8,6    9,7     +7,3°   [453, 436, 453, 436]
306,0  8,7   18,7    +13,3°   [452, 438, 452, 438]
310,5  8,0   26,1    +13,2°   [452, 440, 453, 440]
313,5  7,1   26,1    +13,4°   [454, 440, 454, 441]
```

**Leitura**: 38 s com o nariz em +13° e a traseira no chão. A diferença
frente/trás que o LQR gerou foi de só 13 milésimos (~23 rad/s). Com o ganho do
TCC (50,6 rad/s por rad), 13° = 0,23 rad × 50,6 ≈ 11,5 rad/s por motor: é
exatamente o equilíbrio de um controlador proporcional, em que o erro precisa
ser grande para gerar a correção. O desequilíbrio real pedia ~90.

**Decisão**: o LQG precisa (a) do ponto de operação real (7.2) e (b) de algo
que estime o que sobra (7.6). Ganho maior sozinho só reduz o ângulo (7.4).

### 7.2 `CTL_FF_TH = −90` (desequilíbrio frente/trás fixo)

**Evidência 1**: o valor que o integrador de θ achava em todo hover estável.

```
05/11 manhã, LQI em HOLD a 3–4 cm (relato a cada 2 s, de 14 s a 32 s):
    lqi th=-95  -91  -90  -91  -92  -89  -88  -89  -91  -90
05/11 16:10, LQI, voo 13 em HOLD (de 9 s a 25 s):
    lqi th=-81  -91  -96  -105  -116  -99  -82  -98  -106
05/11 manhã, motores em HOLD:  mot=[498, 394, 507, 403]
```

Nos motores: trás (M1, M3) ≈ 502, frente (M2, M4) ≈ 398, ou seja, ~104
milésimos × 1755 rad/s ≈ 180 rad/s entre frente e trás, ±90 em cada motor. O
integrador dá a mesma coisa: entre −81 e −116, com média de ~−92.

**Evidência 2**: o que acontecia sem esse valor na decolagem (05/11 16:10,
voos 7 a 12, os quatro motores começando iguais):

```
voo 8:   2,80 s  th=+4°   mot=[448, 435, 448, 436]
         3,00 s  th=+18°  mot=[500, 460, 500, 461]   -> FAILSAFE (corte em 20°)
voo 11:  2,75 s  th=+2°   mot=[455, 417, 459, 422]
         3,01 s  th=+18°  mot=[526, 440, 525, 441]   -> FAILSAFE
```

No fim da rampa (3 s), quando o empuxo chega perto do hover, a frente leve
levanta antes que a traseira e θ vai de +2° a +18° em 0,2 s. O integrador
ainda não tinha tido tempo de aprender: 7 de 13 voos acabaram assim.

**Decisão**: aplicar −90 rad/s por motor (frente −90, trás +90) desde a rampa
de partida. Na qualificação, o estimador de arfagem ficou entre −28 e +25 rad/s
em torno de 0, o que confirma o valor (se ele parasse longe de 0, esse resto
deveria ir para o `CTL_FF_TH`).

### 7.3 `CTL_W0 = 820` e "não falta força"

**Evidência 1**: aceleração média dos motores pairando (bateria cheia).

```
16:34 LQG ajustado, voo 5, alvo 4,2 cm:  h = 3,5–4,2 cm,  motores em média 47%
16:43 LQG melhorado (qualificação):      motores em média 46–47%
```

47% × 1755 rad/s ≈ 825 rad/s. Perto do chão o efeito solo ajuda, então o
hover no ar livre deve ser um pouco maior (04/11: ~850).

**Evidência 2**: quando o drone "não subia", os motores tinham folga.

```
16:10, 13 voos:  motor mais alto 61%, limite MOT_MAX 80%, nunca saturou
16:10 voo 13 (LQI em HOLD a 2 cm):  lqi z=-10 -13 -15 -16 ... -36
                                     mot=[535, 420, 535, 420] (48%)
```

O integrador de altura **negativo** quer dizer que o controle estava tirando
força para ficar em 2 cm, e mesmo assim o drone não subia além de ~4 cm com
alvo de 20: algo o segurava (as cordinhas). Aumentar `CTL_W0` ou `MOT_MAX` só
faria ele empurrar mais forte contra elas.

**Evidência 3**: a bateria acabando na qualificação.

```
W0 820, início:    46% para pairar a ~2 cm
W0 912/947, fim:   52–53% para pairar a ~2 cm   (motores nunca acima de 64%)
```

**Decisão**: `CTL_W0 = 820` com bateria cheia; o estimador (7.6) cobre o
resto. W0 de 947 compensava bateria fraca: com bateria cheia ele faria o
drone pular (simulação: 30 rad/s acima do hover leva a ~36 cm num alvo de 20).

### 7.4 Pesos de atitude: Q φ, θ = 8000 e p, q = 100

**Evidência no voo** (16:10, LQI com os pesos do TCC, voo 13 em HOLD a 2 cm):

```
18,0 s  th= -6,7°
18,5 s  th=-12,3°
19,0 s  th=-18,5°
19,5 s  th=-17,2°
20,5 s  th= -7,7°
```

Desvios lentos de até 18° (o corte é em 20°) com o drone parado: o ganho do
TCC deixa a atitude "mole".

**Evidência na simulação**: varredura dos pesos (feita no LQI; o LQG ajustado
usa os mesmos pesos de atitude, com K_θ = 142) com o motor atrasando 40, 80 e
120 ms, desequilíbrio de 125 rad/s e um empurrão de 30 rad/s por 1,5 s
(rebote = maior |θ| depois do empurrão; ganho = K_θ resultante):

| Q φ,θ | Q p,q | K_θ (LQI) | 40 ms | 80 ms | 120 ms |
|---|---|---|---|---|---|
| 500 | 10 | 71 | 16,0° | 22,3° | 28,5° |
| 1000 | 10 | 80 | 14,6° | 20,8° | 28,9° |
| 2000 | 10 | 94 | 12,8° | 18,6° | 29,6° |
| 2000 | 30 | 96 | 11,3° | 16,5° | 26,2° |
| 4000 | 30 | 117 | 9,9° | 14,5° | 23,4° |
| 4000 | 60 | 118 | 8,7° | 13,2° | 18,4° |
| **8000** | **100** | **150** | **6,7°** | **11,1°** | **10,7°** |
| 16000 | 100 | 196 | 5,9° | 9,9° | **instável** |
| 16000 | 200 | 194 | 4,8° | 8,2° | 14,9° |

**Decisão**: 8000/100. É o ponto em que o rebote cai à metade sem perder
estabilidade a 120 ms de atraso (o atraso real dos motores não foi medido).
16000/100 já diverge a 120 ms; 16000/200 volta a ser estável, mas sobra pouca
margem e o ganho de p, q amplifica o ruído do giroscópio. Na qualificação, θ
ficou com RMS de 1,4–4,0°.

### 7.5 Pesos de altura: Q z = 5000 e vz = 50

**Evidência na simulação** (LQG ajustado sem estimador; hover 30 rad/s abaixo
ou acima do real; alvo de 20 cm):

| Q z / vz | K_z | W0 30 abaixo | W0 30 acima |
|---|---|---|---|
| 500 / 5 (TCC) | 35 | não decola | +100,7 cm acima do alvo |
| **5000 / 50** | **110** | não decola | +35,9 cm |
| 20000 / 200 | 220 | não decola | +20,0 cm |
| 50000 / 500 | 346 | −13,8 cm | +13,8 cm |

**Leitura**: sem estimador, o erro de altura é força ÷ ganho; nenhum ganho
razoável zera o erro de um W0 errado. Ganhos muito altos também amplificam o
ruído do MTF (σ ≈ 1 cm parado).

**Decisão**: 5000/50, três vezes o TCC, para o LQG ajustado ser utilizável com o
W0 perto do certo (no voo de 16:34 a 4 cm, o erro foi 0,7 cm). Quem zera o erro
é o estimador (7.6), não o ganho.

### 7.6 Estimador de perturbação: variância de d = 0,1 (altura) e 1e-3 (atitude)

**Evidência na simulação**. Primeiro, a mesma variância nos três canais:

| Variância de d | Erro de altura | Com motor a 120 ms |
|---|---|---|
| 1e-3 | −10,8 cm (lento) | estável |
| 1e-2 | −0,5 cm | **atitude diverge** (θ −219°) |
| 1e-1 | 0 | **diverge** |
| 1 | 0 | **diverge** |

A atitude precisa de um estimador lento e a altura de um rápido. Depois,
atitude fixa em 1e-3 e varrendo a altura (W0 30 acima do real, motor a 80 ms):

| Variância (altura) | Erro de altura | Pico de altura (alvo 20 cm) |
|---|---|---|
| 3e-3 | +2,6 cm | 47,8 cm |
| 1e-2 | +0,3 cm | 42,6 cm |
| 3e-2 | 0 | 36,2 cm |
| **0,1** | **0** | **28,6 cm** |

(Com a correção do chão descrita abaixo, o pico desse caso passou a 35,6 cm:
um W0 30 rad/s acima do real faz o drone passar do alvo antes de o estimador
achar o excesso. Por isso o `CTL_W0` ainda precisa ficar perto do hover.)

**Decisão**: 0,1 na altura e 1e-3 na atitude. Com elas, a simulação zera
altura, φ e θ em todos os casos (inclusive sem o `CTL_FF_TH` e com 120 ms de
atraso) e o rebote fica em 8–13°, perto do LQI (6,7–11°).

**Correção depois do voo** (16:34, primeira versão do melhorado):

```
22,9 s  voo: decolando para 4 cm
25,9 s  dist z=-160 th=53      <- fim da rampa de partida
...     altura máxima 1,9 cm, motores em 36%
```

Na rampa os motores ficam abaixo do hover e o chão segura o drone; o estimador
leu isso como "sobra empuxo" e cortou o motor. A simulação não pegou porque
não tinha a rampa abaixo do hover. **Decisão**: no chão o estimador fica
congelado (também na correção, não só na predição) até o drone subir 1 cm, e o
hover sobe numa rampa fixa até lá. O teste `test_ground_does_not_teach`
reproduz o caso.

### 7.7 Detector de subida presa e velocidade do Kalman

**Evidência 1**: com o drone preso nas cordinhas, a velocidade vertical do
Kalman é falsa (05/11 12:21, alvo de 20 cm):

```
y_z(medido) = -115 mm   x_z(estimado) = -116 mm   ref_z = -278 mm   x_vz = -54 mm/s
y_z(medido) = -117 mm   x_z(estimado) = -120 mm   ref_z = -278 mm   x_vz = -41 mm/s
```

A altura está parada e o Kalman estima 4–5 cm/s de subida: o modelo prevê o
que o LQR pede, e a medida só corrige a posição. **Decisão**: o detector e o
pouso usam a altura do MTF filtrada (0,3 s) e a velocidade derivada dela.

**Evidência 2**: o critério "não subiu 2 cm em 3 s com o alvo 4 cm acima",
simulado sobre os logs de 05/11, teria segurado os 14 voos presos de 7 a 23 s
antes do FAILSAFE, sem disparar nos voos em HOLD.

**Revisão**: na qualificação, sem cordinhas, ele disparou em 7 de 9 voos aos
7–10 s e trocou o alvo de 20 cm por "HOLD em 2 cm". Fora da bancada com
cordinhas, ele atrapalha a subida (próximo passo: deixá-lo opcional).

### 7.8 Pouso que desconta o hover

**Evidência** (05/11 manhã, LQI, "pousar" em t = 83 s; `h` absoluto, chão ≈ 8,6 cm):

```
83,0 s  h=11,5  ref=10,9   mot=[510, 406, 511, 406]
84,5 s  h=12,5  ref= 8,6   mot=[508, 397, 516, 403]
87,0 s  h=12,4  ref= 8,6   mot=[492, 415, 491, 413]   <- 4 s parado no alto
88,0 s  h= 9,7  ref= 8,6                              <- cai de uma vez
```

O integrador de altura guardava o empuxo de hover e desfazia devagar.
**Decisão**: no pouso, se o drone está mais de 1 cm acima da referência e não
desce a 3 cm/s, o hover estimado cai 20 rad/s por segundo. Depois disso, o
pouso passou a levar ~1,3 s.

### 7.9 Equilíbrio lembrado entre voos: removido

**Evidência**: com o drone preso, o equilíbrio guardado foi `th=-128 phi=-10`,
contra ~−90 e 0 de um hover de verdade. No ajuste lento, o equilíbrio
guardado do mesmo jeito (`trim inicial th=-74 phi=-29`) fez o voo seguinte
sair inclinado (φ −6° logo após a decolagem). **Decisão**: retirado no LQI e no LQG
melhorado; o `CTL_FF_TH` fixo faz o papel com um valor medido e conhecido.

### 7.10 Resumo dos valores

| Parâmetro | TCC | Qualificado | Seção |
|---|---|---|---|
| ω0 / `CTL_W0` | 702 | 820 | 7.3 |
| `CTL_FF_TH` | 0 | −90 | 7.2 |
| Q z / vz | 500 / 5 | 5000 / 50 | 7.5 |
| Q φ, θ / p, q | 500 / 10 | 8000 / 100 | 7.4 |
| Q x, y, ψ, vx, vy, r | 5, 5, 50, 5, 5, 5 | iguais | — |
| R | 0,1 I | 0,1 I | — |
| Kalman (Qn, Rn, L, M) | do Simulink | igual | — |
| Variância de d (estimador) | — | 0,1 altura; 1e-3 atitude | 7.6 |
| Subida presa | — | 2 cm em 3 s (a tornar opcional) | 7.7 |
| Desconto no pouso | — | 20 rad/s por s | 7.8 |

## 8. Como reproduzir

```
git checkout lqg-melhorado-aprovado
python3 tools/build.py            # firmware em build/Release/
sh tests/run.sh                   # testes, simulação e conferência das tabelas
python3 tools/lqg_ajustado.py     # regenera Kd/Kfd ajustados e o estimador
python3 tools/controle_gui.py     # painel (modo LQG melhorado, W0 820, FF −90)
```

Documentação técnica: `docs/controle_lqg.md` (seção 12: modos LQG),
`docs/controle_lqi.md` (LQI), `docs/teste_controle.md` (bancada).
