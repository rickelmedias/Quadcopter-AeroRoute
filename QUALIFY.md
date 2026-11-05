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

## 7. Como reproduzir

```
git checkout lqg-melhorado-aprovado
python3 tools/build.py            # firmware em build/Release/
sh tests/run.sh                   # testes, simulação e conferência das tabelas
python3 tools/lqg_ajustado.py     # regenera Kd/Kfd ajustados e o estimador
python3 tools/controle_gui.py     # painel (modo LQG melhorado, W0 820, FF −90)
```

Documentação técnica: `docs/controle_lqg.md` (seção 12: modos LQG),
`docs/controle_lqi.md` (LQI), `docs/teste_controle.md` (bancada).
