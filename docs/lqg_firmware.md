# LQG no firmware

Esta é a versão integrada da `main`: sensores, log por fila, DShot e a malha
LQG do TCC (Filtro de Kalman + LQR). Os ganhos não ficam escritos no código:
`tools/lqg_design.py` monta o modelo do TCC, discretiza, calcula o LQR e gera
`App/Inc/lqg_gains.h` e `App/Src/lqg_gains.c`. O código do controle
(`App/Src/lqg.c`) é genérico e não muda quando o modelo, os pesos ou o período
mudam.

**Os motores não giram com o LQG por padrão.** `LQG_FLIGHT_ENABLED` vale 0 em
`App/Inc/flight_config.h`: o comando `fly` é recusado e só o ensaio a seco
(`dry`) funciona. Isso só deve mudar depois do roteiro da seção 5.

## 1. De onde vem cada número

| Item | Origem |
|---|---|
| Estados, entradas, A e B | `metodologias.tex`, linearização no hover (12 estados, entradas = rotação dos 4 motores) |
| M, l, Ixx, Iyy, Izz, Kt, Kq | Quadros de parâmetros da metodologia |
| Q e R do LQR | `resultados.tex`, eq. `valores_Q_R_sim` |
| C | Saídas medidas: z, vx, vy (MTF-01) e phi, theta, psi, p, q, r (BNO08x) |
| Rv | `resultados.tex`, Quadro `estatistica_sensores` |
| Qw | **Não publicado no TCC.** Valores iniciais em `tools/lqg_design.py`, para ajustar |
| Período | 2,5 ms (400 Hz, a taxa do BNO). O TCC simulou com 100 ms; os ganhos foram recalculados para 2,5 ms com os mesmos Q e R |

Para regerar depois de mudar qualquer valor:

```bash
python tools/lqg_design.py
```

O script imprime os postos de controlabilidade (12) e observabilidade (10, x e
y sem medida absoluta, como no TCC) e o maior módulo dos polos de malha
fechada, que precisa ficar abaixo de 1. No MATLAB, o equivalente é
`sysd = c2d(ss(A,B,eye(12),0), Ts); K = dlqr(sysd.A, sysd.B, Q, R)`.

## 2. Como a malha roda

1. O BNO08x publica um relatório de rotação (400 Hz) e acorda a task `Flight`.
2. Predição do Kalman com a última entrada aplicada.
3. Correção com phi, theta, psi, p, q e r. Quando o deslocamento integrou um
   frame novo do MTF-01 (100 Hz), correção também com z, vx e vy.
4. `u = u0 - K (x_hat - x_ref)`, saturado em `[0,3 w0, 2 w0]`.
5. Cada rotação vira aceleração DShot pela curva de `flight_config.h` e vai
   para a task de motores, que envia um quadro por milissegundo.

O Kalman é o variante no tempo, com uma medida por vez. Isso deixa cada sensor
entrar na sua própria taxa e, com todas as medidas presentes, converge para o
mesmo ganho do Kalman de regime do MATLAB. Para x e y, que não são
observáveis, a variância é limitada para não crescer sem fim. As medidas do
MTF-01 passam por um teste de inovação de 6 desvios padrão, para que um salto
do sensor não contamine o estado.

## 3. Convenção dos motores

A matriz B do TCC fixa a posição e o sentido de cada motor (eixos NED, vista de
cima, nariz para cima na página):

```text
   M1 (horário)      M2 (anti-horário)
            \  frente  /
             \        /
             /        \
            /          \
   M4 (anti-horário) M3 (horário)
```

M1 é o canal PB6, M2 o PB7, M3 o PB8 e M4 o PB9. Esta numeração **não** é a do
Betaflight; ligue os ESCs ou troque a ordem no código para bater com ela. Use
`beep 1` a `beep 4` para conferir.

## 4. Pontos a calibrar antes de voar

1. **Curva rotação × DShot.** `MOTOR_OMEGA_AT_MIN_RAD_S` e
   `MOTOR_OMEGA_AT_MAX_RAD_S` são provisórios. Use os dados do dinamômetro.
2. **Kt e w0.** Com o Kt do TCC (3,6e-5), o hover dá w0 = 146 rad/s
   (~1400 rpm), o que parece baixo para hélices de 3,5" (o esperado é da ordem
   de 1500 rad/s). Vale reconferir a unidade de Kt; um Kt diferente muda w0,
   a matriz B e todos os ganhos. O script também calcula w0 = 146,02 rad/s,
   e não 146,23 como no texto.
3. **Sinais dos sensores** (`SIGN_*` em `flight_config.h`). O padrão supõe o
   BNO com x para a frente e z para cima.
4. **Qw.** Comece pelos valores do script e ajuste com dados de bancada.

## 5. Roteiro de bancada (sem hélices)

1. Grave a `main`, abra `tools\motor_console.ps1` com `-OutputPath`.
2. `dry`. O LQG liga e as linhas `EST` aparecem (estados x1000 em SI e as
   quatro rotações x10).
3. Com o drone na mão, sem hélices:
   - abaixe o lado direito: phi deve ficar positivo e w2 e w3 (lado direito)
     devem subir;
   - levante o nariz: theta positivo, w3 e w4 (traseira) sobem;
   - gire no sentido horário visto de cima: psi aumenta, w1 e w3 sobem;
   - levante o drone: z fica mais negativo e as quatro rotações caem.
   Se algum sinal estiver trocado, corrija o `SIGN_*` correspondente.
4. `lqg` mostra os contadores; `rej z/vx/vy` deve ficar perto de zero.
5. Só então, com a curva do motor medida e o drone preso, faça
   `LQG_FLIGHT_ENABLED 1`, grave e teste `fly` sem hélices.

## 6. Linha EST

```text
EST;t_ms;modo;x;y;z;phi;theta;psi;vx;vy;vz;p;q;r;w1;w2;w3;w4
```

`modo`: 0 parado, 1 ensaio a seco, 2 voo. Os estados estão multiplicados por
1000 (m, rad, m/s, rad/s) e as rotações por 10 (rad/s).
