# Guia de teste dos motores (DShot600, 0 a 100%)

Este guia vale para o console USB da `main` (`App/Inc/usb_console.h`) e para
a branch `motors`. Na bancada não há controle de voo: cada motor recebe
exatamente o que for digitado, com rampa limitada. Os sensores (MTF-01 e
BNO08x) continuam sendo lidos e registrados, para ver a vibração e a
interferência com os motores girando. Os comandos de voo do mesmo console
(`hover`, `dry`, `land`...) estão em [`controle_lqg.md`](controle_lqg.md).

## 1. Segurança antes de ligar

- **Tire as hélices.** Todas. O teste vai até 100%.
- Prenda o frame ou os motores na bancada. Um motor a 100% sem carga gira muito
  rápido e qualquer parafuso solto vira projétil.
- Use, se possível, uma fonte com limite de corrente (2 a 3 A bastam sem
  hélice) ou um "smoke stopper" na bateria no primeiro teste. Um curto no ESC
  aparece como corrente alta logo no armamento.
- A parada de emergência real é **desligar a bateria**. Deixe o conector ao
  alcance da mão.
- Ligue só **GND e sinal** do ESC na Black Pill. Não ligue o 5 V do BEC do ESC
  na placa enquanto ela estiver na USB do PC (as duas fontes brigariam).
- O GND do ESC precisa ser o mesmo da Black Pill, senão o sinal DShot não é
  entendido.

## 2. Ligações

| Motor | Sinal do ESC | Pino STM32 | Canal |
|---|---|---|---|
| 1 | M1 | PB6 | TIM4_CH1 |
| 2 | M2 | PB7 | TIM4_CH2 |
| 3 | M3 | PB8 | TIM4_CH3 |
| 4 | M4 | PB9 | TIM4_CH4 |

O firmware envia DShot600 (1,67 us por bit, um quadro por milissegundo). O ESC
do projeto (Skystars KM55A II, AM32) aceita DShot300 e DShot600. Se o ESC só
aceitar DShot300, mude o período do TIM4 para 319 (`htim4.Init.Period` no
CubeMX); o driver recalcula os pulsos sozinho.

## 3. Gravar e abrir o console

1. `git checkout motors`, compile e grave a placa como de costume (DFU ou
   STM32CubeProgrammer).
2. Com a placa na USB e **a bateria ainda desligada**, abra o console:

```powershell
.\tools\motor_console.ps1 -Port COM5 -OutputPath logs\motores.txt
```

O console liga o DTR (o firmware só arma com a porta aberta), mostra o log e
envia o que você digitar ao apertar Enter. Fechar o console (Ctrl+C) para e
desarma os motores.

3. Digite `log off` para o terminal não ser inundado pelas linhas do BNO
   (400 por segundo). Para gravar dados de vibração, use `log on` com
   `-OutputPath`.

## 4. Comandos

| Comando | O que faz |
|---|---|
| `arm` | Arma a saída. Só aceita com todos os motores em 0. |
| `m <motor> <pct>` | Alvo de um motor (1 a 4) ou de todos (0), de 0 a 100%. |
| `sweep <motor> [max] [passo] [seg]` | Sobe de 0 até `max`% em degraus de `passo`%, segura cada degrau `seg` segundos e desce até 0. Padrão: 100, 10, 3. |
| `rate <pct/s>` | Velocidade máxima da rampa, de 1 a 100 %/s. Padrão 20 %/s (0 a 100% em 5 s). |
| `beep <motor>` | Faz o ESC apitar, para descobrir qual motor é qual. Só desarmado. |
| `log on` / `log off` | Liga ou silencia as linhas MTF, BNO e MOT. |
| `status`, `help` | Mostra o estado ou a ajuda. |
| `stop`, `disarm`, `s` ou **Enter vazio** | Zera tudo na hora e desarma. |

Qualquer linha que o firmware não reconheça também para e desarma, então um
erro de digitação nunca deixa o motor acelerando.

## 5. Roteiro do teste

1. **Ligue a bateria** com o console aberto e `log off`. Os ESCs tocam a
   melodia de inicialização e, como o firmware manda zero desde o boot, o tom
   de "armado" logo depois. Bipes contínuos e repetidos indicam que o ESC não
   está entendendo o sinal (ver seção 7).
2. **Identifique os motores:** `beep 1`, `beep 2`, `beep 3`, `beep 4`. Confira
   se cada um apita no lugar esperado do frame.
3. **Primeiro giro, motor 1:**
   - `arm`
   - `m 1 5` (5%). O motor deve girar devagar e liso.
   - Confira o sentido de rotação contra o esquema do frame.
   - Enter vazio para parar.
4. **Degraus de 0 a 100%, um motor por vez:**
   - `arm`
   - `sweep 1 100 10 3` (0 → 10 → 20 … → 100 → 90 … → 0, 3 s por degrau).
   - Observe ruído, vibração e cheiro. Pare com Enter vazio se algo estranhar.
   - Repita para os motores 2, 3 e 4.
5. **Todos juntos:** `arm`, `sweep 0 100 20 2`.
6. **Com dados:** feche e abra o console com `-OutputPath`, `log on`, `arm`, e
   repita o `sweep 0 100 10 3`. O arquivo terá as linhas `MOT` (comando de
   cada motor) junto com as `BNO` e `MTF` no mesmo eixo de tempo.

Não deixe os motores muito tempo a 100% sem carga: alguns segundos por degrau
bastam. Toque o motor e o ESC depois de cada varredura; morno é normal, quente
demais para segurar não é.

## 6. O que observar e anotar

- Aceleração em que cada motor começa a girar (normalmente entre 2 e 6%).
- Se a rotação é lisa em toda a faixa ou se há trancos em algum degrau.
- Sentido de cada motor.
- Nas linhas `BNO`, o giroscópio e a aceleração crescendo com a rotação: é a
  vibração que o filtro de Kalman do controle vai precisar tolerar.
- Nas linhas `SYS`, o contador de descartes do log deve ficar parado. Se
  subir, o PC não está lendo rápido o bastante (reduza com `LOG_BNO_DIVIDER`
  em `sensor_log.h`).

Formato da linha `MOT`:

```text
MOT;t_ms;armado;thr1;thr2;thr3;thr4;dshot1;dshot2;dshot3;dshot4
```

`thr` está em décimos de por cento (1000 = 100%) e `dshot` é o valor de 11
bits enviado (0 = parado, 48 a 2047 = aceleração).

## 7. Problemas comuns

| Sintoma | Causa provável |
|---|---|
| ESC bipa sem parar e nunca arma | Sem sinal ou protocolo errado: confira GND comum, o pino e se o ESC aceita DShot600 (senão, DShot300). |
| `abra a porta com DTR ligado` | O programa serial não liga o DTR. Use `motor_console.ps1`. |
| Motor gira ao contrário | Inverta dois dos três fios do motor ou o sentido no configurador do ESC (BLHeli/Bluejay/AM32). |
| Motor treme e não parte | Fio de fase solto ou timing do ESC; teste com outro motor. |
| Para sozinho com `PARADO: porta USB fechada` | O console foi fechado ou a USB caiu. É a proteção funcionando. |
