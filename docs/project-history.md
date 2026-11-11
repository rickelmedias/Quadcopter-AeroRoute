# Histórico e conhecimento interno do projeto

Notas da equipe (Rickelme e Thalison) para orientar humanos e agentes. Não é documentação formal do repositório: registra o que sabemos, inclusive o que não aparece nos READMEs. Quando houver conflito com a memória da equipe, o que foi conferido no código está marcado como **conferido**.

## Branches

| Prefixo | O que é | Como tratar |
| --- | --- | --- |
| `archive/*` | Tudo o que foi feito durante o TCC, com os nomes originais. Muito material de passagem, útil para quem lê o TCC e quer ver a evolução. | Só consulta histórica. Não é configuração recomendada. |
| `milestone/*` | Etapas que de fato testamos. | Congeladas. São a evidência de cada etapa. |
| `submission/2026-11-11` e tag `tcc-2026-11-11` | Versão "final" entregue em 2026-11-11 (capturada em 2026-11-10), base do texto e do vídeo do drone. | Congeladas. Nunca mover. |
| `exploration/ekf-2026-11-10` | Melhorias locais que não estavam commitadas (EKF, filtros, controle) e a pasta `drone-better` da Pi. | Exploração, não é resultado do TCC. |
| `main` | Desenvolvimento. Ainda sem definição final porque não chegamos ao resultado esperado. | Onde a Fase 2 acontece. |

## Milestones

| Branch | Etapa |
| --- | --- |
| `01-sensors` | Testes do MTF-01 e do BNO085. |
| `02-motors` | Teste de motores pela GUI Python (`motores_gui.py`). |
| `03-usb-control` | Controle de atitude e altura pelo console USB, sem controle de posição XY. |
| `03-position-lqg` | Teste do controle LQG de posição. |
| `04-dronelink` | Protocolo DroneLink e rotas: Pi e UART reais, planta simulada no STM32 (`FLT_SIM=1`, motores parados). |
| `05-integrated-pi-hover` | Integração do controle de posição com a Pi. **Conferido:** é o mesmo commit da submissão (`af6034e`), ou seja, a entrega real. |

Sobre o "better": `hover-lqg-pi-better` tinha o mesmo commit que `hover-lqg-pi`. As melhorias dele nunca foram commitadas e estão só em `exploration/ekf-2026-11-10`. Portanto a milestone 05 **não** tem EKF.

## O que o TCC afirma e o que o código tem

- **EKF.** O texto cita o EKF só na teoria e como trabalho futuro (conclusão: EKF com quatérnios). **Conferido:** o firmware da submissão não tem EKF. `nav_ekf.c` existe só na branch de exploração.
- **Integrador.** O texto descreve LQR sem ação integral. **Conferido:** o firmware da submissão tem extras fora do texto: integrador de altura e trim (`trim.c`, `update_trim`), estimação de perturbação do "LQG melhorado" (`lqg_dist.c`) e o desconto do hover no pouso. O LQI da branch `controle-lqi` saiu em 06/11.
- Na nossa avaliação, EKF e integradores não foram desenvolvidos nem validados como parte do trabalho. Não citá-los como resultado do TCC.
- O TCC não demonstra voo livre autônomo confiável: posição XY e guinada derivavam.

## Fase 2 (a partir de 2026-11-11)

Objetivo de voo: decolar mantendo XY, atingir a altura definida, pairar, voltar ao ponto após perturbações e depois executar uma sequência de pontos.

As milestones antigas não mudam. Os testes da Fase 2 ficam em branches novas (`phase2/<nn>-<tema>`) a partir de `main`. Quando um ensaio terminar com dados, criar uma tag (`phase2-<nn>-<tema>`) apontando para o código que gerou os dados.

O novo código deve nascer padronizado: `.clang-format` e ruff desde o início, separação clara entre código gerado e manual, e logs com formato e unidades documentados.

### Primeiro ensaio: ruído do BNO085 e do MTF-01

Medir a influência dos motores nos sensores e obter as covariâncias R do Kalman a partir de dados, como o próprio TCC recomenda.

1. Drone parado, alimentado pelo USB, sem bateria.
2. Bateria ligada, ESC energizado, motores parados (interferência elétrica).
3. Motores sem hélice em degraus de throttle, um motor por vez e depois os quatro.
4. Com hélices, drone preso na bancada, nos mesmos degraus.

Em cada condição, gravar com carimbo de tempo: aceleração e giro brutos, orientação do BNO, distância, fluxo e qualidade do MTF, comando DShot e, quando existir, tensão da bateria, corrente e RPM. Relatar média, desvio padrão, deriva e espectro (FFT), e relacionar os picos à rotação dos motores.

## Hardware ainda não ligado

- `power.c`/`power.h` (**conferido**) já leem a bateria no PA1 e a corrente no PB0 pelo ADC1, mas estão desligados (`POWER_VBAT_ENABLED 0`, `POWER_CURRENT_ENABLED 0`). Os valores do divisor (11) e da escala de corrente (50 A/V) são provisórios. Ainda não há failsafe de bateria baixa.
- O pad TX do ESC (telemetria serial, uma linha para os quatro motores) não está ligado. O F411 só tem USART1, USART2 e USART6. As duas primeiras já estão em uso, e o RX da USART6 fica no PA12 (USB) ou no PC7. É preciso confirmar no esquema da Black Pill se há pino livre.
- Alternativa para a RPM sem solda: DShot bidirecional (eRPM) do AM32. Exige mudar `dshot.c` para captura de entrada no mesmo pino.

## Acesso

- Pi: `ssh r1ddax@r1ddax.local`, programa em `~/drone`, ensaios em `~/drone/ensaios`.
- Gravação do STM32: DFU por USB com o STM32CubeProgrammer. O comando de CLI exato que a equipe usa ainda precisa ser registrado.
