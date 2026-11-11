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

Temos cerca de um mês até a entrega oficial final. O `main` deve terminar como a versão que funciona como queríamos. Tudo o que existe até a submissão é a última versão do TCC; daqui em diante são melhorias.

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

## ESC e hardware ainda não ligado

ESC: Skystars KM55A II 4 em 1, AM32 Turbo (2024), MCU AT32F421, 3S–6S, DShot300/600 (TCC, `metodologias.tex`).

**Pinos em uso (conferido no `.ioc`):** PA2/PA3 USART2 (MTF-01), PA9/PA10 USART1 (Pi), PA4–PA7 SPI1 (flash), PA11/PA12 USB, PA13/PA14 SWD, PB3/PB10 I2C2 + PB4 INT + PB5 RST (BNO085), PB6–PB9 TIM4 (DShot). Reservados no `power.c`: PA1 (bateria) e PB0 (corrente). Livres no encapsulamento de 48 pinos: PA8, PA15, PB1, PB12–PB15 (PA0 é o botão KEY, PC13 o LED, PB2 é o BOOT1).

**TX do ESC (telemetria KISS, conferido no código do AM32 em `references/AM32`):** quadro de 10 bytes a 115200 baud com temperatura (°C), tensão (cV), corrente (cA), consumo (mAh), eRPM/100 e CRC8. O ESC responde quando o pacote DShot do motor tem o bit de telemetria ligado. Os quatro ESCs da placa 4 em 1 dividem uma linha, por isso os pedidos são feitos um motor por vez.
- Não há RX de UART livre. A USART6 só tem RX no PA12 (USB) ou no PC7 (não existe na Black Pill), e a USART1 só tem RX no PA10 ou no PB3 (BNO).
- Na bancada, sem a Pi, ligar o TX do ESC no PA10 (USART1_RX) no lugar do TX da Pi. Não muda pino e só troca quem interpreta a USART1.
- Para o voo com a Pi, usar o DShot bidirecional com EDT (Extended DShot Telemetry) do AM32. Ele traz eRPM a cada quadro e temperatura, tensão e corrente, sem fio extra, mas exige reescrever o `dshot.c` para captura de entrada no TIM4.

**CURR:** saída analógica do sensor de corrente total da placa. Vai no PB0 (ADC1, canal 8), já previsto em `power.c`. Cada placa tem uma escala, e a do KM55A II não está documentada: calibrar comparando com a corrente da telemetria KISS ou com um multímetro. A vantagem sobre a telemetria é a taxa (o ADC pode ler a 1 kHz).

**Bateria:** `power.c`/`power.h` (**conferido**) já leem o PA1 (divisor) e o PB0 (CURR), mas estão desligados (`POWER_VBAT_ENABLED 0`, `POWER_CURRENT_ENABLED 0`). O divisor (11) e a escala (50 A/V) são provisórios. Com o TX ligado, a telemetria já dá a tensão, e o divisor vira uma redundância independente do ESC. Ainda não há failsafe de bateria baixa.

## Acesso

- Pi: `ssh r1ddax@r1ddax.local`, programa em `~/drone`, ensaios em `~/drone/ensaios`.
- Gravação do STM32: DFU por USB com `STM32_Programmer_CLI -c port=USB1 -w build/Release/DroneController.hex -v -g`, recuperado do histórico das sessões. O "usb0" de que a equipe lembrava é o `USB1` do CubeProgrammer. Procedimento completo: skill `stm32-flash`.
