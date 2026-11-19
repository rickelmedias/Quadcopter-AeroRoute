# AeroRoute Phase 2 Bench

Firmware de bancada para validar o hardware e os módulos fundamentais antes de
construir o novo controle de voo.

## Escopo atual

- STM32F411 + FreeRTOS/CMSIS-RTOS2;
- aquisição do BNO085 e MTF-01;
- leitura de potência;
- DShot e arbitragem segura dos quatro motores;
- enlace USB binário bench-link v1 (telemetria e comandos, `../docs/phase2/bench-link.md`);
- telemetria KISS dos ESCs (USART1/PA10) e leitura analógica de bateria/corrente;
- IHM no PC em `../hmi`.

O controle novo será construído nesta base depois da validação dos ensaios. Não há Raspberry
Pi, DroneLink, MAVLink ou controlador LQG nesta branch.

## Build e testes

```sh
conda run -n aeroroute python tools/build.py
sh tests/run.sh
```

O artefato é gerado em `build/Release/`. O build não grava a placa.

## Organização

- `App/Src` e `App/Inc`: código manual da bancada;
- `Core`, `Drivers`, `Middlewares` e `USB_DEVICE`: infraestrutura STM32/Cube;
- `tests`: testes host-only dos módulos puros;
- `tools`: build e verificações de código;
- `docs/`: documentação da bancada;
- `../references/legacy-simulink`: cópia histórica do modelo antigo, fora do
  firmware e fora da cadeia de build.

O histórico de controle, protocolo, Pi, ensaios e modelos antigos permanece
acessível nas branches históricas do Git.
