# Captura e replay — formato 3

As correções de 2026-09-16 preservam eventos de aquisição e a execução real da
task de deslocamento separadamente. Não alteram a regra de fluxo zero nem a
matemática dos estimadores.

**Para que serve:** gravar tudo o que os sensores entregaram e o que o
estimador decidiu, e depois reexecutar o mesmo cálculo no PC. Assim, uma
mudança de algoritmo pode ser avaliada sobre o mesmo movimento físico, sem
repetir a bancada. É ferramenta de desenvolvimento: **o firmware
`build/Capture` nunca deve ir para o drone**, porque nele o estimador só roda
enquanto o PC está gravando.

## Validado em bancada (2026-09-16)

| | Ensaio 01 | Ensaio 02 |
|---|---|---|
| Arquivo | `captures/ensaio01.bin` | `captures/ensaio02.bin` |
| Duração / situação | 30 s, parada | 45 s, com movimento |
| Bytes (~25 KB/s) | 750 947 | 1 126 025 |
| Buracos / corrompidos / descartes | 0 / 0 / 0 | 0 / 0 / 0 |
| Replay | REPRODUCED, 3010 passos | REPRODUCED, 4514 passos |
| Maior diferença replay × placa | 4,1 × 10⁻⁸ | 1,0 × 10⁻⁷ |

- O início por DTR funcionou no Windows (PowerShell 5.1 + driver USB serial
  padrão), e a USB acompanhou a taxa sem perdas.
- Firmware identificado na SESSION (`1ce7f9c9de921e4029423727`) igual ao
  `manifest.json` do build Capture.
- Memória medida pelo DIAG no build Capture: menor heap livre **2688 bytes**
  (constante durante a gravação) e margens de pilha de 1100 bytes
  (`Displace`), 1304 (`MTF01`), 1460 (`Capture`) e 2972 (`BNO08X`). A folga de
  heap é pequena: qualquer task ou fila nova nesse build precisa rever
  `configTOTAL_HEAP_SIZE`.

Os resultados de estimação desses ensaios estão em
`docs/evolucao_algoritmo_deslocamento.md`, Seção 6.

## O que mudou

- `OBSERVATION` registra cada snapshot MTF visto pela task: ID, timestamp
  anterior e flags de primeiro frame, intervalo utilizável e validade efetiva
  de distância/fluxo. Inclui primeiro frame e intervalos rejeitados.
- `STEP` continua identificando os quatro IDs consumidos. O replay busca BNO
  e MTF em históricos de 8192 entradas por tipo, em vez de substituir uma
  amostra pela mais nova. Uma referência ausente/expirada reprova a execução.
- Eventos MTF que não foram observados pela task não alteram o `dt` do replay.
- Comparações rejeitam NaN/infinito em ambos os lados. A configuração SESSION
  e as constantes relevantes também são verificadas.
- Descartes por contenção são incrementados em seção crítica. A sequência é
  reservada somente pelo produtor que detém o mutex. Contenção não consome
  sequência: é reportada separadamente no DIAG.
- Cada task (MTF, BNO, deslocamento e captura) solicita DIAG aproximadamente a
  cada segundo enquanto executa seu loop. O nome identifica de quem é a margem
  de stack. Uma task parada/bloqueada não consegue emitir seu próprio DIAG.
- O replay reprova perda reportada em DIAG mesmo sem buracos de sequência.
- SESSION é enfileirado pela task de captura após DTR, antes de liberar os
  produtores. O consumidor USB verifica
  o retorno da transmissão antes de avançar para o próximo bloco.
- Captura e log textual simultâneos são rejeitados em compilação.

O formato 3 é incompatível com versões 1/2, recusadas pelo decoder/replay.

## Builds separados, sem editar os seletores do firmware

No Windows com a toolchain STM32CubeIDE instalada:

```powershell
python tools/build.py
python tools/build.py --capture
```

- Normal: `build/Release/DroneController.hex`.
- Captura binária, sem logs textuais: `build/Capture/DroneController.hex`.
- Acrescentar `--debug` produz `build/Debug` ou `build/CaptureDebug`.

O build padrão permanece sem captura. `--capture` seleciona um perfil próprio
por macro de compilação; não modifica os arquivos de configuração.
Cada diretório contém `manifest.json` com SHA-256 dos artefatos e fingerprint
dos fontes/headers/linker e variante. O SESSION contém o prefixo dessa
fingerprint. Guardar o manifest, binário e cópia dos fontes junto da gravação;
o prefixo não substitui o hash completo do binário nem o arquivamento do código.

## Testes e reprodução

No WSL, a partir da pasta do projeto:

```sh
sh tests/run.sh
gcc -std=c11 -Wall -Wextra -Werror -IApp/Inc tools/replay.c \
    App/Src/disp_estimator.c App/Src/flow_ekf.c App/Src/attitude.c -lm -o /tmp/replay
/tmp/replay captures/ensaio02.bin
python3 tools/decode_capture.py captures/ensaio02.bin --out-dir captures/ensaio02
```

O decoder gera um CSV por tipo de registro (`session`, `mtf`, `bno`,
`observation`, `step`, `diag`). O replay termina com `REPRODUCED` (código 0),
`DIVERGED` ou `INCOMPLETE` (código 1). Em `INCOMPLETE`, leia também os
contadores de pareamento, decisão e valores impressos acima: a última linha
não distingue perda de divergência.

O runner executa oito suítes C e verificação cruzada C/Python, agora também com
ASan/UBSan nos executáveis sintéticos e replay. Os testes incluem preempção
legítima (registros mais novos antes do STEP), frame MTF extra não observado,
rejeição temporal, NaN, infinito, perda por contenção sem buraco e alterações
de amostras/decisões/configuração. A concorrência do incremento protegido foi
verificada por inspeção; testes de host não simulam o scheduler FreeRTOS.

## Procedimento de gravação

### Gravar bytes no Windows

1. Grave `build/Capture/DroneController.hex` por DFU, volte BOOT0 a 0 e
   reinicie a placa.
2. Feche Tera Term, `read_logs.ps1` e qualquer programa que ocupe a COM.
3. Posicione a placa no ponto inicial. A gravação começa assim que o script
   abre a porta, então combine antes o roteiro do movimento (por exemplo:
   5 s parado, movimento até a marca, pausa, volta à marca inicial).
4. Execute, com um nome de arquivo novo a cada ensaio:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\capture_binary.ps1 -Port COM3 -Seconds 45 -OutputPath captures\ensaio03.bin
```

5. Para o próximo ensaio, **reinicie a placa** antes (uma sessão por boot).

Para um ensaio de precisão, anote a distância real medida com trena e a altura.
Prefira 15–25 cm acima de uma superfície com textura: a ~0,9 m um degrau de
fluxo vale ~180 mm/s, e movimentos mais lentos que isso quase não aparecem
(ver a Seção 6.3 do documento de evolução).

Observações práticas:

- Se a porta não abrir (COM errada ou ocupada), o script deixa um `.bin`
  vazio e a próxima tentativa com o mesmo nome falha com "already exists".
  Use outro nome ou apague o vazio. Nesse caso não é preciso reiniciar a
  placa, porque o DTR nunca foi ativado.
- No PowerShell 5.1, os acentos das mensagens do script aparecem corrompidos
  ("sessA�o"). É só visual; os dados não são afetados.
- Guarde junto de cada `.bin` os arquivos `.bin.json` (metadados e SHA-256)
  e `.bin.firmware.json` (manifest), e anote o que foi feito fisicamente.

### Como a sessão funciona

O script prepara um arquivo novo (não sobrescreve), abre a COM com DTR baixo,
ativa DTR e lê bytes sem conversão de texto. Ao término baixa DTR e drena os
bytes já enfileirados. Grava também metadados e SHA-256; uma cópia do manifest
local é incluída quando disponível (confira se corresponde ao firmware gravado).
O caminho binário pode ser verificado sem hardware:

```powershell
.\tools\capture_binary.ps1 -SelfTest
```

O firmware não produz captura antes de DTR. No modo Capture, o estimador espera
a sessão e amostras novas dos três relatórios BNO e do MTF, iniciando do zero.
Isso evita usar estado integrado antes de abrir a COM. Há uma sessão por boot;
fechar/reabrir a porta não reinicia a captura. Reinicie a placa para novo ensaio.
O build normal mantém o comportamento anterior. A task de log textual não é
criada no modo Capture, liberando sua stack de 4 KiB no heap FreeRTOS.

O leitor `read_logs.ps1` continua exclusivo de texto. Arquivos vazios ou sem
registros decodificáveis fazem o decoder retornar erro. Se faltar uma amostra,
o replay mostra offset, sequência, tempo, motivo e quantos passos do prefixo
foram conferidos antes da perda. Isso não significa que o trecho restante foi
validado. Um término entre OBSERVATION e STEP é informado como incompleto.

Nos DIAG de tasks que não possuem o contador de resincronizações, o firmware
envia `UINT32_MAX` e o CSV deixa o campo vazio, em vez de sugerir zero.

## Limites

Gravar desde o início da sessão, sem mudar configuração ou resetar o estimador
durante o ensaio. Se houver reinício do STM32, iniciar outro arquivo. O PC
precisa guardar bytes binários sem transformação de texto.

O replay verifica a parte delimitada pelos registros recebidos, não consegue
provar que o arquivo contém o fim da sessão física. Não há marcador de término;
descarte após o último DIAG pode não aparecer nele. Não interpretar ausência
de um registro final como prova de ausência de perda na cauda da gravação.

A validade numérica não comprova sincronização física ou segurança de voo.
Continuam necessários ensaios de eixos, escala, atrasos e movimento combinado.
