# Continuidade: drone STM32 + Raspberry Pi

**Última revisão do pacote `~/logs_drone/`:** ver [ANALISE_LOGS_DRONE.md](ANALISE_LOGS_DRONE.md). Dois kits completos (180436/181108), histórico tabular de 31 e ZIP duplicado. Compensação angular cruzada já implementada por outro agente; não reaplicar o diagnóstico antigo de FLOW_SWAP sem considerar FLOW_GYRO_XY/YX. Foram corrigidos localmente o alcance e a sinalização de CTL_PSI_MAX e adicionados textos de diagnóstico dos dois observadores verticais. Testes C e build Release passaram; firmware não gravado nesta revisão.

**Atualização posterior:** a sessão prosseguiu com outro agente, que adicionou FLOW_SWAP, opções de tratamento de fluxo, rampa coletiva e limite de yaw. O estado atual e o plano de revisão estão em [PLANO_XY.md](PLANO_XY.md). Os kits 172447–173119 estão locais. A análise posterior encontrou divergência entre subida estimada e altura observada, e suspeita forte na compensação de giroscópio após FLOW_SWAP. O próximo passo manual descrito no fim deste histórico ficou anterior a esses avanços; não reiniciar a sessão por ele automaticamente. Nenhum dos quatro kits mais recentes comprova hover livre no alvo de 15 cm.

Registro preparado em 08/11/2026 a partir da conversa, do repositório e dos kits locais. É um resumo das decisões e das evidências; hipóteses não são causas comprovadas. Preserve o trabalho não commitado e os dados do usuário.

## Objetivo e contexto

Fazer o drone de 551 g decolar, manter atitude e altura e, depois, segurar x/y em (0,0), usando o controle LQG e o filtro de Kalman derivados do MATLAB/Simulink. O usuário quer manter esse conceito para o TCC e aceita ensaios físicos progressivos orientados pelos logs. A planta real, os motores e o empuxo não foram identificados fielmente no modelo. O drone já saiu do chão inesperadamente, girou e bateu antes das correções; não existe auditoria suficiente para determinar a causa desse evento.

Branch atual: `hover-lqg-pi`, HEAD `af6034e` no momento da análise. Comparação de interesse: `controle-lqg-posicao`, onde o usuário relata atitude boa em alguns testes, mas deriva horizontal. Existem também failsafes de inclinação em logs antigos. O worktree contém alterações não commitadas em firmware e ferramentas, mais `tools/logs/` com dados do usuário. Não descartá-los.

RPi: `r1ddax@192.168.87.50`; ferramenta `~/drone/dronectl`; kits `~/real_log_kit/`. Copiar ao PC com `sh tools/pi/buscar_real_log_kit.sh r1ddax@192.168.87.50`. Os kits locais ficam em `real_log_kit/`, ignorado pelo Git. Foto física: `../drone.jpeg`; a frente é a direita da foto. Massa total informada: 551 g. Em hover ideal, o empuxo médio requerido é aproximadamente 1,35 N ou 138 g-força por motor; isso não determina o percentual DShot, a distribuição de massa ou o empuxo individual.

## Alterações já realizadas

O firmware foi compilado e gravado por DFU anteriormente, com verificação reportada e identificação `9db4d15724a3b82fe544607c`. Não há comprovação automática em cada kit de que o binário físico corresponda ao HEAD atual. As alterações locais principais são:

- `App/Src/motors.c`: watchdog de 100 ms para atualização de comando de voo; corta a posse dos motores se os comandos pararem.
- `App/Src/control/flight_ctrl.c`: em ARMED o comando de motor é zero; a marcha lenta só entra em TAKEOFF. A planta simulada recebe o comando físico completo, incluindo feedforward.
- `App/Inc/flight_config.h`: padrões atuais `CTL_W0=1050`, `CTL_XY=0`, `CTL_FF_TH=-20`, `CTL_FF_PHI=-40`, `CTL_FF_PSI=0`, `MOT_MAX=0,95`. O usuário sobrescreveu FF e XY nos ensaios descritos abaixo. O fato de ARMED enviar zero reduz uma possibilidade de movimento inesperado, mas não prova a causa do acidente anterior. Ruídos de inicialização dos ESCs também não foram identificados conclusivamente.
- As ferramentas Pi geram `real_log.csv` separado por ponto e vírgula, `gains.json`, `telemetria.csv`, `meta.json`, `textos.txt`, `resumo.json` e linha do tempo. O CSV inclui comandos físicos M1–M4, medidas disponíveis, estados, referências, du, flags, parâmetros efetivos e ganhos. Os percentuais são comandos, não RPM, DShot efetivamente medido ou empuxo. Os parâmetros são fotografados antes de armar. Quando XY está desligado, velocidades e posições horizontais da malha são mascaradas e não permitem quantificar a deriva real.
- `tools/pi/dronectl/hover_test.py`: após os últimos ensaios, Ctrl+C durante TAKEOFF com amostra recente e válida, altura medida inferior a 10 cm e estimada inferior a 12 cm desarma imediatamente. Acima disso, o primeiro Ctrl+C pede pouso e o segundo corta. Foi implantado na Pi. Isso pode fazer o drone cair de pequena altura; não garante evitar colisão. O STM32 não foi regravado nessa alteração da Pi.
- A suíte de hover na Pi passou com 21 testes antes do último ajuste de limite; os 9 testes direcionados passaram localmente depois do ajuste. O caminho de interrupção perto do chão foi testado sem motores. O firmware e a Pi têm alterações locais adicionais anteriores: consultar `git diff` antes de editar.

## Referenciais e limites da evidência

O usuário confirmou que a frente é a direita da foto e que fiação e montagem não foram alteradas. Segundo o mapeamento adotado no código, M4 e M2 ficam na frente; M3 e M1 atrás. Os quatro motores giraram no ensaio físico. Isso não verifica empuxo igual, RPM ou correspondência física exata de cada canal. Um ensaio manual com bateria desconectada está em `real_log_kit/eixos_manual.csv`: frente levantada aumentou θ, traseira levantada diminuiu θ; esquerda/direita produziram sinais coerentes em φ. No primeiro voo, derivadas angulares e giroscópios apresentaram alta correlação. Não se encontrou inversão evidente de pitch nas medidas.

As matrizes do modelo LQG e a rampa de decolagem de 2 s coincidem com as da branch de posição comparada. Há diferenças de parâmetros, montagem física, integração Pi e outros trechos; os logs antigos não permitem provar a configuração e o binário exatos de todos os voos. A simulação usa planta ideal e não valida o drone físico. `CTL_W0=800` em ensaio anterior não fez o drone levantar; 1050 levantou, mas esse número sozinho não explica estabilidade. A curva `MOT_W_AT_0/100` é provisória.

## Sequência dos ensaios principais

Antes dos voos recentes, a simulação e o ensaio a seco foram executados com o STM32 e a Pi alimentados por USB e sem bateria dos ESCs. Com FF padrão, a simulação teve transitório de atitude, depois atingiu hover. O ensaio a seco usa sensores reais, mas não deve girar motores; comandos calculados não comprovam resposta física.

| Kit local | Configuração e resultado |
| --- | --- |
| `hover_20261108_140136` | Real, W0 1050, FF_TH/PHI/PSI 0, XY 0. Nariz subiu; θ chegou a aproximadamente 20° e houve failsafe em 3,07 s. Pico q próximo de 81°/s. O controle pediu mais aos motores traseiros, direção corretiva esperada, sem saturação indicada. |
| `hover_20261108_143946` | Real, W0 1050, FF_TH -20, PHI/PSI 0, XY 0. θ chegou perto de 19° e começou a recuperar; o drone deslocou-se para trás. Ctrl+C iniciou pouso em 3,71 s, com altura medida de cerca de 2,6 cm; failsafe ocorreu em 3,98 s. A ação do usuário foi para interromper o afastamento. |
| `hover_20261108_144939` | Real, W0 1050, FF_TH -30, PHI/PSI 0, XY 0. θ chegou a cerca de 13,5° e voltou perto de zero antes da interrupção; o usuário disse que o drone foi para o mesmo lado, quase reto. Houve rotação de yaw de aproximadamente -19°. Ctrl+C pediu pouso em 3,80 s com altura medida/estimada aproximadamente 8,4/8,5 cm; failsafe ocorreu em 4,41 s durante o pouso. O limite da Pi foi ampliado posteriormente para tratar esse caso. |
| `hover_20261108_145210` | Simulação no STM32, W0 1050, FF_TH -30, PHI/PSI 0, XY 1. A Pi pediu pouso após estimativa ultrapassar a cerca de 80 cm, antes de alcançar 20 cm de altura. A simulação não prova inversão da malha real: o FF causou inclinação no modelo e a planta acumulou velocidade horizontal antes de XY ser ativado; as medidas horizontais ficam mascaradas até a ativação. |

O efeito aparente de FF_TH -30 foi reduzir o pico de pitch em relação aos dois ensaios anteriores, mas são poucos voos e não isolam todas as diferenças físicas. FF_TH negativo favorece a traseira em relação à frente. Em um instante do voo com -20, os traseiros já recebiam cerca de 9 pontos percentuais de comando a mais que os dianteiros; isso não demonstra o empuxo efetivo. Não há base suficiente para instalar ganhos individuais nos motores, nem para aumentar continuamente o FF só porque o drone manteve velocidade para trás. O drone pode manter velocidade horizontal depois de voltar ao nível quando XY está desligado.

O controle de posição integrado depende de fluxo óptico, sem posição absoluta; deriva, sinal, escala, compensação de giro e confiabilidade perto do piso precisam ser conferidos. O histórico com XY ligado inclui falhas, e há guinada não explicada. Não assumir que o simulador represente corretamente contato com o chão, atraso dos ESCs, empuxo ou desequilíbrio real.

## Próximo passo combinado com o usuário

Foi pedido um ensaio manual do fluxo óptico, com bateria dos ESCs desconectada, Pi/STM32 alimentados por USB e hélices podendo permanecer instaladas. **Não usar `--motors`**:

```sh
~/drone/dronectl hover-test --height 20 --time 30 \
  --set CTL_XY=1,CTL_FF_TH=-30,CTL_FF_PHI=0,CTL_FF_PSI=0 \
  --label "fluxo-xy-manual"
```

Depois de “decolando”, o usuário deve levantar o drone nivelado aproximadamente 15 cm; ao aparecer `XY: ligado`, deslocá-lo cerca de 20 cm para frente, parar 2 s, deslocar para trás e parar, depois direita e esquerda, sem girar. Buscar o kit pela Pi e comparar `y_vx_m_s`, `y_vy_m_s`, flags e tempo com a sequência física. A Pi pode exibir comandos calculados no modo a seco, mas os ESCs devem estar sem bateria. Se XY não engajar ou o fluxo não for válido, o ensaio não comprova os sinais.

Após esse teste, investigar a resposta de XY e a guinada usando sinais e logs antes de recomendar voo com XY ativo. Não afirmar que os 80 cm do simulador provam uma inversão física. O usuário valoriza instruções claras, resultados concretos e evitar verificações físicas repetidas sem propósito.
