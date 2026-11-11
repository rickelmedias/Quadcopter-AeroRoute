# Revisão de ~/logs_drone — 08/11/2026

## Cobertura e limites

Foram lidos os quatro documentos Markdown, a tabela dos 31 ensaios e todos os arquivos dos dois kits completos presentes na pasta: 180436 (2082 amostras) e 181108 (1698). As duas versões CSV têm o mesmo número de amostras em cada kit. O ZIP contém cópias idênticas dos arquivos correspondentes, verificado por SHA-256; não contém os dados completos dos outros 29 voos. Esses outros voos aparecem nesta revisão por meio da tabela e do histórico, não como novas análises de suas amostras.

Os registros não marcam quando a mão apoiou ou corrigiu o drone. Não é possível identificar uma planta de voo livre nem demonstrar estabilidade com esses dados. Os documentos de acompanhamento contêm interpretações que precisam ser distinguidas das medidas. As capturas USB brutas citadas neles não estão neste pacote.

## Achados confirmados

### Z: referência distante e predição de subida com altura quase parada

Ambos os kits têm W0=900, FF_TH=-35, FF_PHI=0, FF_PSI=50, XY=1 e compensação cruzada. Nenhum alcançou 15 cm. O pouso foi solicitado por timeout aos ~26 s; em 181108 houve failsafe durante esse pouso.

Recalculado nas amostras TAKEOFF, janela 5–15 s após o início de TAKEOFF:

| Kit | Mediana de altura | Tendência medida | Subida estimada média (-vz) | Saturação indicada |
|---|---:|---:|---:|---:|
| 180436 | 3,62 cm | +0,20 cm/s | +2,52 cm/s | 0% |
| 181108 | 3,42 cm | +0,08 cm/s | +2,57 cm/s | 0% |

Na janela 15–25 s: alturas medianas 4,12/4,13 cm; tendências +0,16/-0,05 cm/s; subida estimada +2,51/+2,45 cm/s. A tendência medida é regressão linear contra o tempo. Esse padrão persiste após a mudança de compensação angular do MTF.

A referência vertical já está no alvo enquanto a medida permanece baixa. O estimador de perturbação aumenta d_z de poucos rad/s para ~117/118 antes do pouso. Isso é compatível com erro de empuxo de base e adaptação lenta, mas apoio manual, contato e modelo incorreto também produzem esse padrão. Não concluir que basta aumentar W0: o valor equivalente inferido da saída não é medição de RPM/empuxo nem hover livre calibrado.

Próxima correção de controle deve investigar coerência entre os dois observadores verticais: o Kalman principal prediz com du líquido da compensação, enquanto o observador aumentado também estima perturbação. Medir suas velocidades e inovações antes de substituir estados ou alterar Q. Preservar LQG/Kalman; não introduzir ganho maior às cegas.

### Fluxo: pareamento cruzado já existe no código atual

`meas.c` agora tem FLOW_GYRO_XY e FLOW_GYRO_YX. Os kits usam os coeficientes diretos zerados e cruzados +1,45/-1,65, além de FLOW_SWAP=1. A suspeita de associação angular antiga descrita no plano anterior foi tratada por essa mudança; não reaplicar uma troca automaticamente.

Os coeficientes vieram de regressão durante movimento, com interferência manual. Correlação reduzida com giro não demonstra separação perfeita entre translação e rotação. Um braço físico produz velocidade omega × braço, independente da distância ao chão; absorvê-lo num coeficiente angular multiplicado pela distância só aproxima uma altura particular. Calibração multialtura e geometria explícita continuam relevantes.

FLOW_GATE e FLOW_CENS estão desligados. Fluxo válido a 100% significa aceitação, não exatidão ou amostras independentes. O driver aceita a última amostra até 250 ms de idade. Não há registro suficiente de idade e inovação para concluir que o sensor já atingiu seu melhor desempenho.

### Yaw e saída

Os mesmos parâmetros produziram respostas muito diferentes. Entre 15 e 25 s, o canal de yaw médio do du foi ~52 rad/s em 180436 e ~-5 em 181108. Reconstruído do comando final, esse canal foi ~203 e ~28 rad/s: FF e estimador são parcelas importantes, não apenas o ganho do LQR. Reconstrução usa a curva provisória W=1755×throttle e o padrão [-,+,-,+] na ordem u1..u4; não mede torque físico.

Não inferir causalidade por correlação entre x local e θ corporal quando o yaw muda muito. Projetar erro/velocidade no referencial da proa e decompor esforços. Tampouco reduzir os pesos de yaw apenas por haver deriva.

### Relatórios podem sugerir calibrações sem hover no alvo

Os `resumo.json` registram janelas de 19,9 e 18,6 s e sugerem parâmetros apesar de `reached=null`, erro vertical médio de ~11,5 cm e apoio manual. As janelas servem para descrever os dados, não para declarar hover bem-sucedido. Em especial, FF_PSI sugerido=168 num kit e=19 no outro não é uma calibração repetível. Não aplicar automaticamente essas sugestões.

## Melhorias de firmware implementadas nesta revisão

1. **CTL_PSI_MAX cobre o pedido completo.** Antes, somente du do LQG era limitado e FF/estimador podiam ultrapassar o limite depois. Agora a projeção de yaw limita LQG+FF+estimador antes da conversão em throttle, preservando os outros canais nessa etapa. A limitação também marca saturação, para congelar a adaptação do estimador e evitar crescimento oculto contra o limite. A predição usa o comando modificado. O limite continua desligado por padrão (0) e estava desligado nos dois voos; portanto essa falha não é apontada como causa deles. Não ativar um valor arbitrário como se resolvesse yaw.
2. **Diagnóstico vertical nos textos da Pi**, a cada relatório de perturbação: `Z cm h=... ref=... vz=... vd=...`. h é altura filtrada sobre o chão do firmware; ref é a referência sobre esse mesmo chão, em cm. vz é velocidade do Kalman principal; vd é velocidade do observador aumentado de perturbação, ambas em cm/s NED (+ desce). O sinal e as unidades são essenciais. As velocidades são estimativas, não sensores independentes. A mensagem usa a fila STATUSTEXT existente e ficará em textos.txt; não muda o protocolo nem exige USB durante o voo.

Arquivos alterados nesta revisão: `App/Src/control/flight_ctrl.c`, comentários de `App/Inc/control/ctrl_params.h` e `App/Inc/flight_config.h`, e teste adicional em `tests/test_lqg_dist.c`. Preservadas as mudanças anteriores do worktree.

## Validação e implantação

`sh tests/run.sh`: suítes C com ASan/UBSan passaram, incluindo equivalência com Simulink, teste de yaw composto preservando os outros canais e conferência das tabelas/CRC. Suíte Python/Pi pulada pelo script por falta das dependências locais. `python3 tools/build.py`: Release compilado, 92 fontes, ELF/BIN/HEX e manifest em build/Release. `git diff --check` sem erros.

**Não foi gravado o STM32 nem alterada a Pi nesta revisão.** A alteração local não prova que o drone já estabiliza. Não houve ensaio físico novo.

## Próximas decisões

Prioridade: explicar a divergência vertical usando o diagnóstico dos dois observadores, avaliar yaw com esforço total e só então comparar candidatos de Qxy com atraso de motor e incerteza da planta. XY deve trabalhar junto com a subida de Z após desprendimento; voar com XY desligado recria o afastamento. Conferir e registrar a origem XY e o instante real de ativação (nos textos), que não são necessariamente o `lifted` calculado pela Pi.

Para fechar a validação física serão necessários trechos sem apoio manual e referência independente de altura/posição, em configuração de ensaio que dispense segurar hélices em operação. Os dados atuais permitem corrigir caminhos do firmware e formular hipóteses; não permitem garantir erro de posição de 5–10 cm.
