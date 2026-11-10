/**
 * @file capture.h
 * @brief Captura binária, evento a evento, de tudo o que o estimador vê, para
 *        replay fora da placa (formato 3).
 *
 * **Por que eventos e não fotografias.** O projeto óbvio (gravar "o BNO mais
 * recente" junto de cada frame do MTF) congelaria na gravação a associação
 * temporal que o firmware faz hoje. Essa associação é justamente uma das
 * coisas sob investigação (o giroscópio usado é só a amostra mais recente,
 * não uma alinhada ao frame de fluxo). Uma gravação assim nunca serviria para
 * avaliar um alinhamento melhor e ainda descartaria em silêncio os relatórios
 * do BNO que chegaram entre dois frames. Por isso cada relatório de sensor é
 * um registro próprio, com seus carimbos de tempo, e registros separados
 * dizem o que a task observou e o que o estimador fez.
 *
 * **Regras do formato.** Os registros são escritos campo a campo, em
 * little-endian. Nenhuma struct C é entregue ao transporte: padding e
 * representação seriam diferentes entre o firmware e quem lê o arquivo. Todo
 * registro tem byte de sincronismo, tipo, tamanho, número de sequência e
 * checksum, para que o leitor consiga se ressincronizar após perdas e
 * distinguir "nada aconteceu" de "dados se perderam", distinção da qual o
 * replay depende.
 *
 * Leitores: tools/decode_capture.py e tools/replay.c. Procedimento e
 * resultados: docs/captura_replay_v3.md.
 */
#ifndef CAPTURE_H
#define CAPTURE_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/** Byte que abre todo registro. */
#define CAP_SYNC          0xA5U

/**
 * @brief Versão do formato gravada na SESSION.
 *
 * A versão 3 acrescenta OBSERVATION: cada fotografia que a task realmente
 * observou, inclusive intervalos rejeitados, porque os eventos dos sensores
 * sozinhos não reconstroem o escalonamento da task. Nos registros BNO vai o
 * carimbo de medição do SH-2; o t_ms de todo registro é lido antes de o
 * produtor esperar pela trava; a SESSION traz a identidade do firmware e os
 * parâmetros do filtro; o STEP traz o estimador RAW e se a predição teve
 * aceleração real; e o DIAG separa os tipos de perda. Um leitor deve recusar
 * uma versão que não conhece em vez de interpretá-la errado.
 */
#define CAP_FORMAT_VERSION 3U

/**
 * @brief Identifica o binário que produziu a gravação.
 *
 * O tools/build.py define este valor com o prefixo do hash dos fontes (ver
 * manifest.json). Sem ele, usa data e hora da compilação. Não substitui um
 * controle de versão (este projeto não tem), mas distingue duas gravações,
 * que é a pergunta que surge quando um replay discorda de números anotados.
 */
#ifndef CAP_FIRMWARE_ID
#define CAP_FIRMWARE_ID   (__DATE__ " " __TIME__)
#endif
/** Tamanho máximo gravado para ::CAP_FIRMWARE_ID; o excesso é truncado. */
#define CAP_FIRMWARE_ID_MAX 32U

/** Bytes do cabeçalho: sync(1) tipo(1) tamanho(1) seq(2) t_ms(4). */
#define CAP_HEADER_BYTES  9U
/** Bytes do checksum, gravado depois da carga útil. */
#define CAP_CRC_BYTES     1U
/** Bytes fixos de todo registro (cabeçalho + checksum). */
#define CAP_OVERHEAD      (CAP_HEADER_BYTES + CAP_CRC_BYTES)
/** Maior carga útil; o campo tamanho conta só a carga e tem 1 byte. */
#define CAP_MAX_PAYLOAD   255U
/** Maior registro possível, em bytes. */
#define CAP_MAX_RECORD    (CAP_OVERHEAD + CAP_MAX_PAYLOAD)
/** Valor de clock_resyncs no DIAG de tasks que não têm esse contador. O
    decoder deixa o campo vazio em vez de sugerir zero. */
#define CAP_CLOCK_RESYNCS_UNKNOWN UINT32_MAX

/**
 * @brief Tipos de registro e o layout da carga útil de cada um.
 *
 * Tipos de campo: u8/u16/u32/u64 sem sinal, i16 com sinal, f32 IEEE-754 e
 * str = 1 byte de tamanho seguido dos caracteres.
 */
typedef enum {
  /** Versão, identidade do firmware e configuração. Carga: versão u8,
      firmware str, flow_scale_x f32, flow_scale_y f32, gyro_sign_x f32,
      gyro_sign_y f32, deadband f32, flow_lsb f32, max_frame_gap_ms u32,
      max_correction_age_ms u32, ekf accel_noise f32, flow_noise f32,
      gate_sigma f32. */
  CAP_TYPE_SESSION = 1,
  /** Um frame do MTF-01 exatamente como decodificado (21 bytes): flow_x i16,
      flow_y i16, distance_mm u32, quality u8, distance_status u8, strength u8,
      precision u8, flags u8 (bit0 distância válida, bit1 fluxo válido),
      sensor_ms u32, frames u32. */
  CAP_TYPE_MTF     = 2,
  /** Um relatório do BNO08x (35 bytes): tipo u8 (::cap_bno_report_t),
      accuracy u8, sequence u8, 4 x f32, count u32, sensor_us u64,
      delay_us u32. */
  CAP_TYPE_BNO     = 3,
  /** Um passo do estimador (53 bytes): IDs usados (mtf, rotação, giro,
      aceleração) 4 x u32, dt f32, flags u8 (bit0 FUSED aceitou, bit1 EKF
      aceitou, bit2 IMU online), raw x/y, fused vx/vy, ekf vx/vy, ekf x/y
      (8 x f32). */
  CAP_TYPE_STEP    = 4,
  /** Perdas e saúde (29 bytes ou mais): dropped_full u32, dropped_oversize
      u32, dropped_contention u32, clock_resyncs u32, menor heap livre u32,
      heap livre u32, margem de pilha em bytes u32, nome da task str. */
  CAP_TYPE_DIAG    = 5,
  /** Fotografia do MTF observada pela task, inclusive intervalos rejeitados
      (9 bytes): frames u32, previous_sensor_ms u32, flags u8 (bit0 havia
      frame anterior, bit1 intervalo utilizável, bit2 distância válida, bit3
      fluxo válido). */
  CAP_TYPE_OBSERVATION = 6
} cap_type_t;

/**
 * @brief Qual relatório do BNO um registro ::CAP_TYPE_BNO carrega.
 *
 * Numeração própria, e não os IDs do SH-2, para que o formato do arquivo não
 * mude de significado se a biblioteca incorporada renumerar algo.
 */
typedef enum {
  /** Vetor de rotação: 4 valores w,x,y,z. */
  CAP_BNO_ROTATION = 1,
  /** Giroscópio calibrado: x,y,z em rad/s; o quarto valor é 0. */
  CAP_BNO_GYRO     = 2,
  /** Aceleração linear: x,y,z em m/s^2; o quarto valor é 0. */
  CAP_BNO_ACCEL    = 3
} cap_bno_report_t;

/* ---- escrita de bytes -------------------------------------------------- */

/**
 * @brief Montador de um registro sobre um buffer fornecido por quem chama.
 */
typedef struct {
  /** Buffer de destino. */
  uint8_t *buf;
  /** Capacidade do buffer, em bytes. */
  uint16_t cap;
  /** Bytes já escritos. */
  uint16_t len;
  /** Fica verdadeiro quando uma escrita não coube; o registro fica inutilizável. */
  bool overflow;
} cap_writer_t;

/**
 * @brief Prepara o montador para escrever em @p buf.
 *
 * @param[out] w    Montador.
 * @param[in]  buf  Buffer de destino.
 * @param[in]  cap  Capacidade do buffer.
 */
static inline void cap_init(cap_writer_t *w, uint8_t *buf, uint16_t cap) {
  w->buf = buf; w->cap = cap; w->len = 0; w->overflow = false;
}

/**
 * @brief Acrescenta um u8. Se não couber, marca @c overflow.
 * @param[in,out] w  Montador.
 * @param[in]     v  Valor.
 */
static inline void cap_u8(cap_writer_t *w, uint8_t v) {
  if (w->len + 1U > w->cap) { w->overflow = true; return; }
  w->buf[w->len++] = v;
}

/**
 * @brief Acrescenta um u16 em little-endian.
 * @param[in,out] w  Montador.
 * @param[in]     v  Valor.
 */
static inline void cap_u16(cap_writer_t *w, uint16_t v) {
  cap_u8(w, (uint8_t)(v & 0xFFU));
  cap_u8(w, (uint8_t)((v >> 8) & 0xFFU));
}

/**
 * @brief Acrescenta um u32 em little-endian.
 * @param[in,out] w  Montador.
 * @param[in]     v  Valor.
 */
static inline void cap_u32(cap_writer_t *w, uint32_t v) {
  cap_u16(w, (uint16_t)(v & 0xFFFFU));
  cap_u16(w, (uint16_t)((v >> 16) & 0xFFFFU));
}

/**
 * @brief Acrescenta um i16 em little-endian (complemento de dois).
 * @param[in,out] w  Montador.
 * @param[in]     v  Valor.
 */
static inline void cap_i16(cap_writer_t *w, int16_t v) { cap_u16(w, (uint16_t)v); }

/**
 * @brief Acrescenta um u64 em little-endian.
 * @param[in,out] w  Montador.
 * @param[in]     v  Valor.
 */
static inline void cap_u64(cap_writer_t *w, uint64_t v) {
  cap_u32(w, (uint32_t)(v & 0xFFFFFFFFU));
  cap_u32(w, (uint32_t)((v >> 32) & 0xFFFFFFFFU));
}

/**
 * @brief Acrescenta uma string precedida do tamanho.
 *
 * O prefixo de tamanho evita que o leitor precise adivinhar onde a string
 * termina. Strings longas são truncadas em vez de estourar.
 *
 * @param[in,out] w    Montador.
 * @param[in]     s    String terminada em zero.
 * @param[in]     max  Máximo de caracteres gravados.
 */
static inline void cap_str(cap_writer_t *w, const char *s, uint8_t max) {
  uint8_t n = 0;
  while (s[n] != '\0' && n < max) ++n;
  cap_u8(w, n);
  for (uint8_t i = 0; i < n; ++i) cap_u8(w, (uint8_t)s[i]);
}

/**
 * @brief Acrescenta um f32 (IEEE-754 precisão simples) em little-endian.
 *
 * O Cortex-M4F e todo PC que lê esses arquivos usam essa representação. A
 * cópia é feita com memcpy, e não com conversão de ponteiro, para evitar
 * problemas de aliasing. O leitor deve decodificar do mesmo jeito, sem
 * assumir o formato de float da própria máquina.
 *
 * @param[in,out] w  Montador.
 * @param[in]     v  Valor.
 */
static inline void cap_f32(cap_writer_t *w, float v) {
  uint32_t bits;
  memcpy(&bits, &v, sizeof bits);
  cap_u32(w, bits);
}

/* ---- checksum ---------------------------------------------------------- */

/**
 * @brief Calcula o CRC-8 (polinômio 0x07, valor inicial 0xFF).
 *
 * Sem tabela: roda a cada registro num microcontrolador pequeno, e o laço
 * custa menos que a pressão de cache de uma tabela. Basta para detectar a
 * corrupção que um fluxo USB quase sempre confiável produz; detecta, não
 * corrige.
 *
 * @param[in] data  Bytes a verificar.
 * @param[in] len   Quantidade de bytes.
 * @return O CRC-8.
 */
static inline uint8_t cap_crc8(const uint8_t *data, uint16_t len) {
  uint8_t crc = 0xFFU;
  for (uint16_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8U; ++bit) {
      crc = (uint8_t)((crc & 0x80U) ? (((unsigned)crc << 1) ^ 0x07U)
                                    : ((unsigned)crc << 1));
    }
  }
  return crc;
}

/* ---- enquadramento ----------------------------------------------------- */

/**
 * @brief Abre um registro e reserva o cabeçalho.
 *
 * Os campos da carga são acrescentados com as funções cap_*, e depois
 * cap_finish() preenche o tamanho e o checksum.
 *
 * @param[out] w     Montador.
 * @param[in]  buf   Buffer de destino (use ::CAP_MAX_RECORD bytes).
 * @param[in]  cap   Capacidade do buffer.
 * @param[in]  type  Tipo do registro.
 * @param[in]  seq   Número de sequência.
 * @param[in]  t_ms  Tick do STM32 em que o evento chegou, em ms.
 */
static inline void cap_begin(cap_writer_t *w, uint8_t *buf, uint16_t cap,
                             cap_type_t type, uint16_t seq, uint32_t t_ms) {
  cap_init(w, buf, cap);
  cap_u8(w, CAP_SYNC);
  cap_u8(w, (uint8_t)type);
  cap_u8(w, 0U);        /* tamanho, preenchido em cap_finish */
  cap_u16(w, seq);
  cap_u32(w, t_ms);
}

/**
 * @brief Fecha o registro: grava o tamanho e acrescenta o checksum.
 *
 * @param[in,out] w  Montador.
 * @return Tamanho total do registro em bytes, ou 0 se ele estourou o buffer
 *         ou passou do que o campo tamanho consegue expressar. Com 0, quem
 *         chama deve descartar o registro em vez de emitir algo ilegível.
 */
static inline uint16_t cap_finish(cap_writer_t *w) {
  if (w->overflow) return 0;
  const uint16_t payload = (uint16_t)(w->len - CAP_HEADER_BYTES);
  if (payload > CAP_MAX_PAYLOAD) return 0;
  w->buf[2] = (uint8_t)payload;
  const uint8_t crc = cap_crc8(w->buf, w->len);
  cap_u8(w, crc);
  if (w->overflow) return 0;
  return w->len;
}

/* ---- API das tasks ----------------------------------------------------- */

/*
 * Todas as funções abaixo só fazem algo quando CAPTURE_ENABLED está ligado
 * (build Capture; ver sensor_log.h). No build normal são funções vazias.
 *
 * Os produtores (capture_mtf, capture_bno, capture_step, capture_diag e
 * capture_observation) montam um registro e o colocam na fila, esperando no
 * máximo dois ticks pela trava de escrita: espera limitada, nunca ilimitada,
 * para que uma task de aquisição nunca seja cadenciada pela USB. Todos leem o
 * relógio na entrada, ANTES de disputar a trava, para que o tempo gravado seja
 * a chegada do dado e não o momento em que a captura conseguiu atendê-lo. Nada
 * é gravado fora de uma sessão ativa (ver capture_is_active()).
 */

/**
 * @brief Cria a fila, a trava e a task de captura.
 *
 * A task espera o PC ativar o DTR, então enfileira a SESSION e só depois
 * libera os produtores. Só é permitida uma sessão por boot.
 *
 * @return false se a fila, a trava ou a task não puderam ser criadas. No
 *         build normal retorna sempre true.
 */
bool capture_start(void);

/**
 * @brief Diz se os produtores podem gravar agora.
 *
 * @return true só depois que a SESSION foi enfileirada na primeira abertura
 *         da porta (DTR) deste boot, e enquanto o DTR continua ativo na mesma
 *         conexão. No build normal retorna sempre false.
 */
bool capture_is_active(void);

/**
 * @brief Grava um frame do MTF-01 exatamente como decodificado.
 *
 * Chamada pela task do MTF para cada frame válido, inclusive os que a task de
 * deslocamento não chega a observar.
 *
 * @param[in] flow_x           Fluxo X, em cm/s a 1 m.
 * @param[in] flow_y           Fluxo Y, em cm/s a 1 m.
 * @param[in] distance_mm      Distância, em mm.
 * @param[in] quality          Qualidade da imagem.
 * @param[in] distance_status  Estado da distância.
 * @param[in] strength         Intensidade do sinal do telêmetro.
 * @param[in] precision        Precisão do telêmetro.
 * @param[in] range_valid      Distância válida segundo o parser.
 * @param[in] flow_valid       Fluxo válido segundo o parser.
 * @param[in] sensor_ms        Relógio do próprio sensor, em ms. Fica separado
 *                             do tick de recepção: a relação entre os dois é
 *                             justamente o que o replay precisa estudar.
 * @param[in] frames           Contador de frames, usado como ID do frame.
 */
void capture_mtf(int16_t flow_x, int16_t flow_y, uint32_t distance_mm,
                 uint8_t quality, uint8_t distance_status, uint8_t strength,
                 uint8_t precision, bool range_valid, bool flow_valid,
                 uint32_t sensor_ms, uint32_t frames);

/**
 * @brief Grava um relatório do BNO08x.
 *
 * O t_ms do registro é quando o relatório chegou; @p sensor_us e @p delay_us
 * vêm do evento SH-2 e dizem quando o hub mediu. Manter os três separados é o
 * objetivo: relatórios entregues no mesmo lote têm a mesma chegada, mas não o
 * mesmo instante de medição, e decidir qual amostra de giro combina com qual
 * frame de fluxo é justamente a pergunta que esta captura existe para
 * responder.
 *
 * @param[in] report     Tipo do relatório.
 * @param[in] v          Quatro valores; giro e aceleração usam 0 no quarto.
 * @param[in] accuracy   Acurácia 0..3 informada pelo sensor.
 * @param[in] sequence   Sequência informada pelo sensor.
 * @param[in] count      Contador do relatório, usado como ID da amostra.
 * @param[in] sensor_us  Instante da medição no relógio do hub, em us.
 * @param[in] delay_us   Atraso entre a medição e a entrega, em us.
 */
void capture_bno(cap_bno_report_t report, const float v[4], uint8_t accuracy,
                 uint8_t sequence, uint32_t count,
                 uint64_t sensor_us, uint32_t delay_us);

/**
 * @brief Grava um passo do estimador: o que ele usou, o que decidiu e o que
 *        calculou.
 *
 * @param[in] used_mtf_frames          ID do frame MTF usado.
 * @param[in] used_bno_rotation_count  ID da amostra de rotação usada.
 * @param[in] used_bno_gyro_count      ID da amostra de giro usada.
 * @param[in] used_bno_accel_count     ID da amostra de aceleração usada.
 * @param[in] dt_s                     Intervalo integrado, em s.
 * @param[in] fused_accepted           O FUSED aceitou o frame.
 * @param[in] ekf_accepted             O EKF aceitou a correção.
 * @param[in] imu_online               Se a predição usou aceleração real ou
 *                                     seguiu com zero. É gravado porque não dá
 *                                     para deduzir dos valores: um relatório
 *                                     velho parece igual a um novo.
 * @param[in] raw_px                   Posição X do RAW, em m.
 * @param[in] raw_py                   Posição Y do RAW, em m.
 * @param[in] fused_vx                 Velocidade X do FUSED, em m/s.
 * @param[in] fused_vy                 Velocidade Y do FUSED, em m/s.
 * @param[in] ekf_vx                   Velocidade X do EKF, em m/s.
 * @param[in] ekf_vy                   Velocidade Y do EKF, em m/s.
 * @param[in] ekf_px                   Posição X do EKF, em m.
 * @param[in] ekf_py                   Posição Y do EKF, em m.
 */
void capture_step(uint32_t used_mtf_frames, uint32_t used_bno_rotation_count,
                  uint32_t used_bno_gyro_count, uint32_t used_bno_accel_count,
                  float dt_s, bool fused_accepted, bool ekf_accepted,
                  bool imu_online, float raw_px, float raw_py,
                  float fused_vx, float fused_vy, float ekf_vx, float ekf_vy,
                  float ekf_px, float ekf_py);

/**
 * @brief Grava um registro de perdas e saúde.
 *
 * Inclui os três contadores de descarte, o menor heap livre, o heap livre
 * atual, a margem de pilha e o nome da task que chamou. Assim o custo da
 * captura é medido no sistema rodando, e não estimado.
 *
 * @param[in] clock_resyncs  Contador de ressincronizações de relógio da task
 *                           de deslocamento, ou ::CAP_CLOCK_RESYNCS_UNKNOWN
 *                           nas tasks que não o possuem.
 */
void capture_diag(uint32_t clock_resyncs);

/**
 * @brief Chama capture_diag() se já passou 1 s desde a última vez.
 *
 * Cada task produtora chama isto no seu laço, para que a margem de pilha de
 * cada uma seja medida. Uma task parada ou bloqueada não consegue emitir o
 * próprio DIAG.
 *
 * @param[in,out] last_ms        Tick do último DIAG desta task; atualizado
 *                               quando um novo é emitido.
 * @param[in]     clock_resyncs  Repassado para capture_diag().
 */
void capture_diag_poll(uint32_t *last_ms, uint32_t clock_resyncs);

/**
 * @brief Grava uma fotografia do MTF que a task de deslocamento observou.
 *
 * Chamada para todo frame novo visto pela task, inclusive o primeiro e os
 * rejeitados por intervalo inválido. É o que permite ao replay reconstruir o
 * @c dt exatamente como a task calculou.
 *
 * @param[in] frames              ID do frame observado.
 * @param[in] previous_sensor_ms  Carimbo de tempo do frame observado anterior.
 * @param[in] have_previous       Havia um frame observado anterior.
 * @param[in] gap_usable          O intervalo foi considerado utilizável.
 * @param[in] range_valid         Distância válida na fotografia (já considera
 *                                a idade do dado).
 * @param[in] flow_valid          Fluxo válido na fotografia.
 */
void capture_observation(uint32_t frames, uint32_t previous_sensor_ms,
                         bool have_previous, bool gap_usable,
                         bool range_valid, bool flow_valid);

/**
 * @brief Total de registros perdidos, de todos os tipos.
 *
 * O detalhamento por tipo está no DIAG. Fila cheia ou registro grande demais
 * consomem o número de sequência e deixam um buraco visível; um registro
 * perdido por disputa da trava não deixa buraco, só é contado. Um leitor não
 * deve assumir que os dois casos são iguais.
 *
 * @return Soma dos três contadores de descarte.
 */
uint32_t capture_dropped(void);
#endif
