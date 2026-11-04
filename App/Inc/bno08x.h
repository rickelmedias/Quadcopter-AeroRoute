/**
 * @file bno08x.h
 * @brief Driver da IMU BNO08x no I2C2, sobre a pilha SH-2 da CEVA.
 *
 * A task deste módulo é dona do barramento I2C2 e dos pinos RST e INT; nada
 * mais pode mexer neles enquanto ela roda. A pilha SH-2 fica em App/Src/sh2,
 * sem modificações.
 */
#ifndef BNO08X_H
#define BNO08X_H
#include "attitude.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Motivo pelo qual o driver derrubou o sensor pela última vez.
 *
 * O valor é mantido entre as tentativas de reconexão, para que um log lido
 * depois ainda mostre o que deu errado.
 */
typedef enum {
  /** Nenhuma falha registrada. */
  BNO_FAULT_NONE = 0,
  /** Nenhum ACK no I2C em nenhum dos endereços dentro da janela de sondagem. */
  BNO_FAULT_NO_DEVICE,
  /** Uma transferência I2C falhou no meio da sessão. */
  BNO_FAULT_TRANSPORT,
  /** Enquadramento SHTP rejeitado: tamanho inválido, cabeçalho divergente ou canal fora da faixa. */
  BNO_FAULT_PROTOCOL,
  /** A sessão abriu, mas a notificação de reset do SH-2 nunca chegou. */
  BNO_FAULT_NO_RESET,
  /** Falha ao ler o Product ID. */
  BNO_FAULT_IDENTITY,
  /** Falha ao habilitar os relatórios. */
  BNO_FAULT_CONFIG,
  /** Os relatórios pararam de chegar com o sensor online. */
  BNO_FAULT_STALLED
} bno_fault_t;

/**
 * @brief Saúde de um tipo de relatório do BNO (rotação, giro ou aceleração).
 */
typedef struct {
  /** Relatórios recebidos e aceitos desde o boot. */
  uint32_t count;
  /** Relatórios que o sensor numerou mas nunca chegaram. É o jeito mais barato
      de flagrar um barramento que perde dados em silêncio. */
  uint32_t gaps;
  /** Tick do STM32 (ms) em que o último relatório chegou. */
  uint32_t last_ms;
  /** Número de sequência do último relatório, como o sensor informou. */
  uint8_t sequence;
  /** Acurácia informada pelo sensor, 0..3; 0 significa não confiável. */
  uint8_t accuracy;
  /** Verdadeiro depois do primeiro relatório desta sessão do sensor. */
  bool seen;
} bno_report_stats_t;

/**
 * @brief Última amostra e estado de saúde publicados pelo driver do BNO08x.
 */
typedef struct bno_sample_s {
  /** Vetor de rotação w,x,y,z, normalizado. */
  float q[4];
  /** Velocidade angular calibrada p,q,r, em rad/s, eixos do corpo. Estes são
      os estados angulares usuais do modelo linearizado no hover; não são, em
      geral, iguais a roll_dot,pitch_dot,yaw_dot. */
  float gyro[3];
  /** Interface em Euler destinada a logs, comandos e ao futuro LQR. O
      quatérnio acima permanece como representação interna para matemática de
      atitude. */
  attitude_euler_state_t euler;
  /** Verdadeiro quando ::euler, inclusive suas derivadas, foi calculado sem
      singularidade e a partir de relatórios de rotação e gyro já vistos. */
  bool euler_valid;
  /** Aceleração linear, em m/s^2, eixos do sensor, com a gravidade já removida
      pela fusão do próprio sensor (SH2_LINEAR_ACCELERATION). Pegar assim, em
      vez de subtrair a gravidade aqui, evita adivinhar as convenções de sinal
      do sensor. Vai para o log; o modelo do TCC não a usa. */
  float lin_accel[3];
  /** Saúde do relatório de rotação. */
  bno_report_stats_t rotation;
  /** Saúde do relatório de giroscópio. */
  bno_report_stats_t angular;
  /** Saúde do relatório de aceleração linear. */
  bno_report_stats_t acceleration;
  /** Número de peça do software do hub, lido do Product ID. */
  uint32_t part;
  /** Número de build do software do hub, lido do Product ID. */
  uint32_t build;
  /** Relatórios descartados por falha de decodificação ou quatérnio inválido. */
  uint32_t errors;
  /** Quantas vezes o sensor avisou que reiniciou. */
  uint32_t resets;
  /** Tempo entre soltar o RST e o primeiro ACK no I2C, em ms. Esta placa mede
      ~80 ms; uma sondagem única e mais curta que isso era o que mantinha o
      sensor offline. */
  uint16_t probe_ms;
  /** Versão do software do hub: patch. */
  uint16_t version_patch;
  /** Versão do software do hub: principal. */
  uint8_t version_major;
  /** Versão do software do hub: secundária. */
  uint8_t version_minor;
  /** Endereço I2C de 7 bits em uso; 0 até o sensor ser encontrado. */
  uint8_t address;
  /** Motivo da última queda; ver ::bno_fault_t. */
  bno_fault_t fault;
  /** Verdadeiro quando os três relatórios estão chegando em dia. */
  bool online;
} bno_sample_t;

/**
 * @brief Cria a task do BNO08x.
 *
 * A task faz o reset físico, sonda os endereços, configura os relatórios a
 * 400 Hz e tenta reconectar a cada 2 s após qualquer falha.
 *
 * @return true se a task foi criada.
 */
bool bno_start(void);

/**
 * @brief Copia a amostra mais recente, de forma segura entre tasks.
 *
 * O campo @c online é recalculado aqui conforme a idade dos relatórios, então
 * um sensor que para de transmitir aparece offline mesmo antes de a task
 * perceber e reiniciá-lo.
 *
 * @param[out] out  Destino da cópia.
 */
void bno_get_sample(bno_sample_t *out);

/**
 * @brief Registra uma task (TaskHandle_t) para ser notificada a cada novo
 *        relatório de rotação (400 Hz), ou NULL para nenhuma.
 */
void bno_set_listener(void *task);
#endif
