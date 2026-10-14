/**
 * @file motor_test.h
 * @brief Teste de bancada dos motores por comandos de texto na USB.
 *
 * SÓ PARA BANCADA, SEM HÉLICES. Não existe controle de voo neste perfil: os
 * motores recebem exatamente o que for digitado, com rampa limitada.
 *
 * A task envia um quadro DShot600 a cada 1 ms. Até o comando "arm" ela manda
 * só zeros, que é o que o ESC precisa para armar. Comandos (uma linha cada,
 * terminada em Enter):
 *
 *   arm                      arma (todos os motores precisam estar em 0)
 *   m <motor> <pct>          alvo de um motor (1..4) ou de todos (0), 0..100
 *   sweep <motor> [max] [passo] [seg]
 *                            sobe de 0 até max em degraus de passo %, segura
 *                            cada degrau seg segundos e desce até 0
 *   rate <pct_por_s>         velocidade máxima da rampa (padrão 20 %/s)
 *   beep <motor>             beep do ESC para identificar o motor (desarmado)
 *   log on | log off         liga/silencia as linhas MTF/BNO/MOT
 *   status | help
 *   stop, disarm ou Enter vazio  zera tudo na hora e desarma
 *
 * Qualquer linha não reconhecida também para e desarma. Fechar a porta no PC
 * (DTR desligado) para e desarma.
 */
#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H
#include <stdbool.h>

/** Limite absoluto de aceleração, em décimos de por cento (1000 = 100%). */
#define MOTOR_TEST_MAX_PERMILLE 1000U

/**
 * @brief Inicia o DShot e cria a task de teste dos motores.
 *
 * @return true se a task foi criada.
 */
bool motor_test_start(void);
#endif
