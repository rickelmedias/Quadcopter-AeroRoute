"""Ensaio do DroneLink na UART real (Raspberry Pi <-> STM32), com o drone
simulado no STM32 (FLT_SIM = 1: os motores nunca giram).

    cd ~/drone/tools/pi && ~/drone-venv/bin/python ensaios/ensaio_dronelink.py

Etapas (cada uma imprime o resultado e tudo vai para um JSON em
``ensaios/resultados/``):

1. enlace parado: mensagens por tipo e taxa, pacotes perdidos (saltos na
   sequência MAVLink do STM32) e erros de CRC;
2. latência: ida e volta de PARAM_REQUEST_READ -> PARAM_VALUE;
3. recusas e transações, com trechos montados à mão (identificadores
   controlados): sem autorização, no chão, boot errado, destino errado, índice
   fora de ordem, número inválido, ocupado, fora dos limites, pedido repetido
   (executa uma vez só), colisão de identificador e cancelamento;
4. missão ``takeoff 30`` / ``route`` (quadrado de 40 cm) / ``land`` pelo
   ``dronectl``;
5. perda da autorização do operador no meio de um trecho;
6. perda do HEARTBEAT: o processo que comanda o voo morre no meio de um
   trecho, e um leitor passivo (que não transmite) registra o que o STM32 faz.
"""

from __future__ import annotations

import json
import math
import multiprocessing as mp
import os
import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dronectl import dialect  # noqa: E402
from dronectl.link import DroneLink, reason_text  # noqa: E402
from dronectl.missao import Missao  # noqa: E402

PORTA = os.environ.get('DRONECTL_PORT', '/dev/serial0')
SAIDA = Path(__file__).resolve().parent / 'resultados'
ESTADOS = [
    'BOOT',
    'SELF_TEST',
    'STANDBY',
    'PREARM',
    'ARMED',
    'TAKEOFF',
    'HOLD',
    'LEG',
    'LAND',
    'FAILSAFE',
    'TERMINATION',
]


def nome_estado(s: int) -> str:
    return ESTADOS[s] if 0 <= s < len(ESTADOS) else str(s)


class Registro:
    """Liga-se ao DroneLink e guarda o que chega do STM32 (tipo, sequência, estado)."""

    def __init__(self, link: DroneLink) -> None:
        self.link = link
        self.tipos: Counter = Counter()
        self.ultimo_seq = None
        self.perdidos = 0
        self.repetidos = 0
        self.recebidos = 0
        #: Sequências recebidas, "desdobradas" além de 255 (aceita reordenação).
        self.seqs = set()
        self._base = 0
        self.estados = []  # (t, estado, armado)
        #: Frames descartados no STM32 (recepção + transmissão), do SYS_STATUS.
        self.erros_stm32 = 0
        original = link._handle

        def handle(msg):
            self.recebidos += 1
            self.tipos[msg.get_type()] += 1
            seq = msg.get_seq()
            # As duas filas de envio do STM32 (crítica e normal) podem trocar a
            # ordem: um HEARTBEAT empacotado depois sai antes da telemetria já
            # enfileirada. Perda é número que nunca chega, não salto local.
            if self.ultimo_seq is None:
                u = seq
            else:
                salto = (seq - self.ultimo_seq) % 256
                if salto > 128:
                    salto -= 256  # chegou atrasado (fora de ordem)
                u = self._ultimo_u + salto
                if salto <= 0:
                    self.repetidos += 1
            self.seqs.add(u)
            self._ultimo_u = u
            self.ultimo_seq = seq
            original(msg)
            if msg.get_type() == 'SYS_STATUS':
                self.erros_stm32 = msg.errors_comm
            if msg.get_type() in ('HEARTBEAT', 'DRONE_LEG_ACK'):
                estado = msg.custom_mode if msg.get_type() == 'HEARTBEAT' else msg.flight_state
                if not self.estados or self.estados[-1][1] != estado:
                    self.estados.append((time.time(), estado, link.armed))

        link._handle = handle

    def zerar(self) -> None:
        self.tipos.clear()
        self.perdidos = self.recebidos = self.repetidos = 0
        self.seqs = set()
        self.ultimo_seq = None


class Trechos:
    """Monta DRONE_LEG_COMMAND à mão, com missão/comando/índice controlados."""

    def __init__(self, link: DroneLink) -> None:
        self.link = link
        self.mod = link.mod
        self.missao = int.from_bytes(os.urandom(4), 'little') | 1
        self.comando = 0
        self.indice = 0

    def enviar(
        self,
        acao,
        x=0.0,
        y=0.0,
        z=0.0,
        yaw=0.0,
        *,
        frame=None,
        boot=None,
        comando=None,
        indice=None,
        alvo_comp=1,
        timeout_ms=30000,
    ):
        """Envia e devolve (comando, índice, primeira resposta ou None)."""
        m = self.mod
        frame = m.MAV_FRAME_BODY_FRD if frame is None else frame
        if comando is None:
            self.comando += 1
            comando = self.comando
        indice = self.indice + 1 if indice is None else indice
        boot = self.link.boot_id if boot is None else boot
        since = self.link._mark()
        self.link.mav.drone_leg_command_send(
            boot,
            self.missao,
            comando,
            timeout_ms,
            x,
            y,
            z,
            yaw,
            0.0,
            0.0,
            0.0,
            0.0,
            indice,
            0,
            acao,
            frame,
            1,
            alvo_comp,
        )
        ack = self.link._wait(
            lambda a: (
                a.get_type() == 'DRONE_LEG_ACK'
                and a.command_id == comando
                and a.mission_id == self.missao
            ),
            since,
            1.5,
        )
        if ack is not None and ack.result in (m.MAV_RESULT_ACCEPTED, m.MAV_RESULT_IN_PROGRESS):
            self.indice = indice
        return comando, indice, ack

    def fim(self, comando, tempo_s=40.0):
        """Resposta de fim de execução do comando (concluído, falhou ou cancelado).

        Ignora recusas com o mesmo command_id (por exemplo a de colisão de
        identificador), que não dizem nada sobre o trecho em execução.
        """
        m = self.mod
        finais = (m.MAV_RESULT_ACCEPTED, m.MAV_RESULT_FAILED, 6)  # 6 = MAV_RESULT_CANCELLED
        return self.link._wait(
            lambda a: (
                a.get_type() == 'DRONE_LEG_ACK'
                and a.command_id == comando
                and a.mission_id == self.missao
                and a.result in finais
            ),
            0,
            tempo_s,
        )


def resposta(link, ack) -> str:
    """Resultado MAVLink / motivo (texto do dronectl, em português)."""
    if ack is None:
        return 'sem resposta'
    nomes = {
        0: 'ACCEPTED',
        1: 'TEMPORARILY_REJECTED',
        2: 'DENIED',
        3: 'UNSUPPORTED',
        4: 'FAILED',
        5: 'IN_PROGRESS',
        6: 'CANCELLED',
    }
    return f'{nomes.get(ack.result, ack.result)} / {reason_text(ack.reason)}'


def esperar_estado(link, estados, tempo_s):
    fim = time.monotonic() + tempo_s
    while time.monotonic() < fim:
        if nome_estado(link.flight_state) in estados:
            return True
        time.sleep(0.05)
    return False


def posicao(link):
    time.sleep(0.5)
    p = link.position
    return (p.x, p.y, p.z)


def decolar(link, tr, altura=0.3):
    link.arm(True)
    c, _, ack = tr.enviar(
        tr.mod.DRONE_ACTION_TAKEOFF_RELATIVE, z=-altura, frame=tr.mod.MAV_FRAME_LOCAL_FRD
    )
    fim = tr.fim(c, 30)
    return resposta(link, ack), resposta(link, fim)


def pousar(link, tr):
    c, _, ack = tr.enviar(tr.mod.DRONE_ACTION_LAND)
    ok = esperar_estado(link, ('STANDBY',), 20)
    return resposta(link, ack), ok


# ---- etapas ------------------------------------------------------------------


def etapa_enlace(link, reg, segundos=20.0):
    reg.zerar()
    erros0 = link.mav.total_receive_errors
    bytes0 = link.mav.total_bytes_received
    stm0 = reg.erros_stm32
    time.sleep(segundos)
    tipos = {k: round(v / segundos, 2) for k, v in sorted(reg.tipos.items())}
    # Perdidos = números que faltam entre o primeiro e o último recebidos.
    reg.perdidos = (max(reg.seqs) - min(reg.seqs) + 1 - len(reg.seqs)) if reg.seqs else 0
    total = len(reg.seqs) + reg.perdidos
    return {
        'duracao_s': segundos,
        'mensagens': reg.recebidos,
        'taxa_por_tipo_hz': tipos,
        'bytes_por_s': round((link.mav.total_bytes_received - bytes0) / segundos, 1),
        'perdidas_seq': reg.perdidos,
        'fora_de_ordem': reg.repetidos,
        'perda_pct': round(100 * reg.perdidos / total, 3) if total else 0.0,
        'erros_crc': link.mav.total_receive_errors - erros0,
        'descartes_no_stm32': reg.erros_stm32 - stm0,
    }


def etapa_latencia(link, n=100):
    rtts = []
    for _ in range(n):
        since = link._mark()
        t0 = time.perf_counter()
        link.mav.param_request_read_send(1, 1, b'FLT_SIM', -1)
        msg = link._wait(lambda m: m.get_type() == 'PARAM_VALUE', since, 1.0)
        if msg is not None:
            rtts.append((time.perf_counter() - t0) * 1000.0)
        time.sleep(0.05)
    rtts.sort()
    return {
        'amostras': len(rtts),
        'pedidos': n,
        'media_ms': round(sum(rtts) / len(rtts), 1),
        'mediana_ms': round(rtts[len(rtts) // 2], 1),
        'p95_ms': round(rtts[int(len(rtts) * 0.95) - 1], 1),
        'min_ms': round(rtts[0], 1),
        'max_ms': round(rtts[-1], 1),
    }


def etapa_recusas(link):
    m = link.mod
    tr = Trechos(link)
    casos = []

    def caso(nome, esperado, ack, obs=''):
        r = resposta(link, ack)
        ok = esperado in r
        casos.append({'caso': nome, 'esperado': esperado, 'resposta': r, 'ok': ok, 'obs': obs})
        print(f"  {'OK ' if ok else 'ERRO'} {nome}: {r}")

    T = m.DRONE_ACTION_TRANSLATE_RELATIVE
    caso('trecho sem autorização do operador', 'sem autorização do operador', tr.enviar(T, 0.2)[2])
    link.start_session()
    caso('trecho com o drone no chão', 'estado errado', tr.enviar(T, 0.2)[2])
    caso('identidade de boot errada', 'boot diferente', tr.enviar(T, 0.2, boot=link.boot_id + 1)[2])
    caso('destino errado (componente 7)', 'destino errado', tr.enviar(T, 0.2, alvo_comp=7)[2])
    caso('índice de trecho fora de ordem', 'sequência errada', tr.enviar(T, 0.2, indice=5)[2])
    caso('número inválido (NaN)', 'número inválido', tr.enviar(T, float('nan'))[2])

    d0, d1 = decolar(link, tr)
    print(f'  decolagem: {d0} -> {d1}')
    casos.append(
        {
            'caso': 'decolar 30 cm',
            'esperado': 'ACCEPTED',
            'resposta': d1,
            'ok': 'ACCEPTED' in d1,
            'obs': d0,
        }
    )

    c, _, ack = tr.enviar(m.DRONE_ACTION_ROTATE_RELATIVE, yaw=math.radians(90))
    caso('girar 90°', 'IN_PROGRESS', ack)
    caso('trecho durante outro trecho', 'ocupado', tr.enviar(T, 0.2)[2])
    caso('fim do giro', 'ACCEPTED', tr.fim(c))

    caso('descer 2 m (abaixo do chão)', 'fora dos limites', tr.enviar(T, 0.0, 0.0, 2.0)[2])
    caso('translação de 25 m', 'fora dos limites', tr.enviar(T, 25.0)[2])

    p0 = posicao(link)
    c, i, ack = tr.enviar(T, 0.3)
    caso('andar 30 cm', 'IN_PROGRESS', ack)
    # Mesmo pedido de novo (resposta perdida): mesma resposta, executa uma vez.
    _, _, ack2 = tr.enviar(T, 0.3, comando=c, indice=i)
    caso('mesmo pedido repetido', 'IN_PROGRESS', ack2, 'o STM32 devolve a resposta guardada')
    _, _, ack3 = tr.enviar(T, 0.6, comando=c, indice=i)
    caso('mesmos identificadores, conteúdo diferente', 'IDs repetidos', ack3)
    caso('fim de andar 30 cm', 'ACCEPTED', tr.fim(c))
    p1 = posicao(link)
    andou = math.dist(p0[:2], p1[:2])
    ok = abs(andou - 0.3) < 0.05
    casos.append(
        {
            'caso': 'pedido repetido executado uma só vez',
            'esperado': '0,30 m',
            'resposta': f'{andou:.3f} m',
            'ok': ok,
            'obs': '',
        }
    )
    print(
        f"  {'OK ' if ok else 'ERRO'} deslocamento após pedido repetido: {andou:.3f} m (esperado 0,30)"
    )

    c, i, ack = tr.enviar(T, 1.0)
    time.sleep(1.0)
    link.mav.drone_leg_cancel_send(link.boot_id, tr.missao, c, i, 1, 1)
    caso('cancelar trecho em andamento', 'cancel', tr.fim(c, 5))
    esperar_estado(link, ('HOLD',), 5)
    casos.append(
        {
            'caso': 'estado após cancelar',
            'esperado': 'HOLD',
            'resposta': nome_estado(link.flight_state),
            'ok': nome_estado(link.flight_state) == 'HOLD',
            'obs': '',
        }
    )

    p_ack, pousou = pousar(link, tr)
    casos.append(
        {'caso': 'pousar', 'esperado': 'STANDBY', 'resposta': p_ack, 'ok': pousou, 'obs': ''}
    )
    print(f'  pouso: {p_ack}, desarmou: {pousou}')
    link.end_session()
    return casos


def etapa_missao(link):
    saida = []
    missao = Missao(link, saida=lambda s: (saida.append(s), print('  ' + s)))
    linhas = (
        (Path(__file__).resolve().parents[1] / 'missoes' / 'quadrado_40cm.txt')
        .read_text(encoding='utf-8')
        .splitlines()
    )
    t0 = time.monotonic()
    ok = missao.executar_linhas(linhas)
    return {'ok': ok, 'duracao_s': round(time.monotonic() - t0, 1), 'saida': saida}


def etapa_lease(link, reg):
    m = link.mod
    tr = Trechos(link)
    link.start_session()
    decolar(link, tr)
    c, _, ack = tr.enviar(m.DRONE_ACTION_TRANSLATE_RELATIVE, 1.0)
    time.sleep(1.0)
    reg.estados.clear()
    t_corte = time.time()
    link.end_session()  # para a autorização; o HEARTBEAT continua
    fim = tr.fim(c, 5)
    esperar_estado(link, ('STANDBY',), 15)
    eventos = [(round(t - t_corte, 2), nome_estado(s)) for t, s, _ in reg.estados]
    return {'resposta_do_trecho': resposta(link, fim), 'eventos_s': eventos}


def _filho_perde_enlace(fila):
    """Processo que decola, começa um trecho e morre sem avisar."""
    try:
        print('   filho: abrindo o enlace', flush=True)
        link = DroneLink(PORTA, on_text=lambda s: print('   filho [STM32]', s, flush=True))
        link.wait_fc()
        print('   filho: decolando', flush=True)
        tr = Trechos(link)
        link.start_session()
        d = decolar(link, tr)
        if 'ACCEPTED' not in d[1]:
            raise RuntimeError(f'decolagem: {d}')
        tr.enviar(link.mod.DRONE_ACTION_TRANSLATE_RELATIVE, 1.0)
        time.sleep(1.0)
        fila.send(time.time())
    except Exception as exc:  # o pai mostra o erro
        fila.send(repr(exc))
    os._exit(0)  # sem pousar, sem fechar a porta: como uma Pi que travou


def etapa_heartbeat():
    import serial

    mod = dialect.load()
    ctx = mp.get_context('spawn')  # fork depois de threads pode travar o filho
    # Pipe, não Queue: a Queue envia por uma thread que o os._exit do filho mata.
    pai, fila = ctx.Pipe(duplex=False)
    filho = ctx.Process(target=_filho_perde_enlace, args=(fila,))
    filho.start()
    if not pai.poll(120):
        raise RuntimeError('o processo filho não respondeu')
    t_morte = pai.recv()
    filho.join()
    if isinstance(t_morte, str):
        return {'erro': t_morte}
    # Leitor passivo: só lê a UART, nunca transmite (não renova o enlace).
    s = serial.Serial(PORTA, 115200, timeout=0.05)
    mav = mod.MAVLink(None)
    mav.robust_parsing = True
    eventos, ultimo = [], None
    fim = time.time() + 10.0
    while time.time() < fim:
        for msg in mav.parse_buffer(s.read(512)) or []:
            if msg.get_type() == 'HEARTBEAT' and msg.get_srcComponent() == 1:
                armado = bool(msg.base_mode & mod.MAV_MODE_FLAG_SAFETY_ARMED)
                chave = (msg.custom_mode, armado)
                if chave != ultimo:
                    ultimo = chave
                    eventos.append(
                        (round(time.time() - t_morte, 2), nome_estado(msg.custom_mode), armado)
                    )
    s.close()
    return {'eventos_s': eventos}


def main() -> int:
    """Roda as etapas pedidas (argumentos 1..6; sem argumentos, todas)."""
    etapas = {int(a) for a in sys.argv[1:]} or set(range(1, 7))
    SAIDA.mkdir(parents=True, exist_ok=True)
    resultado = {'inicio': time.strftime('%Y-%m-%d %H:%M:%S'), 'porta': PORTA}
    link = DroneLink(PORTA, on_text=lambda s: print(f'  [STM32] {s}'))
    link.wait_fc()
    reg = Registro(link)
    resultado['boot_id'] = link.boot_id
    resultado['capacidades'] = hex(link.capabilities)
    print(f'STM32: boot {link.boot_id}, capacidades {link.capabilities:#x}')
    try:
        if 1 in etapas:
            print('1. enlace parado (60 s)')
            resultado['enlace'] = etapa_enlace(link, reg, 60.0)
            print('  ', resultado['enlace'])
        if 2 in etapas:
            print('2. latência PARAM_REQUEST_READ -> PARAM_VALUE (100 pedidos)')
            resultado['latencia'] = etapa_latencia(link)
            print('  ', resultado['latencia'])
        print('   FLT_SIM ->', link.param_set('FLT_SIM', 1))
        if 3 in etapas:
            print('3. recusas e transações')
            resultado['recusas'] = etapa_recusas(link)
        if 4 in etapas:
            print('4. missão takeoff 30 / route / land')
            resultado['missao'] = etapa_missao(link)
        if 5 in etapas:
            print('5. perda da autorização do operador no meio de um trecho')
            resultado['perda_autorizacao'] = etapa_lease(link, reg)
            print('  ', resultado['perda_autorizacao'])
    finally:
        link.close()
    if 6 in etapas:
        print('6. perda do HEARTBEAT (processo morre no meio de um trecho)')
        resultado['perda_heartbeat'] = etapa_heartbeat()
        print('  ', resultado['perda_heartbeat'])
    with DroneLink(PORTA, on_text=lambda s: None) as link:
        link.wait_fc()
        esperar_estado(link, ('STANDBY',), 10)
        resultado['flt_sim_final'] = link.param_set('FLT_SIM', 0)
    arquivo = SAIDA / f"dronelink_{time.strftime('%Y%m%d_%H%M%S')}.json"
    arquivo.write_text(json.dumps(resultado, indent=1, ensure_ascii=False), encoding='utf-8')
    print(f'resultado em {arquivo}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
