"""Missão passo a passo: a Pi manda um comando, espera o STM32 estabilizar e só
então manda o próximo.

Comandos (shell interativo ``dronectl missao`` ou arquivo, um por linha), com a
nomenclatura aeronáutica em inglês (PX4, ArduPilot, MAVLink). Todas as medidas são
números inteiros: distâncias em centímetros, ângulos em graus, tempo em segundos:

    takeoff <altura cm>         arma e sobe; termina quando o drone está estável no ar
    yaw <graus>                 gira no lugar (+ = horário visto de cima, para a direita)
    forward|back <cm>           anda no eixo do nariz
    right|left <cm>             anda de lado
    climb|descend <cm>          muda a altura
    goto <x> <y> [z]            vai ao waypoint (cm, relativos à decolagem: x = frente
                                inicial, y = direita inicial, z = altura acima do
                                chão; sem z mantém a altura): gira para ficar de
                                frente para ele e anda reto até lá, subindo ou
                                descendo ao mesmo tempo
    route [alg] [x,y[,z] ...]   executa a rota: escolhe a ordem de visita
                                (caixeiro-viajante) com o algoritmo genético, visita
                                todos os pontos e volta à partida; sem pontos, lê o
                                arquivo de pontos (missoes/pontos.txt). ``alg``
                                opcional, só para comparação: greedy, 2opt,
                                2optmulti, bruteforce ou exact
    waypoints <x,y[,z]> ...     ``goto`` para cada waypoint, na ordem dada
    rtl                         return to launch: ``goto 0 0``
    hold                        para onde está
    wait <s>                    espera mostrando o estado
    land                        pousa e desarma
    status | help | exit

O protocolo é o DroneLink (protocol/drone_controller.xml): cada movimento é um
DRONE_LEG_COMMAND; o STM32 responde DRONE_LEG_ACK "em andamento" ao aceitar e
"aceito" quando o drone chegou e ficou parado (1 s dentro da tolerância). Se
o drone não estiver estável em HOLD, o STM32 recusa (estado errado ou
ocupado) — e a Pi nem tenta: confere o estado antes.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, List, Optional, Tuple

from . import rotas
from .link import DroneLink, LinkError, state_name

#: Velocidades das rampas do STM32 (flight_config.h), para estimar o tempo máximo.
Z_RATE_M_S = 0.3
XY_RATE_M_S = 0.3
YAW_RATE_RAD_S = 0.5
#: Folga de tempo de cada trecho além da rampa (estabilizar etc.), em s.
FOLGA_S = 20.0
#: Giro abaixo do qual ``goto`` não gira antes de andar, em graus.
GIRO_MINIMO_GRAUS = 2.0
#: Distância abaixo da qual ``goto`` considera que já chegou, em cm.
DISTANCIA_MINIMA_CM = 3
#: Maior altura de decolagem que o STM32 aceita, em cm (flight_ctrl.c).
ALTURA_MAXIMA_CM = 150
#: Arquivo de pontos do ``route`` sem pontos: um ``x,y[,z]`` (cm) por linha.
ARQUIVO_PONTOS = Path(__file__).resolve().parents[1] / 'missoes' / 'pontos.txt'
#: Algoritmo do ``route`` quando nenhum é pedido (o escolhido no TCC).
ALGORITMO_PADRAO = 'ga'
#: Algoritmos aceitos no ``route`` -> nome em rotas.ALGORITMOS.
ALGORITMOS_ROUTE = {'greedy': 'guloso', '2opt': 'guloso+2opt', '2optmulti': '2opt-multi',
                      'ga': 'genetico', 'bruteforce': 'forca-bruta', 'exact': 'exato'}
#: Faixa de altura aceita nos pontos, em cm (o STM32 recusa abaixo do pouso e acima de 1,5 m).
ALTURA_MINIMA_PONTO_CM = 15
#: Mais pontos que isso no exato estoura a memória da Pi Zero (Held-Karp: n·2ⁿ).
EXATO_MAX_PONTOS = 16
#: Mais pontos que isso na força bruta (n! ordens) passa de alguns segundos na Pi Zero.
FORCA_BRUTA_MAX_PONTOS = 9

#: Nome -> (quantidade de números, ajuda).
COMANDOS = {
    'takeoff': (1, 'takeoff <altura cm>: arma e sobe até ficar estável'),
    'yaw': (1, 'yaw <graus>: gira no lugar; + = horário (direita)'),
    'forward': (1, 'forward <cm>: anda para a frente'),
    'back': (1, 'back <cm>: anda para trás'),
    'right': (1, 'right <cm>: anda para a direita'),
    'left': (1, 'left <cm>: anda para a esquerda'),
    'climb': (1, 'climb <cm>: sobe'),
    'descend': (1, 'descend <cm>: desce'),
    'goto': (-1, 'goto <x cm> <y cm> [z cm]: gira de frente para o waypoint e anda reto'),
    'route': (-1, 'route [alg] [x,y[,z] ...]: ordem pelo algoritmo genético, visita todos e '
                  'volta à partida; sem pontos, lê missoes/pontos.txt'),
    'waypoints': (-1, 'waypoints <x,y[,z]> ...: "goto" para cada waypoint, na ordem dada (cm)'),
    'rtl': (0, 'rtl: return to launch, "goto 0 0" (ponto de decolagem)'),
    'hold': (0, 'hold: para onde está'),
    'wait': (1, 'wait <s>: espera mostrando o estado'),
    'land': (0, 'land: pousa e desarma'),
    'status': (0, 'status: estado e posição'),
    'help': (0, 'help: esta lista'),
    'exit': (0, 'exit: sai (no ar, pousa antes)'),
}
#: Comandos que só fazem sentido com o drone estável no ar (HOLD).
PRECISAM_HOLD = {'yaw', 'forward', 'back', 'right', 'left', 'climb', 'descend',
                 'goto', 'route', 'waypoints', 'rtl'}
#: Deslocamentos no corpo: nome -> sinal de (frente, direita, baixo).
DESLOCAMENTOS = {'forward': (1, 0, 0), 'back': (-1, 0, 0), 'right': (0, 1, 0),
                 'left': (0, -1, 0), 'climb': (0, 0, -1), 'descend': (0, 0, 1)}


@dataclass
class Comando:
    """Uma linha interpretada."""

    nome: str
    #: Centímetros, graus ou segundos, conforme o comando.
    numeros: List[int] = field(default_factory=list)
    #: Pontos da rota, em cm: (x, y) ou (x, y, z).
    pontos: List[Tuple[int, ...]] = field(default_factory=list)
    #: Algoritmo do ``route``.
    algoritmo: str = ''



def interpretar(linha: str) -> Optional[Comando]:
    """Comando de uma linha (None para linha vazia ou comentário ``#``).

    Os números são inteiros (cm, graus ou s); no ``route`` e no ``waypoints`` cada
    ponto é ``x,y`` ou ``x,y,z``.

    Raises:
        ValueError: comando desconhecido ou números errados.
    """
    linha = linha.split('#', 1)[0].strip()
    if not linha:
        return None
    partes = linha.split()
    nome = partes[0].lower()
    if nome not in COMANDOS:
        raise ValueError(f'comando desconhecido: {nome} (digite "help")')
    esperados, ajuda = COMANDOS[nome]
    args = partes[1:]
    if nome in ('route', 'waypoints'):
        algoritmo = ''
        if nome == 'route':
            algoritmo = ALGORITMO_PADRAO
            if args and args[0].lower() in ALGORITMOS_ROUTE:
                algoritmo, args = args[0].lower(), args[1:]
        elif not args:
            raise ValueError(f'uso: {ajuda}')
        pontos = ler_pontos(args)
        _conferir_limite(algoritmo, pontos)
        return Comando(nome, pontos=pontos, algoritmo=algoritmo)
    if (nome == 'goto' and len(args) not in (2, 3)) or (nome != 'goto' and len(args) != esperados):
        raise ValueError(f'uso: {ajuda}')
    try:
        numeros = [int(a) for a in args]
    except ValueError:
        raise ValueError(f'uso: {ajuda} (números inteiros: cm, graus ou s)') from None
    if (nome in DESLOCAMENTOS or nome in ('takeoff', 'wait')) and numeros[0] <= 0:
        raise ValueError(f'{nome}: o valor precisa ser positivo')
    if nome == 'takeoff' and numeros[0] > ALTURA_MAXIMA_CM:
        raise ValueError(f'takeoff: no máximo {ALTURA_MAXIMA_CM} cm')
    if nome == 'goto':
        _conferir_altura(tuple(numeros))
    return Comando(nome, numeros)


def ler_pontos(textos: Iterable[str]) -> List[Tuple[int, ...]]:
    """Pontos ``x,y`` ou ``x,y,z`` (cm inteiros), um por texto."""
    pontos = []
    for texto in textos:
        try:
            ponto = tuple(int(v) for v in texto.split(','))
            if len(ponto) not in (2, 3):
                raise ValueError
        except ValueError:
            raise ValueError(f'ponto inválido "{texto}" (use x,y ou x,y,z em cm, '
                             'ex.: 150,-50,80)') from None
        _conferir_altura(ponto)
        pontos.append(ponto)
    return pontos


def ler_arquivo_pontos(caminho: Path) -> List[Tuple[int, ...]]:
    """Pontos de um arquivo: um ``x,y[,z]`` por linha; ``#`` começa comentário."""
    linhas = [l.split('#', 1)[0].strip() for l in caminho.read_text(encoding='utf-8').splitlines()]
    pontos = ler_pontos(l for l in linhas if l)
    if not pontos:
        raise ValueError(f'{caminho}: nenhum ponto')
    return pontos


def _conferir_limite(algoritmo: str, pontos: List[Tuple[int, ...]]) -> None:
    """Os exatos só até onde rodam na Pi Zero em poucos segundos."""
    if algoritmo == 'exact' and len(pontos) > EXATO_MAX_PONTOS:
        raise ValueError(f'exact: no máximo {EXATO_MAX_PONTOS} pontos (use ga)')
    if algoritmo == 'bruteforce' and len(pontos) > FORCA_BRUTA_MAX_PONTOS:
        raise ValueError(f'bruteforce: no máximo {FORCA_BRUTA_MAX_PONTOS} pontos '
                         '(use exact ou ga)')


def _conferir_altura(ponto: Tuple[int, ...]) -> None:
    if len(ponto) == 3 and not ALTURA_MINIMA_PONTO_CM <= ponto[2] <= ALTURA_MAXIMA_CM:
        raise ValueError(f'altura {ponto[2]} cm fora de {ALTURA_MINIMA_PONTO_CM}..'
                         f'{ALTURA_MAXIMA_CM} cm')


def rumo(x: float, y: float, psi: float, alvo_x: float, alvo_y: float) -> Tuple[float, float]:
    """Quanto girar e quanto andar para ir de (x, y) com a guinada @p psi até o alvo.

    Referencial local NED da decolagem: x para a frente inicial, y para a
    direita inicial, guinada positiva no sentido horário visto de cima.

    Returns:
        (giro em graus, entre -180 e 180; distância na unidade das coordenadas).
    """
    dx, dy = alvo_x - x, alvo_y - y
    distancia = math.hypot(dx, dy)
    if distancia < 1e-9:
        return 0.0, 0.0
    giro = math.atan2(dy, dx) - psi
    giro = (giro + math.pi) % (2.0 * math.pi) - math.pi
    return math.degrees(giro), distancia


class Missao:
    """Executa comandos um de cada vez, esperando o STM32 estabilizar entre eles."""

    def __init__(self, link: DroneLink, saida: Callable[[str], None] = print,
                 arquivo_pontos: Path = ARQUIVO_PONTOS) -> None:
        self.link = link
        self.saida = saida
        #: De onde o ``route`` sem pontos lê os pontos.
        self.arquivo_pontos = Path(arquivo_pontos)
        self.sessao = False
        #: Quando terminou o último trecho: a posição usada na rota tem de ser mais nova.
        self._fim_trecho = 0.0

    # ---- estado ----------------------------------------------------------

    @property
    def estado(self) -> str:
        return state_name(self.link.flight_state)

    def posicao(self, tempo_s: float = 2.0) -> Tuple[float, float, float, float]:
        """(x, y, altura em cm; guinada em rad) da telemetria do STM32.

        Espera uma LOCAL_POSITION_NED e uma ATTITUDE recebidas depois do fim do
        último trecho (chegam a 5 Hz): com a de antes, a rota sairia errada.
        """
        fim = time.monotonic() + tempo_s
        while time.monotonic() < fim and min(self.link.position_time,
                                             self.link.attitude_time) <= self._fim_trecho:
            time.sleep(0.02)
        pos, att = self.link.position, self.link.attitude
        if pos is None or att is None:
            raise LinkError('sem telemetria de posição do STM32 (LOCAL_POSITION_NED/ATTITUDE)')
        return pos.x * 100.0, pos.y * 100.0, -pos.z * 100.0, att.yaw

    def resumo(self) -> str:
        texto = f'estado {self.estado}'
        try:
            x, y, h, psi = self.posicao()
            texto += (f'  x={x:+.0f} y={y:+.0f} cm  altura={h:.0f} cm  '
                      f'guinada={math.degrees(psi):+.0f}°')
        except LinkError:
            pass
        return texto

    # ---- execução --------------------------------------------------------

    def _esperar_trecho(self, command_id: int, tempo_s: float, o_que: str) -> None:
        if not self.link.wait_leg(command_id, tempo_s):
            raise LinkError(f'{o_que} não terminou (estado {self.estado})')
        self._fim_trecho = time.monotonic()
        self.saida(f'ok: {o_que} — estável. {self.resumo()}')

    def _exigir_hold(self, nome: str) -> None:
        # Um trecho recém-concluído pode ainda não ter chegado ao estado local.
        fim = time.monotonic() + 1.0
        while self.estado in ('TAKEOFF', 'LEG') and time.monotonic() < fim:
            time.sleep(0.05)
        if self.estado != 'HOLD':
            raise LinkError(f'"{nome}" só com o drone estável no ar (HOLD); agora: {self.estado}')

    def _girar(self, graus: float) -> None:
        self.saida(f'girando {graus:+.0f}°...')
        tempo = abs(math.radians(graus)) / YAW_RATE_RAD_S + FOLGA_S
        cmd = self.link.rotate(graus, timeout_s=tempo)
        self._esperar_trecho(cmd, tempo + 5.0, f'girar {graus:+.0f}°')

    def _andar(self, frente: float, direita: float, baixo: float, o_que: str) -> None:
        """Translada no corpo; as medidas em cm (o enlace usa m)."""
        self.saida(f'{o_que}...')
        distancia = math.sqrt(frente ** 2 + direita ** 2 + baixo ** 2) / 100.0
        tempo = distancia / min(XY_RATE_M_S, Z_RATE_M_S) + FOLGA_S
        cmd = self.link.move(frente / 100.0, direita / 100.0, baixo / 100.0, timeout_s=tempo)
        self._esperar_trecho(cmd, tempo + 5.0, o_que)

    def _ir(self, alvo_x: float, alvo_y: float, alvo_z: Optional[float] = None) -> None:
        """Vai ao ponto: gira de frente para ele e anda reto, subindo/descendo junto.

        Sem ``alvo_z`` mantém a altura. Se o deslocamento horizontal for pequeno,
        só sobe ou desce (sem girar).
        """
        x, y, h, psi = self.posicao()
        giro, distancia = rumo(x, y, psi, alvo_x, alvo_y)
        dz = 0.0 if alvo_z is None else alvo_z - h
        alvo = f'({alvo_x:+.0f}, {alvo_y:+.0f}' + ('' if alvo_z is None else f', {alvo_z:.0f}') + ')'
        self.saida(f'rota até {alvo} cm: girar {giro:+.0f}°, andar {distancia:.0f} cm'
                   + ('' if alvo_z is None else f', {"subir" if dz >= 0 else "descer"} {abs(dz):.0f} cm'))
        if distancia < DISTANCIA_MINIMA_CM and abs(dz) < DISTANCIA_MINIMA_CM:
            self.saida('já está no ponto')
            return
        if distancia < DISTANCIA_MINIMA_CM:
            self._andar(0.0, 0.0, -dz, f'{"subir" if dz > 0 else "descer"} {abs(dz):.0f} cm')
            return
        if abs(giro) >= GIRO_MINIMO_GRAUS:
            self._girar(giro)
        # Recalcula depois do giro: a distância é a que sobrou de verdade.
        x, y, h, psi = self.posicao()
        giro, distancia = rumo(x, y, psi, alvo_x, alvo_y)
        dz = 0.0 if alvo_z is None else alvo_z - h
        self._andar(distancia, 0.0, -dz, f'frente {distancia:.0f} cm'
                    + ('' if abs(dz) < 1 else f' e {"subir" if dz > 0 else "descer"} {abs(dz):.0f} cm'))

    def _visitar(self, algoritmo: str, pontos: List[Tuple[int, ...]]) -> None:
        """Calcula a ordem com o algoritmo, visita os pontos e volta à partida.

        A partida é (0, 0) na altura atual; pontos sem z ficam nessa altura.
        """
        nome = ALGORITMOS_ROUTE[algoritmo]
        _, _, h0, _ = self.posicao()
        origem = (0.0, 0.0, float(round(h0)))
        pontos3 = [(p[0], p[1], p[2] if len(p) > 2 else origem[2]) for p in pontos]
        t0 = time.perf_counter()
        ordem = rotas.ordenar(nome, pontos3, origem)
        calculo_ms = (time.perf_counter() - t0) * 1000.0
        self.saida(f'{nome}: ordem {" ".join(str(i + 1) for i in ordem)}  '
                   f'circuito {rotas.custo(ordem, pontos3, origem):.0f} cm  '
                   f'(calculado em {calculo_ms:.1f} ms)')
        inicio = time.monotonic()
        for i, indice in enumerate(ordem, 1):
            self.saida(f'ponto {i}/{len(ordem)}: {pontos3[indice]}')
            self._exigir_hold('route')
            self._ir(*pontos3[indice])
        self.saida('voltando à partida')
        self._exigir_hold('route')
        self._ir(*origem)
        self.saida(f'route concluída em {time.monotonic() - inicio:.0f} s de voo')

    def _decolar(self, altura_cm: float) -> None:
        if self.estado not in ('STANDBY', 'FAILSAFE'):
            raise LinkError(f'decolar só no chão; agora: {self.estado}')
        if not self.sessao:
            self.link.start_session()
            self.sessao = True
        self.link.arm(True)
        self.saida(f'armado; decolando para {altura_cm:.0f} cm...')
        altura = altura_cm / 100.0
        cmd = self.link.takeoff(altura, timeout_s=altura / Z_RATE_M_S + FOLGA_S)
        self._esperar_trecho(cmd, altura / Z_RATE_M_S + FOLGA_S + 5.0,
                             f'decolar {altura_cm:.0f} cm')

    def pousar(self, tempo_s: float = 30.0) -> None:
        """Pousa e espera desarmar (também usado ao sair e em erro)."""
        if self.estado in ('STANDBY', 'FAILSAFE'):
            self.saida('já está no chão')
        else:
            self.saida('pousando...')
            self.link.land()
            fim = time.monotonic() + tempo_s
            while time.monotonic() < fim and self.estado not in ('STANDBY', 'FAILSAFE'):
                time.sleep(0.2)
            if self.estado not in ('STANDBY', 'FAILSAFE'):
                self.saida('não desarmou sozinho: desarmando')
                self.link.arm(False)
            self._fim_trecho = time.monotonic()
            self.saida(f'no chão. {self.resumo()}')
        if self.sessao:
            self.link.end_session()
            self.sessao = False

    def executar(self, cmd: Comando) -> bool:
        """Executa um comando até o fim. False = pedir para sair."""
        nome, n = cmd.nome, cmd.numeros
        if nome in PRECISAM_HOLD:
            self._exigir_hold(nome)
        if nome == 'help':
            for _, texto in COMANDOS.values():
                self.saida('  ' + texto)
        elif nome == 'status':
            self.saida(self.resumo())
        elif nome == 'exit':
            return False
        elif nome == 'takeoff':
            self._decolar(n[0])
        elif nome == 'yaw':
            self._girar(n[0])
        elif nome in DESLOCAMENTOS:
            self._andar(*(s * n[0] for s in DESLOCAMENTOS[nome]), f'{nome} {n[0]} cm')
        elif nome == 'goto':
            self._ir(*n)
        elif nome == 'waypoints':
            for i, ponto in enumerate(cmd.pontos, 1):
                self.saida(f'waypoint {i}/{len(cmd.pontos)}')
                self._exigir_hold('waypoints')
                self._ir(*ponto)
        elif nome == 'route':
            pontos = cmd.pontos
            if not pontos:
                pontos = ler_arquivo_pontos(self.arquivo_pontos)
                _conferir_limite(cmd.algoritmo, pontos)
                self.saida(f'{len(pontos)} pontos de {self.arquivo_pontos}')
            self._visitar(cmd.algoritmo, pontos)
        elif nome == 'rtl':
            self._ir(0.0, 0.0)
        elif nome == 'hold':
            self.link.hold()
            self.saida('segurando a posição')
        elif nome == 'wait':
            fim = time.monotonic() + n[0]
            while time.monotonic() < fim:
                self.saida(self.resumo())
                time.sleep(min(1.0, max(0.0, fim - time.monotonic())))
        elif nome == 'land':
            self.pousar()
        return True

    def executar_linhas(self, linhas: Iterable[str]) -> bool:
        """Roda uma lista de comandos; para no primeiro erro e pousa. True = tudo certo."""
        numero = 0
        try:
            for numero, linha in enumerate(linhas, 1):
                cmd = interpretar(linha)
                if cmd is None:
                    continue
                self.saida(f'> {linha.strip()}')
                if not self.executar(cmd):
                    break
            return True
        except (LinkError, ValueError) as exc:
            self.saida(f'erro na linha {numero}: {exc}; parando a missão')
            return False
        finally:
            if self.estado not in ('STANDBY', 'FAILSAFE'):
                self.pousar()
            elif self.sessao:
                self.link.end_session()
                self.sessao = False

    def shell(self, entrada: Callable[[str], str] = input) -> None:
        """Prompt interativo. Ctrl+C pousa; erro num comando não derruba o shell."""
        self.saida('missão: digite "help". Cada comando espera o drone ficar estável.')
        try:
            while True:
                try:
                    linha = entrada(f'[{self.estado}] missao> ')
                except EOFError:
                    break
                try:
                    cmd = interpretar(linha)
                    if cmd is not None and not self.executar(cmd):
                        break
                except (LinkError, ValueError) as exc:
                    self.saida(f'erro: {exc}')
        except KeyboardInterrupt:
            self.saida('\nCtrl+C')
        finally:
            if self.estado not in ('STANDBY', 'FAILSAFE'):
                self.pousar()
            elif self.sessao:
                self.link.end_session()
                self.sessao = False
