"""Missão passo a passo: a Pi manda um comando, espera o STM32 estabilizar e só
então manda o próximo.

Comandos (shell interativo ``dronectl mission`` ou arquivo, um por linha), com a
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
                                arquivo de pontos (missions/points.txt). ``alg``
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
ocupado), e a Pi nem tenta: confere o estado antes.

A origem horizontal (x = y = 0) é onde o fluxo óptico passou a valer na
decolagem (acima de 10 cm), não exatamente onde o drone armou.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, List, Optional, Tuple

from . import routes
from .link import GROUND_STATES, DroneLink, LinkError, state_name

#: Velocidades das referências no STM32 (lqg_config.h / parâmetros padrão), para o
#: tempo máximo de cada trecho.
Z_RATE_M_S = 0.10
XY_RATE_M_S = 0.10
YAW_RATE_RAD_S = 0.5
#: Folga de tempo de cada trecho além da rampa (decolar, estabilizar), em s.
MARGIN_S = 20.0
#: Giro abaixo do qual ``goto`` não gira antes de andar, em graus.
MIN_TURN_DEG = 2.0
#: Distância abaixo da qual ``goto`` considera que já chegou, em cm.
MIN_DISTANCE_CM = 3
#: Faixa de altura que o STM32 aceita, em cm (LQG_PARAM_HEIGHT_MIN_M/MAX_M).
MIN_HEIGHT_CM = 10
MAX_HEIGHT_CM = 80
#: Maior |x| ou |y| aceito pelo STM32, em cm (LQG_XY_LIMIT_M).
MAX_XY_CM = 300
#: Arquivo de pontos do ``route`` sem pontos: um ``x,y[,z]`` (cm) por linha.
POINTS_FILE = Path(__file__).resolve().parents[1] / 'missions' / 'points.txt'
#: Algoritmo do ``route`` quando nenhum é pedido (o escolhido no trabalho).
DEFAULT_ALGORITHM = 'ga'
#: Algoritmos aceitos no ``route`` -> nome em routes.ALGORITHMS.
ROUTE_ALGORITHMS = {
    'greedy': 'greedy',
    '2opt': 'greedy+2opt',
    '2optmulti': '2opt-multi',
    'ga': 'genetic',
    'bruteforce': 'brute-force',
    'exact': 'exact',
}
#: Mais pontos que isso no exato estoura a memória da Pi Zero (Held-Karp: n·2ⁿ).
EXACT_MAX_POINTS = 16
#: Mais pontos que isso na força bruta (n! ordens) passa de alguns segundos na Pi Zero.
BRUTE_FORCE_MAX_POINTS = 9

#: Nome -> (quantidade de números, ajuda).
COMMANDS = {
    'takeoff': (1, 'takeoff <altura cm>: arma e sobe até ficar estável'),
    'yaw': (1, 'yaw <graus>: gira no lugar; + = horário (direita)'),
    'forward': (1, 'forward <cm>: anda para a frente'),
    'back': (1, 'back <cm>: anda para trás'),
    'right': (1, 'right <cm>: anda para a direita'),
    'left': (1, 'left <cm>: anda para a esquerda'),
    'climb': (1, 'climb <cm>: sobe'),
    'descend': (1, 'descend <cm>: desce'),
    'goto': (-1, 'goto <x cm> <y cm> [z cm]: gira de frente para o waypoint e anda reto'),
    'route': (
        -1,
        'route [alg] [x,y[,z] ...]: ordem pelo algoritmo genético, visita todos e '
        'volta à partida; sem pontos, lê missions/points.txt',
    ),
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
NEED_HOLD = {
    'yaw',
    'forward',
    'back',
    'right',
    'left',
    'climb',
    'descend',
    'goto',
    'route',
    'waypoints',
    'rtl',
}
#: Deslocamentos no corpo: nome -> sinal de (frente, direita, baixo).
MOVES = {
    'forward': (1, 0, 0),
    'back': (-1, 0, 0),
    'right': (0, 1, 0),
    'left': (0, -1, 0),
    'climb': (0, 0, -1),
    'descend': (0, 0, 1),
}


@dataclass
class Command:
    """Uma linha interpretada."""

    name: str
    #: Centímetros, graus ou segundos, conforme o comando.
    numbers: List[int] = field(default_factory=list)
    #: Pontos da rota, em cm: (x, y) ou (x, y, z).
    points: List[Tuple[int, ...]] = field(default_factory=list)
    #: Algoritmo do ``route``.
    algorithm: str = ''


def parse(line: str) -> Optional[Command]:
    """Comando de uma linha (None para linha vazia ou comentário ``#``).

    Os números são inteiros (cm, graus ou s); no ``route`` e no ``waypoints`` cada
    ponto é ``x,y`` ou ``x,y,z``.

    Raises:
        ValueError: comando desconhecido ou números errados.
    """
    line = line.split('#', 1)[0].strip()
    if not line:
        return None
    parts = line.split()
    name = parts[0].lower()
    if name not in COMMANDS:
        raise ValueError(f'comando desconhecido: {name} (digite "help")')
    expected, usage = COMMANDS[name]
    args = parts[1:]
    if name in ('route', 'waypoints'):
        algorithm = ''
        if name == 'route':
            algorithm = DEFAULT_ALGORITHM
            if args and args[0].lower() in ROUTE_ALGORITHMS:
                algorithm, args = args[0].lower(), args[1:]
        elif not args:
            raise ValueError(f'uso: {usage}')
        points = parse_points(args)
        _check_size(algorithm, points)
        return Command(name, points=points, algorithm=algorithm)
    if (name == 'goto' and len(args) not in (2, 3)) or (name != 'goto' and len(args) != expected):
        raise ValueError(f'uso: {usage}')
    try:
        numbers = [int(a) for a in args]
    except ValueError:
        raise ValueError(f'uso: {usage} (números inteiros: cm, graus ou s)') from None
    if (name in MOVES or name in ('takeoff', 'wait')) and numbers[0] <= 0:
        raise ValueError(f'{name}: o valor precisa ser positivo')
    if name == 'takeoff' and not MIN_HEIGHT_CM <= numbers[0] <= MAX_HEIGHT_CM:
        raise ValueError(f'takeoff: de {MIN_HEIGHT_CM} a {MAX_HEIGHT_CM} cm')
    if name == 'goto':
        _check_point(tuple(numbers))
    return Command(name, numbers)


def parse_points(texts: Iterable[str]) -> List[Tuple[int, ...]]:
    """Pontos ``x,y`` ou ``x,y,z`` (cm inteiros), um por texto."""
    points = []
    for text in texts:
        try:
            point = tuple(int(v) for v in text.split(','))
            if len(point) not in (2, 3):
                raise ValueError
        except ValueError:
            raise ValueError(
                f'ponto inválido "{text}" (use x,y ou x,y,z em cm, ex.: 150,-50,60)'
            ) from None
        _check_point(point)
        points.append(point)
    return points


def read_points_file(path: Path) -> List[Tuple[int, ...]]:
    """Pontos de um arquivo: um ``x,y[,z]`` por linha; ``#`` começa comentário."""
    lines = [ln.split('#', 1)[0].strip() for ln in path.read_text(encoding='utf-8').splitlines()]
    points = parse_points(ln for ln in lines if ln)
    if not points:
        raise ValueError(f'{path}: nenhum ponto')
    return points


def _check_size(algorithm: str, points: List[Tuple[int, ...]]) -> None:
    """Os exatos só até onde rodam na Pi Zero em poucos segundos."""
    if algorithm == 'exact' and len(points) > EXACT_MAX_POINTS:
        raise ValueError(f'exact: no máximo {EXACT_MAX_POINTS} pontos (use ga)')
    if algorithm == 'bruteforce' and len(points) > BRUTE_FORCE_MAX_POINTS:
        raise ValueError(f'bruteforce: no máximo {BRUTE_FORCE_MAX_POINTS} pontos (use exact ou ga)')


def _check_point(point: Tuple[int, ...]) -> None:
    if abs(point[0]) > MAX_XY_CM or abs(point[1]) > MAX_XY_CM:
        raise ValueError(f'ponto {point[:2]} fora de ±{MAX_XY_CM} cm da decolagem')
    if len(point) == 3 and not MIN_HEIGHT_CM <= point[2] <= MAX_HEIGHT_CM:
        raise ValueError(f'altura {point[2]} cm fora de {MIN_HEIGHT_CM}..{MAX_HEIGHT_CM} cm')


def heading(
    x: float, y: float, psi: float, target_x: float, target_y: float
) -> Tuple[float, float]:
    """Quanto girar e quanto andar para ir de (x, y) com a guinada @p psi até o alvo.

    Referencial local NED da decolagem: x para a frente inicial, y para a
    direita inicial, guinada positiva no sentido horário visto de cima.

    Returns:
        (giro em graus, entre -180 e 180; distância na unidade das coordenadas).
    """
    dx, dy = target_x - x, target_y - y
    dist = math.hypot(dx, dy)
    if dist < 1e-9:
        return 0.0, 0.0
    turn = math.atan2(dy, dx) - psi
    turn = (turn + math.pi) % (2.0 * math.pi) - math.pi
    return math.degrees(turn), dist


class Mission:
    """Executa comandos um de cada vez, esperando o STM32 estabilizar entre eles."""

    def __init__(
        self,
        link: DroneLink,
        out: Callable[[str], None] = print,
        points_file: Path = POINTS_FILE,
    ) -> None:
        self.link = link
        self.out = out
        #: De onde o ``route`` sem pontos lê os pontos.
        self.points_file = Path(points_file)
        self.session = False
        #: Ctrl+C cortou os motores: não pousar depois.
        self.killed = False
        #: Quando terminou o último trecho: a posição usada na rota tem de ser mais nova.
        self._leg_end = 0.0

    # ---- estado ----------------------------------------------------------

    @property
    def state(self) -> str:
        return state_name(self.link.flight_state)

    def pose(self, wait_s: float = 2.0) -> Tuple[float, float, float, float]:
        """(x, y, altura em cm; guinada em rad) da telemetria do STM32.

        Espera uma LOCAL_POSITION_NED e uma ATTITUDE recebidas depois do fim do
        último trecho (chegam a 5 Hz): com as de antes, a rota sairia errada.
        """
        end = time.monotonic() + wait_s
        while (
            time.monotonic() < end
            and min(self.link.position_time, self.link.attitude_time) <= self._leg_end
        ):
            time.sleep(0.02)
        pos, att = self.link.position, self.link.attitude
        if pos is None or att is None:
            raise LinkError('sem telemetria de posição do STM32 (LOCAL_POSITION_NED/ATTITUDE)')
        return pos.x * 100.0, pos.y * 100.0, -pos.z * 100.0, att.yaw

    def summary(self) -> str:
        text = f'estado {self.state}'
        try:
            x, y, h, psi = self.pose()
            text += (
                f'  x={x:+.0f} y={y:+.0f} cm  altura={h:.0f} cm  guinada={math.degrees(psi):+.0f}°'
            )
        except LinkError:
            pass
        if self.link.battery_v is not None:
            text += f'  bateria={self.link.battery_v:.2f} V'
        return text

    # ---- execução --------------------------------------------------------

    def _wait_leg(self, command_id: int, wait_s: float, what: str) -> None:
        if not self.link.wait_leg(command_id, wait_s):
            raise LinkError(f'{what} não terminou (estado {self.state})')
        self._leg_end = time.monotonic()
        self.out(f'ok: {what}: estável. {self.summary()}')

    def _require_hold(self, name: str) -> None:
        # Um trecho recém-concluído pode ainda não ter chegado ao estado local.
        end = time.monotonic() + 1.0
        while self.state in ('TAKEOFF', 'LEG') and time.monotonic() < end:
            time.sleep(0.05)
        if self.state != 'HOLD':
            raise LinkError(f'"{name}" só com o drone estável no ar (HOLD); agora: {self.state}')

    def _turn(self, degrees: float) -> None:
        self.out(f'girando {degrees:+.0f}°...')
        wait = abs(math.radians(degrees)) / YAW_RATE_RAD_S + MARGIN_S
        cmd = self.link.rotate(degrees, timeout_s=wait)
        self._wait_leg(cmd, wait + 5.0, f'girar {degrees:+.0f}°')

    def _move(self, forward: float, right: float, down: float, what: str) -> None:
        """Translada no corpo; as medidas em cm (o enlace usa m)."""
        self.out(f'{what}...')
        dist = math.sqrt(forward**2 + right**2 + down**2) / 100.0
        wait = dist / min(XY_RATE_M_S, Z_RATE_M_S) + MARGIN_S
        cmd = self.link.move(forward / 100.0, right / 100.0, down / 100.0, timeout_s=wait)
        self._wait_leg(cmd, wait + 5.0, what)

    def _go(self, target_x: float, target_y: float, target_z: Optional[float] = None) -> None:
        """Vai ao ponto: gira de frente para ele e anda reto, subindo/descendo junto.

        Sem ``target_z`` mantém a altura. Se o deslocamento horizontal for pequeno,
        só sobe ou desce (sem girar).
        """
        x, y, h, psi = self.pose()
        turn, dist = heading(x, y, psi, target_x, target_y)
        dz = 0.0 if target_z is None else target_z - h
        target = f'({target_x:+.0f}, {target_y:+.0f}' + (
            '' if target_z is None else f', {target_z:.0f}'
        )
        target += ')'
        self.out(
            f'rota até {target} cm: girar {turn:+.0f}°, andar {dist:.0f} cm'
            + ('' if target_z is None else f', {"subir" if dz >= 0 else "descer"} {abs(dz):.0f} cm')
        )
        if dist < MIN_DISTANCE_CM and abs(dz) < MIN_DISTANCE_CM:
            self.out('já está no ponto')
            return
        if dist < MIN_DISTANCE_CM:
            self._move(0.0, 0.0, -dz, f'{"subir" if dz > 0 else "descer"} {abs(dz):.0f} cm')
            return
        if abs(turn) >= MIN_TURN_DEG:
            self._turn(turn)
        # Recalcula depois do giro: a distância é a que sobrou de verdade.
        x, y, h, psi = self.pose()
        turn, dist = heading(x, y, psi, target_x, target_y)
        dz = 0.0 if target_z is None else target_z - h
        self._move(
            dist,
            0.0,
            -dz,
            f'frente {dist:.0f} cm'
            + ('' if abs(dz) < 1 else f' e {"subir" if dz > 0 else "descer"} {abs(dz):.0f} cm'),
        )

    def _visit(self, algorithm: str, points: List[Tuple[int, ...]]) -> None:
        """Calcula a ordem com o algoritmo, visita os pontos e volta à partida.

        A partida é (0, 0) na altura atual; pontos sem z ficam nessa altura.
        """
        name = ROUTE_ALGORITHMS[algorithm]
        _, _, h0, _ = self.pose()
        origin = (0.0, 0.0, float(round(h0)))
        points3 = [(p[0], p[1], p[2] if len(p) > 2 else origin[2]) for p in points]
        t0 = time.perf_counter()
        order = routes.order(name, points3, origin)
        calc_ms = (time.perf_counter() - t0) * 1000.0
        self.out(
            f'{name}: ordem {" ".join(str(i + 1) for i in order)}  '
            f'circuito {routes.cost(order, points3, origin):.0f} cm  '
            f'(calculado em {calc_ms:.1f} ms)'
        )
        start = time.monotonic()
        for i, index in enumerate(order, 1):
            self.out(f'ponto {i}/{len(order)}: {points3[index]}')
            self._require_hold('route')
            self._go(*points3[index])
        self.out('voltando à partida')
        self._require_hold('route')
        self._go(*origin)
        self.out(f'route concluída em {time.monotonic() - start:.0f} s de voo')

    def _takeoff(self, height_cm: float) -> None:
        if self.state not in GROUND_STATES:
            raise LinkError(f'decolar só no chão; agora: {self.state}')
        if not self.session:
            self.link.start_session()
            self.session = True
        self.link.arm(True)
        self.out(f'armado; decolando para {height_cm:.0f} cm...')
        height = height_cm / 100.0
        wait = height / Z_RATE_M_S + MARGIN_S
        cmd = self.link.takeoff(height, timeout_s=wait)
        self._wait_leg(cmd, wait + 5.0, f'decolar {height_cm:.0f} cm')

    def land(self, wait_s: float = 30.0) -> None:
        """Pousa e espera desarmar (também usado ao sair e em erro)."""
        if self.state in GROUND_STATES:
            self.out('já está no chão')
        else:
            self.out('pousando...')
            self.link.land()
            end = time.monotonic() + wait_s
            while time.monotonic() < end and self.state not in GROUND_STATES:
                time.sleep(0.2)
            if self.state not in GROUND_STATES:
                self.out('não desarmou sozinho: desarmando')
                self.link.arm(False)
            self._leg_end = time.monotonic()
            self.out(f'no chão. {self.summary()}')
        if self.session:
            self.link.end_session()
            self.session = False

    def run(self, cmd: Command) -> bool:
        """Executa um comando até o fim. False = pedir para sair."""
        name, n = cmd.name, cmd.numbers
        if name in NEED_HOLD:
            self._require_hold(name)
        if name == 'help':
            for _, text in COMMANDS.values():
                self.out('  ' + text)
        elif name == 'status':
            self.out(self.summary())
        elif name == 'exit':
            return False
        elif name == 'takeoff':
            self._takeoff(n[0])
        elif name == 'yaw':
            self._turn(n[0])
        elif name in MOVES:
            self._move(*(s * n[0] for s in MOVES[name]), f'{name} {n[0]} cm')
        elif name == 'goto':
            self._go(*n)
        elif name == 'waypoints':
            for i, point in enumerate(cmd.points, 1):
                self.out(f'waypoint {i}/{len(cmd.points)}')
                self._require_hold('waypoints')
                self._go(*point)
        elif name == 'route':
            points = cmd.points
            if not points:
                points = read_points_file(self.points_file)
                _check_size(cmd.algorithm, points)
                self.out(f'{len(points)} pontos de {self.points_file}')
            self._visit(cmd.algorithm, points)
        elif name == 'rtl':
            self._go(0.0, 0.0)
        elif name == 'hold':
            self.link.hold()
            self.out('segurando a posição')
        elif name == 'wait':
            end = time.monotonic() + n[0]
            while time.monotonic() < end:
                self.out(self.summary())
                time.sleep(min(1.0, max(0.0, end - time.monotonic())))
        elif name == 'land':
            self.land()
        return True

    def kill(self) -> None:
        """Ctrl+C: PARADA DE EMERGÊNCIA (desarma na hora, sem pousar)."""
        self.killed = True
        ok = self.link.emergency_stop()
        self.session = False
        self.out(
            '\nPARADA DE EMERGÊNCIA: motores cortados'
            + ('' if ok else ' (sem confirmação do STM32: confira e tire a bateria)')
        )

    def _finish(self) -> None:
        if self.killed:
            return
        if self.state not in GROUND_STATES:
            self.land()
        elif self.session:
            self.link.end_session()
            self.session = False

    def run_lines(self, lines: Iterable[str]) -> bool:
        """Roda uma lista de comandos; para no primeiro erro e pousa. True = tudo certo.

        Ctrl+C é a parada de emergência: corta os motores e não pousa.
        """
        number = 0
        try:
            for number, line in enumerate(lines, 1):
                cmd = parse(line)
                if cmd is None:
                    continue
                self.out(f'> {line.strip()}')
                if not self.run(cmd):
                    break
            return True
        except (LinkError, ValueError) as exc:
            self.out(f'erro na linha {number}: {exc}; parando a missão')
            return False
        except KeyboardInterrupt:
            self.kill()
            return False
        finally:
            self._finish()

    def shell(self, read: Callable[[str], str] = input) -> None:
        """Prompt interativo. Ctrl+C = parada de emergência; erro num comando não derruba o shell."""
        self.out(
            'missão: digite "help". Cada comando espera o drone ficar estável. '
            '"land" pousa; Ctrl+C CORTA OS MOTORES.'
        )
        try:
            while True:
                try:
                    line = read(f'[{self.state}] mission> ')
                except EOFError:
                    break
                try:
                    cmd = parse(line)
                    if cmd is not None and not self.run(cmd):
                        break
                except (LinkError, ValueError) as exc:
                    self.out(f'erro: {exc}')
        except KeyboardInterrupt:
            self.kill()
        finally:
            self._finish()
