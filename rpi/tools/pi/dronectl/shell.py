"""Shell de ajuste em tempo real (``hover --debug`` e ``tune``).

Cada linha digitada é um comando. Os pesos do LQR e do Kalman ficam na Pi;
``apply`` recalcula Kd, Kfd, L e M (design.py) e os envia ao STM32, que troca
todos de uma vez no próximo passo de 10 ms. Os parâmetros do STM32 mudam na
hora com ``set``.
"""
from __future__ import annotations

import math
import shlex
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional

from .design import Weights, design, load_profile, save_profile
from .link import DroneLink, LinkError, state_name
from .monitor import FilterStats

HELP = """\
filtragem:  stats | watch [s] | rn auto
pesos:      q <estado|all> <v> | r <motor|all> <v> | qn <estado|all> <v> | rn <medida|all> <v>
            pesos | apply (recalcula e envia) | reset (ganhos do Simulink)
parâmetros: set NOME VALOR | get NOME | params
voo:        ref z <m> | goto <x> <y> <z> | move <frente> <direita> [baixo] | yaw <graus>
            hold | land | disarm
perfil:     save <arquivo.json> | load <arquivo.json>
            sair (em voo, pousa antes)
estados: x y z phi theta psi vx vy vz p q r   medidas: z phi theta psi vx vy p q r"""


class TuningShell:
    """Lê comandos e os aplica pelo enlace."""

    def __init__(self, link: DroneLink, stats: FilterStats, weights: Optional[Weights] = None,
                 flying: bool = False, say: Callable[[str], None] = print) -> None:
        self.link = link
        self.stats = stats
        self.weights = weights or Weights()
        self.flying = flying
        self.say = say
        self.changed_params: Dict[str, float] = {}

    # ---- comandos ------------------------------------------------------

    def _weight(self, kind: str, args: List[str]) -> None:
        if kind == 'rn' and args == ['auto']:
            suggested = self.stats.suggested_rn()
            for i, value in enumerate(suggested):
                if math.isfinite(value) and value > 0:
                    self.say(self.weights.set('rn', i, value))
            self.say('use "apply" para enviar')
            return
        if len(args) != 2:
            raise ValueError(f'uso: {kind} <nome|all> <valor>')
        which = None if args[0] == 'all' else args[0]
        self.say(self.weights.set(kind, which, float(args[1])) + '  (use "apply" para enviar)')

    def _apply(self) -> None:
        gains, info = design(self.weights)
        self.say(f'maior polo: controle {info["controle_maior_polo"]:.6f}, '
                 f'estimador {info["estimador_maior_polo"]:.6f}')
        if info['controle_maior_polo'] >= 1.0 or info['estimador_maior_polo'] >= 1.0:
            raise ValueError('projeto instável; nada foi enviado')
        crc = self.link.upload_gains(gains)
        self.say(f'ganhos novos em uso (crc {crc:08X})')

    def _reset(self) -> None:
        self.weights = Weights()
        crc = self.link.restore_gains()
        self.say(f'ganhos e pesos do Simulink restaurados (crc {crc:08X})')

    def _set(self, name: str, value: str) -> None:
        got = self.link.param_set(name, float(value))
        if not math.isclose(got, float(value), rel_tol=1e-6, abs_tol=1e-9):
            self.say(f'{name.upper()} continua {got:g} (recusado)')
        else:
            self.changed_params[name.upper()] = got
            self.say(f'{name.upper()} = {got:g}')

    def _ref_z(self, height: float) -> None:
        last = self.link.last_debug
        x, y = (last.ref[0], last.ref[1]) if last else (0.0, 0.0)
        psi = math.degrees(last.ref[3]) if last else 0.0
        self.link.goto(x, y, -abs(height), psi)
        self._leg_sent()

    def _leg_sent(self) -> None:
        self.say('trecho aceito; acompanhe com "watch"')

    def _watch(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        try:
            while time.monotonic() < end:
                print('\033[2J\033[H' + self.stats.table(self.weights.rn), flush=True)
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass

    def _save(self, path: str) -> None:
        save_profile(Path(path), self.weights, self.changed_params)
        self.say(f'perfil salvo em {path}')

    def _load(self, path: str) -> None:
        weights, params = load_profile(Path(path))
        for name, value in params.items():
            self._set(name, str(value))
        self.weights = weights
        self._apply()

    def execute(self, line: str) -> bool:
        """Executa uma linha. Devolve False quando o operador quer sair."""
        words = shlex.split(line)
        if not words:
            return True
        cmd, args = words[0].lower(), words[1:]
        try:
            if cmd in ('sair', 'quit', 'exit'):
                return False
            if cmd in ('ajuda', 'help', '?'):
                self.say(HELP)
            elif cmd == 'stats':
                self.say(self.stats.table(self.weights.rn))
            elif cmd == 'watch':
                self._watch(float(args[0]) if args else 10.0)
            elif cmd in ('q', 'r', 'qn', 'rn'):
                self._weight(cmd, args)
            elif cmd == 'pesos':
                w = self.weights
                self.say(f'Q  = {w.q}\nR  = {w.r}\nQn = {w.qn}\nRn = {w.rn}')
            elif cmd == 'apply':
                self._apply()
            elif cmd == 'reset':
                self._reset()
            elif cmd == 'set' and len(args) == 2:
                self._set(args[0], args[1])
            elif cmd == 'get' and len(args) == 1:
                self.say(f'{args[0].upper()} = {self.link.param_get(args[0]):g}')
            elif cmd == 'params':
                for name, value in self.link.param_list().items():
                    self.say(f'{name:>14} = {value:g}')
            elif cmd == 'ref' and len(args) == 2 and args[0] == 'z':
                self._ref_z(float(args[1]))
            elif cmd == 'goto' and len(args) == 3:
                self.link.goto(*(float(a) for a in args))
                self._leg_sent()
            elif cmd == 'move' and len(args) in (2, 3):
                self.link.move(*(float(a) for a in args))
                self._leg_sent()
            elif cmd == 'yaw' and len(args) == 1:
                self.link.rotate(float(args[0]))
                self._leg_sent()
            elif cmd == 'hold':
                self.link.hold()
            elif cmd == 'land':
                self.link.land()
                self.say('pousando')
            elif cmd == 'disarm':
                self.link.arm(False)
            elif cmd == 'save' and len(args) == 1:
                self._save(args[0])
            elif cmd == 'load' and len(args) == 1:
                self._load(args[0])
            else:
                self.say('comando desconhecido (digite ajuda)')
        except (ValueError, IndexError, LinkError) as exc:
            self.say(f'erro: {exc}')
        return True

    def run(self) -> None:
        """Lê comandos até "sair", fim da entrada ou o drone voltar ao chão."""
        self.say('shell de ajuste: digite ajuda. Ctrl+C pousa (em voo) e sai.')
        while True:
            state = state_name(self.link.flight_state)
            if self.flying and state in ('STANDBY', 'FAILSAFE'):
                self.say(f'drone em {state}: saindo do shell')
                return
            try:
                line = input(f'[{state}] > ')
            except EOFError:
                return
            if not self.execute(line):
                return
