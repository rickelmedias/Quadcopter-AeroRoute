"""Linha de comando do computador de missão (Raspberry Pi).

    python -m dronectl status                    estado, identidade de boot, bateria
    python -m dronectl monitor [--time 30]       acompanha estado e posição
    python -m dronectl hover --height 30 --time 20
    python -m dronectl mission [arquivo]         missão (shell interativo sem arquivo)
    python -m dronectl land | disarm
    python -m dronectl routes [alg] x,y[,z] ...  só calcula a ordem (sem drone)

``--port`` (padrão /dev/serial0) e ``--baud`` (115200) valem para todos os que
falam com o STM32. O nome antigo ``missao`` continua aceito.

Em ``mission`` e ``hover``, Ctrl+C é a PARADA DE EMERGÊNCIA: desarma na hora (no ar,
o drone cai). Para pousar, use ``land``. Se a Pi cair sem Ctrl+C (SSH, energia), o
STM32 segura a posição e pousa sozinho.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from datetime import datetime
from pathlib import Path

from . import routes
from .hover import DEFAULT_ROOT, Recorder, analyze, format_summary, run_hover
from .link import GROUND_STATES, DroneLink, LinkError
from .mission import ROUTE_ALGORITHMS, Mission, parse_points


def _connect(args) -> DroneLink:
    link = DroneLink(args.port, args.baud)
    link.wait_fc(timeout=5.0)
    return link


def _status_line(link: DroneLink) -> str:
    text = f'estado {link.state}  armado={"sim" if link.armed else "não"}'
    if link.position is not None and link.attitude is not None:
        p, a = link.position, link.attitude
        text += (
            f'  x={p.x * 100:+.0f} y={p.y * 100:+.0f} cm altura={-p.z * 100:.0f} cm'
            f'  guinada={math.degrees(a.yaw):+.0f}°'
        )
    if link.battery_v is not None:
        text += f'  bateria={link.battery_v:.2f} V'
    if link.simulated:
        text += '  [FLT_SIM]'
    return text


def cmd_status(args) -> int:
    with _connect(args) as link:
        time.sleep(0.6)
        print(
            f'boot {link.boot_id}  DroneLink {link.protocol[0]}.{link.protocol[1]}  '
            f'capacidades 0x{link.capabilities:02X}'
        )
        print(_status_line(link))
    return 0


def cmd_monitor(args) -> int:
    """Uma linha por segundo (e cada aviso do STM32) com hora, boot e motores.

    ``motores`` é o que o FIRMWARE está mandando (DRONE_CTRL_DEBUG, 10 Hz): se os
    motores girarem com isso em 0 %, quem girou foi o ESC, não o controle. Um boot
    novo no meio do log é um reinício do STM32.
    """
    log = open(args.log, 'a', encoding='utf-8') if args.log else None

    def emit(text: str) -> None:
        line = f'{datetime.now().strftime("%H:%M:%S.%f")[:-3]}  {text}'
        print(line, flush=True)
        if log is not None:
            log.write(line + '\n')
            log.flush()

    peak = [0.0]

    def on_debug(sample) -> None:
        peak[0] = max(peak[0], max(sample.throttle))

    link = DroneLink(args.port, args.baud, on_text=lambda text: emit(f'[STM32] {text}'))
    try:
        link.wait_fc(timeout=5.0)
        link.subscribe(on_debug)
        end = time.monotonic() + args.time if args.time else None
        try:
            while end is None or time.monotonic() < end:
                fresh = link.last_debug is not None and link.link_ok
                motors = f'{peak[0] * 100:4.0f} %' if fresh else '   ? '
                peak[0] = 0.0
                emit(
                    f'boot {link.boot_id}  {_status_line(link)}  motores(máx no s) {motors}'
                    f'  enlace {"ok" if link.link_ok else "SEM HEARTBEAT"}'
                    f'  perdas {link.rx_gaps} fora-de-ordem {link.rx_reordered}'
                    f' repetidas {link.rx_repeats} erros-STM32 {link.fc_link_errors}'
                )
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
    finally:
        link.close()
        if log is not None:
            log.close()
    return 0


def cmd_hover(args) -> int:
    with _connect(args) as link:
        result = run_hover(
            link, args.height, args.time, root=Path(args.out), record=not args.no_record
        )
    return 0 if result.get('completed') else 1


def cmd_mission(args) -> int:
    with _connect(args) as link:
        mission = Mission(link)
        recorder = None
        if args.record:
            name = Path(args.file).stem if args.file else 'shell'
            folder = Path(args.out) / datetime.now().strftime(f'mission_{name}_%Y%m%d_%H%M%S')
            recorder = Recorder(link, folder / 'telemetry.csv')
            link.subscribe(recorder)
            show = mission.out
            mission.out = lambda text: (recorder.note(text), show(text))
            link.on_text = lambda text: mission.out(f'[STM32] {text}')
        try:
            if args.file:
                lines = Path(args.file).read_text(encoding='utf-8').splitlines()
                ok = mission.run_lines(lines)
            else:
                mission.shell()
                ok = True
        finally:
            if recorder is not None:
                link.unsubscribe(recorder)
                recorder.close()
                print(format_summary(analyze(recorder.samples)))
                print(f'gravado em {folder}')
    return 0 if ok else 1


def cmd_land(args) -> int:
    with _connect(args) as link:
        if link.state in GROUND_STATES:
            print('já está no chão')
            return 0
        Mission(link).land()
    return 0


def cmd_disarm(args) -> int:
    with _connect(args) as link:
        link.arm(False)
        print('desarmado')
    return 0


def cmd_routes(args) -> int:
    algorithm = args.algorithm
    points = parse_points(args.points)
    origin = (0.0, 0.0, float(args.origin_z))
    points3 = [(p[0], p[1], p[2] if len(p) > 2 else origin[2]) for p in points]
    name = ROUTE_ALGORITHMS[algorithm]
    t0 = time.perf_counter()
    order = routes.order(name, points3, origin)
    ms = (time.perf_counter() - t0) * 1000.0
    print(
        f'{name}: ordem {" ".join(str(i + 1) for i in order)}  '
        f'circuito {routes.cost(order, points3, origin):.0f} cm  ({ms:.1f} ms)'
    )
    for i in order:
        print(f'  {points3[i]}')
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog='dronectl', description=__doc__.split('\n\n')[0])
    sub = parser.add_subparsers(dest='command', required=True)

    def link_options(p):
        p.add_argument('--port', default='/dev/serial0', help='porta serial ou URL do pyserial')
        p.add_argument('--baud', type=int, default=115200)
        return p

    link_options(sub.add_parser('status', help='Estado e identidade do STM32')).set_defaults(
        func=cmd_status
    )
    p = link_options(sub.add_parser('monitor', help='Acompanha estado, posição e bateria'))
    p.add_argument('--time', type=float, default=0.0, help='segundos (0 = até Ctrl+C)')
    p.add_argument('--log', help='também grava as linhas neste arquivo (acrescenta)')
    p.set_defaults(func=cmd_monitor)
    p = link_options(sub.add_parser('hover', help='Decola, segura, grava e pousa'))
    p.add_argument('--height', type=int, default=30, help='altura em cm (10..80)')
    p.add_argument('--time', type=float, default=20.0, help='segundos em HOLD')
    p.add_argument('--out', default=str(DEFAULT_ROOT), help='pasta dos ensaios')
    p.add_argument('--no-record', action='store_true', help='não grava arquivos')
    p.set_defaults(func=cmd_hover)
    p = link_options(
        sub.add_parser(
            'mission', aliases=['missao'], help='Comandos passo a passo (takeoff, goto, route...)'
        )
    )
    p.add_argument('file', nargs='?', help='arquivo de comandos (sem ele: shell interativo)')
    p.add_argument('--record', action='store_true', help='grava a telemetria (DRONE_CTRL_DEBUG)')
    p.add_argument('--out', default=str(DEFAULT_ROOT), help='pasta dos ensaios')
    p.set_defaults(func=cmd_mission)
    link_options(sub.add_parser('land', help='Pousa e desarma')).set_defaults(func=cmd_land)
    link_options(sub.add_parser('disarm', help='Desarma na hora')).set_defaults(func=cmd_disarm)
    p = sub.add_parser('routes', help='Ordem de visita dos pontos (sem drone)')
    p.add_argument('algorithm', nargs='?', default='ga', choices=sorted(ROUTE_ALGORITHMS))
    p.add_argument('points', nargs='+', help='x,y[,z] em cm')
    p.add_argument('--origin-z', type=int, default=50, help='altura da partida em cm')
    p.set_defaults(func=cmd_routes)

    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except (LinkError, ValueError) as exc:
        print(f'erro: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('\ninterrompido', file=sys.stderr)
        return 130
