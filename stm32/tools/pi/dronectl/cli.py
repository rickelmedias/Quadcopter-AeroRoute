"""Linha de comando ``dronectl`` (rodar na Raspberry Pi, por SSH).

Exemplos:

    python -m dronectl status
    python -m dronectl monitor                 # filtragem dos sensores, sem armar
    python -m dronectl hover --height 0.5 --time 20
    python -m dronectl hover --debug           # hover com shell de ajuste em tempo real
    python -m dronectl tune                    # shell de ajuste com o drone no chão
    python -m dronectl param set CTL_W0 690
    python -m dronectl design                  # confere o projeto contra o Simulink
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

from .link import DroneLink, LinkError, state_name


def _status_line(link: DroneLink) -> str:
    """Resumo de uma linha com a estimativa do Kalman (LOCAL_POSITION_NED/ATTITUDE)."""
    pos, att = link.position, link.attitude
    parts = [f'estado {state_name(link.flight_state):<8}']
    if pos is not None:
        parts.append(f'h={-pos.z:5.2f} m  x={pos.x:+5.2f} y={pos.y:+5.2f}  vz={pos.vz:+5.2f}')
    if att is not None:
        parts.append(f'φ={math.degrees(att.roll):+5.1f}° θ={math.degrees(att.pitch):+5.1f}° '
                     f'ψ={math.degrees(att.yaw):+6.1f}°')
    if not link.link_ok:
        parts.append('SEM ENLACE')
    return '  '.join(parts)


def _connect(args) -> DroneLink:
    link = DroneLink(args.port, args.baud)
    link.wait_fc()
    return link


def _start_debug_stream(link: DroneLink, args, default_csv: Optional[str]):
    """Liga o DRONE_CTRL_DEBUG, a estatística da filtragem e o CSV."""
    from .monitor import CsvRecorder, FilterStats

    stats = FilterStats(window=args.window)
    link.subscribe(stats.add)
    csv_path = args.csv or default_csv
    recorder = None
    if csv_path:
        recorder = CsvRecorder(Path(csv_path))
        link.subscribe(recorder.add)
        print(f'gravando telemetria em {csv_path}')
    link.param_set('DBG_RATE_HZ', args.rate)
    return stats, recorder


def _default_csv(prefix: str) -> str:
    return f'logs/{prefix}_{datetime.now():%Y%m%d_%H%M%S}.csv'


def _apply_profile(link: DroneLink, profile: Optional[str]):
    """Aplica um perfil salvo (parâmetros e ganhos) antes de armar."""
    from .design import Weights, design, load_profile

    if not profile:
        return None
    weights, params = load_profile(Path(profile))
    for name, value in params.items():
        print(f'{name} = {link.param_set(name, value):g}')
    if weights.to_dict() != Weights().to_dict():
        gains, _ = design(weights)
        print(f'ganhos do perfil em uso (crc {link.upload_gains(gains):08X})')
    return weights


# ---- comandos --------------------------------------------------------------

def cmd_status(args) -> int:
    with _connect(args) as link:
        time.sleep(0.5)
        crc, custom = link.query_gains()
        print(f'boot {link.boot_id}  capacidades 0x{link.capabilities:X}')
        print(f'ganhos {"enviados pela Pi" if custom else "do Simulink"} (crc {crc:08X})')
        if link.battery_v is None and link.battery_a is None:
            print('bateria: leitura desligada no firmware (power.h)')
        else:
            volts = f'{link.battery_v:.2f} V' if link.battery_v is not None else '? V'
            amps = f'{link.battery_a:.2f} A' if link.battery_a is not None else '? A'
            print(f'bateria: {volts}  {amps}')
        print(_status_line(link))
    return 0


def cmd_monitor(args) -> int:
    """Mostra a filtragem (y × x̂) sem armar; Ctrl+C sai."""
    with _connect(args) as link:
        stats, recorder = _start_debug_stream(link, args, None)
        try:
            while True:
                time.sleep(1.0)
                print('\033[2J\033[H' + stats.table(), flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            link.param_set('DBG_RATE_HZ', 0)
            if recorder:
                recorder.close()
    return 0


def cmd_tune(args) -> int:
    """Shell de ajuste sem armar: ideal para ajustar o Kalman olhando a filtragem."""
    from .shell import TuningShell

    with _connect(args) as link:
        stats, recorder = _start_debug_stream(link, args, None)
        weights = _apply_profile(link, args.profile)
        try:
            TuningShell(link, stats, weights).run()
        except KeyboardInterrupt:
            print()
        finally:
            link.param_set('DBG_RATE_HZ', 0)
            if recorder:
                recorder.close()
    return 0


def _land_and_wait(link: DroneLink, timeout: float = 20.0) -> None:
    """Pede pouso e espera o drone desarmar."""
    try:
        link.land()
    except LinkError as exc:
        print(f'pouso: {exc}')
    end = time.monotonic() + timeout
    while time.monotonic() < end and state_name(link.flight_state) not in ('STANDBY',
                                                                          'FAILSAFE'):
        time.sleep(0.2)
    if state_name(link.flight_state) not in ('STANDBY', 'FAILSAFE'):
        print('não desarmou sozinho: desarmando')
        link.arm(False)


def cmd_hover(args) -> int:
    """Arma, decola, segura e pousa. Com --debug abre o shell de ajuste."""
    from .shell import TuningShell

    with _connect(args) as link:
        stats = recorder = None
        if args.debug or args.csv:
            stats, recorder = _start_debug_stream(
                link, args, _default_csv('hover') if args.debug else None)
        weights = _apply_profile(link, args.profile)
        if args.dry:
            link.param_set('FLT_DRY_RUN', 1)
        dry = link.param_get('FLT_DRY_RUN') >= 0.5
        print('ENSAIO A SECO: motores parados' if dry else
              'MOTORES VÃO GIRAR. Ctrl+C pousa.')
        link.start_session()
        try:
            link.arm(True)
            print(f'armado; decolando para {args.height:.2f} m')
            command = link.takeoff(args.height)
            if not link.wait_leg(command, timeout=30.0):
                raise LinkError('decolagem não concluída')
            print('HOLD')
            if args.debug:
                TuningShell(link, stats, weights, flying=True).run()
            else:
                end = time.monotonic() + args.time
                while time.monotonic() < end:
                    print(_status_line(link), flush=True)
                    if state_name(link.flight_state) in ('STANDBY', 'FAILSAFE'):
                        raise LinkError(f'o drone saiu do voo ({state_name(link.flight_state)})')
                    time.sleep(1.0)
        except KeyboardInterrupt:
            print('\nCtrl+C: pousando')
        except LinkError as exc:
            print(f'erro: {exc}')
        finally:
            if link.armed or state_name(link.flight_state) not in ('STANDBY', 'FAILSAFE'):
                _land_and_wait(link)
            link.end_session()
            if stats is not None:
                link.param_set('DBG_RATE_HZ', 0)
            if recorder:
                recorder.close()
        print(_status_line(link))
    return 0


def cmd_land(args) -> int:
    with _connect(args) as link:
        link.start_session()
        _land_and_wait(link)
        link.end_session()
    return 0


def cmd_disarm(args) -> int:
    with _connect(args) as link:
        link.arm(False)
        print('desarmado')
    return 0


def cmd_param(args) -> int:
    with _connect(args) as link:
        if args.action == 'list':
            for name, value in link.param_list().items():
                print(f'{name:>14} = {value:g}')
        elif args.action == 'get':
            print(f'{args.name.upper()} = {link.param_get(args.name):g}')
        else:
            got = link.param_set(args.name, args.value)
            print(f'{args.name.upper()} = {got:g}')
            if not math.isclose(got, args.value, rel_tol=1e-6, abs_tol=1e-9):
                return 1
    return 0


def cmd_gains(args) -> int:
    from .design import Weights, design, load_profile

    with _connect(args) as link:
        if args.action == 'reset':
            print(f'ganhos do Simulink em uso (crc {link.restore_gains():08X})')
        elif args.action == 'query':
            crc, custom = link.query_gains()
            print(f'{"enviados pela Pi" if custom else "do Simulink"} (crc {crc:08X})')
        else:
            weights = load_profile(Path(args.profile))[0] if args.profile else Weights()
            gains, info = design(weights)
            print(f'maior polo: controle {info["controle_maior_polo"]:.6f}, '
                  f'estimador {info["estimador_maior_polo"]:.6f}')
            print(f'ganhos em uso (crc {link.upload_gains(gains):08X})')
    return 0


def cmd_design(args) -> int:
    """Projeto offline: confere contra o Simulink e mostra os polos (não usa o STM32)."""
    import numpy as np
    from .design import Weights, compare, design, load_profile
    from .gains import default_gains

    weights = load_profile(Path(args.profile))[0] if args.profile else Weights()
    gains, info = design(weights)
    print(f'maior polo: controle {info["controle_maior_polo"]:.6f}, '
          f'estimador {info["estimador_maior_polo"]:.6f}')
    print('diferença relativa para o Simulink:',
          ', '.join(f'{k}={v:.2e}' for k, v in compare(gains, default_gains()).items()))
    np.set_printoptions(precision=4, suppress=True, linewidth=150)
    print('Kd =\n', np.array(gains.Kd))
    print(f'crc {gains.crc32():08X}')
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog='dronectl', description='Comando e ajuste do drone')
    parser.add_argument('-p', '--port', default='/dev/serial0',
                        help='porta serial do STM32 (ou URL do pyserial)')
    parser.add_argument('-b', '--baud', type=int, default=115200)
    sub = parser.add_subparsers(dest='command', required=True)

    def stream_options(p):
        p.add_argument('--rate', type=float, default=25.0, help='DRONE_CTRL_DEBUG, em Hz')
        p.add_argument('--window', type=int, default=250, help='amostras na estatística')
        p.add_argument('--csv', help='grava a telemetria neste CSV')
        p.add_argument('--profile', help='perfil de ajuste (JSON) aplicado antes de começar')

    sub.add_parser('status', help='estado, ganhos e estimativa').set_defaults(func=cmd_status)
    p = sub.add_parser('monitor', help='filtragem dos sensores ao vivo, sem armar')
    stream_options(p)
    p.set_defaults(func=cmd_monitor)
    p = sub.add_parser('tune', help='shell de ajuste com o drone no chão')
    stream_options(p)
    p.set_defaults(func=cmd_tune)
    p = sub.add_parser('hover', help='decola, segura e pousa')
    p.add_argument('--height', type=float, default=0.5, help='altura, em m')
    p.add_argument('--time', type=float, default=20.0, help='tempo em HOLD, em s (sem --debug)')
    p.add_argument('--dry', action='store_true', help='força FLT_DRY_RUN=1 (motores parados)')
    p.add_argument('--debug', action='store_true', help='shell de ajuste em tempo real')
    stream_options(p)
    p.set_defaults(func=cmd_hover)
    sub.add_parser('land', help='pousa e desarma').set_defaults(func=cmd_land)
    sub.add_parser('disarm', help='desarma na hora').set_defaults(func=cmd_disarm)
    p = sub.add_parser('param', help='parâmetros do STM32')
    p.add_argument('action', choices=['list', 'get', 'set'])
    p.add_argument('name', nargs='?')
    p.add_argument('value', nargs='?', type=float)
    p.set_defaults(func=cmd_param)
    p = sub.add_parser('gains', help='ganhos do LQG no STM32')
    p.add_argument('action', choices=['query', 'reset', 'upload'])
    p.add_argument('--profile', help='perfil com os pesos (upload)')
    p.set_defaults(func=cmd_gains)
    p = sub.add_parser('design', help='projeto offline e comparação com o Simulink')
    p.add_argument('--profile', help='perfil com os pesos')
    p.set_defaults(func=cmd_design)

    args = parser.parse_args(argv)
    if args.command == 'param' and args.action != 'list' and not args.name:
        parser.error('informe o nome do parâmetro')
    if args.command == 'param' and args.action == 'set' and args.value is None:
        parser.error('informe o valor')
    try:
        return args.func(args)
    except LinkError as exc:
        print(f'erro: {exc}', file=sys.stderr)
        return 1
