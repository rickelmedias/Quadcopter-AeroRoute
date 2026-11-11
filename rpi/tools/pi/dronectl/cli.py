"""Linha de comando ``dronectl`` (rodar na Raspberry Pi, por SSH).

Exemplos:

    python -m dronectl status
    python -m dronectl monitor                 # filtragem dos sensores, sem armar
    python -m dronectl hover --height 0.5 --time 20
    python -m dronectl hover --debug           # hover com shell de ajuste em tempo real
    python -m dronectl tune                    # shell de ajuste com o drone no chão
    python -m dronectl param set CTL_W0 690
    python -m dronectl design                  # confere o projeto contra o Simulink
    python -m dronectl missao --sim            # shell de missão contra o drone simulado
    python -m dronectl missao --sim rota.txt   # missão de um arquivo
    python -m dronectl hover-test --motors     # ensaio de hover gravado e analisado
    python -m dronectl hover-report            # reanalisa o último ensaio
    python -m dronectl hover-profile           # perfil com que o próximo ensaio começa
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
        parts.append(
            f'φ={math.degrees(att.roll):+5.1f}° θ={math.degrees(att.pitch):+5.1f}° '
            f'ψ={math.degrees(att.yaw):+6.1f}°'
        )
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
    while time.monotonic() < end and state_name(link.flight_state) not in ('STANDBY', 'FAILSAFE'):
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
                link, args, _default_csv('hover') if args.debug else None
            )
        weights = _apply_profile(link, args.profile)
        if args.dry:
            link.param_set('FLT_DRY_RUN', 1)
        dry = link.param_get('FLT_DRY_RUN') >= 0.5
        print('ENSAIO A SECO: motores parados' if dry else 'MOTORES VÃO GIRAR. Ctrl+C pousa.')
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
            print(
                f'maior polo: controle {info["controle_maior_polo"]:.6f}, '
                f'estimador {info["estimador_maior_polo"]:.6f}'
            )
            print(f'ganhos em uso (crc {link.upload_gains(gains):08X})')
    return 0


def cmd_design(args) -> int:
    """Projeto offline: confere contra o Simulink e mostra os polos (não usa o STM32)."""
    import numpy as np
    from .design import Weights, compare, design, load_profile
    from .gains import default_gains

    weights = load_profile(Path(args.profile))[0] if args.profile else Weights()
    gains, info = design(weights)
    print(
        f'maior polo: controle {info["controle_maior_polo"]:.6f}, '
        f'estimador {info["estimador_maior_polo"]:.6f}'
    )
    print(
        'diferença relativa para o Simulink:',
        ', '.join(f'{k}={v:.2e}' for k, v in compare(gains, default_gains()).items()),
    )
    np.set_printoptions(precision=4, suppress=True, linewidth=150)
    print('Kd =\n', np.array(gains.Kd))
    print(f'crc {gains.crc32():08X}')
    return 0


def cmd_missao(args) -> int:
    """Shell de missão ou arquivo: cada comando espera o drone ficar estável."""
    from .missao import Missao

    with _connect(args) as link:
        if args.sim:
            if link.param_set('FLT_SIM', 1) < 0.5:
                print('erro: o STM32 recusou FLT_SIM=1 (desarme antes)')
                return 1
            print('DRONE SIMULADO no STM32 (FLT_SIM=1): motores parados')
        else:
            sim = link.param_get('FLT_SIM') >= 0.5
            dry = link.param_get('FLT_DRY_RUN') >= 0.5
            if sim:
                print('DRONE SIMULADO no STM32 (FLT_SIM=1): motores parados')
            elif dry:
                print(
                    'aviso: FLT_DRY_RUN=1 sem FLT_SIM: os motores não giram e a decolagem '
                    'nunca termina. Use --sim para o teste de comunicação.'
                )
            else:
                print('MOTORES VÃO GIRAR (voo real). Ctrl+C pousa.')
        missao = Missao(link, arquivo_pontos=args.pontos) if args.pontos else Missao(link)
        if args.arquivo:
            linhas = Path(args.arquivo).read_text(encoding='utf-8').splitlines()
            ok = missao.executar_linhas(linhas)
        else:
            missao.shell()
            ok = True
        if args.sim and args.desligar_sim:
            link.param_set('FLT_SIM', 0)
    return 0 if ok else 1


def _hover_profile(args):
    from .hover_test import PROFILE_PATH, Profile

    return Profile.load(Path(args.profile) if args.profile else PROFILE_PATH)


def cmd_hover_test(args) -> int:
    """Ensaio de hover: perfil + ajustes, voo com proteções, gravação e análise."""
    from .hover_test import RESULTS_DIR, Options, parse_assignments, run_hover_test

    profile = _hover_profile(args)
    mode = 'motors' if args.motors else 'sim' if args.sim else 'dry'
    try:
        params = parse_assignments(args.set)
        q = parse_assignments(args.q, upper=False)
        opts = Options(
            height_m=(args.height if args.height is not None else profile.height_cm) / 100,
            time_s=args.time if args.time is not None else profile.time_s,
            mode=mode,
            params=params,
            q=q,
            r=args.r,
            rate_hz=args.rate,
            geofence_m=args.geofence / 100,
            label=args.label or '',
            out_dir=Path(args.out) if args.out else RESULTS_DIR,
            countdown_s=args.countdown,
        )
        opts.check()
    except ValueError as exc:
        print(f'erro: {exc}', file=sys.stderr)
        return 2
    if args.save:
        profile.params.update(params)
        profile.q.update(q)
        if args.r is not None:
            profile.r = args.r
        if args.height is not None:
            profile.height_cm = args.height
        if args.time is not None:
            profile.time_s = args.time
        profile.save()
        print('perfil atualizado com os ajustes deste ensaio')
    print(
        {
            'motors': 'VOO REAL',
            'sim': 'DRONE SIMULADO no STM32 (motores parados)',
            'dry': 'A SECO: sensores reais, motores parados',
        }[mode]
        + f': {opts.height_m * 100:g} cm por {opts.time_s:g} s'
    )
    with _connect(args) as link:
        test = run_hover_test(link, opts, profile)
    return 1 if test.error else 0


def _hover_folder(args) -> Path:
    from .hover_test import RESULTS_DIR

    if args.folder and args.folder != 'latest':
        return Path(args.folder)
    base = Path(args.dir) if args.dir else RESULTS_DIR
    folders = sorted(base.glob('hover_*'))
    if not folders:
        raise LinkError(f'nenhum ensaio em {base}')
    return folders[-1]


def cmd_hover_report(args) -> int:
    """Reanalisa um ensaio gravado (não usa o drone)."""
    from .hover_report import format_report, report_folder

    folder = _hover_folder(args)
    report = report_folder(folder)
    print(folder)
    if args.timeline:
        print('\n'.join(report.timeline) + '\n')
    print(format_report(report))
    if args.plot:
        from .hover_plot import plot_folder

        print(f'gráfico em {plot_folder(folder)}')
    return 0


def cmd_hover_profile(args) -> int:
    """Mostra ou muda o perfil dos ensaios de hover (não usa o drone)."""
    from .hover_test import MANAGED, Profile, check_q_names, parse_assignments

    profile = _hover_profile(args)
    changed = []
    try:
        if args.reset:
            profile = Profile(
                path=profile.path,
                history=profile.history,
                boot_id=profile.boot_id,
                boot_values=profile.boot_values,
            )
            changed.append('perfil zerado (fábrica)')
        params = parse_assignments(args.set)
        managed = sorted(set(params) & set(MANAGED))
        if managed:
            raise ValueError(f'{", ".join(managed)} é do próprio ensaio')
        q = parse_assignments(args.q, upper=False)
        check_q_names(q)
    except ValueError as exc:
        print(f'erro: {exc}', file=sys.stderr)
        return 2
    if args.apply_suggestion is not None:
        chosen = profile.apply_suggestion(args.apply_suggestion)
        changed.append(
            'sugestão aplicada: '
            + (', '.join(f'{k}={v:g}' for k, v in chosen.items()) or 'nada a aplicar')
        )
    profile.params.update(params)
    profile.q.update(q)
    for name in args.unset or []:
        if profile.params.pop(name.upper(), None) is not None:
            changed.append(f'{name.upper()} volta ao de fábrica')
    for name in args.unset_q or []:
        profile.q.pop(name, None)
    if args.clear_q:
        profile.q, profile.r = {}, None
    if args.r is not None:
        profile.r = args.r
    if args.height is not None:
        profile.height_cm = args.height
    if args.time is not None:
        profile.time_s = args.time
    if (
        changed
        or params
        or q
        or args.unset
        or args.unset_q
        or args.clear_q
        or args.r is not None
        or args.height is not None
        or args.time is not None
    ):
        profile.save()
    for line in changed:
        print(line)
    print(profile.text())
    return 0


def cmd_rotas_bench(args) -> int:
    """Benchmark dos algoritmos de rota (não usa o drone)."""
    from .rotas_bench import rodar

    resumo = rodar(
        args.n_min,
        args.n_max,
        args.instancias,
        args.exato_max_s,
        args.exato_max_mb,
        Path(args.saida),
        forca_max_s=args.forca_max_s,
    )
    print(f"\nassinatura {resumo['assinatura_sha256']}")
    print(f"total {resumo['tempo_total_s']} s; resultados em {args.saida}/")
    return 0


def cmd_rotas_limite(args) -> int:
    """Até quantos pontos o TSP exato (Held-Karp) roda nesta máquina."""
    from .rotas_limite import rodar

    r = rodar(args.n_max, args.tempo_max_s, args.mem_max_mb, Path(args.saida))
    print(f"\nparou em N={r['parou_em']}: {r['motivo']}")
    print(f"resultados em {args.saida}/limite_exato.md")
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog='dronectl', description='Comando e ajuste do drone')
    parser.add_argument(
        '-p', '--port', default='/dev/serial0', help='porta serial do STM32 (ou URL do pyserial)'
    )
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
    p = sub.add_parser('missao', help='comandos passo a passo (takeoff, yaw, goto, route, land...)')
    p.add_argument('arquivo', nargs='?', help='arquivo de missão (sem ele: shell interativo)')
    p.add_argument(
        '--sim',
        action='store_true',
        help='liga FLT_SIM=1: drone simulado no STM32, motores parados',
    )
    p.add_argument(
        '--pontos',
        help='arquivo de pontos do "route" sem pontos (padrão: tools/pi/missoes/pontos.txt)',
    )
    p.add_argument('--desligar-sim', action='store_true', help='volta FLT_SIM=0 ao terminar')
    p.set_defaults(func=cmd_missao)
    p = sub.add_parser(
        'hover-test',
        help='ensaio de hover: perfil, voo protegido, gravação e '
        'análise (sem --motors/--sim: a seco)',
    )
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--motors', action='store_true', help='voo real: os motores giram')
    mode.add_argument('--sim', action='store_true', help='drone simulado no STM32 (FLT_SIM=1)')
    p.add_argument('--height', type=float, help='altura, em cm (padrão: a do perfil, 30)')
    p.add_argument('--time', type=float, help='tempo de hover depois de chegar, em s (perfil: 20)')
    p.add_argument(
        '--set',
        action='append',
        metavar='NOME=VALOR[,...]',
        help='parâmetro só deste ensaio (pode repetir)',
    )
    p.add_argument(
        '--q',
        action='append',
        metavar='ESTADO=PESO[,...]',
        help='peso Q do LQG ajustado só deste ensaio (x y z phi theta psi vx vy vz p q r)',
    )
    p.add_argument('--r', type=float, help='peso R do LQG ajustado (padrão 0,1)')
    p.add_argument(
        '--save', action='store_true', help='grava --set/--q/--r/--height/--time no perfil'
    )
    p.add_argument('--label', help='anotação do ensaio (vai para o registro)')
    p.add_argument('--rate', type=float, default=50.0, help='DRONE_CTRL_DEBUG, em Hz')
    p.add_argument(
        '--geofence',
        type=float,
        default=80.0,
        help='pousa a esta distância do ponto de decolagem, em cm',
    )
    p.add_argument('--countdown', type=float, default=3.0, help='contagem antes de armar, em s')
    p.add_argument('--profile', help='perfil (padrão ~/drone/perfil_hover.json)')
    p.add_argument('--out', help='pasta dos ensaios (padrão ~/real_log_kit)')
    p.set_defaults(func=cmd_hover_test)
    p = sub.add_parser('hover-report', help='reanalisa um ensaio de hover gravado')
    p.add_argument('folder', nargs='?', default='latest', help='pasta do ensaio (padrão: o último)')
    p.add_argument('--dir', help='onde procurar o último (padrão ~/real_log_kit)')
    p.add_argument('--timeline', action='store_true', help='mostra a linha do tempo (1 linha/s)')
    p.add_argument('--plot', action='store_true', help='grava grafico.png na pasta (matplotlib)')
    p.set_defaults(func=cmd_hover_report)
    p = sub.add_parser('hover-profile', help='perfil com que o próximo hover-test começa')
    p.add_argument('--set', action='append', metavar='NOME=VALOR[,...]')
    p.add_argument('--unset', action='append', metavar='NOME', help='volta ao de fábrica')
    p.add_argument('--q', action='append', metavar='ESTADO=PESO[,...]')
    p.add_argument('--unset-q', action='append', metavar='ESTADO')
    p.add_argument('--clear-q', action='store_true', help='volta ao Q e R do ajustado do firmware')
    p.add_argument('--r', type=float)
    p.add_argument('--height', type=float, help='altura padrão, em cm')
    p.add_argument('--time', type=float, help='tempo de hover padrão, em s')
    p.add_argument(
        '--apply-suggestion',
        nargs='*',
        metavar='NOME',
        help='passa a sugestão do último ensaio (toda, ou só estes nomes) para o perfil',
    )
    p.add_argument('--reset', action='store_true', help='perfil de fábrica (mantém o histórico)')
    p.add_argument('--profile', help='perfil (padrão ~/drone/perfil_hover.json)')
    p.set_defaults(func=cmd_hover_profile)
    p = sub.add_parser(
        'rotas-bench', help='benchmark guloso x 2-opt x genético x força bruta x exato (TSP)'
    )
    p.add_argument('--n-min', type=int, default=3)
    p.add_argument('--n-max', type=int, default=30)
    p.add_argument('--instancias', type=int, default=10, help='instâncias (sementes) por N')
    p.add_argument(
        '--exato-max-s',
        type=float,
        default=10.0,
        help='não roda o exato se o tempo previsto por instância passar disso',
    )
    p.add_argument(
        '--exato-max-mb',
        type=float,
        default=600.0,
        help='nem se a memória do Held-Karp passar disso (Pi Zero: use 150)',
    )
    p.add_argument(
        '--forca-max-s',
        type=float,
        default=10.0,
        help='não roda a força bruta se o tempo previsto por instância passar disso',
    )
    p.add_argument('--saida', default='resultados_rotas', help='pasta dos resultados')
    p.set_defaults(func=cmd_rotas_bench)
    p = sub.add_parser('rotas-limite', help='até quantos pontos o TSP exato roda (tempo e RAM)')
    p.add_argument('--n-max', type=int, default=30)
    p.add_argument(
        '--tempo-max-s',
        type=float,
        default=60.0,
        help='tempo por instância a partir do qual o exato é inviável',
    )
    p.add_argument('--mem-max-mb', type=float, help='padrão: 80 %% da RAM livre')
    p.add_argument('--saida', default='resultados_rotas', help='pasta dos resultados')
    p.set_defaults(func=cmd_rotas_limite)
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
