"""Command line entry point.

python -m benchlink serve --simulate          # UI with the simulated STM32
python -m benchlink serve --port /dev/ttyACM0 # UI with the real board
python -m benchlink serve --replay <session>  # UI replaying a recording
python -m benchlink monitor --port /dev/ttyACM0  # read-only text monitor
python -m benchlink sessions                  # list recordings
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from . import protocol as p
from .hub import Hub
from .server import serve
from .sources import SerialSource, SimulatorSource

DEFAULT_DATA = Path(__file__).resolve().parents[1] / 'data'


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog='benchlink', description=__doc__.splitlines()[0])
    parser.add_argument('--data', type=Path, default=DEFAULT_DATA, help='sessions/config folder')
    sub = parser.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('serve', help='start the web HMI')
    group = s.add_mutually_exclusive_group()
    group.add_argument('--port', help='STM32 USB CDC device, e.g. /dev/ttyACM0')
    group.add_argument('--simulate', action='store_true', help='use the simulated STM32')
    group.add_argument('--replay', help='session id under <data>/sessions')
    s.add_argument('--host', default='127.0.0.1')
    s.add_argument('--http-port', type=int, default=8765)
    m = sub.add_parser(
        'monitor', help='print TEXT/STATUS from the board (never sends motor commands)'
    )
    m.add_argument('--port', required=True)
    m.add_argument('--seconds', type=float, default=0, help='0 = until Ctrl+C')
    sub.add_parser('sessions', help='list recorded sessions')
    args = parser.parse_args(argv)

    hub = Hub(args.data)
    if args.cmd == 'sessions':
        for item in hub.sessions():
            print(f"{item['id']}  {item.get('duration_s') or '?'} s  fw={item.get('firmware_id')}")
        hub.close()
        return 0
    if args.cmd == 'monitor':
        return monitor(hub, args.port, args.seconds)
    if args.port:
        hub.set_source(SerialSource(args.port))
    elif args.simulate:
        hub.set_source(SimulatorSource())
    elif args.replay:
        hub.open_replay(args.replay)
    serve(hub, args.host, args.http_port)
    return 0


def monitor(hub: Hub, port: str, seconds: float) -> int:
    hub.set_source(SerialSource(port))
    printed = 0
    end = time.monotonic() + seconds if seconds else None
    try:
        while end is None or time.monotonic() < end:
            snap = hub.snapshot()
            texts = snap['texts']
            for item in texts[printed:] if len(texts) > printed else []:
                print(f"[{item['t_ms']:>9} ms] {item['text']}")
            printed = len(texts)
            status = snap['status']
            if status:
                armed = 'ARMADO' if status['flags'] & p.STATUS_ARMED else 'desarmado'
                values = snap['values']
                dist = values.get('mtf.distance', {}).get('v')
                rate = values.get('imu.rate', {}).get('v')
                line = (
                    f"\r{armed} cap={status['cap_permille'] / 10:.0f}% "
                    f"imu={rate or 0:.0f}Hz dist={dist if dist is not None else '-'} m "
                    f"erros={snap['link']['errors']} perdidos={snap['link']['lost']}   "
                )
                sys.stdout.write(line)
                sys.stdout.flush()
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        hub.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
