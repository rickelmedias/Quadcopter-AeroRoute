"""HTTP server for the bench HMI (stdlib only): static UI, SSE stream, JSON API.

Binds to 127.0.0.1 by default. Anyone who can reach the port can request motor
commands, so do not expose it to a network.
"""

from __future__ import annotations

import json
import mimetypes
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from .hub import Hub
from .signals import SIGNALS
from .sources import NullSource, SerialSource, SimulatorSource

WEB_DIR = Path(__file__).resolve().parents[1] / 'web'
STREAM_PERIOD_S = 0.1


def serial_ports() -> list[dict]:
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [{'device': p.device, 'description': p.description} for p in list_ports.comports()]


def make_handler(hub: Hub):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'AeroRouteBench/1'

        def log_message(self, fmt, *args):
            pass

        # ---- helpers -------------------------------------------------------

        def _json(self, data, status: int = 200) -> None:
            body = json.dumps(data, ensure_ascii=False, allow_nan=False, default=str).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _error(self, status: int, message: str) -> None:
            self._json({'error': message}, status)

        def _body(self) -> dict:
            length = int(self.headers.get('Content-Length') or 0)
            if length > 1_000_000:
                raise ValueError('body too large')
            raw = self.rfile.read(length) if length else b'{}'
            data = json.loads(raw or b'{}')
            if not isinstance(data, dict):
                raise TypeError('JSON object expected')
            return data

        def _static(self, path: str) -> None:
            name = 'index.html' if path in ('', '/') else path.lstrip('/')
            target = (WEB_DIR / name).resolve()
            if WEB_DIR.resolve() not in target.parents or not target.is_file():
                self._error(404, 'not found')
                return
            body = target.read_bytes()
            ctype = mimetypes.guess_type(target.name)[0] or 'application/octet-stream'
            if target.suffix == '.js':
                ctype = 'text/javascript'
            self.send_response(200)
            self.send_header('Content-Type', f'{ctype}; charset=utf-8')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _stream(self) -> None:
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            try:
                while True:
                    data = json.dumps(hub.snapshot(), ensure_ascii=False, default=str)
                    self.wfile.write(f'data: {data}\n\n'.encode())
                    self.wfile.flush()
                    time.sleep(STREAM_PERIOD_S)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

        # ---- routes --------------------------------------------------------

        def do_GET(self):
            url = urlparse(self.path)
            query = parse_qs(url.query)
            try:
                if url.path == '/api/stream':
                    self._stream()
                elif url.path == '/api/signals':
                    self._json([s.__dict__ for s in SIGNALS])
                elif url.path == '/api/series':
                    names = [n for n in query.get('names', [''])[0].split(',') if n]
                    seconds = min(30.0, float(query.get('seconds', ['10'])[0]))
                    self._json(hub.series(names, seconds))
                elif url.path == '/api/spectrum':
                    seconds = min(30.0, float(query.get('seconds', ['4'])[0]))
                    self._json(hub.spectrum(query['name'][0], seconds))
                elif url.path == '/api/freezes':
                    self._json(hub.freezes.all())
                elif url.path == '/api/sessions':
                    self._json(hub.sessions())
                elif url.path == '/api/config':
                    self._json(hub.config)
                elif url.path == '/api/ports':
                    self._json(serial_ports())
                elif url.path.startswith('/api/'):
                    self._error(404, 'unknown endpoint')
                else:
                    self._static(url.path)
            except (KeyError, ValueError) as exc:
                self._error(400, str(exc))

        def do_POST(self):
            url = urlparse(self.path)
            try:
                body = self._body()
                client = str(body.get('client_id', ''))
                route = url.path
                if route == '/api/heartbeat':
                    self._json({'controller': hub.client_heartbeat(client)})
                elif route == '/api/control':
                    if body.get('release'):
                        hub.release_control(client)
                        self._json({'controller': False})
                    else:
                        self._json(hub.claim_control(client, bool(body.get('force'))))
                elif route == '/api/command':
                    self._json(hub.command(client, str(body['name']), body.get('args') or {}))
                elif route == '/api/freezes':
                    names = body.get('names')
                    self._json(
                        hub.freeze(
                            list(names) if names else None,
                            str(body.get('label', '')),
                            body.get('group'),
                        )
                    )
                elif route.startswith('/api/freezes/'):
                    item_id = route.rsplit('/', 1)[-1]
                    if body.get('delete'):
                        self._json({'deleted': hub.freezes.delete(item_id)})
                    else:
                        item = hub.freezes.update(item_id, **body)
                        if item is None:
                            self._error(404, 'freeze not found')
                        else:
                            self._json(item)
                elif route == '/api/freezes-clear':
                    hub.freezes.clear()
                    self._json({'ok': True})
                elif route == '/api/record/start':
                    self._json(
                        hub.start_recording(
                            str(body.get('name', 'sessao')),
                            str(body.get('notes', '')),
                            body.get('conditions') or {},
                        )
                    )
                elif route == '/api/record/stop':
                    self._json(hub.stop_recording())
                elif route == '/api/marker':
                    self._json(hub.marker(str(body.get('label', 'marcador'))))
                elif route == '/api/notes':
                    hub.notes(str(body.get('text', '')))
                    self._json({'ok': True})
                elif route == '/api/config':
                    self._json(hub.update_config(body))
                elif route == '/api/estimator/reset':
                    hub.estimator.reset_position()
                    self._json({'ok': True})
                elif route == '/api/source':
                    self._json(set_source(hub, body))
                elif route == '/api/replay':
                    source = hub.source
                    if not hasattr(source, 'control'):
                        self._error(409, 'not in replay')
                    else:
                        source.control(body.get('playing'), body.get('speed'))
                        self._json(source.describe())
                else:
                    self._error(404, 'unknown endpoint')
            except PermissionError as exc:
                self._error(HTTPStatus.FORBIDDEN, str(exc))
            except (KeyError, ValueError, TypeError, FileNotFoundError, RuntimeError) as exc:
                self._error(400, str(exc))
            except OSError as exc:
                self._error(503, f'falha de comunicação: {exc}')

    return Handler


def set_source(hub: Hub, body: dict) -> dict:
    mode = body.get('mode')
    if hub.status and hub.status['flags'] & 1 and hub.source.live:
        raise RuntimeError('pare os motores antes de trocar a fonte')
    if mode == 'serial':
        hub.set_source(SerialSource(str(body['port'])))
    elif mode == 'simulator':
        hub.set_source(SimulatorSource())
    elif mode == 'replay':
        hub.open_replay(str(body['session']))
    elif mode == 'none':
        hub.set_source(NullSource())
    else:
        raise ValueError('mode: serial | simulator | replay | none')
    return hub.source.describe()


def serve(hub: Hub, host: str = '127.0.0.1', port: int = 8765) -> None:
    server = ThreadingHTTPServer((host, port), make_handler(hub))
    server.daemon_threads = True
    print(f'IHM de bancada em http://{host}:{port}  (Ctrl+C encerra e para os motores)', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        hub.close()


def start_background(hub: Hub, host: str = '127.0.0.1', port: int = 0):
    """Start a server in a thread (tests); returns (server, url)."""
    server = ThreadingHTTPServer((host, port), make_handler(hub))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, f'http://{host}:{server.server_address[1]}'
