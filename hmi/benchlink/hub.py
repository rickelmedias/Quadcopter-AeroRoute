"""Core state of the bench HMI: decoding, live values, recording, freezes, control.

Safety model (the STM32 has the final word; see ``app/bench_link.h``):

- Only one browser client holds control; others are view-only.
- The hub forwards HEARTBEAT to the STM32 only while the controlling client is
  sending its own heartbeat (< ``CLIENT_TIMEOUT_S``). If the page freezes,
  closes or the tab is backgrounded, heartbeats stop, the hub sends STOP, and
  the firmware also disarms by itself after 300 ms.
- Replay sources never accept commands.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from pathlib import Path

from . import protocol as p
from .estimator import EstimatorConfig, HostEstimator
from .freezes import FreezeStore
from .recorder import Recorder, list_sessions, utc_now
from .signals import BY_NAME, SIGNALS, extract
from .sources import NullSource, ReplaySource, Source
from .stats import SignalStore, spectrum

CLIENT_TIMEOUT_S = 0.25
CONTROL_RELEASE_S = 3.0
HEARTBEAT_PERIOD_S = 0.1
ACK_TIMEOUT_S = 0.6
STALE_S = 0.5

DEFAULT_CONFIG = {
    'motor_poles': 14,
    'power_cal': {
        'vbat_gain': 0.0,
        'current_a_per_v': 0.0,
        'current_offset_v': 0.0,
        'vbat_min_v': 0.0,
    },
    'esc_telem': {'enabled': False, 'period_ms': 10},
    'stream': {'enabled': True, 'bno_divider': 1},
    'estimator': EstimatorConfig().to_dict(),
    'reference_reversed': [1, 4],
    'control': dict(p.CTRL_DEFAULTS),
    'gains': dict(p.GAIN_DEFAULTS),
}


def _finite(value):
    return value if isinstance(value, (int, float)) and math.isfinite(value) else None


class Hub:
    def __init__(self, data_dir: Path) -> None:
        self.data_dir = data_dir
        self.sessions_dir = data_dir / 'sessions'
        self.sessions_dir.mkdir(parents=True, exist_ok=True)
        self.config_path = data_dir / 'config.json'
        self.config = self._load_config()
        self.freezes = FreezeStore(data_dir / 'freezes.json')
        self.lock = threading.RLock()
        self.source: Source = NullSource()
        self.recorder: Recorder | None = None
        self.estimator = HostEstimator(EstimatorConfig(**self.config['estimator']))
        self._reset_state()
        self.controller: str | None = None
        self.controller_seen = 0.0
        self.controller_beat = 0.0
        self.stop_sent = False
        #: A saída foi armada por esta IHM (ARM da bancada ou Armar do controle).
        #: Só nesse caso o STOP automático por falta de heartbeat do navegador vale:
        #: num voo armado pela Raspberry Pi a IHM só grava (e o STOP manual continua).
        self.armed_here = False
        self.pending: dict[int, dict] = {}
        self.host_seq = 0
        self.send_lock = threading.Lock()
        self._snapshot_cache: tuple[float, dict] | None = None
        self._running = True
        self._thread = threading.Thread(
            target=self._heartbeat_loop, name='hub-heartbeat', daemon=True
        )
        self._thread.start()

    # ---- configuration ---------------------------------------------------------

    def _load_config(self) -> dict:
        config = json.loads(json.dumps(DEFAULT_CONFIG))
        if self.config_path.is_file():
            try:
                saved = json.loads(self.config_path.read_text())
            except json.JSONDecodeError:
                saved = {}
            for key, value in saved.items():
                if isinstance(value, dict) and isinstance(config.get(key), dict):
                    config[key].update(value)
                elif key in config:
                    config[key] = value
        return config

    def _save_config(self) -> None:
        self.config_path.write_text(json.dumps(self.config, indent=2))

    def update_config(self, changes: dict) -> dict:
        with self.lock:
            for key in ('motor_poles',):
                if key in changes:
                    poles = int(changes[key])
                    if poles < 2 or poles % 2:
                        raise ValueError('motor poles must be an even number >= 2')
                    self.config[key] = poles
            for key in ('power_cal', 'esc_telem', 'stream', 'estimator', 'control', 'gains'):
                if key in changes:
                    self.config[key].update(changes[key])
            if 'reference_reversed' in changes:
                self.config['reference_reversed'] = [int(m) for m in changes['reference_reversed']]
            self.estimator.config = EstimatorConfig(**self.config['estimator'])
            self._save_config()
            return self.config

    # ---- sources ---------------------------------------------------------------

    def _reset_state(self) -> None:
        self.decoder = p.FrameDecoder()
        self.store = SignalStore(window_s=30.0)
        self.values: dict[str, list] = {}
        self.hello: dict | None = None
        self.status: dict | None = None
        self.sys: dict | None = None
        self.ctrl: dict | None = None
        self.texts: deque = deque(maxlen=200)
        self.events: deque = deque(maxlen=200)
        self.last_seq: int | None = None
        self.lost = 0
        self.rx_bytes = 0
        self.connected_at = time.monotonic()
        self.estimator.reset()

    def set_source(self, source: Source) -> None:
        old = self.source
        if self.recorder and not source.live:
            self.stop_recording()
        old.close()
        with self.lock:
            self._reset_state()
            self.source = source
        source.start(self.on_bytes)
        self._event('fonte', descricao=source.describe())

    def close(self) -> None:
        self._running = False
        self._stop_now('IHM encerrada')
        if self.recorder:
            self.stop_recording()
        self.source.close()

    # ---- reception ---------------------------------------------------------------

    def on_bytes(self, data: bytes) -> None:
        now = time.monotonic()
        with self.lock:
            self.rx_bytes += len(data)
            if self.recorder:
                self.recorder.write_raw(data)
            errors_before = self.decoder.errors
            for packet in self.decoder.feed(data):
                self._on_packet(packet, now)
            if self.decoder.errors != errors_before:
                self._set('link.errors', self.decoder.errors, True, now)

    def _set(self, name: str, value, valid: bool, now: float, t_s: float | None = None) -> None:
        value = _finite(value)
        if value is None:
            return
        self.values[name] = [value, valid, now]
        if t_s is not None:
            self.store.add(name, t_s, value)

    def _on_packet(self, packet: p.Packet, now: float) -> None:
        if self.last_seq is not None:
            gap = (packet.seq - self.last_seq - 1) & 0xFF
            if gap:
                self.lost += gap
                self._set('link.lost', self.lost, True, now)
        self.last_seq = packet.seq
        msg = p.decode_message(packet.msg_id, packet.payload)
        if msg is None:
            return
        if self.recorder:
            self.recorder.write_message(msg)
        kind = msg['type']
        if kind == 'hello':
            self.hello = msg
            self._event('hello', firmware_id=msg['firmware_id'], variant=msg['variant'])
            if self.recorder:
                self.recorder.update_meta(hello=msg)
            if self.source.live:
                threading.Thread(target=self._push_config, daemon=True).start()
            return
        if kind == 'status':
            self.status = msg
            if not msg['flags'] & p.STATUS_ARMED:
                self.stop_sent = False
                self.armed_here = False
            return
        if kind == 'text':
            self.texts.append(
                {'t_ms': msg['t_ms'], 'level': msg['level'], 'text': msg['text'], 'host': utc_now()}
            )
            return
        if kind == 'ack':
            waiter = self.pending.get(msg['seq'])
            if waiter and waiter['command'] == msg['command']:
                waiter['ack'] = msg
                waiter['event'].set()
            return
        if kind == 'sys':
            self.sys = msg
        if kind == 'ctrl':
            self.ctrl = msg
        t_s = msg['t_ms'] / 1000.0
        values, valid = extract(msg, self.config['motor_poles'])
        for name, value in values.items():
            self._set(name, value, valid, now, t_s)
        if kind == 'imu':
            self.estimator.on_imu(values['imu.roll'], values['imu.pitch'], msg['gyro'])
        elif kind == 'range':
            est = self.estimator.on_range(
                t_s,
                msg['distance_mm'] / 1000.0,
                bool(msg['flags'] & 1),
                (msg['flow_x'], msg['flow_y'], msg['quality'], bool(msg['flags'] & 2)),
            )
            for name, value in est.items():
                self._set(name, value, True, now, t_s)
            for name in ('mtf.flow_x', 'mtf.flow_y', 'mtf.quality'):
                if name in self.values:
                    self.values[name][1] = bool(msg['flags'] & 2)

    # ---- commands ----------------------------------------------------------------

    def _send_frame(self, msg_id: int, payload: bytes) -> int:
        with self.send_lock:
            seq = self.host_seq
            self.host_seq = (self.host_seq + 1) & 0xFF
            self.source.send(p.encode_frame(msg_id, seq, payload))
        return seq

    def _transact(self, name: str, args: dict, wait: bool = True) -> dict:
        if not self.source.live:
            raise PermissionError('fonte sem comandos (replay ou desconectado)')
        msg_id, payload = p.command_payload(name, **args)
        waiter = {'command': msg_id, 'event': threading.Event(), 'ack': None}
        with self.send_lock:
            seq = self.host_seq
            self.pending[seq] = waiter
            self.host_seq = (self.host_seq + 1) & 0xFF
            self.source.send(p.encode_frame(msg_id, seq, payload))
        result = {'command': name, 'seq': seq}
        if wait:
            got = waiter['event'].wait(ACK_TIMEOUT_S)
            ack = waiter['ack']
            result.update(
                ack=got,
                result=p.RESULT_NAMES.get(ack['result']) if ack else 'sem resposta',
                ok=bool(ack and ack['result'] == p.RESULT_OK),
                value=ack['value'] if ack else None,
            )
        self.pending.pop(seq, None)
        self._event('comando', nome=name, args=args, resultado=result.get('result', 'enviado'))
        return result

    def command(self, client_id: str, name: str, args: dict | None = None) -> dict:
        args = args or {}
        if name != 'stop' and client_id != self.controller:
            raise PermissionError('este cliente não está no controle')
        if name == 'heartbeat':
            raise ValueError('heartbeat é gerado pelo hub')
        if name == 'stop':
            # STOP vale de qualquer cliente que esteja vendo a IHM.
            return self._transact('stop', {})
        arming = name == 'arm' or (name == 'ctrl' and args.get('action') == 'arm')
        if name == 'arm' or (name == 'ctrl' and args.get('action') in ('arm', 'takeoff')):
            if not self.controller_alive():
                raise PermissionError('controle sem heartbeat do navegador')
            self._send_heartbeat()
            time.sleep(0.02)
            if arming:
                self.armed_here = True  # antes do ACK: o STOP automático já vale
        if name in ('power_cal', 'esc_telem', 'stream'):
            p.command_payload(name, **args)  # valida antes de persistir
            self.update_config({name: args})
        if name == 'ctrl_gains':
            p.command_payload(name, **args)
            self.update_config({'gains': args})
            args = dict(self.config['gains'])
        if name == 'ctrl_params':
            p.command_payload(name, **args)
            self.update_config({'control': args})
            args = dict(self.config['control'])
        return self._transact(name, args)

    def _push_config(self) -> None:
        """New firmware session: re-apply calibration and telemetry settings."""
        time.sleep(0.05)
        for name, key in (
            ('power_cal', 'power_cal'),
            ('esc_telem', 'esc_telem'),
            ('stream', 'stream'),
            ('ctrl_params', 'control'),
            ('ctrl_gains', 'gains'),
        ):
            minor = self.hello['minor'] if self.hello else 0
            if name == 'ctrl_gains' and minor < 3:
                continue  # firmware anterior: comando desconhecido pararia a saída
            if name == 'ctrl_params' and not (self.hello and self.hello['capabilities'] & 8):
                # Firmware sem controle: o comando seria desconhecido e pararia a saída.
                continue
            try:
                self._transact(name, dict(self.config[key]))
            except (PermissionError, ValueError, OSError) as exc:
                self._event('erro_config', nome=name, erro=str(exc))

    def _send_heartbeat(self) -> None:
        try:
            self._send_frame(p.CMD_HEARTBEAT, b'')
        except (RuntimeError, OSError):
            pass

    # ---- control ownership ---------------------------------------------------

    def controller_alive(self) -> bool:
        return (
            self.controller is not None
            and time.monotonic() - self.controller_beat < CLIENT_TIMEOUT_S
        )

    # Regra de travas: nunca enviar (send_lock) segurando self.lock; a recepção
    # pode precisar de self.lock para entregar o ACK que o envio aguarda.

    def claim_control(self, client_id: str, force: bool = False) -> dict:
        now = time.monotonic()
        transferred = False
        with self.lock:
            stale = self.controller is None or now - self.controller_seen > CONTROL_RELEASE_S
            if self.controller == client_id or stale or force:
                transferred = self.controller not in (None, client_id)
                self.controller = client_id
                self.controller_seen = now
                self.controller_beat = now
            mine = self.controller == client_id
        if transferred:
            self._stop_now('controle transferido para outro cliente')
        return {'controller': mine}

    def release_control(self, client_id: str) -> None:
        with self.lock:
            released = self.controller == client_id
            if released:
                self.controller = None
        if released:
            self._stop_now('controle liberado')

    def client_heartbeat(self, client_id: str) -> bool:
        now = time.monotonic()
        with self.lock:
            if client_id == self.controller:
                self.controller_seen = now
                self.controller_beat = now
                return True
        return False

    def _stop_now(self, reason: str) -> None:
        if self.source.live:
            try:
                self._send_frame(p.CMD_STOP, b'')
            except (RuntimeError, OSError):
                pass
            self._event('stop_automatico', motivo=reason)

    def _heartbeat_loop(self) -> None:
        next_beat = 0.0
        while self._running:
            now = time.monotonic()
            if self.source.live and now >= next_beat:
                next_beat = now + HEARTBEAT_PERIOD_S
                armed = bool(self.status and self.status['flags'] & p.STATUS_ARMED)
                if self.controller_alive():
                    self._send_heartbeat()
                elif armed and self.armed_here and not self.stop_sent:
                    self.stop_sent = True
                    self._stop_now('IHM sem heartbeat do navegador')
            time.sleep(0.02)

    # ---- events, recording ---------------------------------------------------

    def _event(self, kind: str, **data) -> None:
        record = {'utc': utc_now(), 'event': kind, **data}
        self.events.append(record)
        if self.recorder:
            self.recorder.event(kind, **data)

    def start_recording(self, name: str, notes: str = '', conditions: dict | None = None) -> dict:
        with self.lock:
            if self.recorder:
                raise RuntimeError('já está gravando')
            if not self.source.live:
                raise PermissionError('grave somente com STM32 ou simulador')
            self.recorder = Recorder(
                self.sessions_dir,
                name or 'sessao',
                {
                    'notes': notes,
                    'conditions': conditions or {},
                    'source': self.source.describe(),
                    'hello': self.hello,
                    'config': self.config,
                },
            )
            self._event('gravacao_inicio', nome=name)
            return {'path': str(self.recorder.path)}

    def stop_recording(self) -> dict:
        with self.lock:
            if not self.recorder:
                return {'path': None}
            self._event('gravacao_fim')
            path = self.recorder.stop(self.freezes.all())
            self.recorder = None
            return {'path': str(path)}

    def marker(self, label: str) -> dict:
        t_ms = self.status['t_ms'] if self.status else None
        self._event('marcador', rotulo=label, t_ms=t_ms)
        return {'ok': True}

    def notes(self, text: str) -> None:
        with self.lock:
            if self.recorder:
                self.recorder.update_meta(notes=text)
            self._event('notas', texto=text)

    def open_replay(self, session_id: str) -> None:
        folder = (self.sessions_dir / session_id).resolve()
        if folder.parent != self.sessions_dir.resolve() or not (folder / 'raw.blr').is_file():
            raise FileNotFoundError(session_id)
        self.set_source(ReplaySource(folder))

    def sessions(self) -> list[dict]:
        return list_sessions(self.sessions_dir)

    # ---- freezes ---------------------------------------------------------------

    def freeze(self, names: list[str] | None, label: str, group: str | None = None) -> dict:
        with self.lock:
            if names is None:
                names = [s.name for s in SIGNALS if group is None or s.group == group]
            values = {}
            for name in names:
                current = self.values.get(name)
                if current is None or name not in BY_NAME:
                    continue
                stats = self.store.stats(name, 1.0)
                signal = BY_NAME[name]
                values[name] = {
                    'value': current[0],
                    'valid': current[1],
                    'mean_1s': stats[0] if stats else None,
                    'std_1s': stats[1] if stats else None,
                    'n_1s': stats[2] if stats else 0,
                    'unit': signal.unit,
                    'label': signal.label,
                    'group': signal.group,
                }
            if not values:
                raise ValueError('nenhum valor disponível para congelar')
            context = {
                'source': self.source.kind,
                'device_t_ms': self.status['t_ms'] if self.status else None,
                'firmware_id': self.hello['firmware_id'] if self.hello else None,
                'window_s': 1.0,
            }
            item = self.freezes.add(label or 'amostra', values, context)
            self._event('congelado', id=item['id'], rotulo=item['label'], sinais=list(values))
            return item

    # ---- views --------------------------------------------------------------

    def series(self, names: list[str], seconds: float) -> dict:
        with self.lock:
            return {n: self.store.series(n, seconds) for n in names if n in BY_NAME}

    def spectrum(self, name: str, seconds: float) -> dict:
        with self.lock:
            samples = self.store.window(name, seconds)
            now = time.monotonic()
            markers = []
            for i in range(1, 5):
                entry = self.values.get(f'esc.m{i}.hz')
                if entry and now - entry[2] < 1.0 and entry[0] > 0:
                    markers.append({'motor': i, 'hz': entry[0]})
        result = spectrum(samples) or {'freq_hz': [], 'amplitude': [], 'fs_hz': None, 'n': 0}
        result['markers'] = markers
        result['unit'] = BY_NAME[name].unit if name in BY_NAME else ''
        return result

    def snapshot(self) -> dict:
        now = time.monotonic()
        cached = self._snapshot_cache
        if cached and now - cached[0] < 0.08:
            return cached[1]
        with self.lock:
            values = {}
            for name, (value, valid, seen) in self.values.items():
                stats = self.store.stats(name, 1.0)
                values[name] = {
                    'v': value,
                    'ok': bool(valid) and now - seen < STALE_S,
                    'age': round(now - seen, 2),
                    'mean': stats[0] if stats else None,
                    'std': stats[1] if stats else None,
                }
            for name, base in (('imu.rate', 'imu.qw'), ('mtf.rate', 'mtf.distance')):
                rate = self.store.rate(base)
                if rate is not None:
                    values[name] = {'v': rate, 'ok': True, 'age': 0, 'mean': None, 'std': None}
            snap = {
                'source': self.source.describe(),
                'hello': self.hello,
                'status': self.status,
                'sys': self.sys,
                'ctrl': self.ctrl,
                'link': {
                    'rx_bytes': self.rx_bytes,
                    'packets': self.decoder.packets,
                    'errors': self.decoder.errors,
                    'lost': self.lost,
                },
                'control': {
                    'controller': self.controller,
                    'alive': self.controller_alive(),
                },
                'recording': (
                    {
                        'path': str(self.recorder.path),
                        'elapsed_s': round(self.recorder.elapsed(), 1),
                    }
                    if self.recorder
                    else None
                ),
                'values': values,
                'texts': list(self.texts)[-40:],
                'events': list(self.events)[-40:],
            }
        self._snapshot_cache = (now, snap)
        return snap
