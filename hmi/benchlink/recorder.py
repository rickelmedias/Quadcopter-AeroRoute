"""Session recording: raw bytes for exact replay, CSV per message type, events.

Layout of ``<sessions>/<YYYYmmdd_HHMMSS>_<name>/``:

- ``meta.json``: session metadata (operator notes, conditions, firmware HELLO,
  configuration, counts, start/stop in UTC);
- ``raw.blr``: every byte received from the STM32, in arrival chunks:
  magic ``BLR1`` then records ``<d I`` (host seconds since start, length) + bytes.
  Replay feeds these bytes through the same decoder used live;
- ``<type>.csv``: one file per message type, columns with units in the name;
- ``events.jsonl``: markers, commands with their ACK, notes, freezes;
- ``freezes.json``: frozen samples at the end of the session.
"""

from __future__ import annotations

import csv
import json
import re
import struct
import time
from datetime import datetime, timezone
from pathlib import Path

MAGIC = b'BLR1'
RECORD = struct.Struct('<dI')

CSV_COLUMNS = {
    'imu': [
        ('t_ms', 't_ms'),
        ('qw', 'q.0'),
        ('qx', 'q.1'),
        ('qy', 'q.2'),
        ('qz', 'q.3'),
        ('gyro_x_rad_s', 'gyro.0'),
        ('gyro_y_rad_s', 'gyro.1'),
        ('gyro_z_rad_s', 'gyro.2'),
        ('lin_acc_x_m_s2', 'lin_accel.0'),
        ('lin_acc_y_m_s2', 'lin_accel.1'),
        ('lin_acc_z_m_s2', 'lin_accel.2'),
        ('acc_rot', 'accuracy.0'),
        ('acc_gyro', 'accuracy.1'),
        ('acc_lin', 'accuracy.2'),
        ('online', 'flags'),
        ('n_rot', 'count.0'),
        ('n_gyro', 'count.1'),
        ('n_lin', 'count.2'),
        ('gaps_rot', 'gaps.0'),
        ('gaps_gyro', 'gaps.1'),
        ('gaps_lin', 'gaps.2'),
    ],
    'range': [
        ('t_ms', 't_ms'),
        ('sensor_ms', 'sensor_ms'),
        ('distance_mm', 'distance_mm'),
        ('flow_x_cm_s_at_1m', 'flow_x'),
        ('flow_y_cm_s_at_1m', 'flow_y'),
        ('quality', 'quality'),
        ('distance_status', 'distance_status'),
        ('flow_status', 'flow_status'),
        ('strength', 'strength'),
        ('precision', 'precision'),
        ('flags', 'flags'),
        ('frames', 'frames'),
        ('crc_errors', 'crc_errors'),
        ('gaps', 'gaps'),
        ('uart_errors', 'uart_errors'),
        ('dropped_bytes', 'dropped_bytes'),
    ],
    'motor': [
        ('t_ms', 't_ms'),
        ('armed', 'flags'),
        ('m1_permille', 'permille.0'),
        ('m2_permille', 'permille.1'),
        ('m3_permille', 'permille.2'),
        ('m4_permille', 'permille.3'),
        ('m1_dshot', 'dshot.0'),
        ('m2_dshot', 'dshot.1'),
        ('m3_dshot', 'dshot.2'),
        ('m4_dshot', 'dshot.3'),
        ('kiss_request_mask', 'telemetry_mask'),
    ],
    'esc': [
        ('t_ms', 't_ms'),
        ('motor_index0', 'motor'),
        ('temperature_c', 'temperature_c'),
        ('voltage_cv', 'voltage_cv'),
        ('current_ca', 'current_ca'),
        ('consumption_mah', 'consumption_mah'),
        ('erpm_div100', 'erpm_100'),
        ('frames_ok', 'frames_ok'),
        ('timeouts', 'timeouts'),
    ],
    'power': [
        ('t_ms', 't_ms'),
        ('vbat_pin_v', 'vbat_pin_v'),
        ('current_pin_v', 'current_pin_v'),
        ('vbat_v', 'vbat_v'),
        ('current_a', 'current_a'),
        ('flags', 'flags'),
    ],
    'status': [
        ('t_ms', 't_ms'),
        ('flags', 'flags'),
        ('owner', 'owner'),
        ('cap_permille', 'cap_permille'),
        ('rate_permille_s', 'rate_permille_s'),
        ('m1_target_permille', 'target_permille.0'),
        ('m2_target_permille', 'target_permille.1'),
        ('m3_target_permille', 'target_permille.2'),
        ('m4_target_permille', 'target_permille.3'),
        ('reversed_mask', 'reversed'),
        ('direction_sent_mask', 'direction_sent'),
        ('tx_failures', 'tx_failures'),
        ('heartbeat_age_ms', 'heartbeat_age_ms'),
    ],
    'sys': [
        ('t_ms', 't_ms'),
        ('telemetry_dropped', 'telemetry_dropped'),
        ('heap_free_b', 'heap_free'),
        ('link_packets', 'link_packets'),
        ('link_errors', 'link_errors'),
        ('link_overflows', 'link_overflows'),
        ('esc_crc_errors', 'esc_crc_errors'),
        ('esc_length_errors', 'esc_length_errors'),
        ('esc_unsolicited', 'esc_unsolicited'),
        ('esc_uart_errors', 'esc_uart_errors'),
        ('esc_timeouts_m1', 'esc_timeouts.0'),
        ('esc_timeouts_m2', 'esc_timeouts.1'),
        ('esc_timeouts_m3', 'esc_timeouts.2'),
        ('esc_timeouts_m4', 'esc_timeouts.3'),
        ('bno_errors', 'bno_errors'),
        ('bno_resets', 'bno_resets'),
        ('bno_fault', 'bno_fault'),
    ],
    'ctrl': [
        ('t_ms', 't_ms'),
        ('state', 'state'),
        ('flags', 'flags'),
        ('fail', 'fail'),
        ('z_meas_m_ned', 'y.0'),
        ('phi_meas_rad', 'y.1'),
        ('theta_meas_rad', 'y.2'),
        ('psi_meas_rad', 'y.3'),
        ('u_meas_m_s', 'y.4'),
        ('v_meas_m_s', 'y.5'),
        ('p_meas_rad_s', 'y.6'),
        ('q_meas_rad_s', 'y.7'),
        ('r_meas_rad_s', 'y.8'),
        ('x_est_m', 'pos.0'),
        ('y_est_m', 'pos.1'),
        ('z_est_m_ned', 'pos.2'),
        ('u_est_m_s', 'vel.0'),
        ('v_est_m_s', 'vel.1'),
        ('w_est_m_s_ned', 'vel.2'),
        ('h_ref_m', 'h_ref'),
        ('trim_permille', 'trim'),
        ('base_permille', 'base'),
        ('x_ref_m', 'ref_xy.0'),
        ('y_ref_m', 'ref_xy.1'),
        ('psi_ref_rad', 'ref_psi'),
        ('vbat_factor', 'vbat_factor'),
    ],
    'text': [('t_ms', 't_ms'), ('level', 'level'), ('text', 'text')],
}


def _get(msg: dict, path: str):
    key, _, index = path.partition('.')
    value = msg[key]
    return value[int(index)] if index else value


def _slug(text: str) -> str:
    slug = re.sub(r'[^A-Za-z0-9_-]+', '-', text.strip()).strip('-')
    return slug[:40] or 'sessao'


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


class Recorder:
    def __init__(self, root: Path, name: str, meta: dict) -> None:
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = root / f'{stamp}_{_slug(name)}'
        self.path.mkdir(parents=True, exist_ok=False)
        self.start = time.monotonic()
        self.meta = {
            'format': 'aeroroute-bench-session/1',
            'name': name,
            'started_utc': utc_now(),
            'protocol': 'bench-link v1',
            **meta,
        }
        self.counts: dict[str, int] = {}
        self.raw = open(self.path / 'raw.blr', 'wb')  # noqa: SIM115 - closed in stop()
        self.raw.write(MAGIC)
        self.events = open(self.path / 'events.jsonl', 'w', encoding='utf-8')  # noqa: SIM115
        self.files: dict[str, tuple] = {}
        self._write_meta()

    def _write_meta(self) -> None:
        (self.path / 'meta.json').write_text(json.dumps(self.meta, indent=2, ensure_ascii=False))

    def elapsed(self) -> float:
        return time.monotonic() - self.start

    def write_raw(self, data: bytes) -> None:
        self.raw.write(RECORD.pack(self.elapsed(), len(data)))
        self.raw.write(data)

    def write_message(self, msg: dict) -> None:
        kind = msg['type']
        columns = CSV_COLUMNS.get(kind)
        if columns is None:
            return
        entry = self.files.get(kind)
        if entry is None:
            handle = open(self.path / f'{kind}.csv', 'w', newline='', encoding='utf-8')  # noqa: SIM115
            writer = csv.writer(handle)
            writer.writerow(['host_s', *(c for c, _ in columns)])
            entry = self.files[kind] = (handle, writer)
        entry[1].writerow([f'{self.elapsed():.4f}', *(_get(msg, path) for _, path in columns)])
        self.counts[kind] = self.counts.get(kind, 0) + 1

    def event(self, kind: str, **data) -> None:
        record = {'host_s': round(self.elapsed(), 4), 'utc': utc_now(), 'event': kind, **data}
        self.events.write(json.dumps(record, ensure_ascii=False) + '\n')
        self.events.flush()

    def update_meta(self, **fields) -> None:
        self.meta.update(fields)
        self._write_meta()

    def stop(self, freezes: list[dict]) -> Path:
        self.meta['stopped_utc'] = utc_now()
        self.meta['duration_s'] = round(self.elapsed(), 3)
        self.meta['counts'] = self.counts
        self._write_meta()
        (self.path / 'freezes.json').write_text(json.dumps(freezes, indent=2, ensure_ascii=False))
        self.raw.close()
        self.events.close()
        for handle, _ in self.files.values():
            handle.close()
        return self.path


def read_raw(path: Path) -> list[tuple[float, bytes]]:
    """All chunks of a ``raw.blr`` file; a truncated tail is ignored."""
    data = path.read_bytes()
    if not data.startswith(MAGIC):
        raise ValueError(f'{path} is not a bench-link recording')
    out = []
    offset = len(MAGIC)
    while offset + RECORD.size <= len(data):
        t, length = RECORD.unpack_from(data, offset)
        offset += RECORD.size
        if offset + length > len(data):
            break
        out.append((t, data[offset : offset + length]))
        offset += length
    return out


def list_sessions(root: Path) -> list[dict]:
    out = []
    if not root.is_dir():
        return out
    for folder in sorted(root.iterdir(), reverse=True):
        meta_path = folder / 'meta.json'
        if not meta_path.is_file():
            continue
        try:
            meta = json.loads(meta_path.read_text())
        except json.JSONDecodeError:
            continue
        out.append(
            {
                'id': folder.name,
                'name': meta.get('name'),
                'started_utc': meta.get('started_utc'),
                'duration_s': meta.get('duration_s'),
                'notes': meta.get('notes', ''),
                'firmware_id': (meta.get('hello') or {}).get('firmware_id'),
            }
        )
    return out
