"""Golden bench-link frames shared with the C host test.

Regenerate (only after a deliberate protocol change):

    cd hmi && conda run -n aeroroute python -m tests.vectors --write

``stm32/tests/test_bench_link.c`` builds the same messages with the firmware
encoders and must produce identical bytes.
"""

from __future__ import annotations

import sys
from pathlib import Path

from benchlink import protocol as p

VECTOR_FILE = Path(__file__).resolve().parents[2] / 'stm32/tests/vectors/bench_link.txt'

#: (name, seq, message dict) — values exactly representable in float32.
TELEMETRY = [
    (
        'hello',
        1,
        {
            'type': 'hello',
            'capabilities': 7,
            'uptime_ms': 123456,
            'firmware_id': '0123456789abcdef01234567',
            'variant': 'Release',
        },
    ),
    (
        'status',
        2,
        {
            'type': 'status',
            't_ms': 1000,
            'flags': p.STATUS_ARMED | p.STATUS_HEARTBEAT_OK | p.STATUS_STREAM,
            'owner': 1,
            'cap_permille': 400,
            'rate_permille_s': 200,
            'target_permille': [100, 0, 250, 400],
            'reversed': 9,
            'direction_sent': 9,
            'direction_known': 0,
            'commands_pending': 0,
            'tx_failures': 3,
            'heartbeat_age_ms': 42,
            'bno_divider': 1,
        },
    ),
    (
        'imu',
        3,
        {
            'type': 'imu',
            't_ms': 2000,
            'q': [1.0, 0.0, -0.5, 0.25],
            'gyro': [0.125, -0.0625, 2.0],
            'lin_accel': [-9.5, 0.0, 0.75],
            'accuracy': [3, 2, 1],
            'flags': 1,
            'count': [400, 401, 402],
            'gaps': [0, 1, 2],
        },
    ),
    (
        'range',
        4,
        {
            'type': 'range',
            't_ms': 3000,
            'sensor_ms': 2990,
            'distance_mm': 1234,
            'flow_x': -12,
            'flow_y': 34,
            'quality': 200,
            'distance_status': 1,
            'flow_status': 1,
            'strength': 90,
            'precision': 5,
            'flags': 3,
            'frames': 100,
            'crc_errors': 1,
            'gaps': 2,
            'uart_errors': 0,
            'dropped_bytes': 0,
        },
    ),
    (
        'motor',
        5,
        {
            'type': 'motor',
            't_ms': 4000,
            'flags': 1,
            'permille': [100, 200, 0, 1000],
            'dshot': [248, 448, 0, 2047],
            'telemetry_mask': 4,
        },
    ),
    (
        'esc',
        6,
        {
            'type': 'esc',
            't_ms': 5000,
            'motor': 2,
            'temperature_c': -5,
            'voltage_cv': 1680,
            'current_ca': 250,
            'consumption_mah': 12,
            'erpm_100': 345,
            'frames_ok': 77,
            'timeouts': 3,
        },
    ),
    (
        'power',
        7,
        {
            'type': 'power',
            't_ms': 6000,
            'vbat_pin_v': 1.5,
            'current_pin_v': 0.25,
            'vbat_v': 16.5,
            'current_a': -2.0,
            'flags': 0x0F,
        },
    ),
    (
        'sys',
        8,
        {
            'type': 'sys',
            't_ms': 7000,
            'telemetry_dropped': 1,
            'heap_free': 20000,
            'link_packets': 500,
            'link_errors': 2,
            'link_overflows': 0,
            'esc_crc_errors': 3,
            'esc_length_errors': 4,
            'esc_unsolicited': 5,
            'esc_uart_errors': 6,
            'esc_timeouts': [7, 8, 9, 10],
            'bno_errors': 11,
            'bno_resets': 12,
            'bno_fault': 7,
        },
    ),
    (
        'ctrl',
        11,
        {
            'type': 'ctrl',
            't_ms': 9000,
            'state': 4,
            'flags': 0x1F,
            'fail': 5,
            'y': [-0.25, 0.0625, -0.125, 0.5, 0.03125, -0.015625, 0.25, -0.5, 1.0],
            'pos': [0.5, -0.25, -0.3125],
            'vel': [0.0625, 0.0, -0.125],
            'h_ref': 0.3125,
            'trim': -12.5,
            'base': 537.5,
            'ref_xy': [0.25, -0.125],
            'ref_psi': 0.5,
            'vbat_factor': 1.0625,
        },
    ),
    ('ack', 9, {'type': 'ack', 'command': p.CMD_CAP, 'seq': 17, 'result': 0, 'value': 400}),
    ('text', 10, {'type': 'text', 't_ms': 8000, 'level': 1, 'text': 'ola bancada'}),
]

#: (name, seq, command name, kwargs)
COMMANDS = [
    ('cmd_motor_set', 20, 'motor_set', {'values': {1: 10, 3: 25}}),
    ('cmd_cap', 21, 'cap', {'percent': 40}),
    ('cmd_direction', 22, 'direction', {'motors': [1, 4], 'reversed': [1], 'save': False}),
    ('cmd_sweep', 23, 'sweep', {'motors': [2], 'max_percent': 30, 'step_percent': 5, 'hold_s': 2}),
    (
        'cmd_power_cal',
        24,
        'power_cal',
        {'vbat_gain': 11.0, 'current_a_per_v': 50.0, 'current_offset_v': 0.5, 'vbat_min_v': 10.5},
    ),
    ('cmd_heartbeat', 25, 'heartbeat', {}),
    ('cmd_ctrl_takeoff', 26, 'ctrl', {'action': 'takeoff'}),
    (
        'cmd_ctrl_gains',
        28,
        'ctrl_gains',
        {'z': 8.0, 'w': 4.0, 'att': 1.5, 'rate': 1.0, 'yaw': 2.0, 'yaw_rate': 1.0, 'xy': 1.0, 'vel': 0.5},
    ),
    (
        'cmd_ctrl_params',
        27,
        'ctrl_params',
        {
            'height_m': 0.5,
            'hover_percent': 60.0,
            'cap_percent': 80.0,
            'climb_m_s': 0.125,
            'land_m_s': 0.25,
            'spool_percent_per_s': 5.0,
            'trim_ki': 1.0,
            'ki_xy': 0.125,
            'ki_yaw': 0.25,
            'autotrim': True,
            'use_ekf': True,
        },
    ),
]


def frames() -> list[tuple[str, bytes]]:
    out = []
    for name, seq, message in TELEMETRY:
        msg_id, payload = p.encode_message(message)
        out.append((name, p.encode_frame(msg_id, seq, payload)))
    for name, seq, command, kwargs in COMMANDS:
        msg_id, payload = p.command_payload(command, **kwargs)
        out.append((name, p.encode_frame(msg_id, seq, payload)))
    return out


def render() -> str:
    lines = ['# bench-link v1 golden frames: <name> <hex of COBS frame including 0x00>']
    lines += [f'{name} {frame.hex()}' for name, frame in frames()]
    return '\n'.join(lines) + '\n'


def read() -> dict[str, bytes]:
    out = {}
    for line in VECTOR_FILE.read_text().splitlines():
        if line and not line.startswith('#'):
            name, hexed = line.split()
            out[name] = bytes.fromhex(hexed)
    return out


if __name__ == '__main__':
    if '--write' in sys.argv:
        VECTOR_FILE.parent.mkdir(parents=True, exist_ok=True)
        VECTOR_FILE.write_text(render())
        print(f'wrote {VECTOR_FILE}')
    else:
        print(render(), end='')
