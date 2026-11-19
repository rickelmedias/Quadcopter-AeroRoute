"""bench-link v1: COBS + CRC16 framing and message codecs.

Mirror of ``stm32/App/Inc/link/bench_frame.h`` and ``bench_proto.h``. Message IDs
and payload sizes are checked against the C header by ``tests/test_contract.py``
and the byte-level framing by shared golden vectors
(``stm32/tests/vectors/bench_link.txt``). Specification:
``docs/phase2/bench-link.md``.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field

PROTO_MAJOR = 1
PROTO_MINOR = 4
MAX_PAYLOAD = 96

# Device -> host
MSG_HELLO = 0x01
MSG_STATUS = 0x02
MSG_IMU = 0x10
MSG_RANGE = 0x11
MSG_MOTOR = 0x12
MSG_ESC = 0x13
MSG_POWER = 0x14
MSG_SYS = 0x15
MSG_CTRL = 0x16
MSG_TEXT = 0x20
MSG_ACK = 0x21

# Host -> device
CMD_HELLO = 0x80
CMD_HEARTBEAT = 0x81
CMD_ARM = 0x82
CMD_STOP = 0x83
CMD_MOTOR_SET = 0x84
CMD_CAP = 0x86
CMD_RATE = 0x87
CMD_DIRECTION = 0x88
CMD_BEEP = 0x89
CMD_SWEEP = 0x8A
CMD_STREAM = 0x8B
CMD_POWER_CAL = 0x8C
CMD_ESC_TELEM = 0x8D
CMD_CTRL = 0x90
CMD_CTRL_PARAMS = 0x91
CMD_CTRL_GAINS = 0x92

STATUS_ARMED = 0x01
STATUS_SWEEPING = 0x02
STATUS_HEARTBEAT_OK = 0x04
STATUS_RX_BLOCKED = 0x08
STATUS_ESC_TELEM = 0x10
STATUS_STREAM = 0x20
STATUS_POWER_LOW = 0x40

#: Control (LQR) states reported in MSG_CTRL, and BL_CMD_CTRL actions.
CTRL_STATES = ['desarmado', 'armado', 'subindo aceleracao', 'subindo', 'hover', 'pouso']
CTRL_ACTIONS = {'arm': 1, 'takeoff': 2, 'land': 3}
CTRL_FAILS = [
    'nenhuma',
    'BNO sem dado',
    'inclinacao excessiva',
    'MTF sem distancia',
    'nao descolou',
    'pouso concluido',
    'estimador invalido',
]
CTRL_FLAG_FILTER_FREE = 0x01
CTRL_FLAG_RANGE = 0x02
CTRL_FLAG_FLOW = 0x04
CTRL_FLAG_ATTITUDE = 0x08
CTRL_FLAG_TRIM = 0x10
CTRL_FLAG_SATURATED = 0x20
CTRL_FLAG_XY_ACTIVE = 0x40
CTRL_FLAG_VBAT = 0x80
#: Editable control parameters: (min, max) in the units of ``command_payload``.
#: Mirror of the LQG_PARAM_* limits in ``stm32/App/Inc/control/lqg_ctrl.h``.
CTRL_LIMITS = {
    'height_m': (0.10, 0.80),
    'hover_percent': (10.0, 90.0),
    'cap_percent': (20.0, 100.0),
    'climb_m_s': (0.02, 0.50),
    'land_m_s': (0.02, 0.50),
    'spool_percent_per_s': (0.5, 20.0),
    'trim_ki': (0.0, 20.0),
    'ki_xy': (0.0, 0.5),
    'ki_yaw': (0.0, 2.0),
}
CTRL_DEFAULTS = {
    'height_m': 0.30,
    'hover_percent': 55.0,
    'cap_percent': 95.0,
    'climb_m_s': 0.10,
    'land_m_s': 0.10,
    'spool_percent_per_s': 4.0,
    'trim_ki': 6.0,
    'ki_xy': 0.05,
    'ki_yaw': 0.3,
    'autotrim': True,
    'use_ekf': True,
}

#: Multiplicadores dos ganhos Kd gerados (1 = original). Ordem do comando e limites
#: de ``lqg_gains_valid`` em ``stm32/App/Inc/control/lqg_ctrl.h``.
GAIN_KEYS = ['z', 'w', 'att', 'rate', 'yaw', 'yaw_rate', 'xy', 'vel']
GAIN_LIMITS = {
    'z': (0.0, 20.0),
    'w': (0.0, 20.0),
    'att': (0.25, 5.0),
    'rate': (0.25, 5.0),
    'yaw': (0.0, 10.0),
    'yaw_rate': (0.0, 10.0),
    'xy': (0.0, 10.0),
    'vel': (0.0, 10.0),
}
#: Guinada 5x como nos voos de 14/11/2026. 10x foi testado e não ajudou: a autoridade de guinada
#: medida é bem menor que a do modelo e o ganho alto só deu picos de comando (ver control-lqg.md).
GAIN_DEFAULTS = {**dict.fromkeys(GAIN_KEYS, 1.0), 'yaw': 5.0, 'yaw_rate': 5.0}

RESULT_OK = 0
RESULT_REJECTED = 1
RESULT_INVALID = 2
RESULT_UNKNOWN = 3
RESULT_NAMES = {0: 'ok', 1: 'recusado', 2: 'invalido', 3: 'desconhecido'}

BNO_FAULTS = [
    'nenhuma',
    'sem dispositivo',
    'transporte',
    'protocolo',
    'sem reset',
    'identidade',
    'configuracao',
    'relatorios parados',
]

#: Payload layouts (little-endian). Names follow the C structs.
TELEMETRY_FORMATS = {
    MSG_HELLO: '<BBHI24s8s',
    MSG_STATUS: '<IBBHH4HBBBBIIBB',
    MSG_IMU: '<I4f3f3f3BB3I3I',
    MSG_RANGE: '<IIIhhBBBBBB5I',
    MSG_MOTOR: '<IB4H4HB',
    MSG_ESC: '<IBbHHHHIIB',
    MSG_POWER: '<I4fB',
    MSG_SYS: '<I9I4I2IB3x',
    MSG_CTRL: '<IBBBB9f3f3f3f4f',
    MSG_ACK: '<BBBH',
}
COMMAND_FORMATS = {
    CMD_HELLO: '<',
    CMD_HEARTBEAT: '<',
    CMD_ARM: '<',
    CMD_STOP: '<',
    CMD_MOTOR_SET: '<B4H',
    CMD_CAP: '<H',
    CMD_RATE: '<H',
    CMD_DIRECTION: '<BBB',
    CMD_BEEP: '<B',
    CMD_SWEEP: '<BHHI',
    CMD_STREAM: '<BB',
    CMD_POWER_CAL: '<4fB',
    CMD_ESC_TELEM: '<BH',
    CMD_CTRL: '<B',
    CMD_CTRL_PARAMS: '<9fB',
    CMD_CTRL_GAINS: '<8f',
}
TEXT_HEADER = '<IB'
TEXT_MAX = 60

MESSAGE_NAMES = {
    MSG_HELLO: 'hello',
    MSG_STATUS: 'status',
    MSG_IMU: 'imu',
    MSG_RANGE: 'range',
    MSG_MOTOR: 'motor',
    MSG_ESC: 'esc',
    MSG_POWER: 'power',
    MSG_SYS: 'sys',
    MSG_CTRL: 'ctrl',
    MSG_TEXT: 'text',
    MSG_ACK: 'ack',
}
COMMAND_NAMES = {
    CMD_HELLO: 'hello',
    CMD_HEARTBEAT: 'heartbeat',
    CMD_ARM: 'arm',
    CMD_STOP: 'stop',
    CMD_MOTOR_SET: 'motor_set',
    CMD_CAP: 'cap',
    CMD_RATE: 'rate',
    CMD_DIRECTION: 'direction',
    CMD_BEEP: 'beep',
    CMD_SWEEP: 'sweep',
    CMD_STREAM: 'stream',
    CMD_POWER_CAL: 'power_cal',
    CMD_ESC_TELEM: 'esc_telem',
    CMD_CTRL: 'ctrl',
    CMD_CTRL_PARAMS: 'ctrl_params',
    CMD_CTRL_GAINS: 'ctrl_gains',
}


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    """COBS without delimiter; identical byte output to ``bl_cobs_encode``."""
    out = bytearray([0])
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_index] = code
                code_index = len(out)
                out.append(0)
                code = 1
    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes | None:
    """Decode a COBS frame (without delimiter); None when malformed."""
    out = bytearray()
    i = 0
    if not data:
        return None
    while i < len(data):
        code = data[i]
        i += 1
        if code == 0 or i + code - 1 > len(data):
            return None
        block = data[i : i + code - 1]
        if 0 in block:
            return None
        out += block
        i += code - 1
        if code < 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def encode_frame(msg_id: int, seq: int, payload: bytes = b'') -> bytes:
    """Complete wire frame: COBS(id, seq, payload, crc16 LE) + 0x00."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('payload too large')
    packet = bytes([msg_id & 0xFF, seq & 0xFF]) + payload
    packet += struct.pack('<H', crc16(packet))
    return cobs_encode(packet) + b'\x00'


@dataclass
class Packet:
    msg_id: int
    seq: int
    payload: bytes


@dataclass
class FrameDecoder:
    """Incremental frame decoder; mirrors ``bl_decoder_t`` limits."""

    max_frame: int = MAX_PAYLOAD + 4 + 2 + 2
    buffer: bytearray = field(default_factory=bytearray)
    overflow: bool = False
    packets: int = 0
    errors: int = 0

    def feed(self, data: bytes) -> list[Packet]:
        out: list[Packet] = []
        for byte in data:
            if byte == 0:
                if self.buffer or self.overflow:
                    packet = self._finish()
                    if packet is not None:
                        out.append(packet)
            elif len(self.buffer) < self.max_frame:
                self.buffer.append(byte)
            else:
                self.overflow = True
        return out

    def reset(self) -> None:
        self.buffer.clear()
        self.overflow = False

    def _finish(self) -> Packet | None:
        raw = None if self.overflow else cobs_decode(bytes(self.buffer))
        self.reset()
        if raw is None or len(raw) < 4:
            self.errors += 1
            return None
        body, crc = raw[:-2], struct.unpack('<H', raw[-2:])[0]
        if crc16(body) != crc:
            self.errors += 1
            return None
        self.packets += 1
        return Packet(body[0], body[1], bytes(body[2:]))


# ---- Telemetry decoding ----------------------------------------------------------


def _cstr(raw: bytes) -> str:
    return raw.split(b'\x00', 1)[0].decode('ascii', 'replace')


def decode_message(msg_id: int, payload: bytes) -> dict | None:
    """Decode a device message into a dict with ``type``; None if unknown/bad size."""
    name = MESSAGE_NAMES.get(msg_id)
    if name is None:
        return None
    if msg_id == MSG_TEXT:
        size = struct.calcsize(TEXT_HEADER)
        if len(payload) < size or len(payload) > size + TEXT_MAX:
            return None
        t_ms, level = struct.unpack_from(TEXT_HEADER, payload)
        text = payload[size:].decode('latin-1')
        return {'type': name, 't_ms': t_ms, 'level': level, 'text': text}
    fmt = TELEMETRY_FORMATS[msg_id]
    if len(payload) != struct.calcsize(fmt):
        return None
    v = struct.unpack(fmt, payload)
    if msg_id == MSG_HELLO:
        return {
            'type': name,
            'major': v[0],
            'minor': v[1],
            'capabilities': v[2],
            'uptime_ms': v[3],
            'firmware_id': _cstr(v[4]),
            'variant': _cstr(v[5]),
        }
    if msg_id == MSG_STATUS:
        return {
            'type': name,
            't_ms': v[0],
            'flags': v[1],
            'owner': v[2],
            'cap_permille': v[3],
            'rate_permille_s': v[4],
            'target_permille': list(v[5:9]),
            'reversed': v[9],
            'direction_sent': v[10],
            'direction_known': v[11],
            'commands_pending': v[12],
            'tx_failures': v[13],
            'heartbeat_age_ms': v[14],
            'bno_divider': v[15],
        }
    if msg_id == MSG_IMU:
        return {
            'type': name,
            't_ms': v[0],
            'q': list(v[1:5]),
            'gyro': list(v[5:8]),
            'lin_accel': list(v[8:11]),
            'accuracy': list(v[11:14]),
            'flags': v[14],
            'count': list(v[15:18]),
            'gaps': list(v[18:21]),
        }
    if msg_id == MSG_RANGE:
        return {
            'type': name,
            't_ms': v[0],
            'sensor_ms': v[1],
            'distance_mm': v[2],
            'flow_x': v[3],
            'flow_y': v[4],
            'quality': v[5],
            'distance_status': v[6],
            'flow_status': v[7],
            'strength': v[8],
            'precision': v[9],
            'flags': v[10],
            'frames': v[11],
            'crc_errors': v[12],
            'gaps': v[13],
            'uart_errors': v[14],
            'dropped_bytes': v[15],
        }
    if msg_id == MSG_MOTOR:
        return {
            'type': name,
            't_ms': v[0],
            'flags': v[1],
            'permille': list(v[2:6]),
            'dshot': list(v[6:10]),
            'telemetry_mask': v[10],
        }
    if msg_id == MSG_ESC:
        return {
            'type': name,
            't_ms': v[0],
            'motor': v[1],
            'temperature_c': v[2],
            'voltage_cv': v[3],
            'current_ca': v[4],
            'consumption_mah': v[5],
            'erpm_100': v[6],
            'frames_ok': v[7],
            'timeouts': v[8],
        }
    if msg_id == MSG_POWER:
        return {
            'type': name,
            't_ms': v[0],
            'vbat_pin_v': v[1],
            'current_pin_v': v[2],
            'vbat_v': v[3],
            'current_a': v[4],
            'flags': v[5],
        }
    if msg_id == MSG_SYS:
        return {
            'type': name,
            't_ms': v[0],
            'telemetry_dropped': v[1],
            'heap_free': v[2],
            'link_packets': v[3],
            'link_errors': v[4],
            'link_overflows': v[5],
            'esc_crc_errors': v[6],
            'esc_length_errors': v[7],
            'esc_unsolicited': v[8],
            'esc_uart_errors': v[9],
            'esc_timeouts': list(v[10:14]),
            'bno_errors': v[14],
            'bno_resets': v[15],
            'bno_fault': v[16],
        }
    if msg_id == MSG_CTRL:
        return {
            'type': name,
            't_ms': v[0],
            'state': v[1],
            'flags': v[2],
            'fail': v[3],
            'y': list(v[5:14]),
            'pos': list(v[14:17]),
            'vel': list(v[17:20]),
            'h_ref': v[20],
            'trim': v[21],
            'base': v[22],
            'ref_xy': [v[23], v[24]],
            'ref_psi': v[25],
            'vbat_factor': v[26],
        }
    if msg_id == MSG_ACK:
        return {
            'type': name,
            'command': v[0],
            'seq': v[1],
            'result': v[2],
            'value': v[3],
        }
    return None


# ---- Telemetry encoding (simulator and golden vectors) ------------------------


def _fixed(text: str, width: int) -> bytes:
    return text.encode('ascii')[:width].ljust(width, b'\x00')


def encode_message(m: dict) -> tuple[int, bytes]:
    """Inverse of :func:`decode_message`."""
    t = m['type']
    if t == 'text':
        raw = m['text'].encode('latin-1')[:TEXT_MAX]
        return MSG_TEXT, struct.pack(TEXT_HEADER, m['t_ms'], m['level']) + raw
    if t == 'hello':
        args = (
            m.get('major', PROTO_MAJOR),
            m.get('minor', PROTO_MINOR),
            m['capabilities'],
            m['uptime_ms'],
            _fixed(m['firmware_id'], 24),
            _fixed(m['variant'], 8),
        )
        return MSG_HELLO, struct.pack(TELEMETRY_FORMATS[MSG_HELLO], *args)
    if t == 'status':
        args = (
            m['t_ms'],
            m['flags'],
            m['owner'],
            m['cap_permille'],
            m['rate_permille_s'],
            *m['target_permille'],
            m['reversed'],
            m['direction_sent'],
            m['direction_known'],
            m['commands_pending'],
            m['tx_failures'],
            m['heartbeat_age_ms'],
            m['bno_divider'],
            0,
        )
        return MSG_STATUS, struct.pack(TELEMETRY_FORMATS[MSG_STATUS], *args)
    if t == 'imu':
        args = (
            m['t_ms'],
            *m['q'],
            *m['gyro'],
            *m['lin_accel'],
            *m['accuracy'],
            m['flags'],
            *m['count'],
            *m['gaps'],
        )
        return MSG_IMU, struct.pack(TELEMETRY_FORMATS[MSG_IMU], *args)
    if t == 'range':
        keys = [
            't_ms',
            'sensor_ms',
            'distance_mm',
            'flow_x',
            'flow_y',
            'quality',
            'distance_status',
            'flow_status',
            'strength',
            'precision',
            'flags',
            'frames',
            'crc_errors',
            'gaps',
            'uart_errors',
            'dropped_bytes',
        ]
        return MSG_RANGE, struct.pack(TELEMETRY_FORMATS[MSG_RANGE], *(m[k] for k in keys))
    if t == 'motor':
        args = (m['t_ms'], m['flags'], *m['permille'], *m['dshot'], m['telemetry_mask'])
        return MSG_MOTOR, struct.pack(TELEMETRY_FORMATS[MSG_MOTOR], *args)
    if t == 'esc':
        keys = [
            't_ms',
            'motor',
            'temperature_c',
            'voltage_cv',
            'current_ca',
            'consumption_mah',
            'erpm_100',
            'frames_ok',
            'timeouts',
        ]
        return MSG_ESC, struct.pack(TELEMETRY_FORMATS[MSG_ESC], *(m[k] for k in keys), 0)
    if t == 'power':
        keys = ['t_ms', 'vbat_pin_v', 'current_pin_v', 'vbat_v', 'current_a', 'flags']
        return MSG_POWER, struct.pack(TELEMETRY_FORMATS[MSG_POWER], *(m[k] for k in keys))
    if t == 'sys':
        args = (
            m['t_ms'],
            m['telemetry_dropped'],
            m['heap_free'],
            m['link_packets'],
            m['link_errors'],
            m['link_overflows'],
            m['esc_crc_errors'],
            m['esc_length_errors'],
            m['esc_unsolicited'],
            m['esc_uart_errors'],
            *m['esc_timeouts'],
            m['bno_errors'],
            m['bno_resets'],
            m['bno_fault'],
        )
        return MSG_SYS, struct.pack(TELEMETRY_FORMATS[MSG_SYS], *args)
    if t == 'ctrl':
        args = (
            m['t_ms'],
            m['state'],
            m['flags'],
            m['fail'],
            0,
            *m['y'],
            *m['pos'],
            *m['vel'],
            m['h_ref'],
            m['trim'],
            m['base'],
            *m.get('ref_xy', [0.0, 0.0]),
            m.get('ref_psi', 0.0),
            m.get('vbat_factor', 1.0),
        )
        return MSG_CTRL, struct.pack(TELEMETRY_FORMATS[MSG_CTRL], *args)
    if t == 'ack':
        args = (m['command'], m['seq'], m['result'], m['value'])
        return MSG_ACK, struct.pack(TELEMETRY_FORMATS[MSG_ACK], *args)
    raise ValueError(f'unknown message type {t}')


# ---- Commands ------------------------------------------------------------------


def _mask(motors) -> int:
    mask = 0
    for m in motors:
        if not 1 <= int(m) <= 4:
            raise ValueError('motors are numbered 1..4')
        mask |= 1 << (int(m) - 1)
    return mask


def command_payload(name: str, **kw) -> tuple[int, bytes]:
    """Build a command payload from friendly arguments (percent, seconds, motors 1..4).

    Ranges are validated here too, so the HMI never sends something the firmware
    would answer with INVALID (which also stops the motors).
    """

    def permille(pct: float) -> int:
        value = round(float(pct) * 10)
        if not 0 <= value <= 1000:
            raise ValueError('percentage outside 0..100')
        return value

    if name in ('hello', 'heartbeat', 'arm', 'stop'):
        return {'hello': CMD_HELLO, 'heartbeat': CMD_HEARTBEAT, 'arm': CMD_ARM, 'stop': CMD_STOP}[
            name
        ], b''
    if name == 'motor_set':
        values = kw['values']  # dict motor(1..4) -> percent
        mask = _mask(values.keys())
        if mask == 0:
            raise ValueError('no motor selected')
        per = [0, 0, 0, 0]
        for motor, pct in values.items():
            per[int(motor) - 1] = permille(pct)
        return CMD_MOTOR_SET, struct.pack('<B4H', mask, *per)
    if name == 'cap':
        return CMD_CAP, struct.pack('<H', permille(kw['percent']))
    if name == 'rate':
        value = permille(kw['percent_per_s'])
        if value < 10:
            raise ValueError('ramp below 1 %/s')
        return CMD_RATE, struct.pack('<H', value)
    if name == 'direction':
        mask = _mask(kw['motors'])
        reversed_bits = _mask(kw.get('reversed', []))
        if reversed_bits & ~mask or mask == 0:
            raise ValueError('reversed motors must be selected')
        return CMD_DIRECTION, struct.pack('<BBB', mask, reversed_bits, 1 if kw.get('save') else 0)
    if name == 'beep':
        mask = _mask(kw['motors'])
        if mask == 0:
            raise ValueError('no motor selected')
        return CMD_BEEP, struct.pack('<B', mask)
    if name == 'sweep':
        mask = _mask(kw['motors'])
        step = permille(kw['step_percent'])
        hold = round(float(kw['hold_s']) * 1000)
        if mask == 0 or step < 1 or not 100 <= hold <= 30000:
            raise ValueError('invalid sweep')
        return CMD_SWEEP, struct.pack('<BHHI', mask, permille(kw['max_percent']), step, hold)
    if name == 'stream':
        divider = int(kw.get('bno_divider', 1))
        if not 1 <= divider <= 16:
            raise ValueError('divider 1..16')
        return CMD_STREAM, struct.pack('<BB', 1 if kw.get('enabled', True) else 0, divider)
    if name == 'power_cal':
        values = [
            float(kw.get('vbat_gain', 0.0)),
            float(kw.get('current_a_per_v', 0.0)),
            float(kw.get('current_offset_v', 0.0)),
            float(kw.get('vbat_min_v', 0.0)),
        ]
        if not all(math.isfinite(v) for v in values) or min(values[0], values[1], values[3]) < 0:
            raise ValueError('invalid calibration')
        flags = (1 if values[0] > 0 else 0) | (2 if values[1] > 0 else 0)
        return CMD_POWER_CAL, struct.pack('<4fB', *values, flags)
    if name == 'esc_telem':
        period = int(kw.get('period_ms', 10))
        if not 4 <= period <= 1000:
            raise ValueError('period 4..1000 ms')
        return CMD_ESC_TELEM, struct.pack('<BH', 1 if kw.get('enabled') else 0, period)
    if name == 'ctrl':
        action = CTRL_ACTIONS.get(str(kw.get('action')))
        if action is None:
            raise ValueError('action: arm | takeoff | land')
        return CMD_CTRL, struct.pack('<B', action)
    if name == 'ctrl_params':
        values = {k: float(kw.get(k, CTRL_DEFAULTS[k])) for k in CTRL_LIMITS}
        for key, (low, high) in CTRL_LIMITS.items():
            if not (math.isfinite(values[key]) and low <= values[key] <= high):
                raise ValueError(f'{key} fora de {low}..{high}')
        wire = (
            values['height_m'],
            values['hover_percent'] * 10.0,
            values['cap_percent'] * 10.0,
            values['climb_m_s'],
            values['land_m_s'],
            values['spool_percent_per_s'] * 10.0,
            values['trim_ki'] * 10.0,
            values['ki_xy'],
            values['ki_yaw'],
        )
        flags = (1 if kw.get('autotrim', True) else 0) | (2 if kw.get('use_ekf', True) else 0)
        return CMD_CTRL_PARAMS, struct.pack('<9fB', *wire, flags)
    if name == 'ctrl_gains':
        values = [float(kw.get(k, GAIN_DEFAULTS[k])) for k in GAIN_KEYS]
        for key, value in zip(GAIN_KEYS, values, strict=True):
            low, high = GAIN_LIMITS[key]
            if not (math.isfinite(value) and low <= value <= high):
                raise ValueError(f'ganho {key} fora de {low}..{high}')
        return CMD_CTRL_GAINS, struct.pack('<8f', *values)
    raise ValueError(f'unknown command {name}')


def decode_command(msg_id: int, payload: bytes) -> dict | None:
    """Parse a command like ``bl_command_parse`` (simulator). None = invalid."""
    fmt = COMMAND_FORMATS.get(msg_id)
    if fmt is None or len(payload) != struct.calcsize(fmt):
        return None
    v = struct.unpack(fmt, payload)
    name = COMMAND_NAMES[msg_id]
    out = {'name': name}

    def valid_mask(mask: int) -> bool:
        return 1 <= mask <= 15

    if msg_id == CMD_MOTOR_SET:
        if not valid_mask(v[0]) or any(x > 1000 for x in v[1:]):
            return None
        out.update(mask=v[0], values=list(v[1:]))
    elif msg_id == CMD_CAP:
        if v[0] > 1000:
            return None
        out['value'] = v[0]
    elif msg_id == CMD_RATE:
        if not 10 <= v[0] <= 1000:
            return None
        out['value'] = v[0]
    elif msg_id == CMD_DIRECTION:
        if not valid_mask(v[0]) or v[1] & ~v[0] or v[2] > 1:
            return None
        out.update(mask=v[0], reversed=v[1], save=bool(v[2]))
    elif msg_id == CMD_BEEP:
        if not valid_mask(v[0]):
            return None
        out['mask'] = v[0]
    elif msg_id == CMD_SWEEP:
        if not valid_mask(v[0]) or v[1] > 1000 or not 1 <= v[2] <= 1000:
            return None
        if not 100 <= v[3] <= 30000:
            return None
        out.update(mask=v[0], max=v[1], step=v[2], hold_ms=v[3])
    elif msg_id == CMD_STREAM:
        if v[0] > 1 or not 1 <= v[1] <= 16:
            return None
        out.update(enabled=bool(v[0]), bno_divider=v[1])
    elif msg_id == CMD_POWER_CAL:
        if not all(math.isfinite(x) for x in v[:4]) or min(v[0], v[1], v[3]) < 0 or v[4] & ~3:
            return None
        out.update(cal=list(v[:4]), flags=v[4])
    elif msg_id == CMD_ESC_TELEM:
        if v[0] > 1 or not 4 <= v[1] <= 1000:
            return None
        out.update(enabled=bool(v[0]), period_ms=v[1])
    elif msg_id == CMD_CTRL:
        if not 1 <= v[0] <= 3:
            return None
        out['action'] = {1: 'arm', 2: 'takeoff', 3: 'land'}[v[0]]
    elif msg_id == CMD_CTRL_GAINS:
        gains = dict(zip(GAIN_KEYS, v, strict=True))
        for key, value in gains.items():
            low, high = GAIN_LIMITS[key]
            if not (math.isfinite(value) and low <= value <= high):
                return None
        out['gains'] = gains
    elif msg_id == CMD_CTRL_PARAMS:
        limits = [
            (0.10, 0.80),
            (100.0, 900.0),
            (200.0, 1000.0),
            (0.02, 0.50),
            (0.02, 0.50),
            (5.0, 200.0),
            (0.0, 200.0),  # LQG_PARAM_TRIM_KI_MAX (‰/(m·s))
            (0.0, 0.5),
            (0.0, 2.0),
        ]
        if v[9] & ~3 or not all(
            math.isfinite(x) and low <= x <= high
            for x, (low, high) in zip(v[:9], limits, strict=True)
        ):
            return None
        out.update(
            height_m=v[0],
            hover_permille=v[1],
            cap_permille=v[2],
            climb_m_s=v[3],
            land_m_s=v[4],
            spool_permille_s=v[5],
            trim_ki_permille=v[6],
            ki_xy=v[7],
            ki_yaw=v[8],
            autotrim=bool(v[9] & 1),
            use_ekf=bool(v[9] & 2),
        )
    return out
