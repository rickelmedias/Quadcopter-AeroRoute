"""Simulated STM32 bench (Python), speaking bench-link v1 at byte level.

Used to develop and test the HMI without hardware. It mirrors the firmware
rules that matter to the operator: arming requires a recent HEARTBEAT, outputs
ramp and are clamped to the cap, 300 ms without HEARTBEAT disarms, direction and
beep only while disarmed, malformed/unknown commands stop the output.

The sensor values are synthetic: a level drone at ~0.30 m with noise, plus a
vibration component at each motor's rotation frequency proportional to its
command, so the spectrum view shows peaks next to the measured RPM markers.
This is not a model of the real drone and proves nothing about hardware.
"""

from __future__ import annotations

import math
import random

from . import protocol as p

KEEPALIVE_MS = 300
DEFAULT_CAP = 400
DEFAULT_RATE = 200
POLES = 14
#: Synthetic eRPM at full command (not a motor model).
ERPM_AT_FULL = 160_000


class SimulatedBench:
    def __init__(self, seed: int = 1, start_ms: int = 0) -> None:
        self.rng = random.Random(seed)
        self.decoder = p.FrameDecoder()
        self.out = bytearray()
        self.seq = 0
        self.now_ms = start_ms
        self.boot_ms = start_ms
        self.owner = 0
        self.cap = DEFAULT_CAP
        self.rate = DEFAULT_RATE
        self.target = [0, 0, 0, 0]
        self.current = [0.0, 0.0, 0.0, 0.0]
        self.reversed = 0
        self.direction_sent = 0
        self.heartbeat_ms: int | None = None
        self.sweep: dict | None = None
        self.stream = True
        self.bno_divider = 1
        self.esc_enabled = False
        self.esc_period = 10
        self.esc_next_ms = 0
        self.esc_motor = 0
        self.esc_ok = [0, 0, 0, 0]
        self.cal = [0.0, 0.0, 0.0, 0.0]
        self.cal_flags = 0
        self.link_errors = 0
        self.counts = [0, 0, 0]
        self.next = {
            'imu': 0.0,
            'range': 0.0,
            'motor': 0.0,
            'status': 0.0,
            'power': 0.0,
            'sys': 0.0,
        }
        self.mtf_frames = 0
        self.imu_index = 0
        self.connected = False
        self.yaw = 0.0
        # Controle LQR simulado: só a sequência armar/decolar/pairar/pousar, sem LQR.
        self.ctrl_state = 0
        self.ctrl_params = dict(p.CTRL_DEFAULTS)
        self.ctrl_next = 0.0
        self.ctrl_fail = 0
        self.h = 0.0
        self.h_ref = 0.0
        self.base = 0.0
        self.trim = 0.0
        #: Hover "real" da planta simulada, em ‰ (difere do parâmetro de propósito).
        self.true_hover = 600.0
        self.ctrl_gains = dict(p.GAIN_DEFAULTS)

    # ---- link ---------------------------------------------------------------

    def connect(self) -> None:
        """DTR rising: new session (stops motors, sends HELLO)."""
        self._stop('nova sessao USB')
        self.connected = True
        self.decoder.reset()
        self.heartbeat_ms = None
        self._hello()
        self._text(0, 'bench-link v1 (SIMULADOR); SEM HELICES sem suporte fixo')

    def disconnect(self) -> None:
        self._stop('sessao USB encerrada')
        self.connected = False

    def receive(self, data: bytes) -> None:
        if not self.connected:
            return
        before = self.decoder.errors
        for packet in self.decoder.feed(data):
            self._handle(packet)
        if self.decoder.errors != before:
            self.link_errors += self.decoder.errors - before
            self._stop('quadro USB corrompido')
            self._text(2, 'quadro corrompido (CRC/COBS): saida parada')

    def take_output(self) -> bytes:
        data = bytes(self.out)
        self.out.clear()
        return data

    def _send(self, msg: dict) -> None:
        if not self.connected:
            return
        msg_id, payload = p.encode_message(msg)
        self.out += p.encode_frame(msg_id, self.seq, payload)
        self.seq = (self.seq + 1) & 0xFF

    def _text(self, level: int, text: str) -> None:
        self._send({'type': 'text', 't_ms': self.now_ms, 'level': level, 'text': text})

    def _hello(self) -> None:
        self._send(
            {
                'type': 'hello',
                'capabilities': 15,
                'uptime_ms': self.now_ms - self.boot_ms,
                'firmware_id': 'simulador',
                'minor': p.PROTO_MINOR,
                'variant': 'sim',
            }
        )

    def _ack(self, packet: p.Packet, result: int, value: int = 0) -> None:
        self._send(
            {
                'type': 'ack',
                'command': packet.msg_id,
                'seq': packet.seq,
                'result': result,
                'value': value,
            }
        )

    # ---- motor rules -------------------------------------------------------

    def _armed(self) -> bool:
        return self.owner != 0

    def _all_zero(self) -> bool:
        return all(t == 0 for t in self.target) and all(c == 0 for c in self.current)

    def _stop(self, why: str) -> None:
        if self._armed():
            self._text(1, f'SAIDA DESARMADA: {why}')
        self.owner = 0
        self.sweep = None
        self.target = [0, 0, 0, 0]
        self.current = [0.0, 0.0, 0.0, 0.0]
        self.ctrl_state = 0
        self.h = self.h_ref = self.base = self.trim = 0.0

    def _heartbeat_fresh(self) -> bool:
        return self.heartbeat_ms is not None and self.now_ms - self.heartbeat_ms < KEEPALIVE_MS

    def _handle(self, packet: p.Packet) -> None:
        cmd = p.decode_command(packet.msg_id, packet.payload)
        if cmd is None:
            self._stop('comando USB invalido')
            self._text(2, 'comando invalido: saida parada')
            known = packet.msg_id in p.COMMAND_FORMATS
            self._ack(packet, p.RESULT_INVALID if known else p.RESULT_UNKNOWN)
            return
        name = cmd['name']
        if name == 'heartbeat':
            self.heartbeat_ms = self.now_ms
            return
        ok, value = True, 0
        if name == 'hello':
            self._hello()
        elif name == 'arm':
            if not self._heartbeat_fresh():
                self._text(1, 'ARM recusado: sem HEARTBEAT recente')
                ok = False
            elif self._low_battery():
                self._text(1, 'ARM recusado: tensao abaixo do minimo ou invalida')
                ok = False
            elif self._armed() or not self._all_zero():
                ok = False
            else:
                self.owner = 1
        elif name == 'ctrl':
            ok = self._ctrl_action(cmd['action'])
        elif name == 'ctrl_gains':
            self.ctrl_gains = cmd['gains']
        elif name == 'ctrl_params':
            self.ctrl_params = {
                'height_m': cmd['height_m'],
                'hover_percent': cmd['hover_permille'] / 10.0,
                'cap_percent': cmd['cap_permille'] / 10.0,
                'climb_m_s': cmd['climb_m_s'],
                'land_m_s': cmd['land_m_s'],
                'spool_percent_per_s': cmd['spool_permille_s'] / 10.0,
                'trim_ki': cmd['trim_ki_permille'] / 10.0,
                'ki_xy': cmd['ki_xy'],
                'ki_yaw': cmd['ki_yaw'],
                'autotrim': cmd['autotrim'],
                'use_ekf': cmd['use_ekf'],
            }
        elif name == 'stop':
            self._stop('STOP pela IHM')
        elif name == 'motor_set':
            ok = self._armed()
            if ok:
                self.sweep = None
                for i in range(4):
                    if cmd['mask'] >> i & 1:
                        self.target[i] = min(cmd['values'][i], self.cap)
        elif name == 'cap':
            self.cap = cmd['value']
            self.target = [min(t, self.cap) for t in self.target]
            self.current = [min(c, self.cap) for c in self.current]
            value = self.cap
        elif name == 'rate':
            self.rate = cmd['value']
        elif name == 'direction':
            ok = not self._armed() and self._all_zero()
            if ok:
                m = cmd['mask']
                self.reversed = (self.reversed & ~m) | (cmd['reversed'] & m)
                self.direction_sent |= m
        elif name == 'beep':
            ok = not self._armed() and self._all_zero()
        elif name == 'sweep':
            ok = self._armed()
            if ok:
                self.sweep = {
                    'mask': cmd['mask'],
                    'max': min(cmd['max'], self.cap),
                    'step': cmd['step'],
                    'hold': cmd['hold_ms'],
                    'level': 0,
                    'down': False,
                    'next': self.now_ms,
                }
        elif name == 'stream':
            self.stream = cmd['enabled']
            self.bno_divider = cmd['bno_divider']
        elif name == 'power_cal':
            self.cal = cmd['cal']
            self.cal_flags = cmd['flags']
        elif name == 'esc_telem':
            self.esc_enabled = cmd['enabled']
            self.esc_period = cmd['period_ms']
        self._ack(packet, p.RESULT_OK if ok else p.RESULT_REJECTED, value)

    # ---- controle simulado -----------------------------------------------------

    def _ctrl_action(self, action: str) -> bool:
        if action == 'arm':
            if not self._heartbeat_fresh():
                self._text(1, 'controle recusado: sem HEARTBEAT recente')
                return False
            if self._low_battery():
                self._text(1, 'ARM recusado: tensao abaixo do minimo ou invalida')
                return False
            if self._armed() or not self._all_zero():
                return False
            self.owner = 2
            self.ctrl_state = 1
            self._text(0, 'controle armado: medindo o chao, motores em zero')
            return True
        if action == 'takeoff' and self.ctrl_state == 1 and self.owner == 2:
            self.ctrl_state = 2
            self.base = 50.0
            self._text(0, 'decolagem: subindo a aceleracao')
            return True
        if action == 'land' and self.ctrl_state in (2, 3, 4) and self.owner == 2:
            self.ctrl_state = 5
            self._text(0, 'pouso')
            return True
        return False

    def _ctrl_tick(self) -> None:
        """1 ms: sequência do controle (altura de primeira ordem, não é o LQR)."""
        prm = self.ctrl_params
        cap = prm['cap_percent'] * 10
        hover = prm['hover_percent'] * 10
        s = self.ctrl_state
        if s == 2:
            self.base += prm['spool_percent_per_s'] * 10 / 1000.0
            if self.base >= self.true_hover:
                self.ctrl_state = 3
                self.trim = self.base - hover
                self.h_ref = self.h
                self._text(0, 'descolou')
            elif self.base >= cap - 10:
                self._stop('nao descolou: motores no limite, cortados')
                self.ctrl_fail = 4
                return
        elif s in (3, 4):
            goal = prm['height_m']
            step = prm['climb_m_s'] / 1000.0
            self.h_ref += max(-step, min(step, goal - self.h_ref))
            self.ctrl_state = 4 if abs(self.h_ref - goal) < 0.005 else 3
            self.h += (self.h_ref - self.h) * 0.002
            self.base = hover + self.trim
        elif s == 5:
            step = prm['land_m_s'] / 1000.0
            self.h_ref = max(0.0, self.h_ref - step)
            self.h += (self.h_ref - self.h) * 0.002
            if self.h < 0.03:
                self.base -= 0.3
                if self.base <= 50:
                    self._stop('pouso concluido')
                    self.ctrl_fail = 5
                    return
        level = max(0.0, min(self.base, cap)) if self.ctrl_state >= 2 else 0.0
        self.target = [round(level)] * 4
        self.current = [level] * 4

    def _ctrl_msg(self) -> None:
        z = -(self.h + self.rng.gauss(0, 0.002))
        flags = 0x0A | (0x01 if self.ctrl_state >= 3 else 0) | (0x04 if self.ctrl_state >= 2 else 0)
        self._send(
            {
                'type': 'ctrl',
                't_ms': self.now_ms,
                'state': self.ctrl_state,
                'flags': flags,
                'fail': self.ctrl_fail,
                'y': [z, 0.01, -0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'pos': [0.0, 0.0, -self.h],
                'vel': [0.0, 0.0, 0.0],
                'h_ref': self.h_ref,
                'trim': self.trim,
                'base': self.base,
                'ref_xy': [0.0, 0.0],
                'ref_psi': 0.0,
                'vbat_factor': 1.0,
            }
        )

    # ---- simulation ---------------------------------------------------------

    def _vbat_pin(self) -> float:
        load = sum(self.current) / 4000.0
        return 1.52 - 0.08 * load + self.rng.gauss(0, 0.002)

    def _current_pin(self) -> float:
        load = sum(self.current) / 4000.0
        return 0.5 + 0.6 * load**1.5 + self.rng.gauss(0, 0.003)

    def _low_battery(self) -> bool:
        if self.cal[3] <= 0:
            return False
        if not self.cal_flags & 1:
            return True
        return self._vbat_pin() * self.cal[0] < self.cal[3]

    def _rotation_hz(self, i: int) -> float:
        erpm = self.current[i] / 1000.0 * ERPM_AT_FULL
        return erpm / (POLES / 2) / 60.0

    def advance(self, now_ms: int) -> None:
        """Run the simulated firmware up to ``now_ms`` (1 ms motor loop)."""
        while self.now_ms < now_ms:
            self.now_ms += 1
            self._motor_tick()
            if not self.connected:
                continue
            t = self.now_ms
            if t >= self.next['imu']:
                self._reschedule('imu', 2.5)
                self._imu()
            if t >= self.next['range']:
                self._reschedule('range', 10)
                self._range()
            if t >= self.next['motor']:
                self._reschedule('motor', 10)
                self._motor_msg()
            if t >= self.next['status']:
                self._reschedule('status', 100)
                self._status()
            if t >= self.next['power']:
                self._reschedule('power', 100)
                self._power()
            if t >= self.ctrl_next:
                self.ctrl_next = t + 50
                self._ctrl_msg()
            if t >= self.next['sys']:
                self._reschedule('sys', 1000)
                self._sys()
            if self.esc_enabled and t >= self.esc_next_ms:
                self.esc_next_ms = t + self.esc_period
                self._esc(self.esc_motor)
                self.esc_motor = (self.esc_motor + 1) % 4

    def _reschedule(self, key: str, period: float) -> None:
        """Keep the average rate exact with 1 ms steps (2.5 ms -> 2/3 ms alternating)."""
        nxt = self.next[key] + period
        self.next[key] = nxt if nxt > self.now_ms - period else self.now_ms + period

    def _motor_tick(self) -> None:
        if self._armed() and (
            self.heartbeat_ms is None or self.now_ms - self.heartbeat_ms >= KEEPALIVE_MS
        ):
            self._stop('bancada sem keepalive por 300 ms')
        if not self._armed():
            return
        if self.owner == 2:
            self._ctrl_tick()
            return
        s = self.sweep
        if s and self.now_ms >= s['next']:
            if not s['down']:
                if s['level'] >= s['max']:
                    s['down'] = True
                else:
                    s['level'] = min(s['max'], s['level'] + s['step'])
            if s['down'] and s['level'] == 0:
                self.sweep = None
                self._text(0, 'sweep terminou')
            else:
                if s['down']:
                    s['level'] = max(0, s['level'] - s['step'])
                for i in range(4):
                    if s['mask'] >> i & 1:
                        self.target[i] = s['level']
                s['next'] = self.now_ms + s['hold'] + s['step'] * 1000 // self.rate
        step = self.rate / 1000.0
        for i in range(4):
            goal = float(self.target[i])
            c = self.current[i]
            self.current[i] = min(goal, c + step) if c < goal else max(goal, c - step)

    def _imu(self) -> None:
        self.imu_index += 1
        dt = 0.0025
        noise = 0.002
        vib = [0.0, 0.0, 0.0]
        t = self.now_ms / 1000.0
        for i in range(4):
            amp = self.current[i] / 1000.0 * 0.4
            f = self._rotation_hz(i)
            if amp > 0:
                phase = 2 * math.pi * f * t + i
                vib[0] += amp * math.sin(phase)
                vib[1] += amp * math.cos(phase)
                vib[2] += 0.3 * amp * math.sin(2 * phase)
        gyro = [self.rng.gauss(0, noise) + vib[k] * 0.1 for k in range(3)]
        lacc = [self.rng.gauss(0, 0.03) + vib[k] for k in range(3)]
        self.yaw += gyro[2] * dt
        roll = 0.01 + self.rng.gauss(0, 0.0005)
        pitch = -0.02 + self.rng.gauss(0, 0.0005)
        cy, sy = math.cos(self.yaw / 2), math.sin(self.yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        q = [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
        self.counts = [c + 1 for c in self.counts]
        if not self.stream or self.imu_index % self.bno_divider:
            return
        self._send(
            {
                'type': 'imu',
                't_ms': self.now_ms,
                'q': q,
                'gyro': gyro,
                'lin_accel': lacc,
                'accuracy': [3, 3, 3],
                'flags': 1,
                'count': list(self.counts),
                'gaps': [0, 0, 0],
            }
        )

    def _range(self) -> None:
        self.mtf_frames += 1
        if not self.stream:
            return
        lift = sum(self.current) / 4000.0 * 0.01
        if self.owner == 2:
            distance = 50 + self.h * 1000 + self.rng.gauss(0, 2.0)
        else:
            distance = 300 + lift * 1000 + self.rng.gauss(0, 2.0)
        self._send(
            {
                'type': 'range',
                't_ms': self.now_ms,
                'sensor_ms': self.now_ms + 12345,
                'distance_mm': max(0, round(distance)),
                'flow_x': round(self.rng.gauss(0, 2)),
                'flow_y': round(self.rng.gauss(0, 2)),
                'quality': 180 + self.rng.randint(-10, 10),
                'distance_status': 1,
                'flow_status': 1,
                'strength': 120,
                'precision': 3,
                'flags': 3,
                'frames': self.mtf_frames,
                'crc_errors': 0,
                'gaps': 0,
                'uart_errors': 0,
                'dropped_bytes': 0,
            }
        )

    def _motor_msg(self) -> None:
        if not self.stream:
            return
        permille = [round(c) for c in self.current]
        dshot = [0 if v == 0 else 48 + round(v * 1999 / 1000) for v in permille]
        self._send(
            {
                'type': 'motor',
                't_ms': self.now_ms,
                'flags': 1 if self._armed() else 0,
                'permille': permille,
                'dshot': dshot,
                'telemetry_mask': 0,
            }
        )

    def _status(self) -> None:
        flags = 0
        flags |= p.STATUS_ARMED if self._armed() else 0
        flags |= p.STATUS_SWEEPING if self.sweep else 0
        flags |= p.STATUS_HEARTBEAT_OK if self._heartbeat_fresh() else 0
        flags |= p.STATUS_ESC_TELEM if self.esc_enabled else 0
        flags |= p.STATUS_STREAM if self.stream else 0
        flags |= p.STATUS_POWER_LOW if self._low_battery() else 0
        age = 0xFFFFFFFF if self.heartbeat_ms is None else self.now_ms - self.heartbeat_ms
        self._send(
            {
                'type': 'status',
                't_ms': self.now_ms,
                'flags': flags,
                'owner': self.owner,
                'cap_permille': self.cap,
                'rate_permille_s': self.rate,
                'target_permille': list(self.target),
                'reversed': self.reversed,
                'direction_sent': self.direction_sent,
                'direction_known': 0,
                'commands_pending': 0,
                'tx_failures': 0,
                'heartbeat_age_ms': min(age, 0xFFFFFFFF),
                'bno_divider': self.bno_divider,
            }
        )

    def _power(self) -> None:
        vpin, cpin = self._vbat_pin(), self._current_pin()
        flags = 0x03
        vbat = math.nan
        current = math.nan
        if self.cal_flags & 1 and self.cal[0] > 0:
            flags |= 0x04
            vbat = vpin * self.cal[0]
        if self.cal_flags & 2 and self.cal[1] > 0:
            flags |= 0x08
            current = (cpin - self.cal[2]) * self.cal[1]
        self._send(
            {
                'type': 'power',
                't_ms': self.now_ms,
                'vbat_pin_v': vpin,
                'current_pin_v': cpin,
                'vbat_v': vbat,
                'current_a': current,
                'flags': flags,
            }
        )

    def _sys(self) -> None:
        self._send(
            {
                'type': 'sys',
                't_ms': self.now_ms,
                'telemetry_dropped': 0,
                'heap_free': 21000,
                'link_packets': self.decoder.packets,
                'link_errors': self.link_errors,
                'link_overflows': 0,
                'esc_crc_errors': 0,
                'esc_length_errors': 0,
                'esc_unsolicited': 0,
                'esc_uart_errors': 0,
                'esc_timeouts': [0, 0, 0, 0],
                'bno_errors': 0,
                'bno_resets': 0,
                'bno_fault': 0,
            }
        )

    def _esc(self, i: int) -> None:
        erpm = self.current[i] / 1000.0 * ERPM_AT_FULL
        self.esc_ok[i] += 1
        self._send(
            {
                'type': 'esc',
                't_ms': self.now_ms,
                'motor': i,
                'temperature_c': 28 + round(self.current[i] / 100),
                'voltage_cv': round(self._vbat_pin() * 1100),
                'current_ca': round(self.current[i] * 0.8),
                'consumption_mah': 0,
                'erpm_100': round(erpm / 100),
                'frames_ok': self.esc_ok[i],
                'timeouts': 0,
            }
        )
