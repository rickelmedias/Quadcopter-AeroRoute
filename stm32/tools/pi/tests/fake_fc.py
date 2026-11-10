"""STM32 simulado para testar o enlace da Pi sem hardware.

Fala o mesmo dialeto por um pseudo-terminal e imita as respostas do firmware
(App/Src/dronelink): HEARTBEAT, DRONE_PROTOCOL_INFO, parâmetros, envio de
ganhos com CRC, armar, trechos e o DRONE_CTRL_DEBUG. Não simula a dinâmica:
cada trecho leva ``leg_time`` segundos e o drone "chega" no alvo (posição e
guinada em LOCAL_POSITION_NED e ATTITUDE). Como o firmware, só aceita girar e
andar em HOLD (ocupado durante a decolagem ou um trecho).
"""
from __future__ import annotations

import math
import os
import struct
import threading
import time
import zlib

from dronectl import dialect

#: Formas das matrizes na ordem do envio (as mesmas de ctrl_gains.c).
SHAPES = [(4, 12), (4, 4), (12, 9), (12, 9)]
FIRST_ROW = [0, 4, 8, 20]


class FakeFC:
    """Controladora simulada do outro lado de um pty."""

    def __init__(self, drop_first_row: bool = False) -> None:
        self.mod = dialect.load()
        self.master, slave = os.openpty()
        self.port = os.ttyname(slave)
        self._slave = slave
        self.mav = self.mod.MAVLink(self, srcSystem=1, srcComponent=1)
        self.mav.robust_parsing = True
        self.boot_id = 4242
        self.state = 2  # STANDBY
        self.params = {'CTL_W0': [702.0, 100.0, 3000.0], 'FLT_DRY_RUN': [1.0, 0.0, 1.0],
                       'DBG_RATE_HZ': [0.0, 0.0, 50.0], 'FLT_SIM': [0.0, 0.0, 1.0]}
        self.active_crc = 0
        self.custom = 0
        self.staged = {}
        self.session = 0
        self.drop_first_row = drop_first_row
        self.received_rows = 0
        self.lease_seen = False
        #: Pose do drone simulado: x, y, z (NED, m) e guinada (rad).
        self.x = self.y = self.z = self.yaw = 0.0
        #: Duração de cada trecho, em s.
        self.leg_time = 0.2
        #: Trechos recebidos: (ação, x, y, z, guinada).
        self.legs = []
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def write(self, data: bytes) -> None:
        with self._lock:
            os.write(self.master, data)

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        os.close(self.master)
        os.close(self._slave)

    # ---- respostas -----------------------------------------------------

    def _param_value(self, name: str) -> None:
        names = list(self.params)
        self.mav.param_value_send(name.encode(), self.params[name][0],
                                  self.mod.MAV_PARAM_TYPE_REAL32, len(names), names.index(name))

    def _gain_ack(self, session, action, status, staged_crc=0, rows=0) -> None:
        self.mav.drone_gain_ack_send(session, staged_crc, self.active_crc, rows, action, status,
                                     self.custom, 1, 191)

    def _handle(self, msg) -> None:
        kind = msg.get_type()
        if kind == 'PARAM_REQUEST_LIST':
            for name in self.params:
                self._param_value(name)
        elif kind == 'PARAM_REQUEST_READ':
            name = msg.param_id.rstrip('\x00') if isinstance(msg.param_id, str) else \
                msg.param_id.decode().rstrip('\x00')
            if msg.param_index >= 0:
                name = list(self.params)[msg.param_index]
            if name in self.params:
                self._param_value(name)
        elif kind == 'PARAM_SET':
            name = msg.param_id.rstrip('\x00') if isinstance(msg.param_id, str) else \
                msg.param_id.decode().rstrip('\x00')
            if name not in self.params:
                self.mav.statustext_send(4, f'param {name}: desconhecido'.encode())
                return
            value, lo, hi = self.params[name]
            if lo <= msg.param_value <= hi:
                self.params[name][0] = msg.param_value
            else:
                self.mav.statustext_send(4, f'{name}: fora dos limites'.encode())
            self._param_value(name)
        elif kind == 'DRONE_GAIN_ROW':
            if self.drop_first_row:
                self.drop_first_row = False  # perde só a primeira linha da primeira sessão
                return
            if msg.session_id != self.session:
                self.session, self.staged = msg.session_id, {}
            rows, cols = SHAPES[msg.matrix]
            if msg.row >= rows or msg.count != cols:
                self._gain_ack(msg.session_id, 255, 2)
                return
            self.staged[FIRST_ROW[msg.matrix] + msg.row] = list(msg.values[:cols])
            self.received_rows += 1
        elif kind == 'DRONE_GAIN_COMMIT':
            if msg.action == self.mod.DRONE_GAIN_ACTION_COMMIT:
                rows = sum(1 << i for i in self.staged)
                values = [v for i in sorted(self.staged) for v in self.staged[i]]
                crc = zlib.crc32(b''.join(struct.pack('<f', v) for v in values)) & 0xFFFFFFFF
                if msg.session_id != self.session:
                    status = 1
                elif len(self.staged) != 32:
                    status = 3
                elif crc != msg.crc32:
                    status = 4
                else:
                    status, self.active_crc, self.custom = 0, crc, 1
                self._gain_ack(msg.session_id, msg.action, status, crc, rows)
            elif msg.action == self.mod.DRONE_GAIN_ACTION_RESTORE_DEFAULT:
                self.active_crc, self.custom = 0x21F15F2A, 0
                self._gain_ack(msg.session_id, msg.action, 0)
            else:
                self._gain_ack(msg.session_id, msg.action, 0)
        elif kind == 'DRONE_CONTROL_LEASE':
            self.lease_seen = msg.boot_id == self.boot_id
        elif kind == 'COMMAND_LONG' and msg.command == self.mod.MAV_CMD_COMPONENT_ARM_DISARM:
            ok = msg.param1 < 0.5 or self.lease_seen
            if ok:
                self.state = 4 if msg.param1 >= 0.5 else 2
            self.mav.command_ack_send(msg.command, 0 if ok else 2, 0, 0 if ok else 11, 1, 191)
        elif kind == 'DRONE_LEG_COMMAND':
            self._leg(msg)

    def _leg(self, msg) -> None:
        def ack(result, progress, reason=0):
            self.mav.drone_leg_ack_send(self.boot_id, msg.mission_id, msg.command_id,
                                        0xFFFFFFFF, float('nan'), msg.leg_index, reason, result,
                                        progress, self.state, 1, 191)

        def later(apply):
            def finish():
                time.sleep(self.leg_time)
                apply()
                self.state = 6
                ack(self.mod.MAV_RESULT_ACCEPTED, 100)
            threading.Thread(target=finish, daemon=True).start()

        a = self.mod
        self.legs.append((msg.action, msg.x, msg.y, msg.z, msg.yaw))
        if msg.action == a.DRONE_ACTION_LAND:
            self.state, self.z = 2, 0.0
            ack(a.MAV_RESULT_ACCEPTED, 100)
        elif msg.action == a.DRONE_ACTION_HOLD:
            ack(a.MAV_RESULT_ACCEPTED, 100)
        elif msg.action == a.DRONE_ACTION_TAKEOFF_RELATIVE and self.state == 4:
            self.state = 5
            ack(a.MAV_RESULT_IN_PROGRESS, 0)
            later(lambda: setattr(self, 'z', msg.z))
        elif msg.action in (a.DRONE_ACTION_ROTATE_RELATIVE, a.DRONE_ACTION_TRANSLATE_RELATIVE):
            if self.state in (5, 7):
                ack(a.MAV_RESULT_TEMPORARILY_REJECTED, 0, 5)  # ocupado
                return
            if self.state != 6:
                ack(a.MAV_RESULT_DENIED, 0, 1)  # estado errado
                return
            self.state = 7
            ack(a.MAV_RESULT_IN_PROGRESS, 0)
            if msg.action == a.DRONE_ACTION_ROTATE_RELATIVE:
                def girar():
                    self.yaw = (self.yaw + msg.yaw + math.pi) % (2 * math.pi) - math.pi
                later(girar)
            else:
                def andar():
                    c, s = math.cos(self.yaw), math.sin(self.yaw)
                    self.x += msg.x * c - msg.y * s
                    self.y += msg.x * s + msg.y * c
                    self.z += msg.z
                later(andar)
        else:
            ack(a.MAV_RESULT_DENIED, 0, 1)

    # ---- laço ----------------------------------------------------------

    def _loop(self) -> None:
        import select

        last_hb = last_info = last_dbg = last_pose = 0.0
        step = 0
        while self._running:
            ready, _, _ = select.select([self.master], [], [], 0.01)
            if ready:
                try:
                    data = os.read(self.master, 1024)
                except OSError:
                    return
                try:
                    messages = self.mav.parse_buffer(data) or []
                except Exception:  # lixo de uma sessão anterior do pty
                    messages = []
                for msg in messages:
                    self._handle(msg)
            now = time.monotonic()
            if now - last_hb > 0.5:
                last_hb = now
                armed = self.mod.MAV_MODE_FLAG_SAFETY_ARMED if self.state in (4, 5, 6, 7, 8) else 0
                self.mav.heartbeat_send(self.mod.MAV_TYPE_QUADROTOR,
                                        self.mod.MAV_AUTOPILOT_GENERIC, armed, self.state, 4)
            if now - last_info > 1.0:
                last_info = now
                self.mav.drone_protocol_info_send(self.boot_id, 0x7F, 120000, 1, 1, self.state,
                                                  0, 0)
            if now - last_pose > 0.2:
                last_pose = now
                t = int(now * 1000) & 0xFFFFFFFF
                self.mav.attitude_send(t, 0.0, 0.0, self.yaw, 0.0, 0.0, 0.0)
                self.mav.local_position_ned_send(t, self.x, self.y, self.z, 0.0, 0.0, 0.0)
            rate = self.params['DBG_RATE_HZ'][0]
            if rate > 0 and now - last_dbg > 1.0 / rate:
                last_dbg = now
                step += 1
                noise = (step % 7 - 3) * 1e-3
                y = [-0.5 + noise, 0.01, -0.02, 0.0, noise, -noise, 0.0, 0.0, 0.0]
                x = [0.0, 0.0, -0.5, 0.01, -0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.mav.drone_ctrl_debug_send(int(now * 1000) & 0xFFFFFFFF, step, y, x,
                                               [0.0] * 4, [0.0, 0.0, -0.5, 0.0],
                                               [4000] * 4, self.state, 15, 1)
