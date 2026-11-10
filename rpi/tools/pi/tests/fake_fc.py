"""STM32 simulado para testar o enlace da Pi sem hardware.

Fala o mesmo dialeto por um pseudo-terminal e imita as respostas do firmware
(App/Src/dronelink): HEARTBEAT, DRONE_PROTOCOL_INFO, parâmetros, envio de
ganhos com CRC, armar, trechos e o DRONE_CTRL_DEBUG. Não simula a dinâmica:
cada trecho leva ``leg_time`` segundos e o drone "chega" no alvo (posição e
guinada em LOCAL_POSITION_NED e ATTITUDE). Como o firmware, só aceita girar e
andar em HOLD (ocupado durante a decolagem ou um trecho).

Com ``hover=True`` a decolagem e o pouso seguem um modelo de voo simples, para
os ensaios de hover (``dronectl hover-test``): partida dos motores, rampa de
subida em FLT_Z_RATE, altura seguindo a referência, XY parado em
``hover_xy`` (ou andando a ``drift``), φ/θ medidos com o erro de nível
``level_deg`` menos SNS_LVL_*, relatos ``dist`` a cada 2 s e pouso até o
chão. O relógio do STM32 falso anda ``time_scale`` vezes mais rápido que o
real, para um ensaio de 20 s caber em 2 s de teste.
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
#: Parâmetros do voo que o ensaio de hover lê e muda: [valor, mínimo, máximo].
HOVER_PARAMS = {
    'CTL_FF_TH': [-50.0, -300.0, 300.0], 'CTL_FF_PHI': [-40.0, -300.0, 300.0],
    'CTL_FF_PSI': [50.0, -300.0, 300.0], 'SNS_LVL_PHI': [0.0, -5.0, 5.0],
    'SNS_LVL_TH': [0.0, -5.0, 5.0], 'CTL_XY': [1.0, 0.0, 1.0], 'CTL_LQG_AJ': [1.0, 0.0, 1.0],
    'CTL_DIST': [1.0, 0.0, 1.0], 'CTL_DIST_MAX': [300.0, 0.0, 600.0],
    'MOT_MAX': [0.95, 0.1, 1.0], 'MOT_W_AT_0': [0.0, 0.0, 2000.0],
    'MOT_W_AT_100': [1755.0, 100.0, 8000.0], 'FLT_Z_RATE': [0.05, 0.0, 1.0],
    'FLT_TILT_MAX': [0.35, 0.2, 1.2]}
#: z medido pelo MTF com o drone no chão (SIM_PLANT_GROUND_Z), em m (NED).
GROUND_Z = -0.08


class FakeFC:
    """Controladora simulada do outro lado de um pty."""

    def __init__(self, drop_first_row: bool = False, hover: bool = False,
                 time_scale: float = 1.0) -> None:
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
        #: Modelo de voo para o ensaio de hover (ver o cabeçalho).
        self.hover = hover
        self.time_scale = time_scale
        self.sim_t = 0.0
        self.z_ref = self.zf = GROUND_Z
        self.takeoff_target = None
        self.spinup_until = 0.0
        self.hover_xy = (0.0, 0.0)
        self.drift = (0.0, 0.0)
        self.level_deg = (0.0, 0.0)
        self.dist_report = (35, -12, 8, 20)
        self.last_dist = 0.0
        #: Descida do pouso, em m/s (0 = nunca chega ao chão).
        self.land_rate = 0.1
        #: Quando saiu do chão (relógio simulado), None no chão.
        self.lift_t = None
        #: Resposta final da decolagem em andamento (chamada ao chegar em HOLD).
        self._takeoff_ack = None
        #: Comandos de armar/desarmar recebidos (True = armar).
        self.arm_commands = []
        #: Para de mandar o DRONE_CTRL_DEBUG (telemetria cai).
        self.mute_debug = False
        if hover:
            self.params['CTL_W0'][0] = 1050.0
            self.params.update({k: list(v) for k, v in HOVER_PARAMS.items()})
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
            self.arm_commands.append(msg.param1 >= 0.5)
            if ok:
                self.state = 4 if msg.param1 >= 0.5 else 2
                if self.hover and msg.param1 < 0.5:
                    self.takeoff_target = None
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
        if self.hover and msg.action == a.DRONE_ACTION_TAKEOFF_RELATIVE and self.state == 4:
            # Partida (3 s), depois rampa até o alvo; HOLD quando chegar.
            self.state = 5
            self.takeoff_target = GROUND_Z + msg.z
            self.spinup_until = self.sim_t + 3.0
            self.z_ref = self.zf = GROUND_Z
            self._takeoff_ack = ack
            ack(a.MAV_RESULT_IN_PROGRESS, 0)
            return
        if self.hover and msg.action == a.DRONE_ACTION_LAND and self.state in (5, 6, 7):
            self.state = 8
            ack(a.MAV_RESULT_ACCEPTED, 100)
            return
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

    # ---- modelo de voo do ensaio de hover ------------------------------

    def _p(self, name: str) -> float:
        return self.params[name][0]

    def _say(self, text: str) -> None:
        self.mav.statustext_send(6, text.encode())

    def _hover_step(self, dt: float) -> None:
        """Avança o voo simulado ``dt`` s do relógio do STM32 falso."""
        self.sim_t += dt
        motors = self._p('FLT_DRY_RUN') < 0.5 or self._p('FLT_SIM') >= 0.5
        if self.state == 5 and self.takeoff_target is not None and self.sim_t >= self.spinup_until:
            self.z_ref = max(self.takeoff_target, self.z_ref - self._p('FLT_Z_RATE') * dt)
        elif self.state == 8:
            self.z_ref = min(GROUND_Z, self.z_ref + self.land_rate * dt)
        target = self.z_ref if motors else GROUND_Z
        self.zf += (target - self.zf) * min(1.0, dt / 0.3)
        lifted = self.zf < GROUND_Z - 0.03
        if self.state == 5 and self.takeoff_target is not None and \
                self.z_ref == self.takeoff_target and abs(self.zf - self.takeoff_target) < 0.005:
            self.state = 6
            self._takeoff_ack(self.mod.MAV_RESULT_ACCEPTED, 100)
        if self.state == 8 and self.zf > GROUND_Z - 0.003:
            self.state = 2
            self.takeoff_target = None
            self._say('voo: desarmado (pousou)')
            z, th, phi, psi = self.dist_report
            self._say(f'dist z={z // 4} th={th} phi={phi} psi={psi} rad/s')
        if lifted and self.state in (5, 6) and self.sim_t - self.last_dist >= 2.0:
            self.last_dist = self.sim_t
            self._say('dist z=%d th=%d phi=%d psi=%d' % self.dist_report)
        if lifted and self.lift_t is None:
            self.lift_t = self.sim_t
        elif not lifted:
            self.lift_t = None

    def _hover_debug(self, step: int) -> None:
        """DRONE_CTRL_DEBUG do voo simulado."""
        noise = (step % 7 - 3) * 1e-3
        lifted = self.zf < GROUND_Z - 0.03
        phi = math.radians(self.level_deg[0] + self._p('SNS_LVL_PHI')) + noise * 0.2
        theta = math.radians(self.level_deg[1] + self._p('SNS_LVL_TH')) - noise * 0.2
        x = y = vx = vy = 0.0
        if lifted:
            # Sem integrador de posição, o erro de nível vira deslocamento (~14 cm por 0,5°).
            k = 0.28 / math.radians(1.0)
            t_air = self.sim_t - (self.lift_t if self.lift_t is not None else self.sim_t)
            vx, vy = self.drift
            x = self.hover_xy[0] + k * theta + vx * t_air
            y = self.hover_xy[1] - k * phi + vy * t_air
        flying = self.state in (5, 6, 7, 8) and (self._p('FLT_DRY_RUN') < 0.5 or
                                                 self._p('FLT_SIM') >= 0.5)
        thr = [6200, 6000, 5300, 6100] if flying else [0] * 4
        ys = [self.zf + noise, phi, theta, 0.0, vx, vy, 0.0, 0.0, 0.0]
        xs = [x, y, self.zf, phi, theta, 0.0, vx, vy, 0.0, 0.0, 0.0, 0.0]
        self.mav.drone_ctrl_debug_send(int(self.sim_t * 1000) & 0xFFFFFFFF, step, ys, xs,
                                       [0.0] * 4, [0.0, 0.0, self.z_ref, 0.0], thr, self.state,
                                       15, 1)

    # ---- laço ----------------------------------------------------------

    def _loop(self) -> None:
        import select

        last_hb = last_info = last_dbg = last_pose = 0.0
        last_step = time.monotonic()
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
            if self.hover:
                self._hover_step((now - last_step) * self.time_scale)
                last_step = now
            rate = self.params['DBG_RATE_HZ'][0]
            if rate > 0 and now - last_dbg > 1.0 / rate and self.hover:
                last_dbg = now
                step += 1
                if not self.mute_debug:
                    self._hover_debug(step)
            elif rate > 0 and now - last_dbg > 1.0 / rate:
                last_dbg = now
                step += 1
                noise = (step % 7 - 3) * 1e-3
                y = [-0.5 + noise, 0.01, -0.02, 0.0, noise, -noise, 0.0, 0.0, 0.0]
                x = [0.0, 0.0, -0.5, 0.01, -0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.mav.drone_ctrl_debug_send(int(now * 1000) & 0xFFFFFFFF, step, y, x,
                                               [0.0] * 4, [0.0, 0.0, -0.5, 0.0],
                                               [4000] * 4, self.state, 15, 1)
