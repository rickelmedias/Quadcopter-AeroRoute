"""STM32 simulado para testar o lado da Pi sem hardware.

Fala o mesmo dialeto por um pseudo-terminal e imita as respostas do firmware
(stm32/App/Src/dronelink): HEARTBEAT, DRONE_PROTOCOL_INFO, armar só com a
autorização do operador, trechos transacionais (o mesmo pedido repetido recebe
a mesma resposta), ATTITUDE, LOCAL_POSITION_NED (z relativo ao chão),
SYS_STATUS e DRONE_CTRL_DEBUG. Não simula a dinâmica: cada trecho leva
``leg_time`` segundos e o drone "chega" no alvo. Como o firmware, só aceita girar
e andar em HOLD (ocupado durante a decolagem ou um trecho).
"""

from __future__ import annotations

import math
import os
import select
import threading
import time

from dronectl import dialect

STANDBY, ARMED, TAKEOFF, HOLD, LEG, LAND = 2, 4, 5, 6, 7, 8


class FakeFC:
    """Controladora simulada do outro lado de um pty."""

    def __init__(self) -> None:
        self.mod = dialect.load()
        self.master, slave = os.openpty()
        self.port = os.ttyname(slave)
        self._slave = slave
        self.mav = self.mod.MAVLink(self, srcSystem=1, srcComponent=1)
        self.mav.robust_parsing = True
        self.boot_id = 4242
        self.state = STANDBY
        self.lease_seen = False
        #: Pose do drone simulado: x, y, z (NED relativo ao chão, m) e guinada (rad).
        self.x = self.y = self.z = self.yaw = 0.0
        #: Duração de cada trecho, em s.
        self.leg_time = 0.2
        #: Trechos executados: (ação, x, y, z, guinada).
        self.legs = []
        #: Respostas por (missão, comando) para repetir o mesmo pedido.
        self._answers = {}
        #: Pedidos de armar (True) e desarmar (False) recebidos.
        self.arm_commands = []
        self.vbat = 11.8
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

    def _handle(self, msg) -> None:
        kind = msg.get_type()
        if kind == 'DRONE_CONTROL_LEASE':
            self.lease_seen = msg.boot_id == self.boot_id and bool(msg.flags & 1)
        elif kind == 'COMMAND_LONG' and msg.command == self.mod.MAV_CMD_COMPONENT_ARM_DISARM:
            arm = msg.param1 >= 0.5
            self.arm_commands.append(arm)
            ok = (not arm) or (self.lease_seen and self.state == STANDBY)
            if ok:
                self.state = ARMED if arm else STANDBY
                if not arm:
                    self.z = 0.0
            reason = 0 if ok else (11 if not self.lease_seen else 1)
            self.mav.command_ack_send(msg.command, 0 if ok else 2, 0, reason, 1, 191)
        elif kind == 'DRONE_LEG_COMMAND':
            key = (msg.mission_id, msg.command_id)
            if key in self._answers:
                self._answers[key]()  # repetido: mesma resposta, não executa de novo
                return
            self._leg(msg)

    def _leg(self, msg) -> None:
        a = self.mod
        key = (msg.mission_id, msg.command_id)

        def ack(result, progress, reason=0):
            def send():
                self.mav.drone_leg_ack_send(
                    self.boot_id,
                    msg.mission_id,
                    msg.command_id,
                    0xFFFFFFFF,
                    float('nan'),
                    msg.leg_index,
                    reason,
                    result,
                    progress,
                    self.state,
                    1,
                    191,
                )

            self._answers[key] = send
            send()

        def later(apply):
            def finish():
                time.sleep(self.leg_time)
                apply()
                self.state = HOLD
                ack(a.MAV_RESULT_ACCEPTED, 100)

            threading.Thread(target=finish, daemon=True).start()

        if msg.boot_id != self.boot_id:
            ack(a.MAV_RESULT_DENIED, 0, 6)
            return
        self.legs.append((msg.action, msg.x, msg.y, msg.z, msg.yaw))
        if msg.action == a.DRONE_ACTION_LAND:
            if self.state in (TAKEOFF, HOLD, LEG):
                self.state = LAND
                ack(a.MAV_RESULT_ACCEPTED, 100)

                def touch():
                    time.sleep(self.leg_time)
                    self.z, self.state = 0.0, STANDBY

                threading.Thread(target=touch, daemon=True).start()
            else:
                ack(a.MAV_RESULT_DENIED, 0, 1)
        elif msg.action == a.DRONE_ACTION_HOLD:
            ack(a.MAV_RESULT_ACCEPTED, 100)
        elif msg.action == a.DRONE_ACTION_TAKEOFF_RELATIVE:
            if self.state != ARMED:
                ack(a.MAV_RESULT_DENIED, 0, 1)
                return
            self.state = TAKEOFF
            ack(a.MAV_RESULT_IN_PROGRESS, 0)
            later(lambda: setattr(self, 'z', msg.z))
        elif msg.action in (
            a.DRONE_ACTION_ROTATE_RELATIVE,
            a.DRONE_ACTION_TRANSLATE_RELATIVE,
            a.DRONE_ACTION_GOTO_LOCAL,
        ):
            if self.state in (TAKEOFF, LEG):
                ack(a.MAV_RESULT_TEMPORARILY_REJECTED, 0, 5)  # ocupado
                return
            if self.state != HOLD:
                ack(a.MAV_RESULT_DENIED, 0, 1)  # estado errado
                return
            if (
                msg.action == a.DRONE_ACTION_TRANSLATE_RELATIVE
                and not -0.80 <= self.z - msg.z <= -0.10
            ):
                ack(a.MAV_RESULT_DENIED, 0, 3)  # altura fora de 10..80 cm
                return
            self.state = LEG
            ack(a.MAV_RESULT_IN_PROGRESS, 0)
            if msg.action == a.DRONE_ACTION_ROTATE_RELATIVE:

                def turn():
                    self.yaw = (self.yaw + msg.yaw + math.pi) % (2 * math.pi) - math.pi

                later(turn)
            elif msg.action == a.DRONE_ACTION_TRANSLATE_RELATIVE:

                def move():
                    c, s = math.cos(self.yaw), math.sin(self.yaw)
                    self.x += msg.x * c - msg.y * s
                    self.y += msg.x * s + msg.y * c
                    self.z += msg.z

                later(move)
            else:

                def go():
                    self.x, self.y, self.yaw = msg.x, msg.y, msg.yaw
                    if msg.z != 0.0:
                        self.z = msg.z

                later(go)
        else:
            ack(a.MAV_RESULT_DENIED, 0, 10)

    # ---- laço ----------------------------------------------------------

    def _loop(self) -> None:
        last_hb = last_info = last_pose = last_dbg = last_status = 0.0
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
                    if msg.get_srcComponent() == 191:
                        self._handle(msg)
            now = time.monotonic()
            t = int(now * 1000) & 0xFFFFFFFF
            if now - last_hb > 0.5:
                last_hb = now
                armed = self.mod.MAV_MODE_FLAG_SAFETY_ARMED if self.state >= ARMED else 0
                self.mav.heartbeat_send(
                    self.mod.MAV_TYPE_QUADROTOR,
                    self.mod.MAV_AUTOPILOT_GENERIC,
                    armed,
                    self.state,
                    4,
                )
            if now - last_info > 1.0:
                last_info = now
                self.mav.drone_protocol_info_send(
                    self.boot_id, 0x3F, 120000, 1, 2, self.state, 0, 0
                )
            if now - last_status > 0.5:
                last_status = now
                # Só os 13 campos base: o pymavlink 2.4.49 da Pi não tem as extensões.
                self.mav.sys_status_send(
                    0, 0, 0, 0, int(self.vbat * 1000), -1, -1, 0, 0, 0, 0, 0, 0
                )
            if now - last_pose > 0.2:
                last_pose = now
                self.mav.attitude_send(t, 0.0, 0.0, self.yaw, 0.0, 0.0, 0.0)
                self.mav.local_position_ned_send(t, self.x, self.y, self.z, 0.0, 0.0, 0.0)
            if now - last_dbg > 0.1:
                last_dbg = now
                step += 1
                noise = (step % 7 - 3) * 1e-3
                xs = [
                    self.x + noise,
                    self.y - noise,
                    self.z + noise,
                    0.0,
                    0.0,
                    self.yaw,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ]
                ys = [self.z, 0.0, 0.0, self.yaw, 0.0, 0.0, 0.0, 0.0, 0.0]
                flying = self.state in (TAKEOFF, HOLD, LEG, LAND)
                self.mav.drone_ctrl_debug_send(
                    t,
                    step,
                    ys,
                    xs,
                    [0.0] * 4,
                    [self.x, self.y, self.z, self.yaw],
                    [5600] * 4 if flying else [0] * 4,
                    self.state,
                    15,
                    0x01 | (0x02 if flying else 0),
                )
