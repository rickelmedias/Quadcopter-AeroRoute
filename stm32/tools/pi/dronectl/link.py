"""Enlace MAVLink (DroneLink) entre a Raspberry Pi e o STM32.

Implementa o lado da Pi do protocolo de ``protocol/drone_controller.xml``:

- HEARTBEAT a 4 Hz e autorização do operador (DRONE_CONTROL_LEASE) a 5 Hz
  enquanto há uma sessão de voo;
- armar/desarmar (COMMAND_LONG) e trechos transacionais (DRONE_LEG_COMMAND,
  com missão, comando e índice de trecho em sequência, e repetição segura);
- parâmetros (PARAM_*) e envio atômico de ganhos (DRONE_GAIN_*);
- recepção da telemetria (DRONE_CTRL_DEBUG, ATTITUDE, LOCAL_POSITION_NED,
  STATUSTEXT).

Uma thread lê a serial e despacha as mensagens; outra envia os HEARTBEATs e a
autorização. Os métodos públicos podem ser chamados da thread principal.
"""
from __future__ import annotations

import math
import random
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, Dict, List, Optional, Tuple

from . import dialect
from .gains import Gains

#: Identidades MAVLink (App/Inc/dronelink/dronelink_config.h).
SYSTEM_ID = 1
FC_COMPONENT_ID = 1
PI_COMPONENT_ID = 191

#: Nomes dos estados de voo (DRONE_FLIGHT_STATE).
FLIGHT_STATES = ['BOOT', 'SELF_TEST', 'STANDBY', 'PREARM', 'ARMED', 'TAKEOFF', 'HOLD',
                 'LEG', 'LAND', 'FAILSAFE', 'TERMINATION']
#: Motivos de recusa (DRONE_FAILURE_REASON).
FAILURE_REASONS = ['nenhum', 'estado errado', 'sensor com problema', 'fora dos limites',
                   'bateria baixa', 'ocupado', 'boot diferente', 'sequência errada',
                   'número inválido', 'IDs repetidos', 'não implementado',
                   'sem autorização do operador', 'enlace perdido', 'tempo esgotado',
                   'destino errado', 'cancelado']
#: Situação do envio de ganhos (DRONE_GAIN_STATUS).
GAIN_STATUS = ['ok', 'sessão errada', 'índice errado', 'incompleto', 'CRC não confere',
               'valor não finito']


class LinkError(RuntimeError):
    """Falha de comunicação ou pedido recusado pelo STM32."""


def state_name(state: int) -> str:
    """Nome de um DRONE_FLIGHT_STATE."""
    return FLIGHT_STATES[state] if 0 <= state < len(FLIGHT_STATES) else str(state)


def reason_text(reason: int) -> str:
    """Texto de um DRONE_FAILURE_REASON."""
    return FAILURE_REASONS[reason] if 0 <= reason < len(FAILURE_REASONS) else str(reason)


@dataclass
class DebugSample:
    """Um passo do LQG recebido em DRONE_CTRL_DEBUG."""

    time_ms: int
    step: int
    y: List[float]
    x_hat: List[float]
    du: List[float]
    ref: List[float]
    throttle: List[float]
    state: int
    meas_flags: int
    ctrl_flags: int


class _Writer:
    """Arquivo para o MAVLink escrever, protegido entre threads."""

    def __init__(self, port) -> None:
        self._port = port
        self._lock = threading.Lock()

    def write(self, data: bytes) -> None:
        with self._lock:
            self._port.write(data)


class DroneLink:
    """Conexão com o STM32 pela UART (ou qualquer URL do pyserial)."""

    def __init__(self, port: str = '/dev/serial0', baud: int = 115200,
                 on_text: Optional[Callable[[str], None]] = None) -> None:
        import serial  # pyserial

        self.mod = dialect.load()
        self.on_text = on_text or (lambda text: print(f'[STM32] {text}'))
        self._serial = serial.serial_for_url(port, baudrate=baud, timeout=0.05)
        self.mav = self.mod.MAVLink(_Writer(self._serial), srcSystem=SYSTEM_ID,
                                    srcComponent=PI_COMPONENT_ID)
        self.mav.robust_parsing = True

        self._cond = threading.Condition()
        self._inbox: Deque[Tuple[int, object]] = deque(maxlen=400)
        self._seq = 0
        self._subscribers: List[Callable[[DebugSample], None]] = []
        self._nav_subscribers: List[Callable[[object], None]] = []

        self.boot_id = 0
        self.capabilities = 0
        self.flight_state = 0
        self.armed = False
        self.last_fc_heartbeat = 0.0
        self.params: Dict[str, Tuple[float, int, int]] = {}
        self.last_debug: Optional[DebugSample] = None
        self.attitude = None
        self.position = None
        #: Quando chegaram (time.monotonic) o último ATTITUDE e LOCAL_POSITION_NED.
        self.attitude_time = 0.0
        self.position_time = 0.0
        #: Bateria do SYS_STATUS: tensão (V) e corrente (A), None = sem leitura.
        self.battery_v: Optional[float] = None
        self.battery_a: Optional[float] = None

        self._lease_active = False
        self._lease_id = random.randint(1, 0xFFFFFFFF)
        self._mission_id = random.randint(1, 0xFFFFFFFF)
        self._last_leg_index = 0
        self._command_id = 0
        #: Posição na caixa de entrada quando cada trecho foi enviado.
        self._leg_marks: Dict[int, int] = {}

        self._running = True
        self._threads = [threading.Thread(target=self._rx_loop, daemon=True),
                         threading.Thread(target=self._tx_loop, daemon=True)]
        for t in self._threads:
            t.start()

    # ---- ciclo de vida -------------------------------------------------

    def close(self) -> None:
        """Para as threads e fecha a porta."""
        self._lease_active = False
        self._running = False
        for t in self._threads:
            t.join(timeout=1.0)
        self._serial.close()

    def __enter__(self) -> 'DroneLink':
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def wait_fc(self, timeout: float = 5.0) -> None:
        """Espera o HEARTBEAT e a identidade de boot do STM32."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while not (self.boot_id and self.last_fc_heartbeat):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise LinkError('o STM32 não respondeu (confira a porta, o baud e o cabo)')
                self._cond.wait(remaining)

    @property
    def link_ok(self) -> bool:
        """HEARTBEAT do STM32 recebido no último segundo."""
        return time.monotonic() - self.last_fc_heartbeat < 1.5

    def subscribe(self, callback: Callable[[DebugSample], None]) -> None:
        """Chama @p callback (na thread de recepção) a cada DRONE_CTRL_DEBUG."""
        self._subscribers.append(callback)

    def subscribe_nav(self, callback: Callable[[object], None]) -> None:
        """Chama @p callback (na thread de recepção) a cada DRONE_NAV_DEBUG (NavSample)."""
        self._nav_subscribers.append(callback)

    # ---- threads -------------------------------------------------------

    def _rx_loop(self) -> None:
        while self._running:
            try:
                data = self._serial.read(self._serial.in_waiting or 1)
            except Exception:  # porta fechada durante o encerramento
                if self._running:
                    time.sleep(0.1)
                continue
            if not data:
                continue
            try:
                messages = self.mav.parse_buffer(data) or []
            except Exception:
                continue
            for msg in messages:
                if msg.get_srcSystem() == SYSTEM_ID and msg.get_srcComponent() == FC_COMPONENT_ID:
                    self._handle(msg)

    def _tx_loop(self) -> None:
        last_heartbeat = last_lease = 0.0
        while self._running:
            now = time.monotonic()
            try:
                if now - last_heartbeat >= 0.25:
                    last_heartbeat = now
                    self.mav.heartbeat_send(self.mod.MAV_TYPE_ONBOARD_CONTROLLER,
                                            self.mod.MAV_AUTOPILOT_INVALID, 0, 0,
                                            self.mod.MAV_STATE_ACTIVE)
                if self._lease_active and self.boot_id and now - last_lease >= 0.2:
                    last_lease = now
                    self.mav.drone_control_lease_send(
                        self.boot_id, self._mission_id, self._lease_id, 1000,
                        self.mod.DRONE_LEASE_FLAG_ACTIVE, SYSTEM_ID, FC_COMPONENT_ID)
            except Exception:
                pass
            time.sleep(0.02)

    def _handle(self, msg) -> None:
        kind = msg.get_type()
        notify = True
        if kind == 'HEARTBEAT':
            self.last_fc_heartbeat = time.monotonic()
            self.flight_state = msg.custom_mode
            self.armed = bool(msg.base_mode & self.mod.MAV_MODE_FLAG_SAFETY_ARMED)
        elif kind == 'DRONE_PROTOCOL_INFO':
            if self.boot_id and msg.boot_id != self.boot_id:
                self.on_text(f'STM32 reiniciou (boot {self.boot_id} -> {msg.boot_id})')
                self._last_leg_index = 0
                self._mission_id = random.randint(1, 0xFFFFFFFF)
            self.boot_id = msg.boot_id
            self.capabilities = msg.capabilities
            self.flight_state = msg.flight_state
        elif kind == 'STATUSTEXT':
            text = msg.text.decode(errors='replace') if isinstance(msg.text, bytes) else msg.text
            self.on_text(text.rstrip('\x00'))
        elif kind == 'PARAM_VALUE':
            name = msg.param_id.decode() if isinstance(msg.param_id, bytes) else msg.param_id
            self.params[name.rstrip('\x00')] = (msg.param_value, msg.param_index, msg.param_count)
        elif kind == 'DRONE_CTRL_DEBUG':
            sample = DebugSample(msg.time_boot_ms, msg.step, list(msg.y), list(msg.x_hat),
                                 list(msg.du), list(msg.ref),
                                 [t / 10000.0 for t in msg.throttle], msg.flight_state,
                                 msg.meas_flags, msg.ctrl_flags)
            self.last_debug = sample
            for callback in list(self._subscribers):
                try:
                    callback(sample)
                except Exception as exc:  # um assinante com erro não derruba o enlace
                    self.on_text(f'erro no assinante da telemetria: {exc}')
            notify = False
        elif kind == 'DRONE_NAV_DEBUG':
            from .nav_report import NavSample
            sample = NavSample.from_msg(msg)
            for callback in list(self._nav_subscribers):
                try:
                    callback(sample)
                except Exception as exc:  # um assinante com erro não derruba o enlace
                    self.on_text(f'erro no assinante do EKF: {exc}')
            notify = False
        elif kind == 'ATTITUDE':
            self.attitude = msg
            self.attitude_time = time.monotonic()
            notify = False
        elif kind == 'LOCAL_POSITION_NED':
            self.position = msg
            self.position_time = time.monotonic()
            notify = False
        elif kind == 'SYS_STATUS':
            # UINT16_MAX e -1 = leitura desligada no STM32 (power.h).
            self.battery_v = None if msg.voltage_battery == 0xFFFF else msg.voltage_battery / 1000.0
            self.battery_a = None if msg.current_battery < 0 else msg.current_battery / 100.0
            notify = False
        elif kind == 'DRONE_LEG_ACK':
            # O ACK traz o estado de voo: no fim de um trecho ele já é HOLD,
            # antes do próximo HEARTBEAT (2 Hz).
            self.flight_state = msg.flight_state
        elif kind not in ('DRONE_GAIN_ACK', 'COMMAND_ACK'):
            return
        if notify:
            with self._cond:
                self._seq += 1
                self._inbox.append((self._seq, msg))
                self._cond.notify_all()

    def _mark(self) -> int:
        with self._cond:
            return self._seq

    def _wait(self, match: Callable[[object], bool], since: int, timeout: float):
        """Espera uma mensagem recebida depois de @p since que satisfaça @p match."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                for seq, msg in self._inbox:
                    if seq > since and match(msg):
                        return msg
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cond.wait(remaining)

    # ---- parâmetros ----------------------------------------------------

    def param_list(self, timeout: float = 5.0) -> Dict[str, float]:
        """Lê todos os parâmetros (PARAM_REQUEST_LIST, completando os que faltarem)."""
        self.params.clear()
        self.mav.param_request_list_send(SYSTEM_ID, FC_COMPONENT_ID)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            time.sleep(0.1)
            if self.params:
                count = next(iter(self.params.values()))[2]
                have = {index for _, index, _ in self.params.values()}
                if len(have) >= count:
                    break
                if time.monotonic() > deadline - timeout / 2:
                    for index in set(range(count)) - have:
                        self.mav.param_request_read_send(SYSTEM_ID, FC_COMPONENT_ID, b'', index)
        if not self.params:
            raise LinkError('o STM32 não respondeu à lista de parâmetros')
        return {name: value for name, (value, _, _) in sorted(self.params.items(),
                                                             key=lambda kv: kv[1][1])}

    def param_get(self, name: str, timeout: float = 1.0) -> float:
        """Lê um parâmetro pelo nome."""
        name = name.upper()
        for _ in range(3):
            self.params.pop(name, None)
            self.mav.param_request_read_send(SYSTEM_ID, FC_COMPONENT_ID, name.encode(), -1)
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                if name in self.params:
                    return self.params[name][0]
                time.sleep(0.02)
        raise LinkError(f'parâmetro {name} não respondeu (nome existe?)')

    def param_set(self, name: str, value: float, timeout: float = 1.0) -> float:
        """Altera um parâmetro e devolve o valor que ficou em uso.

        Se o STM32 recusar (limites, drone armado), o valor devolvido é o
        antigo e o motivo chega por STATUSTEXT.
        """
        name = name.upper()
        for _ in range(3):
            self.params.pop(name, None)
            self.mav.param_set_send(SYSTEM_ID, FC_COMPONENT_ID, name.encode(), float(value),
                                    self.mod.MAV_PARAM_TYPE_REAL32)
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                if name in self.params:
                    return self.params[name][0]
                time.sleep(0.02)
        raise LinkError(f'parâmetro {name} não respondeu (nome existe?)')

    # ---- ganhos --------------------------------------------------------

    def _gain_request(self, session: int, action: int, crc: int, timeout: float = 1.5):
        for _ in range(3):
            since = self._mark()
            self.mav.drone_gain_commit_send(session, crc, action, SYSTEM_ID, FC_COMPONENT_ID)
            ack = self._wait(lambda m: m.get_type() == 'DRONE_GAIN_ACK' and
                             m.session_id == session and m.action == action, since, timeout)
            if ack is not None:
                return ack
        raise LinkError('o STM32 não confirmou a operação de ganhos')

    def upload_gains(self, gains: Gains) -> int:
        """Envia Kd, Kfd, L e M e troca os ganhos de uma vez (commit com CRC).

        Returns:
            O CRC dos ganhos em uso depois do envio.
        """
        crc = gains.crc32()
        for attempt in range(3):
            session = random.randint(1, 0xFFFFFFFF)
            since = self._mark()
            for matrix_id, row, values in gains.rows():
                padded = values + [0.0] * (12 - len(values))
                self.mav.drone_gain_row_send(session, padded, matrix_id, row, len(values),
                                             SYSTEM_ID, FC_COMPONENT_ID)
                time.sleep(0.003)
            rejected = self._wait(lambda m: m.get_type() == 'DRONE_GAIN_ACK' and
                                  m.session_id == session and m.action == 255, since, 0.05)
            if rejected is not None:
                raise LinkError(f'linha recusada: {GAIN_STATUS[rejected.status]}')
            ack = self._gain_request(session, self.mod.DRONE_GAIN_ACTION_COMMIT, crc)
            if ack.status == self.mod.DRONE_GAIN_STATUS_OK and ack.active_crc32 == crc:
                return ack.active_crc32
            if ack.status != self.mod.DRONE_GAIN_STATUS_INCOMPLETE:
                raise LinkError(f'ganhos recusados: {GAIN_STATUS[ack.status]}')
            # Linha perdida no caminho: manda tudo de novo numa sessão nova.
        raise LinkError('ganhos incompletos depois de 3 tentativas')

    def restore_gains(self) -> int:
        """Volta aos ganhos do Simulink; devolve o CRC em uso."""
        return self._gain_request(0, self.mod.DRONE_GAIN_ACTION_RESTORE_DEFAULT, 0).active_crc32

    def query_gains(self) -> Tuple[int, bool]:
        """CRC dos ganhos em uso e se eles são diferentes dos do Simulink."""
        ack = self._gain_request(0, self.mod.DRONE_GAIN_ACTION_QUERY, 0)
        return ack.active_crc32, bool(ack.custom)

    # ---- voo -----------------------------------------------------------

    def start_session(self) -> None:
        """Começa a autorizar o voo (lease) e espera ela chegar ao STM32."""
        self.wait_fc()
        self._lease_active = True
        time.sleep(0.4)

    def end_session(self) -> None:
        """Para de autorizar o voo (o STM32 pede HOLD e depois pouso se estiver no ar)."""
        self._lease_active = False

    def arm(self, arm: bool = True, timeout: float = 1.5) -> None:
        """Arma ou desarma (COMMAND_LONG ARM_DISARM)."""
        cmd = self.mod.MAV_CMD_COMPONENT_ARM_DISARM
        for _ in range(3):
            since = self._mark()
            self.mav.command_long_send(SYSTEM_ID, FC_COMPONENT_ID, cmd, 0,
                                       1.0 if arm else 0.0, 0, 0, 0, 0, 0, 0)
            ack = self._wait(lambda m: m.get_type() == 'COMMAND_ACK' and m.command == cmd,
                             since, timeout)
            if ack is None:
                continue
            if ack.result != self.mod.MAV_RESULT_ACCEPTED:
                raise LinkError(f'{"armar" if arm else "desarmar"} recusado: '
                                f'{reason_text(ack.result_param2)}')
            return
        raise LinkError('o STM32 não respondeu ao armar/desarmar')

    def send_leg(self, action: int, frame: Optional[int] = None, x: float = 0.0,
                 y: float = 0.0, z: float = 0.0, yaw: float = 0.0, speed: float = 0.0,
                 position_tolerance: float = 0.0, yaw_tolerance: float = 0.0,
                 timeout_ms: int = 30000) -> int:
        """Envia um trecho e espera a primeira resposta.

        Returns:
            O command_id, para acompanhar o fim com :meth:`wait_leg`.
        """
        frame = self.mod.MAV_FRAME_LOCAL_FRD if frame is None else frame
        self._command_id += 1
        command_id = self._command_id
        mission_id = self._mission_id
        leg_index = self._last_leg_index + 1
        for _ in range(3):
            since = self._mark()
            self.mav.drone_leg_command_send(
                self.boot_id, mission_id, command_id, timeout_ms, x, y, z, yaw, speed,
                0.0, position_tolerance, yaw_tolerance, leg_index, 0, action, frame,
                SYSTEM_ID, FC_COMPONENT_ID)
            ack = self._wait(lambda m: m.get_type() == 'DRONE_LEG_ACK' and
                             m.mission_id == mission_id and m.command_id == command_id,
                             since, 1.0)
            if ack is None:
                continue  # mesmo pedido de novo: o STM32 repete a resposta
            accepted = ack.result in (self.mod.MAV_RESULT_ACCEPTED,
                                      self.mod.MAV_RESULT_IN_PROGRESS)
            if not accepted:
                raise LinkError(f'trecho recusado: {reason_text(ack.reason)}')
            self._last_leg_index = leg_index
            self._leg_marks[command_id] = since
            return command_id
        raise LinkError('o STM32 não respondeu ao trecho')

    def wait_leg(self, command_id: int, timeout: float) -> bool:
        """Espera o fim de um trecho; True se ele foi concluído.

        Só vale a resposta desta missão e recebida depois do envio do trecho:
        uma resposta antiga com o mesmo número de comando (de outra missão ou
        de outro programa no mesmo enlace) não pode encerrar este trecho.
        """
        mission_id = self._mission_id
        since = self._leg_marks.pop(command_id, 0)
        ack = self._wait(lambda m: m.get_type() == 'DRONE_LEG_ACK' and
                         m.mission_id == mission_id and m.command_id == command_id and
                         m.result != self.mod.MAV_RESULT_IN_PROGRESS, since, timeout)
        if ack is None:
            raise LinkError('o trecho não terminou no tempo esperado')
        if ack.result != self.mod.MAV_RESULT_ACCEPTED:
            self.on_text(f'trecho terminou sem concluir: {reason_text(ack.reason)}')
            return False
        return True

    def takeoff(self, height: float, timeout_s: float = 20.0) -> int:
        """Decola até @p height m acima do chão (o fim chega por :meth:`wait_leg`)."""
        return self.send_leg(self.mod.DRONE_ACTION_TAKEOFF_RELATIVE, z=-abs(height),
                             timeout_ms=int(timeout_s * 1000))

    def hold(self) -> None:
        """Para e mantém a posição."""
        self.send_leg(self.mod.DRONE_ACTION_HOLD)

    def land(self) -> None:
        """Pousa (o STM32 desarma ao tocar o chão)."""
        self.send_leg(self.mod.DRONE_ACTION_LAND)

    def goto(self, x: float, y: float, z: float, yaw_deg: float = 0.0,
             speed: float = 0.0) -> int:
        """Vai ao ponto local (m, z negativo acima do chão; guinada absoluta em graus)."""
        return self.send_leg(self.mod.DRONE_ACTION_GOTO_LOCAL, self.mod.MAV_FRAME_LOCAL_FRD,
                             x, y, z, math.radians(yaw_deg), speed)

    def move(self, forward: float, right: float, down: float = 0.0, speed: float = 0.0,
             timeout_s: float = 30.0) -> int:
        """Translada no referencial do corpo (m)."""
        return self.send_leg(self.mod.DRONE_ACTION_TRANSLATE_RELATIVE,
                             self.mod.MAV_FRAME_BODY_FRD, forward, right, down, 0.0, speed,
                             timeout_ms=int(timeout_s * 1000))

    def rotate(self, yaw_deg: float, timeout_s: float = 30.0) -> int:
        """Gira em relação à guinada atual (graus, positivo = horário visto de cima)."""
        return self.send_leg(self.mod.DRONE_ACTION_ROTATE_RELATIVE,
                             self.mod.MAV_FRAME_BODY_FRD, yaw=math.radians(yaw_deg),
                             timeout_ms=int(timeout_s * 1000))
