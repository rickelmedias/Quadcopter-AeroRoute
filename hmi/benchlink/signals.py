"""Named bench signals with units, extracted from decoded bench-link messages.

Frames: BNO085 values are in the sensor axes reported by SH-2; Euler angles are
ZYX (yaw, pitch, roll) from the BNO quaternion, in degrees for display. MTF-01
distance is along the sensor axis; flow is the sensor's raw unit (cm/s at 1 m),
not vehicle velocity. Motor commands are percent of full DShot range. ESC values
come only from KISS telemetry; a command is never shown as measured RPM.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Signal:
    name: str
    group: str
    label: str
    unit: str
    digits: int = 3


def _build() -> list[Signal]:
    s: list[Signal] = []
    add = s.append
    g = 'Orientação (BNO085)'
    add(Signal('imu.roll', g, 'Rolagem φ', 'deg', 2))
    add(Signal('imu.pitch', g, 'Arfagem θ', 'deg', 2))
    add(Signal('imu.yaw', g, 'Guinada ψ', 'deg', 2))
    for axis in 'wxyz':
        add(Signal(f'imu.q{axis}', g, f'Quatérnio {axis}', '', 4))
    g = 'IMU (BNO085)'
    for axis, name in zip('xyz', 'pqr'):
        add(Signal(f'imu.gyro_{axis}', g, f'Giro {name} ({axis})', 'rad/s', 4))
    for axis in 'xyz':
        add(Signal(f'imu.lacc_{axis}', g, f'Acel. linear {axis} (sem g)', 'm/s²', 3))
    add(Signal('imu.lacc_norm', g, '|Acel. linear|', 'm/s²', 3))
    add(Signal('imu.acc_rot', g, 'Acurácia rotação', '0–3', 0))
    add(Signal('imu.acc_gyro', g, 'Acurácia giro', '0–3', 0))
    add(Signal('imu.acc_lin', g, 'Acurácia acel.', '0–3', 0))
    add(Signal('imu.rate', g, 'Taxa recebida', 'Hz', 0))
    add(Signal('imu.gaps', g, 'Relatórios perdidos (soma)', '', 0))
    g = 'MTF-01'
    add(Signal('mtf.distance', g, 'Distância (eixo do sensor)', 'm', 3))
    add(Signal('mtf.strength', g, 'Intensidade', '', 0))
    add(Signal('mtf.precision', g, 'Precisão', '', 0))
    add(Signal('mtf.flow_x', g, 'Fluxo x', 'cm/s@1m', 0))
    add(Signal('mtf.flow_y', g, 'Fluxo y', 'cm/s@1m', 0))
    add(Signal('mtf.quality', g, 'Qualidade do fluxo', '0–255', 0))
    add(Signal('mtf.rate', g, 'Taxa recebida', 'Hz', 0))
    add(Signal('mtf.crc_errors', g, 'Erros de CRC', '', 0))
    add(Signal('mtf.gaps', g, 'Frames perdidos', '', 0))
    g = 'Estimativa no PC'
    add(Signal('est.height', g, 'Altura (dist·cosφ·cosθ, filtrada)', 'm', 3))
    add(Signal('est.vz', g, 'Velocidade vertical', 'm/s', 3))
    add(Signal('est.vx', g, 'Velocidade x (eixos do MTF)', 'm/s', 3))
    add(Signal('est.vy', g, 'Velocidade y (eixos do MTF)', 'm/s', 3))
    add(Signal('est.px', g, 'Deslocamento x integrado', 'm', 3))
    add(Signal('est.py', g, 'Deslocamento y integrado', 'm', 3))
    g = 'Controle LQR (STM32)'
    add(Signal('ctrl.h_ref', g, 'Altura de referência', 'm', 3))
    add(Signal('ctrl.x_ref', g, 'Referência x (frente)', 'm', 3))
    add(Signal('ctrl.y_ref', g, 'Referência y (direita)', 'm', 3))
    add(Signal('ctrl.vbat_factor', g, 'Fator da bateria na saída', '', 3))
    add(Signal('ctrl.h_meas', g, 'Altura medida (acima do chão do armamento)', 'm', 3))
    add(Signal('ctrl.h_est', g, 'Altura estimada (Kalman)', 'm', 3))
    add(Signal('ctrl.vz_est', g, 'Velocidade vertical estimada (para cima)', 'm/s', 3))
    add(Signal('ctrl.x_est', g, 'Posição x estimada (frente)', 'm', 3))
    add(Signal('ctrl.y_est', g, 'Posição y estimada (direita)', 'm', 3))
    add(Signal('ctrl.u_est', g, 'Velocidade u estimada (frente)', 'm/s', 3))
    add(Signal('ctrl.v_est', g, 'Velocidade v estimada (direita)', 'm/s', 3))
    add(Signal('ctrl.phi', g, 'φ medido (direita para baixo +)', 'deg', 2))
    add(Signal('ctrl.theta', g, 'θ medido (nariz para cima +)', 'deg', 2))
    add(Signal('ctrl.psi', g, 'ψ medido (horário +)', 'deg', 2))
    add(Signal('ctrl.p', g, 'p medido', 'rad/s', 3))
    add(Signal('ctrl.q', g, 'q medido', 'rad/s', 3))
    add(Signal('ctrl.r', g, 'r medido', 'rad/s', 3))
    add(Signal('ctrl.u_meas', g, 'u medido (fluxo)', 'm/s', 3))
    add(Signal('ctrl.v_meas', g, 'v medido (fluxo)', 'm/s', 3))
    add(Signal('ctrl.trim', g, 'Ajuste lento do hover', '%', 2))
    add(Signal('ctrl.base', g, 'Aceleração base (hover + ajuste)', '%', 1))
    g = 'Motores'
    for i in range(1, 5):
        add(Signal(f'motor.m{i}.cmd', g, f'M{i} comando enviado', '%', 1))
        add(Signal(f'motor.m{i}.dshot', g, f'M{i} valor DShot', '', 0))
    g = 'ESC (KISS)'
    for i in range(1, 5):
        add(Signal(f'esc.m{i}.rpm', g, f'M{i} rotação medida', 'rpm', 0))
        add(Signal(f'esc.m{i}.hz', g, f'M{i} rotação medida', 'Hz', 1))
        add(Signal(f'esc.m{i}.voltage', g, f'M{i} tensão', 'V', 2))
        add(Signal(f'esc.m{i}.current', g, f'M{i} corrente', 'A', 2))
        add(Signal(f'esc.m{i}.temp', g, f'M{i} temperatura', '°C', 0))
        add(Signal(f'esc.m{i}.mah', g, f'M{i} consumo', 'mAh', 0))
    g = 'Energia'
    add(Signal('power.vbat', g, 'Bateria (PA1 calibrado)', 'V', 2))
    add(Signal('power.current', g, 'Corrente CURR (PB0 calibrado)', 'A', 2))
    add(Signal('power.vbat_pin', g, 'PA1 bruto', 'V', 3))
    add(Signal('power.current_pin', g, 'PB0 bruto', 'V', 3))
    g = 'Sistema'
    add(Signal('sys.heap', g, 'Heap livre', 'B', 0))
    add(Signal('sys.dropped', g, 'Telemetria descartada (STM32)', '', 0))
    add(Signal('sys.link_errors', g, 'Quadros USB inválidos (STM32)', '', 0))
    add(Signal('link.lost', g, 'Quadros perdidos (PC)', '', 0))
    add(Signal('link.errors', g, 'Quadros inválidos (PC)', '', 0))
    return s


SIGNALS: list[Signal] = _build()
BY_NAME = {s.name: s for s in SIGNALS}


def quaternion_to_euler_deg(q: list[float]) -> tuple[float, float, float]:
    """ZYX Euler (roll, pitch, yaw) in degrees from w, x, y, z."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def extract(msg: dict, motor_poles: int = 14) -> tuple[dict[str, float], bool]:
    """Return ({signal: value}, valid) for one decoded message."""
    t = msg['type']
    out: dict[str, float] = {}
    valid = True
    if t == 'imu':
        roll, pitch, yaw = quaternion_to_euler_deg(msg['q'])
        out.update({'imu.roll': roll, 'imu.pitch': pitch, 'imu.yaw': yaw})
        for axis, value in zip('wxyz', msg['q']):
            out[f'imu.q{axis}'] = value
        for axis, value in zip('xyz', msg['gyro']):
            out[f'imu.gyro_{axis}'] = value
        for axis, value in zip('xyz', msg['lin_accel']):
            out[f'imu.lacc_{axis}'] = value
        out['imu.lacc_norm'] = math.sqrt(sum(v * v for v in msg['lin_accel']))
        out['imu.acc_rot'], out['imu.acc_gyro'], out['imu.acc_lin'] = msg['accuracy']
        out['imu.gaps'] = sum(msg['gaps'])
        valid = bool(msg['flags'] & 1)
    elif t == 'range':
        out['mtf.distance'] = msg['distance_mm'] / 1000.0
        out['mtf.strength'] = msg['strength']
        out['mtf.precision'] = msg['precision']
        out['mtf.flow_x'] = msg['flow_x']
        out['mtf.flow_y'] = msg['flow_y']
        out['mtf.quality'] = msg['quality']
        out['mtf.crc_errors'] = msg['crc_errors']
        out['mtf.gaps'] = msg['gaps']
        valid = bool(msg['flags'] & 1)
    elif t == 'motor':
        for i in range(4):
            out[f'motor.m{i + 1}.cmd'] = msg['permille'][i] / 10.0
            out[f'motor.m{i + 1}.dshot'] = msg['dshot'][i]
    elif t == 'esc':
        i = msg['motor'] + 1
        erpm = msg['erpm_100'] * 100.0
        rpm = erpm / (motor_poles / 2.0)
        out[f'esc.m{i}.rpm'] = rpm
        out[f'esc.m{i}.hz'] = rpm / 60.0
        out[f'esc.m{i}.voltage'] = msg['voltage_cv'] / 100.0
        out[f'esc.m{i}.current'] = msg['current_ca'] / 100.0
        out[f'esc.m{i}.temp'] = msg['temperature_c']
        out[f'esc.m{i}.mah'] = msg['consumption_mah']
    elif t == 'power':
        flags = msg['flags']
        if flags & 0x01:
            out['power.vbat_pin'] = msg['vbat_pin_v']
        if flags & 0x02:
            out['power.current_pin'] = msg['current_pin_v']
        if flags & 0x04:
            out['power.vbat'] = msg['vbat_v']
        if flags & 0x08:
            out['power.current'] = msg['current_a']
    elif t == 'ctrl':
        y = msg['y']
        out['ctrl.h_ref'] = msg['h_ref']
        out['ctrl.h_meas'] = -y[0]
        out['ctrl.h_est'] = -msg['pos'][2]
        out['ctrl.vz_est'] = -msg['vel'][2]
        out['ctrl.x_est'], out['ctrl.y_est'] = msg['pos'][0], msg['pos'][1]
        out['ctrl.u_est'], out['ctrl.v_est'] = msg['vel'][0], msg['vel'][1]
        out['ctrl.phi'], out['ctrl.theta'] = math.degrees(y[1]), math.degrees(y[2])
        out['ctrl.psi'] = math.degrees(y[3])
        out['ctrl.u_meas'], out['ctrl.v_meas'] = y[4], y[5]
        out['ctrl.p'], out['ctrl.q'], out['ctrl.r'] = y[6], y[7], y[8]
        out['ctrl.trim'] = msg['trim'] / 10.0
        out['ctrl.vbat_factor'] = msg.get('vbat_factor', 1.0)
        ref_xy = msg.get('ref_xy', [0.0, 0.0])
        out['ctrl.x_ref'], out['ctrl.y_ref'] = ref_xy[0], ref_xy[1]
        out['ctrl.base'] = msg['base'] / 10.0
    elif t == 'sys':
        out['sys.heap'] = msg['heap_free']
        out['sys.dropped'] = msg['telemetry_dropped']
        out['sys.link_errors'] = msg['link_errors']
    return out, valid
