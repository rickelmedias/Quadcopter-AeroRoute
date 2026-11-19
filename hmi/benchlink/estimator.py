"""Read-only host-side estimate for bench validation (not a flight estimator).

Height: MTF-01 distance projected with BNO roll/pitch, ``h = d·cos(φ)·cos(θ)``,
then a first-order low-pass filter (time constant ``height_tau_s``). Vertical
speed is the filtered derivative of that height (positive up).

Horizontal speed in the MTF axes: ``v = (flow − k·ω)·h / 100`` with flow in
cm/s at 1 m, ω the BNO gyro in rad/s and ``k`` a per-axis rotation
compensation in (cm/s@1m)/(rad/s). The sign/gain of ``k`` depends on how the
sensors are mounted and must be measured on the bench (rotate without
translating and fit flow against gyro). Defaults are 0 = no compensation.
The integrated displacement drifts; reset it before each movement.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass


@dataclass
class EstimatorConfig:
    height_tau_s: float = 0.05
    vz_tau_s: float = 0.2
    flow_gyro_kx: float = 0.0  # pairs flow_x with gyro_y
    flow_gyro_ky: float = 0.0  # pairs flow_y with gyro_x
    min_quality: int = 30

    def to_dict(self) -> dict:
        return asdict(self)


class HostEstimator:
    def __init__(self, config: EstimatorConfig | None = None) -> None:
        self.config = config or EstimatorConfig()
        self.reset()

    def reset(self) -> None:
        self.roll = 0.0
        self.pitch = 0.0
        self.gyro = (0.0, 0.0, 0.0)
        self.imu_seen = False
        self.height: float | None = None
        self.vz = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.px = 0.0
        self.py = 0.0
        self.last_t: float | None = None

    def reset_position(self) -> None:
        self.px = 0.0
        self.py = 0.0

    def on_imu(self, roll_deg: float, pitch_deg: float, gyro: list[float]) -> None:
        self.roll = math.radians(roll_deg)
        self.pitch = math.radians(pitch_deg)
        self.gyro = (gyro[0], gyro[1], gyro[2])
        self.imu_seen = True

    def on_range(
        self, t_s: float, distance_m: float, range_ok: bool, flow: tuple[int, int, int, bool]
    ) -> dict[str, float]:
        """Update with one MTF frame; returns the estimate signals that are valid."""
        out: dict[str, float] = {}
        dt = 0.0 if self.last_t is None else t_s - self.last_t
        if dt < 0 or dt > 0.5:
            dt = 0.0  # gap or replay jump: do not integrate across it
        self.last_t = t_s
        if not range_ok or not self.imu_seen:
            return out
        h_raw = distance_m * math.cos(self.roll) * math.cos(self.pitch)
        if self.height is None or dt == 0.0:
            previous = h_raw if self.height is None else self.height
            self.height = h_raw
        else:
            previous = self.height
            alpha = dt / (self.config.height_tau_s + dt)
            self.height += alpha * (h_raw - self.height)
        if dt > 0:
            beta = dt / (self.config.vz_tau_s + dt)
            self.vz += beta * ((self.height - previous) / dt - self.vz)
        out['est.height'] = self.height
        out['est.vz'] = self.vz
        flow_x, flow_y, quality, flow_ok = flow
        if flow_ok and quality >= self.config.min_quality:
            fx = flow_x - self.config.flow_gyro_kx * self.gyro[1]
            fy = flow_y - self.config.flow_gyro_ky * self.gyro[0]
            self.vx = fx * self.height / 100.0
            self.vy = fy * self.height / 100.0
            self.px += self.vx * dt
            self.py += self.vy * dt
            out.update({'est.vx': self.vx, 'est.vy': self.vy, 'est.px': self.px, 'est.py': self.py})
        return out
