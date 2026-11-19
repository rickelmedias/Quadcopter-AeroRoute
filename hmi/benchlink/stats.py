"""Rolling signal storage, window statistics and spectrum (pure Python)."""

from __future__ import annotations

import bisect
import cmath
import math
from collections import deque


class SignalStore:
    """Per-signal ring of (t_s, value), with t in device seconds."""

    def __init__(self, window_s: float = 30.0) -> None:
        self.window_s = window_s
        self.data: dict[str, deque[tuple[float, float]]] = {}

    def clear(self) -> None:
        self.data.clear()

    def add(self, name: str, t: float, value: float) -> None:
        if value is None or not math.isfinite(value):
            return
        ring = self.data.get(name)
        if ring is None:
            ring = self.data[name] = deque()
        elif ring and t < ring[-1][0] - 1.0:
            ring.clear()  # device reboot or replay jump: timeline restarted
        ring.append((t, float(value)))
        limit = t - self.window_s
        while ring and ring[0][0] < limit:
            ring.popleft()

    def latest(self, name: str) -> tuple[float, float] | None:
        ring = self.data.get(name)
        return ring[-1] if ring else None

    def window(self, name: str, seconds: float) -> list[tuple[float, float]]:
        ring = self.data.get(name)
        if not ring:
            return []
        start = ring[-1][0] - seconds
        items = list(ring)
        index = bisect.bisect_left(items, (start, -math.inf))
        return items[index:]

    def stats(self, name: str, seconds: float = 1.0) -> tuple[float, float, int] | None:
        """Mean, population standard deviation and count over the last ``seconds``."""
        values = [v for _, v in self.window(name, seconds)]
        if not values:
            return None
        mean = sum(values) / len(values)
        var = sum((v - mean) ** 2 for v in values) / len(values)
        return mean, math.sqrt(var), len(values)

    def series(self, name: str, seconds: float, max_points: int = 1500) -> dict:
        items = self.window(name, seconds)
        if len(items) > max_points:
            step = len(items) / max_points
            items = [items[int(i * step)] for i in range(max_points)]
        return {'t': [round(t, 4) for t, _ in items], 'v': [v for _, v in items]}

    def rate(self, name: str, seconds: float = 2.0) -> float | None:
        items = self.window(name, seconds)
        if len(items) < 2 or items[-1][0] <= items[0][0]:
            return None
        return (len(items) - 1) / (items[-1][0] - items[0][0])


def _fft(x: list[complex]) -> list[complex]:
    n = len(x)
    if n == 1:
        return x
    even = _fft(x[0::2])
    odd = _fft(x[1::2])
    out = [0j] * n
    for k in range(n // 2):
        tw = cmath.exp(-2j * math.pi * k / n) * odd[k]
        out[k] = even[k] + tw
        out[k + n // 2] = even[k] - tw
    return out


def spectrum(samples: list[tuple[float, float]], max_points: int = 4096) -> dict | None:
    """Single-sided amplitude spectrum with a Hann window.

    The sample rate is the mean rate of the timestamps (device ticks have 1 ms
    resolution, so individual intervals jitter). Returns amplitude in the
    signal's unit and frequency in Hz; the mean is removed first.
    """
    if len(samples) < 16:
        return None
    n = 1
    while n * 2 <= min(len(samples), max_points):
        n *= 2
    samples = samples[-n:]
    duration = samples[-1][0] - samples[0][0]
    if duration <= 0:
        return None
    fs = (n - 1) / duration
    values = [v for _, v in samples]
    mean = sum(values) / n
    window = [0.5 - 0.5 * math.cos(2 * math.pi * i / (n - 1)) for i in range(n)]
    gain = sum(window) / n
    data = [complex((values[i] - mean) * window[i]) for i in range(n)]
    bins = _fft(data)
    half = n // 2
    amplitude = [abs(bins[k]) * 2 / (n * gain) for k in range(half)]
    amplitude[0] /= 2
    return {
        'fs_hz': fs,
        'n': n,
        'freq_hz': [k * fs / n for k in range(half)],
        'amplitude': amplitude,
    }
