"""Test helpers: a simulator source on a manual clock (deterministic)."""

from __future__ import annotations

import threading

from benchlink.simulator import SimulatedBench
from benchlink.sources import Source


class ManualSimSource(Source):
    """Simulated STM32 whose clock only advances in :meth:`step`.

    Replies (ACK, TEXT) are delivered synchronously from :meth:`send`.
    """

    live = True
    kind = 'simulator'

    def __init__(self) -> None:
        self.sim = SimulatedBench(seed=3, start_ms=10_000)
        self.lock = threading.RLock()
        self.on_bytes = None
        self.sent: list[bytes] = []

    def start(self, on_bytes) -> None:
        self.on_bytes = on_bytes
        with self.lock:
            self.sim.connect()
            self._deliver()

    def _deliver(self) -> None:
        data = self.sim.take_output()
        if data and self.on_bytes:
            self.on_bytes(data)

    def send(self, data: bytes) -> None:
        with self.lock:
            self.sent.append(data)
            self.sim.receive(data)
            self._deliver()

    def step(self, ms: int) -> None:
        with self.lock:
            self.sim.advance(self.sim.now_ms + ms)
            self._deliver()

    def close(self) -> None:
        with self.lock:
            self.sim.disconnect()
