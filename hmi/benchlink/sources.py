"""Byte sources for the hub: real serial port, simulated STM32, recorded replay."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from pathlib import Path

from .recorder import read_raw
from .simulator import SimulatedBench

OnBytes = Callable[[bytes], None]


class Source:
    """Interface. ``live`` sources accept commands; replay never does."""

    live = False
    kind = 'none'

    def start(self, on_bytes: OnBytes) -> None:
        raise NotImplementedError

    def send(self, data: bytes) -> None:
        raise RuntimeError('this source does not accept commands')

    def close(self) -> None:
        pass

    def describe(self) -> dict:
        return {'kind': self.kind, 'live': self.live}


class SerialSource(Source):
    """USB CDC of the STM32. Opening asserts DTR, which starts a firmware session."""

    live = True
    kind = 'serial'

    def __init__(self, port: str) -> None:
        import serial  # pyserial; imported here so tests/replay do not need it

        self.port = port
        self.serial = serial.Serial(port, 115200, timeout=0.02)
        self.serial.dtr = True
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self.error: str | None = None

    def start(self, on_bytes: OnBytes) -> None:
        def run() -> None:
            while not self._stop.is_set():
                try:
                    data = self.serial.read(4096)
                except Exception as exc:  # noqa: BLE001 - report any port failure
                    self.error = str(exc)
                    break
                if data:
                    on_bytes(data)

        self._thread = threading.Thread(target=run, name='serial-rx', daemon=True)
        self._thread.start()

    def send(self, data: bytes) -> None:
        with self._lock:
            self.serial.write(data)

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)
        try:
            self.serial.dtr = False
        finally:
            self.serial.close()

    def describe(self) -> dict:
        return {'kind': self.kind, 'live': True, 'port': self.port, 'error': self.error}


class SimulatorSource(Source):
    """Simulated STM32 in a thread, on the host monotonic clock."""

    live = True
    kind = 'simulator'

    def __init__(self, seed: int = 1) -> None:
        self.sim = SimulatedBench(seed=seed, start_ms=int(time.monotonic() * 1000))
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self, on_bytes: OnBytes) -> None:
        with self._lock:
            self.sim.connect()

        def run() -> None:
            while not self._stop.is_set():
                with self._lock:
                    self.sim.advance(int(time.monotonic() * 1000))
                    data = self.sim.take_output()
                if data:
                    on_bytes(data)
                time.sleep(0.005)

        self._thread = threading.Thread(target=run, name='simulator', daemon=True)
        self._thread.start()

    def send(self, data: bytes) -> None:
        with self._lock:
            self.sim.receive(data)

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)
        with self._lock:
            self.sim.disconnect()


class ReplaySource(Source):
    """Feeds a recorded ``raw.blr`` at its original pace (times ``speed``)."""

    live = False
    kind = 'replay'

    def __init__(self, session_dir: Path) -> None:
        self.session_dir = session_dir
        self.chunks = read_raw(session_dir / 'raw.blr')
        self.duration = self.chunks[-1][0] if self.chunks else 0.0
        self.speed = 1.0
        self.playing = True
        self.position = 0.0
        self.index = 0
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self.finished = False

    def start(self, on_bytes: OnBytes) -> None:
        def run() -> None:
            last = time.monotonic()
            while not self._stop.is_set():
                now = time.monotonic()
                with self._lock:
                    if self.playing:
                        self.position += (now - last) * self.speed
                    batch = []
                    while (
                        self.index < len(self.chunks)
                        and self.chunks[self.index][0] <= self.position
                    ):
                        batch.append(self.chunks[self.index][1])
                        self.index += 1
                    self.finished = self.index >= len(self.chunks)
                last = now
                if batch:
                    on_bytes(b''.join(batch))
                time.sleep(0.01)

        self._thread = threading.Thread(target=run, name='replay', daemon=True)
        self._thread.start()

    def control(self, playing: bool | None = None, speed: float | None = None) -> None:
        with self._lock:
            if playing is not None:
                self.playing = playing
            if speed is not None:
                self.speed = max(0.1, min(20.0, float(speed)))

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)

    def describe(self) -> dict:
        with self._lock:
            return {
                'kind': self.kind,
                'live': False,
                'session': self.session_dir.name,
                'position_s': round(self.position, 2),
                'duration_s': round(self.duration, 2),
                'playing': self.playing,
                'speed': self.speed,
                'finished': self.finished,
            }


class NullSource(Source):
    kind = 'none'

    def start(self, on_bytes: OnBytes) -> None:
        pass
