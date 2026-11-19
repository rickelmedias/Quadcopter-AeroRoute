"""Persistent frozen samples ("congelar").

A freeze stores, instantly, the current value of one signal, a group or all
signals, plus mean and standard deviation over the last second already
buffered (window and sample count recorded). Rows persist in a JSON file until
deleted, so they survive HMI restarts and can be compared with live values or
with each other.
"""

from __future__ import annotations

import json
import threading
import uuid
from pathlib import Path

from .recorder import utc_now


class FreezeStore:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._lock = threading.Lock()
        self.items: list[dict] = []
        if path.is_file():
            try:
                self.items = json.loads(path.read_text())
            except json.JSONDecodeError:
                backup = path.with_suffix('.corrupt.json')
                path.replace(backup)
                self.items = []

    def _save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix('.tmp')
        tmp.write_text(json.dumps(self.items, indent=2, ensure_ascii=False))
        tmp.replace(self.path)

    def add(self, label: str, values: dict, context: dict) -> dict:
        item = {
            'id': uuid.uuid4().hex[:10],
            'label': label,
            'note': '',
            'created_utc': utc_now(),
            **context,
            'values': values,
        }
        with self._lock:
            self.items.append(item)
            self._save()
        return item

    def update(self, item_id: str, **fields) -> dict | None:
        with self._lock:
            for item in self.items:
                if item['id'] == item_id:
                    for key in ('label', 'note'):
                        if key in fields:
                            item[key] = str(fields[key])[:200]
                    self._save()
                    return item
        return None

    def delete(self, item_id: str) -> bool:
        with self._lock:
            before = len(self.items)
            self.items = [i for i in self.items if i['id'] != item_id]
            if len(self.items) != before:
                self._save()
                return True
        return False

    def clear(self) -> None:
        with self._lock:
            self.items = []
            self._save()

    def all(self) -> list[dict]:
        with self._lock:
            return json.loads(json.dumps(self.items))
