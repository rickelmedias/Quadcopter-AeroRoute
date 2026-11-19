"""Gera e carrega o módulo Python do dialeto MAVLink do projeto.

O módulo é gerado com o pymavlink a partir de ``protocol/drone_controller.xml``
(o mesmo arquivo que gera os headers C do STM32 em ``stm32/tools/gen_mavlink.py``)
e guardado em ``~/.cache/dronectl``. Ele só é refeito quando o XML muda, então a
Pi nunca fala uma versão do protocolo diferente da que está no repositório.
"""

from __future__ import annotations

import contextlib
import hashlib
import importlib.util
import io
import shutil
import sys
from pathlib import Path
from types import ModuleType

#: Raiz do repositório (rpi/dronectl/dialect.py -> ../..).
REPO_ROOT = Path(__file__).resolve().parents[2]
#: XML do dialeto.
DIALECT_XML = REPO_ROOT / 'protocol' / 'drone_controller.xml'
#: Onde o módulo gerado fica guardado.
CACHE_DIR = Path.home() / '.cache' / 'dronectl'

_loaded: ModuleType | None = None


def load() -> ModuleType:
    """Devolve o módulo do dialeto, gerando-o se o XML mudou."""
    global _loaded
    if _loaded is not None:
        return _loaded
    import pymavlink
    from pymavlink.generator import mavgen

    digest = hashlib.sha256(DIALECT_XML.read_bytes()).hexdigest()[:16]
    target = CACHE_DIR / f'drone_controller_{digest}.py'
    if not target.exists():
        work = CACHE_DIR / f'xml_{digest}'
        work.mkdir(parents=True, exist_ok=True)
        definitions = Path(pymavlink.__file__).parent / 'dialects' / 'v20'
        for name in ('common.xml', 'standard.xml', 'minimal.xml'):
            shutil.copy(definitions / name, work / name)
        shutil.copy(DIALECT_XML, work / DIALECT_XML.name)
        opts = mavgen.Opts(
            output=str(target), wire_protocol='2.0', language='Python3', validate=False
        )
        log = io.StringIO()
        with contextlib.redirect_stdout(log):
            ok = mavgen.mavgen(opts, [str(work / DIALECT_XML.name)])
        if not ok:
            raise RuntimeError('falha ao gerar o dialeto MAVLink:\n' + log.getvalue())
    spec = importlib.util.spec_from_file_location('drone_controller', target)
    module = importlib.util.module_from_spec(spec)
    sys.modules.setdefault('drone_controller', module)
    spec.loader.exec_module(module)
    _loaded = module
    return module
