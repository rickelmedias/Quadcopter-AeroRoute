"""Gera os headers C do MAVLink a partir de ../protocol/drone_controller.xml.

Usa o gerador oficial (pymavlink.tools.mavgen) e os XMLs padrão que vêm com a
mesma versão do pymavlink (common.xml, standard.xml, minimal.xml). O
resultado substitui Middlewares/Third_Party/mavlink por completo, para que
todos os headers saiam da mesma versão do gerador.

Uso (no PC, no ambiente conda ``aeroroute``, que já tem o pymavlink):

    cd stm32 && conda run -n aeroroute python tools/gen_mavlink.py

O lado da Raspberry Pi (rpi/dronectl) gera o próprio módulo Python a partir do
mesmo XML sempre que ele muda.
"""
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

#: Raiz do firmware (stm32/).
ROOT = Path(__file__).resolve().parents[1]
#: Dialeto do projeto, compartilhado com a Raspberry Pi.
DIALECT = ROOT.parent / 'protocol/drone_controller.xml'
#: Destino dos headers C.
OUTPUT = ROOT / 'Middlewares/Third_Party/mavlink'


def main():
    """Copia os XMLs para uma pasta temporária, roda o mavgen e troca a saída."""
    try:
        import pymavlink
    except ImportError:
        sys.exit('instale o gerador: pip install pymavlink')
    definitions = Path(pymavlink.__file__).parent / 'dialects/v20'
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        for name in ('common.xml', 'standard.xml', 'minimal.xml'):
            shutil.copy(definitions / name, tmp / name)
        shutil.copy(DIALECT, tmp / DIALECT.name)
        out = tmp / 'out'
        subprocess.run([sys.executable, '-m', 'pymavlink.tools.mavgen', '--lang=C',
                        '--wire-protocol=2.0', '-o', str(out), str(tmp / DIALECT.name)],
                       check=True)
        if OUTPUT.exists():
            shutil.rmtree(OUTPUT)
        shutil.copytree(out, OUTPUT)
    print(f'headers gerados em {OUTPUT.relative_to(ROOT)} (pymavlink {pymavlink.__version__})')


if __name__ == '__main__':
    main()
