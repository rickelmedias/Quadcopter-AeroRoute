"""Testes da parte de protocolo de tools/motores_gui.py (sem tela, sem placa).

Uma porta serial virtual (pty) faz o papel do STM32: devolve linhas MOT e
texto, e recebe os comandos.

    python3 tests/test_motores_gui.py
"""
import os
import pty
import sys
import time
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import serial  # noqa: E402

import motores_gui as g  # noqa: E402


def test_interpretar_mot():
    e = g.interpretar_mot("MOT;1234;1;0;100;250;0;0;204;550;0\r\n")
    assert e is not None and e.armado
    assert e.permille == [0, 100, 250, 0] and e.dshot == [0, 204, 550, 0]
    assert g.interpretar_mot("# console: SEM HELICES") is None
    assert g.interpretar_mot("MOT;1;1;x;0;0;0;0;0;0;0") is None
    assert g.interpretar_mot("MOT;1;1;0;0") is None


def test_limitar():
    assert g.limitar(0, 300) == 0
    assert g.limitar(47, 300) == 0  # comandos especiais do DShot nunca saem como aceleração
    assert g.limitar(48, 300) == 48
    assert g.limitar(2000, 300) == 300
    assert g.limitar(2000, 20) == 48  # limite abaixo do mínimo ainda respeita 48


def test_proximo_comando():
    enviado = [0, 0, 0, 0]
    assert g.proximo_comando([0, 0, 0, 0], enviado) is None
    assert g.proximo_comando([100, 100, 100, 100], enviado) == "d 0 100"
    assert enviado == [100] * 4
    assert g.proximo_comando([100, 200, 100, 100], enviado) == "d 2 200"
    assert g.proximo_comando([100, 200, 100, 100], enviado) is None
    assert g.proximo_comando([0, 200, 0, 300], enviado) == "d 1 0"
    assert g.proximo_comando([0, 200, 0, 300], enviado) == "d 3 0"
    assert g.proximo_comando([0, 200, 0, 300], enviado) == "d 4 300"


def test_enlace_com_porta_virtual():
    mestre, escravo = pty.openpty()
    tty.setraw(mestre)
    porta = serial.Serial(os.ttyname(escravo), 115200, timeout=0.05)
    enlace = g.EnlaceMotores(porta)
    try:
        os.write(mestre, b"# console: SEM HELICES\r\nMOT;10;1;0;0;0;0;0;0;0;0\r\nlixo\r\n")
        eventos = []
        fim = time.time() + 1.0
        while time.time() < fim and len(eventos) < 2:
            try:
                eventos.append(enlace.eventos.get(timeout=0.1))
            except Exception:
                pass
        tipos = [t for t, _ in eventos]
        assert tipos == ["texto", "mot"], tipos
        assert eventos[1][1].armado

        assert enlace.enviar("d 0 100")
        time.sleep(0.1)
        assert os.read(mestre, 64) == b"d 0 100\r\n"

        enlace.fechar()  # manda "stop" antes de fechar
    finally:
        os.close(mestre)


if __name__ == "__main__":
    for nome, funcao in sorted(globals().items()):
        if nome.startswith("test_"):
            funcao()
            print("ok ", nome)
    print("test_motores_gui: ok")
