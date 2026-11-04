"""Testes da parte de protocolo de tools/controle_gui.py (sem tela, sem placa).

    python3 tests/test_controle_gui.py
"""
import os
import pty
import re
import sys
import time
import tty
from pathlib import Path

RAIZ = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ / "tools"))
import serial  # noqa: E402

import controle_gui as g  # noqa: E402

#: Linha EST de exemplo: estado SEGURANDO(6), BNO+MTF ok (7), y z=-0.165 φ=0.010 θ=-0.020 ψ=0.030.
EST = ("EST;1000;6;7;0;-165;10;-20;30;0;0;1;2;3;"
       "-1;-2;-165;10;-20;30;0;0;0;1;2;3;"
       "-65;25;-40;0;0;0;-165;0")


def test_interpretar_est():
    e = g.interpretar_est(EST + "\r\n")
    assert e is not None
    assert e.estado == 6 and e.nome_estado == "SEGURANDO" and e.meas == 7
    assert abs(e.altura_m - 0.165) < 1e-9
    assert abs(e.y[1] - 0.010) < 1e-9 and abs(e.y[2] + 0.020) < 1e-9 and abs(e.y[3] - 0.030) < 1e-9
    assert e.du == [-6.5, 2.5, -4.0, 0.0]  # décimos de rad/s
    assert abs(e.ref[2] + 0.165) < 1e-9
    assert g.interpretar_est("EST;1;2;3") is None
    assert g.interpretar_est("MOT;1;1;0;0;0;0;0;0;0;0") is None
    assert g.interpretar_est(EST.replace("-165;10", "x;10", 1)) is None


def test_interpretar_mot():
    m = g.interpretar_mot("MOT;10;1;0;100;250;0;0;204;550;0")
    assert m is not None and m.armado and m.permille == [0, 100, 250, 0] and m.dshot == [0, 204, 550, 0]
    assert g.interpretar_mot("MOT;1;1;0") is None


def test_du_por_motor_segue_o_mixer_do_firmware():
    """u1->M4, u2->M2, u3->M1, u4->M3, igual ao firmware (App/Src/control/mixer.c)."""
    codigo = (RAIZ / "App/Src/control/mixer.c").read_text()
    tabela = re.search(r"MOTOR_OF_INPUT\[MIXER_COUNT\]\s*=\s*\{([^}]*)\}", codigo).group(1)
    firmware = [int(x.strip().rstrip("U")) + 1 for x in tabela.split(",")]  # índice 0-based -> M1..M4
    assert firmware == [g.MOTOR_DA_ENTRADA[i] for i in (1, 2, 3, 4)]
    assert g.du_por_motor([1.0, 2.0, 3.0, 4.0]) == [3.0, 2.0, 4.0, 1.0]


def test_comando_ligar():
    assert g.comando_ligar(True) == ["param set FLT_DRY_RUN 1", "hover 0.10"]
    assert g.comando_ligar(False) == ["param set FLT_DRY_RUN 0", "hover 0.10"]


def test_altura_do_painel_igual_ao_limite_do_firmware():
    config = (RAIZ / "App/Inc/flight_config.h").read_text()
    limite = float(re.search(r"#define FLIGHT_MAX_HEIGHT_M\s+([0-9.]+)F", config).group(1))
    assert abs(g.ALTURA_M - limite) < 1e-9


def test_enlace_com_porta_virtual():
    mestre, escravo = pty.openpty()
    tty.setraw(mestre)
    enlace = g.Enlace(serial.Serial(os.ttyname(escravo), 115200, timeout=0.05))
    try:
        os.write(mestre, ("# console\r\n" + EST + "\r\nMOT;10;0;0;0;0;0;0;0;0;0\r\nlixo\r\n").encode())
        tipos = []
        fim = time.time() + 1.0
        while time.time() < fim and len(tipos) < 5:
            try:
                tipos.append(enlace.eventos.get(timeout=0.1)[0])
            except Exception:
                pass
        assert tipos == ["texto", "est", "bruta", "mot", "bruta"], tipos
        assert enlace.enviar("land")
        time.sleep(0.1)
        assert os.read(mestre, 64) == b"land\r\n"
        enlace.fechar()
    finally:
        os.close(mestre)


if __name__ == "__main__":
    for nome, funcao in sorted(globals().items()):
        if nome.startswith("test_"):
            funcao()
            print("ok ", nome)
    print("test_controle_gui: ok")
