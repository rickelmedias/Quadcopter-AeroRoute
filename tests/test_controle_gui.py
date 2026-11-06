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
    def params(cmds):
        return {c.split()[2]: c.split()[3] for c in cmds if c.startswith("param set")}

    # Padrão: LQG melhorado (ajustado + estimador), com o desequilíbrio fixo.
    cmds = g.comando_ligar(True, ff_th=-90)
    assert cmds[-1] == "hover 0.25"
    assert params(cmds) == {"FLT_DRY_RUN": "1", "CTL_LQI": "0", "CTL_TRIM": "0",
                            "CTL_LQG_AJ": "1", "CTL_DIST": "1", "CTL_FF_TH": "-90",
                            "CTL_XY": "0", "FLT_PRESO": "0"}
    # Original: o LQG do TCC puro, sem desequilíbrio fixo mesmo que o painel tenha um.
    assert params(g.comando_ligar(False, "original", ff_th=-90)) == {
        "FLT_DRY_RUN": "0", "CTL_LQI": "0", "CTL_TRIM": "0", "CTL_LQG_AJ": "0", "CTL_DIST": "0",
        "CTL_FF_TH": "0", "CTL_XY": "0", "FLT_PRESO": "0"}
    # Segurar posição e detector de subida presa saem das caixas do painel.
    com_xy = params(g.comando_ligar(False, "melhorado", ff_th=-90, xy=True, preso=True))
    assert com_xy["CTL_XY"] == "1" and com_xy["FLT_PRESO"] == "1"
    assert params(g.comando_ligar(False, "ajustado", ff_th=-80))["CTL_DIST"] == "0"
    assert params(g.comando_ligar(False, "ajustado", ff_th=-80))["CTL_FF_TH"] == "-80"
    assert params(g.comando_ligar(False, "lqi", ff_th=-90))["CTL_LQI"] == "1"


def test_modos_do_painel_existem_no_firmware():
    params = (RAIZ / "App/Src/control/ctrl_params.c").read_text()
    for nome in ("CTL_LQI", "CTL_TRIM", "CTL_LQG_AJ", "CTL_DIST", "CTL_FF_TH", "CTL_XY",
                 "FLT_PRESO"):
        assert f'"{nome}"' in params, nome
    assert set(g.MODOS) == {"original", "ajustado", "melhorado", "lqi"}
    assert g.MODO_PADRAO in g.MODOS


def test_comando_ligar_respeita_a_altura_escolhida():
    assert g.comando_ligar(True, "lqi", 0.05)[-1] == "hover 0.05"
    assert g.comando_ligar(True, "lqi", 0.50)[-1] == "hover 0.25"  # nunca acima do limite
    assert g.comando_ligar(True, "lqi", 0.0)[-1] == "hover 0.03"


def test_sugestao_w0_com_mensagem_do_lqi():
    assert g.sugestao_w0(850, "z=20 th=-110 phi=-6 rad/s") == 870


def test_sugestao_w0():
    assert g.sugestao_w0(768, "z=132 th=-145 phi=-18 rad/s") == 900
    assert g.sugestao_w0(768, "z=-10 th=0 phi=0 NO LIMITE") == 758
    assert g.sugestao_w0(768, "sem nada") is None


def test_altura_do_painel_igual_ao_limite_do_firmware():
    config = (RAIZ / "App/Inc/flight_config.h").read_text()
    limite = float(re.search(r"#define FLIGHT_MAX_HEIGHT_M\s+([0-9.]+)F", config).group(1))
    assert abs(g.ALTURA_M - limite) < 1e-9


def test_pesos_do_quadro_ida_e_volta():
    padrao = g.lqi_projeto.Pesos()
    textos = g.textos_pesos(padrao)
    assert set(textos) == {chave for _rotulo, chave in g.CAMPOS_PESOS}
    assert g.pesos_de_textos(textos) == padrao
    textos["theta"] = "1000,5"  # vírgula decimal
    assert g.pesos_de_textos(textos).q["theta"] == 1000.5
    textos["R"] = ""
    try:
        g.pesos_de_textos(textos)
        raise AssertionError("campo vazio deveria falhar")
    except ValueError:
        pass


def test_envio_de_ganhos_cabe_no_console_e_reconstroi_o_crc():
    """Cada linha cabe no console (64 caracteres, 6 palavras) e o strtof do
    STM32 chega nos mesmos float32 que entraram no CRC do commit."""
    import struct
    import zlib
    pesos = g.lqi_projeto.Pesos()
    pesos.q["theta"] = 2000.0
    linhas = g.lqi_projeto.comandos(g.lqi_projeto.projetar(pesos))
    console = (RAIZ / "App/Src/usb_console.c").read_text()
    line_max = int(re.search(r"#define LINE_MAX (\d+)U", console).group(1))
    max_args = int(re.search(r"#define MAX_ARGS (\d+)", console).group(1))
    assert linhas[0] == "lqi begin" and linhas[-1].startswith("lqi commit ")
    assert len(linhas) == 2 + 76
    assert all(len(l) < line_max and len(l.split()) <= max_args for l in linhas)
    valores = [struct.unpack("<f", struct.pack("<f", float(l.split()[5])))[0] for l in linhas[1:-1]]
    assert int(linhas[-1].split()[2], 16) == zlib.crc32(struct.pack("<76f", *valores))


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
