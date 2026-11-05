#!/usr/bin/env python3
"""Painel do teste de controle: sobe 15 cm em rampa, segura e pousa em rampa.

Fala com o console USB do firmware (``App/Inc/usb_console.h``) pela porta serial
do STM32 (CDC, normalmente ``/dev/ttyACM0``) e mostra a telemetria do LQG:

    python3 tools/controle_gui.py [porta]

Depende de ``pyserial`` e de ``tkinter``.

O que o painel faz:

* **LIGAR** arma e sobe 15 cm em rampa (o firmware limita qualquer pedido a
  15 cm e usa a altura lida pelo MTF-01); **DESLIGAR** pousa em rampa e desarma.
* **EMERGÊNCIA** (ou barra de espaço / Esc) corta os motores na hora.
* **Ensaio a seco** (marcado por padrão): o controle roda e a tela mostra o que
  cada motor receberia, mas os motores não giram. Use com o drone na mão para
  conferir os sinais antes de soltar os motores.

O controle é só de atitude e altura (``CTL_XY = 0``): rolagem e arfagem ficam
em zero, a guinada fica no zero da energização e a altura segue a rampa. A
posição horizontal **não** é controlada: um pequeno erro de nível faz o drone
deslizar. Prenda-o.

Compensação automática (marcada por padrão): o LQG do TCC não tem integrador,
então um hover errado (o drone não sai do chão) ou um peso mal distribuído
(a frente levanta e a traseira fica) deixam erro de altura e de inclinação. O
firmware soma um ajuste lento à rotação dos motores (``control/trim.h``) e
registra no log o que aprendeu ao pousar (``trim z=.. th=.. phi=..``).

Pouso: ao encostar no chão a aceleração desce em rampa até a marcha lenta
(1,5 s) e só então desarma, em vez de cortar de uma vez.

Altura: escolha no painel (3 a 15 cm) uma altura que as cordinhas da bancada
deixem alcançar. Se o drone fica preso abaixo do alvo, a compensação de hover
continua somando empuxo contra as cordinhas, que puxam torto.

CTL_W0: com a compensação ligada, ajuste só antes de LIGAR. Ao pousar, o painel
sugere o próximo valor (``CTL_W0 + z`` aprendido).

Segurança no firmware: inclinação acima de 20 graus corta os motores; altura
acima de 35 cm pousa; se o PC calar por 1 s (watchdog), o drone pousa em rampa.
"""
from __future__ import annotations

import queue
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import serial
from serial.tools import list_ports

#: Porta padrão do STM32 (CDC).
PORTA_PADRAO = "/dev/ttyACM0"
#: Maior altura da subida, em m (o firmware também limita a este valor).
ALTURA_M = 0.15
#: Menor altura escolhível no painel, em m.
ALTURA_MIN_M = 0.03
#: Altura com que o painel abre, em m. Na bancada de 04/11 as cordinhas deixam o
#: drone subir só ~6 cm; acima disso a compensação de hover empurra contra elas.
ALTURA_PADRAO_M = 0.04
#: CTL_W0 com que o painel abre, em rad/s: o hover aprendido no ensaio de 04/11
#: (768 + 80 de compensação com o drone nivelado no ar).
W0_PADRAO = 850
#: Prazo do watchdog do firmware, em ms.
WATCHDOG_MS = 1000
#: Intervalo entre mensagens de "estou vivo", em s.
INTERVALO_PING_S = 0.25
#: Taxa da linha EST pedida ao firmware, em Hz.
TAXA_EST_HZ = 20
#: Depois de LIGAR, o botão ignora cliques por este tempo (evita um clique
#: duplo virar DESLIGAR logo após armar), em s.
TRAVA_BOTAO_S = 1.5
#: Pasta dos CSVs de cada ensaio.
PASTA_LOGS = Path(__file__).resolve().parent / "logs"

#: Nomes dos estados do voo (flight_ctrl_state_name).
ESTADOS = ["BOOT", "SELF_TEST", "STANDBY", "PREARM", "ARMADO", "DECOLANDO", "SEGURANDO",
           "TRECHO", "POUSANDO", "FAILSAFE", "TERMINADO"]
ESTADO_STANDBY, ESTADO_ARMADO, ESTADO_DECOLANDO, ESTADO_HOLD, ESTADO_POUSANDO = 2, 4, 5, 6, 8
ESTADO_FAILSAFE = 9
#: Bits de meas (MEAS_*) e de flags (FLIGHT_FLAG_*).
MEAS_ATITUDE, MEAS_GIRO, MEAS_ALTURA, MEAS_FLUXO = 1, 2, 4, 8
FLAG_SECO, FLAG_ATIVO, FLAG_GANHOS_PI, FLAG_SATURADO = 1, 2, 4, 8
#: Entrada do modelo (u1..u4) -> motor físico (control/mixer.h).
MOTOR_DA_ENTRADA = {1: 4, 2: 2, 3: 1, 4: 3}
#: Radianos para graus.
RAD_GRAUS = 57.29577951


@dataclass
class Estimativa:
    """Linha ``EST`` do firmware, já em unidades de engenharia."""

    estado: int = 0
    meas: int = 0
    flags: int = 0
    y: list[float] = field(default_factory=lambda: [0.0] * 9)    # z φ θ ψ vx vy p q r
    x: list[float] = field(default_factory=lambda: [0.0] * 12)
    du: list[float] = field(default_factory=lambda: [0.0] * 4)   # rad/s, por entrada u1..u4
    ref: list[float] = field(default_factory=lambda: [0.0] * 4)  # x y z ψ

    @property
    def altura_m(self) -> float:
        """Altura medida pelo MTF-01 (o z do modelo é para baixo)."""
        return -self.y[0]

    @property
    def nome_estado(self) -> str:
        return ESTADOS[self.estado] if 0 <= self.estado < len(ESTADOS) else f"?{self.estado}"


@dataclass
class Motores:
    """Linha ``MOT``: aceleração enviada a cada motor físico (índice 0 = M1)."""

    armado: bool = False
    permille: list[int] = field(default_factory=lambda: [0] * 4)
    dshot: list[int] = field(default_factory=lambda: [0] * 4)


def interpretar_est(linha: str) -> Estimativa | None:
    """Lê ``EST;t;estado;meas;flags;`` e 29 inteiros; None se não for essa linha.

    As medidas, os estados e as referências vêm em milésimos; ``du`` em décimos.
    """
    partes = linha.strip().split(";")
    if len(partes) != 5 + 29 or partes[0] != "EST":
        return None
    try:
        n = [int(p) for p in partes[2:]]
    except ValueError:
        return None
    return Estimativa(
        estado=n[0], meas=n[1], flags=n[2],
        y=[v / 1000.0 for v in n[3:12]],
        x=[v / 1000.0 for v in n[12:24]],
        du=[v / 10.0 for v in n[24:28]],
        ref=[v / 1000.0 for v in n[28:32]],
    )


def interpretar_mot(linha: str) -> Motores | None:
    """Lê ``MOT;t_ms;armado;pct1..pct4;dshot1..dshot4``; None se não for essa linha."""
    partes = linha.strip().split(";")
    if len(partes) != 11 or partes[0] != "MOT":
        return None
    try:
        n = [int(p) for p in partes[1:]]
    except ValueError:
        return None
    return Motores(armado=n[1] != 0, permille=n[2:6], dshot=n[6:10])


def du_por_motor(du_entradas: list[float]) -> list[float]:
    """Reordena du (u1..u4 do modelo) em um valor por motor físico M1..M4."""
    por_motor = [0.0] * 4
    for entrada, motor in MOTOR_DA_ENTRADA.items():
        por_motor[motor - 1] = du_entradas[entrada - 1]
    return por_motor


def comando_ligar(seco: bool, compensar: bool = True, altura_m: float = ALTURA_M) -> list[str]:
    """Comandos que armam e sobem ``altura_m`` (ensaio a seco ou real).

    ``compensar`` liga o ajuste lento do hover e do equilíbrio (``CTL_TRIM``,
    ``control/trim.h``): sem ele, um peso mal distribuído ou um hover errado
    deixam o drone no chão ou inclinado.
    """
    return [f"param set FLT_DRY_RUN {1 if seco else 0}",
            f"param set CTL_TRIM {1 if compensar else 0}",
            f"hover {min(max(altura_m, ALTURA_MIN_M), ALTURA_M):.2f}"]


def sugestao_w0(w0_atual: float, texto_trim: str) -> int | None:
    """Novo ``CTL_W0`` a partir da última mensagem ``trim z=.. th=.. phi=..``.

    O hover real é ``CTL_W0 + z``: começar o próximo voo já nele deixa a
    compensação de hover perto de zero. None se o texto não tiver ``z=``.
    """
    for parte in texto_trim.split():
        if parte.startswith("z="):
            try:
                return int(round(w0_atual + float(parte[2:])))
            except ValueError:
                return None
    return None


class Enlace:
    """Porta serial do STM32: envia comandos e separa o que chega.

    Uma thread lê a porta; o que chega fica em ``eventos`` como tuplas
    ``("est", Estimativa)``, ``("mot", Motores)``, ``("texto", str)``,
    ``("bruta", str)`` (linha original) ou ``("fechada", motivo)``.
    """

    def __init__(self, serial_port) -> None:
        self._serial = serial_port
        self._trava = threading.Lock()
        self._vivo = True
        self.eventos: queue.Queue = queue.Queue()
        threading.Thread(target=self._ler, daemon=True).start()

    @classmethod
    def abrir(cls, porta: str) -> "Enlace":
        """Abre a porta com DTR ligado (sem DTR o firmware recusa armar)."""
        s = serial.Serial(porta, 115200, timeout=0.1)
        s.dtr = True
        return cls(s)

    def enviar(self, comando: str) -> bool:
        """Manda uma linha de comando. Devolve False se a porta caiu."""
        with self._trava:
            try:
                self._serial.write((comando + "\r\n").encode("ascii"))
                return True
            except (serial.SerialException, OSError) as erro:
                self.eventos.put(("fechada", str(erro)))
                return False

    def fechar(self, parar: bool = True) -> None:
        """Fecha a porta; antes, manda ``stop`` se ``parar``."""
        if self._vivo and parar:
            self.enviar("stop")
            time.sleep(0.05)
        self._vivo = False
        try:
            self._serial.close()
        except (serial.SerialException, OSError):
            pass

    def _ler(self) -> None:
        resto = b""
        while self._vivo:
            try:
                dados = self._serial.read(512)
            except Exception as erro:  # noqa: BLE001 - fechar() derruba a leitura de formas variadas
                if self._vivo:
                    self.eventos.put(("fechada", str(erro)))
                return
            if not dados:
                continue
            resto += dados
            while b"\n" in resto:
                bruta, resto = resto.split(b"\n", 1)
                linha = bruta.decode("ascii", errors="replace").strip()
                est = interpretar_est(linha)
                mot = None if est else interpretar_mot(linha)
                if est is not None:
                    self.eventos.put(("est", est))
                    self.eventos.put(("bruta", linha))
                elif mot is not None:
                    self.eventos.put(("mot", mot))
                    self.eventos.put(("bruta", linha))
                elif linha.startswith("#"):
                    self.eventos.put(("texto", linha.lstrip("# ").strip()))


# --------------------------------------------------------------------------
# Tela
# --------------------------------------------------------------------------

def executar_tela(porta_inicial: str) -> None:
    """Monta e roda a janela."""
    import tkinter as tk
    from tkinter import messagebox, ttk

    raiz = tk.Tk()
    raiz.title("Teste de controle · LQG atitude + altura")
    raiz.geometry("1040x800")

    enlace: Enlace | None = None
    est = Estimativa()
    mot = Motores()
    chao_m = [None]            # altura medida no chão (referência da "subida")
    arquivo = [None]           # CSV do ensaio
    ultimo_envio = [0.0]
    botao_travado_ate = [0.0]

    porta_var = tk.StringVar(value=porta_inicial)
    situacao_var = tk.StringVar(value="Desconectado")
    seco_var = tk.BooleanVar(value=True)
    trim_var = tk.BooleanVar(value=True)
    gravar_var = tk.BooleanVar(value=True)
    w0_var = tk.IntVar(value=W0_PADRAO)
    altura_var = tk.IntVar(value=int(round(ALTURA_PADRAO_M * 100)))  # cm
    ultimo_trim = [""]
    armado_antes = [False]
    maxmot_var = tk.IntVar(value=80)
    rampa_var = tk.IntVar(value=5)
    valores: dict[str, tk.StringVar] = {k: tk.StringVar(value="--") for k in (
        "estado", "sensores", "modo", "roll", "pitch", "yaw", "altura", "ref", "subida", "trim")}
    barras_du: dict[int, tk.Canvas] = {}
    textos_motor: dict[int, tk.StringVar] = {n: tk.StringVar(value="--") for n in (1, 2, 3, 4)}

    def escrever_log(texto: str) -> None:
        caixa.configure(state="normal")
        caixa.insert("end", time.strftime("%H:%M:%S ") + texto + "\n")
        caixa.see("end")
        caixa.configure(state="disabled")

    def mandar(comando: str) -> None:
        if enlace is not None and enlace.enviar(comando):
            ultimo_envio[0] = time.monotonic()

    # ---- ações -----------------------------------------------------------

    def conectar() -> None:
        nonlocal enlace
        if enlace is not None:
            desconectar()
            return
        try:
            enlace = Enlace.abrir(porta_var.get())
        except (serial.SerialException, OSError) as erro:
            situacao_var.set(f"Erro: {erro}")
            enlace = None
            return
        botao_conectar.configure(text="Desconectar")
        situacao_var.set(f"Conectado em {porta_var.get()}")
        for cmd in ("stop", "log on", f"wd {WATCHDOG_MS}", f"param set LOG_EST_HZ {TAXA_EST_HZ}",
                    "param set FLT_DRY_RUN 1", f"param set FLT_HOVER_H {ALTURA_M:.2f}",
                    f"param set CTL_W0 {w0_var.get()}", f"param set MOT_MAX {maxmot_var.get() / 100:.2f}",
                    f"param set FLT_Z_RATE {rampa_var.get() / 100:.2f}"):
            mandar(cmd)
        if gravar_var.get():
            abrir_csv()
        escrever_log("conectado")

    def desconectar() -> None:
        nonlocal enlace, est, mot
        if enlace is not None:
            enlace.fechar()
        enlace = None
        est, mot = Estimativa(), Motores()
        if arquivo[0] is not None:
            arquivo[0].close()
            arquivo[0] = None
        botao_conectar.configure(text="Conectar")
        situacao_var.set("Desconectado")

    def abrir_csv() -> None:
        PASTA_LOGS.mkdir(parents=True, exist_ok=True)
        caminho = PASTA_LOGS / time.strftime("controle_%Y%m%d_%H%M%S.log")
        # Linha a linha: se o painel for fechado à força, o log do ensaio fica.
        arquivo[0] = open(caminho, "w", encoding="utf-8", buffering=1)
        escrever_log(f"gravando {caminho}")

    def emergencia(_evento=None) -> None:
        """Corta os motores antes de qualquer outra coisa."""
        mandar("stop")
        escrever_log("EMERGÊNCIA: motores cortados")

    def ligar_ou_desligar() -> None:
        if enlace is None or time.monotonic() < botao_travado_ate[0]:
            return
        if est.estado in (ESTADO_ARMADO, ESTADO_DECOLANDO, ESTADO_HOLD):
            mandar("land")
            escrever_log("DESLIGAR: pousando em rampa")
            return
        if est.estado == ESTADO_POUSANDO:
            return
        if est.estado not in (ESTADO_STANDBY, ESTADO_FAILSAFE):
            escrever_log(f"não dá para ligar no estado {est.nome_estado}")
            return
        if not (est.meas & MEAS_ATITUDE and est.meas & MEAS_ALTURA):
            escrever_log("sensores sem dado (BNO e MTF precisam estar OK)")
            return
        if not seco_var.get() and not messagebox.askokcancel(
            "Soltar os motores?",
            f"Ensaio a seco DESLIGADO: os motores vão girar e o drone vai subir {altura_var.get()} cm.\n\n"
            "• Drone PRESO (barbante) e em área livre?\n• Hélices conferidas?\n"
            "• Mão no botão de EMERGÊNCIA (barra de espaço)?",
        ):
            return
        for cmd in comando_ligar(seco_var.get(), trim_var.get(), altura_var.get() / 100):
            mandar(cmd)
        botao_travado_ate[0] = time.monotonic() + TRAVA_BOTAO_S
        escrever_log("LIGAR: " + ("ensaio a seco" if seco_var.get() else "MOTORES REAIS") + f", subindo {altura_var.get()} cm")

    def ajustar(nome: str, valor: str) -> None:
        armado = est.estado in (ESTADO_ARMADO, ESTADO_DECOLANDO, ESTADO_HOLD, ESTADO_POUSANDO)
        if nome == "CTL_W0" and armado and trim_var.get():
            # Com a compensação ligada, mudar o hover em voo soma um degrau nos
            # quatro motores por cima do que ela já achou (ensaio de 04/11).
            escrever_log("CTL_W0 não muda em voo com a compensação ligada; ajuste antes de LIGAR")
            return
        mandar(f"param set {nome} {valor}")

    # ---- laço periódico --------------------------------------------------

    def consumir_eventos() -> None:
        nonlocal est, mot
        if enlace is None:
            return
        while True:
            try:
                tipo, dado = enlace.eventos.get_nowait()
            except queue.Empty:
                break
            if tipo == "est":
                armado = dado.estado in (ESTADO_ARMADO, ESTADO_DECOLANDO, ESTADO_HOLD, ESTADO_POUSANDO)
                if armado_antes[0] and not armado and ultimo_trim[0]:
                    sugestao = sugestao_w0(w0_var.get(), ultimo_trim[0])
                    if sugestao is not None:
                        escrever_log(f"sugestão para o próximo voo: CTL_W0 = {sugestao} "
                                     f"(hover aprendido: {ultimo_trim[0]})")
                armado_antes[0] = armado
                est = dado
                if est.estado == ESTADO_STANDBY or chao_m[0] is None:
                    chao_m[0] = est.altura_m
            elif tipo == "mot":
                mot = dado
            elif tipo == "texto":
                escrever_log(dado)
                if dado.startswith("trim "):
                    valores["trim"].set(dado[len("trim "):])
                    ultimo_trim[0] = dado[len("trim "):]
                if arquivo[0] is not None:
                    arquivo[0].write(f"{time.time():.3f};TXT;{dado}\n")
            elif tipo == "bruta" and arquivo[0] is not None:
                arquivo[0].write(f"{time.time():.3f};{dado}\n")
            elif tipo == "fechada":
                escrever_log(f"porta fechada: {dado}")
                desconectar()
                return

    def desenhar() -> None:
        if enlace is None:
            valores["estado"].set("Desconectado")
        else:
            valores["estado"].set(est.nome_estado)
        cor = {ESTADO_FAILSAFE: "#b91c1c", ESTADO_POUSANDO: "#b45309", ESTADO_HOLD: "#166534",
               ESTADO_DECOLANDO: "#1d4ed8", ESTADO_ARMADO: "#1d4ed8"}.get(est.estado, "#374151")
        rotulo_estado.configure(foreground=cor)
        ok_bno = bool(est.meas & MEAS_ATITUDE and est.meas & MEAS_GIRO)
        ok_mtf = bool(est.meas & MEAS_ALTURA)
        valores["sensores"].set(f"BNO {'OK' if ok_bno else 'SEM DADO'} · MTF {'OK' if ok_mtf else 'SEM DADO'}")
        valores["modo"].set("motores parados (a seco)" if est.flags & FLAG_SECO
                            else "MOTORES SEGUINDO O CONTROLE")
        valores["roll"].set(f"{est.y[1] * RAD_GRAUS:+6.2f}°")      # φ
        valores["pitch"].set(f"{est.y[2] * RAD_GRAUS:+6.2f}°")     # θ
        valores["yaw"].set(f"{est.y[3] * RAD_GRAUS:+6.2f}°")       # ψ
        valores["altura"].set(f"{est.altura_m * 1000:6.0f} mm")
        valores["ref"].set(f"{-est.ref[2] * 1000:6.0f} mm")
        subida = 0.0 if chao_m[0] is None else est.altura_m - chao_m[0]
        valores["subida"].set(f"{subida * 100:+5.1f} cm de {altura_var.get()}")
        du = du_por_motor(est.du)
        for n in (1, 2, 3, 4):
            c = barras_du[n]
            c.delete("all")
            meio = 100
            largura = max(-95, min(95, du[n - 1] * 3.0))  # 1 rad/s = 3 px
            c.create_line(meio, 0, meio, 18, fill="#9ca3af")
            c.create_rectangle(meio, 3, meio + largura, 15,
                               fill="#2563eb" if largura >= 0 else "#dc2626", width=0)
            textos_motor[n].set(f"du {du[n - 1]:+6.1f} rad/s · motor {mot.permille[n - 1] / 10:4.1f}% "
                                f"(DShot {mot.dshot[n - 1]})")
        botao_ligar.configure(text=texto_ligar())
        faixa_modo.configure(
            background="#fee2e2" if (not seco_var.get()) else "#dcfce7",
            text=("MOTORES REAIS: drone preso, mão na EMERGÊNCIA" if not seco_var.get()
                  else "Ensaio a seco: os motores NÃO giram"),
        )

    def texto_ligar() -> str:
        if enlace is None:
            return "LIGAR"
        if est.estado in (ESTADO_ARMADO, ESTADO_DECOLANDO, ESTADO_HOLD):
            return "DESLIGAR (pousar em rampa)"
        if est.estado == ESTADO_POUSANDO:
            return "Pousando..."
        return f"LIGAR (subir {altura_var.get()} cm)"

    def tique() -> None:
        consumir_eventos()
        if enlace is not None and time.monotonic() - ultimo_envio[0] >= INTERVALO_PING_S:
            mandar("ping")
        desenhar()
        raiz.after(50, tique)

    def fechar_janela() -> None:
        if enlace is not None:
            enlace.fechar()
        if arquivo[0] is not None:
            arquivo[0].close()
        raiz.destroy()

    # ---- montagem da tela -----------------------------------------------

    topo = ttk.Frame(raiz, padding=8)
    topo.pack(fill="x")
    ttk.Label(topo, text="Porta:").pack(side="left")
    ttk.Combobox(topo, textvariable=porta_var, width=18,
                 values=[p.device for p in list_ports.comports()]).pack(side="left", padx=4)
    botao_conectar = ttk.Button(topo, text="Conectar", command=conectar)
    botao_conectar.pack(side="left", padx=4)
    ttk.Label(topo, textvariable=situacao_var).pack(side="left", padx=12)
    ttk.Checkbutton(topo, text="Gravar log do ensaio", variable=gravar_var).pack(side="right")

    tk.Label(raiz, bg="#fde68a", fg="#7c2d12", font=("TkDefaultFont", 11, "bold"), pady=4,
             text="PRENDA O DRONE · sem controle horizontal ele desliza · EMERGÊNCIA = barra de espaço ou Esc"
             ).pack(fill="x")
    faixa_modo = tk.Label(raiz, font=("TkDefaultFont", 11, "bold"), pady=3)
    faixa_modo.pack(fill="x")

    estado_quadro = ttk.Frame(raiz, padding=8)
    estado_quadro.pack(fill="x")
    rotulo_estado = ttk.Label(estado_quadro, textvariable=valores["estado"],
                              font=("TkDefaultFont", 22, "bold"))
    rotulo_estado.pack(side="left", padx=(0, 24))
    ttk.Label(estado_quadro, textvariable=valores["sensores"], font=("TkDefaultFont", 11)).pack(side="left")
    ttk.Label(estado_quadro, textvariable=valores["modo"], font=("TkDefaultFont", 11, "bold")).pack(side="right")

    medidas = ttk.LabelFrame(raiz, text="Medidas (relativas ao zero configurado)", padding=8)
    medidas.pack(fill="x", padx=8, pady=4)
    linhas = [
        ("Arfagem θ (frente/trás, nariz p/ cima +) · roll do BNO", "pitch"),
        ("Rolagem φ (lados, direita p/ baixo +) · pitch do BNO", "roll"),
        ("Guinada ψ (horário +) · zero na energização", "yaw"),
        ("Altura medida (MTF-01)", "altura"),
        ("Altura de referência (rampa)", "ref"),
        ("Subida", "subida"),
        ("Compensação (rad/s): hover, frente/trás, lados", "trim"),
    ]
    for i, (texto, chave) in enumerate(linhas):
        ttk.Label(medidas, text=texto).grid(row=i, column=0, sticky="w", padx=4, pady=1)
        ttk.Label(medidas, textvariable=valores[chave], font=("TkFixedFont", 13, "bold")).grid(
            row=i, column=1, sticky="e", padx=16)
    medidas.columnconfigure(0, weight=1)

    motores_quadro = ttk.LabelFrame(
        raiz, text="O que cada motor recebe (barra = du do controle: azul aumenta, vermelho diminui)",
        padding=8)
    motores_quadro.pack(fill="x", padx=8, pady=4)
    nomes = {1: "M1 trás/dir. (amarela)", 2: "M2 frente/dir. (preta)", 3: "M3 trás/esq. (preta)",
             4: "M4 frente/esq. (amarela)"}
    for n, (lin, col) in {4: (0, 0), 2: (0, 1), 3: (1, 0), 1: (1, 1)}.items():
        quadro = ttk.Frame(motores_quadro, padding=4)
        quadro.grid(row=lin, column=col, sticky="ew", padx=10)
        ttk.Label(quadro, text=nomes[n], font=("TkDefaultFont", 10, "bold")).pack(anchor="w")
        c = tk.Canvas(quadro, width=200, height=18, bg="white", highlightthickness=1,
                      highlightbackground="#9ca3af")
        c.pack(anchor="w")
        barras_du[n] = c
        ttk.Label(quadro, textvariable=textos_motor[n]).pack(anchor="w")
        motores_quadro.columnconfigure(col, weight=1)

    acoes = ttk.Frame(raiz, padding=8)
    acoes.pack(fill="x")
    botao_ligar = tk.Button(acoes, text="LIGAR", font=("TkDefaultFont", 14, "bold"), bg="#16a34a",
                            fg="white", activebackground="#166534", width=26, command=ligar_ou_desligar)
    botao_ligar.pack(side="left", padx=4)
    tk.Button(acoes, text="EMERGÊNCIA\nPARAR MOTORES", font=("TkDefaultFont", 12, "bold"),
              bg="#dc2626", fg="white", activebackground="#7f1d1d", command=emergencia
              ).pack(side="left", padx=16)
    ttk.Checkbutton(acoes, text="Ensaio a seco (motores parados)", variable=seco_var).pack(side="left", padx=12)
    ttk.Checkbutton(acoes, text="Compensação automática\n(hover + frente/trás/lados)",
                    variable=trim_var).pack(side="left", padx=12)

    ajustes = ttk.LabelFrame(raiz, text="Ajustes ao vivo", padding=6)
    ajustes.pack(fill="x", padx=8, pady=4)
    ttk.Label(ajustes, text="Rotação de hover CTL_W0 (rad/s):").grid(row=0, column=0, sticky="w")
    tk.Scale(ajustes, from_=400, to=1200, orient="horizontal", length=260, variable=w0_var,
             command=lambda v: ajustar("CTL_W0", str(int(float(v))))).grid(row=0, column=1)
    ttk.Label(ajustes, text="Limite de aceleração MOT_MAX (%):").grid(row=0, column=2, sticky="w", padx=(16, 0))
    tk.Scale(ajustes, from_=20, to=100, orient="horizontal", length=200, variable=maxmot_var,
             command=lambda v: ajustar("MOT_MAX", f"{float(v) / 100:.2f}")).grid(row=0, column=3)
    ttk.Label(ajustes, text="Altura da subida (cm):").grid(row=2, column=0, sticky="w")
    tk.Scale(ajustes, from_=int(ALTURA_MIN_M * 100), to=int(round(ALTURA_M * 100)), orient="horizontal",
             length=260, variable=altura_var).grid(row=2, column=1)
    ttk.Label(ajustes, text="Fique abaixo do que as cordinhas deixam subir.").grid(
        row=2, column=2, columnspan=2, sticky="w", padx=(16, 0))
    ttk.Label(ajustes, text="Rampa FLT_Z_RATE (cm/s):").grid(row=1, column=0, sticky="w")
    tk.Scale(ajustes, from_=2, to=20, orient="horizontal", length=260, variable=rampa_var,
             command=lambda v: ajustar("FLT_Z_RATE", f"{float(v) / 100:.2f}")).grid(row=1, column=1)
    ttk.Label(ajustes, text="Com a compensação ligada o CTL_W0 só precisa ficar perto; sem ela, suba até levantar.").grid(
        row=1, column=2, columnspan=2, sticky="w", padx=(16, 0))

    caixa = tk.Text(raiz, height=8, state="disabled", bg="#111827", fg="#d1d5db")
    caixa.pack(fill="both", expand=True, padx=8, pady=(4, 8))

    raiz.bind("<space>", emergencia)
    raiz.bind("<Escape>", emergencia)
    raiz.protocol("WM_DELETE_WINDOW", fechar_janela)
    tique()
    raiz.mainloop()


def main() -> None:
    """Ponto de entrada: ``controle_gui.py [porta]``."""
    executar_tela(sys.argv[1] if len(sys.argv) > 1 else PORTA_PADRAO)


if __name__ == "__main__":
    main()
