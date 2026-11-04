#!/usr/bin/env python3
"""Painel de teste dos motores: arma o ESC, controla o DShot e o sentido.

Fala com o console USB do firmware da branch ``motores`` (``App/Inc/usb_console.h``)
pela porta serial do STM32 (CDC, normalmente ``/dev/ttyACM0``).

    python3 tools/motores_gui.py [porta]

Depende de ``pyserial`` e de ``tkinter``.

Segurança (a bateria alimenta os ESCs, a USB alimenta só o STM32):

* **tire as hélices** antes de armar;
* o painel liga o watchdog do firmware: se o PC parar de falar por 1 s, os
  motores são cortados, e fechar a porta também os corta;
* **PARAR TUDO** (ou a barra de espaço / Esc) zera tudo na hora;
* o limite de DShot começa baixo (``LIMITE_PADRAO``).

O que a tela mostra é o que o STM32 *enviou* ao ESC (linha ``MOT`` do log). O
ESC não devolve a rotação real, então a animação é proporcional ao DShot, não
medida.
"""
from __future__ import annotations

import math
import queue
import sys
import threading
import time
from dataclasses import dataclass, field

import serial
from serial.tools import list_ports

#: Porta padrão do STM32 (CDC).
PORTA_PADRAO = "/dev/ttyACM0"
#: Menor e maior valor DShot de aceleração.
DSHOT_MIN, DSHOT_MAX = 48, 2047
#: Limite inicial dos controles (~14%); sobe pela tela.
LIMITE_PADRAO = 300
#: Prazo do watchdog do firmware, em ms.
WATCHDOG_MS = 1000
#: Intervalo entre mensagens de "estou vivo" e entre envios de valores, em s.
INTERVALO_PING_S = 0.25
INTERVALO_ENVIO_S = 0.08
#: Rampa do firmware, em % por segundo, ao conectar.
RAMPA_PADRAO = 40

#: Motores na ordem da tela (cima = frente do drone): (número, rótulo, cor da hélice).
LAYOUT = [
    [(4, "M4 · frente/esquerda", "#e0b400"), (2, "M2 · frente/direita", "#222222")],
    [(3, "M3 · trás/esquerda", "#222222"), (1, "M1 · trás/direita", "#e0b400")],
]


@dataclass
class EstadoMotores:
    """Último estado informado pela linha ``MOT`` do firmware."""

    armado: bool = False
    dshot: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    permille: list[int] = field(default_factory=lambda: [0, 0, 0, 0])


def limitar(valor: int, limite: int) -> int:
    """Leva ``valor`` para 0 (parado) ou para a faixa 48..limite do DShot."""
    if valor < DSHOT_MIN:
        return 0
    return min(valor, max(limite, DSHOT_MIN), DSHOT_MAX)


def proximo_comando(pedido: list[int], enviado: list[int]) -> str | None:
    """Próximo comando ``d`` a mandar para o firmware, ou None se já está tudo enviado.

    Atualiza ``enviado``. Quando os quatro pedidos são iguais, um único ``d 0``
    cobre todos os motores; senão manda um motor por vez.
    """
    if pedido == enviado:
        return None
    if len(set(pedido)) == 1:
        enviado[:] = list(pedido)
        return f"d 0 {pedido[0]}"
    for i in range(4):
        if pedido[i] != enviado[i]:
            enviado[i] = pedido[i]
            return f"d {i + 1} {pedido[i]}"
    return None


def interpretar_mot(linha: str) -> EstadoMotores | None:
    """Lê ``MOT;t_ms;armado;pct1..pct4;dshot1..dshot4``; None se não for essa linha."""
    partes = linha.strip().split(";")
    if len(partes) != 11 or partes[0] != "MOT":
        return None
    try:
        numeros = [int(p) for p in partes[1:]]
    except ValueError:
        return None
    return EstadoMotores(armado=numeros[1] != 0, permille=numeros[2:6], dshot=numeros[6:10])


class EnlaceMotores:
    """Porta serial do STM32: envia comandos e separa o que chega em estado e texto.

    Uma thread lê a porta; o que chega fica em ``eventos`` como tuplas
    ``("mot", EstadoMotores)``, ``("texto", str)`` ou ``("fechada", motivo)``.
    """

    def __init__(self, serial_port) -> None:
        self._serial = serial_port
        self._trava = threading.Lock()
        self._vivo = True
        self.eventos: queue.Queue = queue.Queue()
        self._thread = threading.Thread(target=self._ler, daemon=True)
        self._thread.start()

    @classmethod
    def abrir(cls, porta: str) -> "EnlaceMotores":
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

    def fechar(self) -> None:
        """Para os motores e fecha a porta."""
        if self._vivo:
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
                dados = self._serial.read(256)
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
                estado = interpretar_mot(linha)
                if estado is not None:
                    self.eventos.put(("mot", estado))
                elif linha.startswith("#"):
                    self.eventos.put(("texto", linha.lstrip("# ").strip()))


# --------------------------------------------------------------------------
# Tela
# --------------------------------------------------------------------------

def executar_tela(porta_inicial: str) -> None:
    """Monta e roda a janela."""
    import tkinter as tk
    from tkinter import ttk

    raiz = tk.Tk()
    raiz.title("Teste de motores · DShot")
    raiz.geometry("980x760")

    enlace: EnlaceMotores | None = None
    estado = EstadoMotores()
    # Valor pedido por motor (índice 0 = M1) e último valor já enviado.
    pedido = [0, 0, 0, 0]
    enviado = [-1, -1, -1, -1]
    sentido = {n: "?" for n in (1, 2, 3, 4)}  # "n", "r" ou "?" (não definido por aqui)
    angulo = {n: 0.0 for n in (1, 2, 3, 4)}
    ultimo_envio = [0.0]
    ultimo_ping = [0.0]

    limite_var = tk.IntVar(value=LIMITE_PADRAO)
    rampa_var = tk.IntVar(value=RAMPA_PADRAO)
    juntos_var = tk.BooleanVar(value=False)
    porta_var = tk.StringVar(value=porta_inicial)
    situacao_var = tk.StringVar(value="Desconectado")
    sliders: dict[int, tk.Scale] = {}
    canvases: dict[int, tk.Canvas] = {}
    rotulos_dshot: dict[int, tk.StringVar] = {}
    botoes_sentido: dict[int, ttk.Button] = {}

    def escrever_log(texto: str) -> None:
        caixa.configure(state="normal")
        caixa.insert("end", time.strftime("%H:%M:%S ") + texto + "\n")
        caixa.see("end")
        caixa.configure(state="disabled")

    def mandar(comando: str) -> None:
        if enlace is None:
            return
        if enlace.enviar(comando):
            ultimo_envio[0] = time.monotonic()

    # ---- ações -----------------------------------------------------------

    def conectar() -> None:
        nonlocal enlace
        if enlace is not None:
            desconectar()
            return
        try:
            enlace = EnlaceMotores.abrir(porta_var.get())
        except (serial.SerialException, OSError) as erro:
            situacao_var.set(f"Erro: {erro}")
            enlace = None
            return
        botao_conectar.configure(text="Desconectar")
        situacao_var.set(f"Conectado em {porta_var.get()}")
        for cmd in ("stop", "log on", f"rate {rampa_var.get()}", f"wd {WATCHDOG_MS}"):
            mandar(cmd)
        escrever_log("conectado; sem hélices!")

    def desconectar() -> None:
        nonlocal enlace, estado
        if enlace is not None:
            enlace.fechar()
        enlace = None
        estado = EstadoMotores()
        parar_sliders()
        botao_conectar.configure(text="Conectar")
        situacao_var.set("Desconectado")

    def parar_sliders() -> None:
        for n in sliders:
            sliders[n].set(0)
        for i in range(4):
            pedido[i] = 0
            enviado[i] = -1

    def parar_tudo(_evento=None) -> None:
        """Corta tudo, antes de qualquer outra coisa."""
        mandar("stop")
        parar_sliders()
        escrever_log("PARAR TUDO")

    def armar() -> None:
        if enlace is None:
            return
        parar_sliders()
        mandar("arm")
        mandar(f"rate {rampa_var.get()}")
        enviado[:] = [0, 0, 0, 0]  # o firmware arma com todos os motores em 0

    def desarmar() -> None:
        parar_tudo()

    def alterou_rampa(_v=None) -> None:
        mandar(f"rate {max(1, rampa_var.get())}")

    def bipar(n: int) -> None:
        if estado.armado:
            escrever_log("beep só com a saída desarmada")
            return
        mandar(f"beep {n}")

    def alternar_sentido(n: int) -> None:
        if estado.armado:
            escrever_log("sentido só com a saída desarmada")
            return
        novo = "n" if sentido[n] == "r" else "r"
        sentido[n] = novo
        botoes_sentido[n].configure(text=texto_sentido(n))
        mandar(f"dir {n} {novo}")
        escrever_log(f"M{n}: sentido {'invertido' if novo == 'r' else 'normal'} (gravando no ESC)")

    def texto_sentido(n: int) -> str:
        return {"n": "Sentido: normal", "r": "Sentido: INVERTIDO", "?": "Sentido: ? (inverter)"}[
            sentido[n]
        ]

    def mudou_slider(n: int, valor: str) -> None:
        v = limitar(int(float(valor)), limite_var.get())
        if juntos_var.get():
            for m in range(1, 5):
                pedido[m - 1] = v
                if m != n:
                    sliders[m].set(v)
        else:
            pedido[n - 1] = v

    def mudou_limite(_v=None) -> None:
        for n, s in sliders.items():
            s.configure(to=limite_var.get())
            if s.get() > limite_var.get():
                s.set(limite_var.get())

    # ---- laço periódico --------------------------------------------------

    def enviar_pedidos() -> None:
        """Manda só o que mudou; sem mudança, um ping mantém o watchdog vivo."""
        agora = time.monotonic()
        if enlace is None:
            return
        if estado.armado and agora - ultimo_envio[0] >= INTERVALO_ENVIO_S:
            comando = proximo_comando(pedido, enviado)
            if comando is not None:
                mandar(comando)
        if agora - ultimo_envio[0] >= INTERVALO_PING_S:
            mandar("ping")

    def consumir_eventos() -> None:
        nonlocal enlace, estado
        if enlace is None:
            return
        while True:
            try:
                tipo, dado = enlace.eventos.get_nowait()
            except queue.Empty:
                break
            if tipo == "mot":
                estado = dado
            elif tipo == "texto":
                escrever_log(dado)
            elif tipo == "fechada":
                escrever_log(f"porta fechada: {dado}")
                desconectar()
                return
        if not estado.armado and any(enviado[i] > 0 for i in range(4)):
            parar_sliders()  # o firmware desarmou (watchdog, stop...): limpa a tela

    def desenhar() -> None:
        for n, canvas in canvases.items():
            valor = estado.dshot[n - 1]
            rodando = valor >= DSHOT_MIN
            if rodando:
                passo = 2.0 + 40.0 * (valor - DSHOT_MIN) / (DSHOT_MAX - DSHOT_MIN)
                angulo[n] = (angulo[n] + (-passo if sentido[n] == "r" else passo)) % 360.0
            canvas.delete("pas")
            cx = cy = 60
            for k in (0, 90, 180, 270):
                a = math.radians(angulo[n] + k)
                canvas.create_line(
                    cx, cy, cx + 48 * math.cos(a), cy + 48 * math.sin(a),
                    width=7, fill="#3b82f6" if rodando else "#9ca3af", capstyle="round", tags="pas",
                )
            rotulos_dshot[n].set(
                f"DShot {valor}  ({estado.permille[n - 1] / 10:.1f}%)" if rodando else "parado (0)"
            )
        texto = "ARMADO" if estado.armado else "desarmado"
        cor = "#b91c1c" if estado.armado else "#166534"
        faixa_estado.configure(text=texto, foreground=cor)

    def tique() -> None:
        consumir_eventos()
        enviar_pedidos()
        desenhar()
        raiz.after(30, tique)

    def fechar_janela() -> None:
        if enlace is not None:
            enlace.fechar()
        raiz.destroy()

    # ---- montagem da tela -----------------------------------------------

    topo = ttk.Frame(raiz, padding=8)
    topo.pack(fill="x")
    ttk.Label(topo, text="Porta:").pack(side="left")
    combo = ttk.Combobox(topo, textvariable=porta_var, width=18,
                         values=[p.device for p in list_ports.comports()])
    combo.pack(side="left", padx=4)
    botao_conectar = ttk.Button(topo, text="Conectar", command=conectar)
    botao_conectar.pack(side="left", padx=4)
    ttk.Label(topo, textvariable=situacao_var).pack(side="left", padx=12)
    faixa_estado = ttk.Label(topo, text="desarmado", font=("TkDefaultFont", 12, "bold"))
    faixa_estado.pack(side="right")

    aviso = tk.Label(
        raiz, bg="#fde68a", fg="#7c2d12", font=("TkDefaultFont", 11, "bold"), pady=4,
        text="SEM HÉLICES · bateria nos ESCs · PARAR TUDO = barra de espaço ou Esc",
    )
    aviso.pack(fill="x")

    acoes = ttk.Frame(raiz, padding=8)
    acoes.pack(fill="x")
    ttk.Button(acoes, text="ARMAR ESC", command=armar).pack(side="left", padx=4)
    ttk.Button(acoes, text="Desarmar", command=desarmar).pack(side="left", padx=4)
    tk.Button(acoes, text="PARAR TUDO", bg="#dc2626", fg="white", activebackground="#991b1b",
              font=("TkDefaultFont", 12, "bold"), command=parar_tudo).pack(side="left", padx=16)
    ttk.Checkbutton(acoes, text="Mover os 4 juntos", variable=juntos_var).pack(side="left", padx=8)
    ttk.Label(acoes, text="Limite DShot:").pack(side="left", padx=(16, 2))
    tk.Scale(acoes, from_=DSHOT_MIN, to=DSHOT_MAX, orient="horizontal", length=160,
             variable=limite_var, command=mudou_limite).pack(side="left")
    ttk.Label(acoes, text="Rampa %/s:").pack(side="left", padx=(16, 2))
    tk.Scale(acoes, from_=5, to=100, orient="horizontal", length=120, variable=rampa_var,
             command=alterou_rampa).pack(side="left")

    drone = ttk.Frame(raiz, padding=8)
    drone.pack(fill="both", expand=True)
    ttk.Label(drone, text="▲  FRENTE DO DRONE  ▲", font=("TkDefaultFont", 11, "bold")).grid(
        row=0, column=0, columnspan=2, pady=(0, 4))
    for r, linha in enumerate(LAYOUT, start=1):
        for c, (n, rotulo, cor) in enumerate(linha):
            quadro = ttk.LabelFrame(drone, text=rotulo, padding=6)
            quadro.grid(row=r, column=c, padx=10, pady=6, sticky="nsew")
            drone.columnconfigure(c, weight=1)
            canvas = tk.Canvas(quadro, width=120, height=120, bg="white", highlightthickness=2,
                               highlightbackground=cor)
            canvas.create_oval(8, 8, 112, 112, outline=cor, width=4)
            canvas.pack()
            canvases[n] = canvas
            rotulos_dshot[n] = tk.StringVar(value="parado (0)")
            ttk.Label(quadro, textvariable=rotulos_dshot[n]).pack()
            s = tk.Scale(quadro, from_=0, to=LIMITE_PADRAO, orient="horizontal", length=240,
                         command=lambda v, n=n: mudou_slider(n, v))
            s.pack()
            sliders[n] = s
            linha_btn = ttk.Frame(quadro)
            linha_btn.pack(pady=2)
            ttk.Button(linha_btn, text="Beep", width=6, command=lambda n=n: bipar(n)).pack(
                side="left", padx=2)
            b = ttk.Button(linha_btn, text=texto_sentido(n), width=22,
                           command=lambda n=n: alternar_sentido(n))
            b.pack(side="left", padx=2)
            botoes_sentido[n] = b

    caixa = tk.Text(raiz, height=7, state="disabled", bg="#111827", fg="#d1d5db")
    caixa.pack(fill="x", padx=8, pady=(0, 8))

    raiz.bind("<space>", parar_tudo)
    raiz.bind("<Escape>", parar_tudo)
    raiz.protocol("WM_DELETE_WINDOW", fechar_janela)
    tique()
    raiz.mainloop()


def main() -> None:
    """Ponto de entrada: ``motores_gui.py [porta]``."""
    executar_tela(sys.argv[1] if len(sys.argv) > 1 else PORTA_PADRAO)


if __name__ == "__main__":
    main()
