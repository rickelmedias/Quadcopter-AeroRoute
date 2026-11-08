"""Até quantos pontos os algoritmos exatos do TSP são viáveis nesta máquina.

Primeiro a **força bruta** (testa as N! ordens): mede N = 3, 4, ... até o tempo
previsto do próximo passar do limite e prevê o resto por t = c · N!.
Depois o **Held-Karp**:

Para N = 3, 4, ... roda o Held-Karp numa instância (semente 1), **cada N num
processo separado**, e mede o tempo e a memória de pico real do processo
(``ru_maxrss``). Para quando o N seguinte passaria de um dos limites:

- tempo por instância > ``tempo_max_s`` (padrão 60 s), previsto pelo último
  medido × crescimento observado;
- memória estimada (N · 2ᴺ estados × 9 bytes) > ``mem_max_mb`` (padrão: a RAM
  disponível agora).

    python -m dronectl rotas-limite --saida ../../docs/resultados_rotas/pc

Grava ``limite_exato.json`` e ``limite_exato.md`` com o que foi medido, o
primeiro N inviável por tempo e por memória, e as previsões (t = c · N! e
t = c · N² · 2ᴺ) para os N que não rodam.
"""
from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

from .rotas_bench import _maquina

#: Programa do processo filho: mede um exato numa instância e o próprio pico de RAM.
_FILHO = r'''
import json, resource, sys, time
sys.path.insert(0, {raiz!r})
from dronectl import rotas
n = {n}
pontos = rotas.pontos_aleatorios(n, 1)
base = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
t0 = time.perf_counter()
ordem = rotas.{funcao}(pontos)
dt = time.perf_counter() - t0
pico = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
print(json.dumps({{"n": n, "tempo_s": dt, "pico_mb": pico / 1024, "base_mb": base / 1024,
                  "custo_cm": rotas.custo(ordem, pontos)}}))
'''


def memoria_estimada_mb(n: int) -> float:
    """n · 2ⁿ estados de 8 bytes (custo) + 1 byte (predecessor)."""
    return n * (1 << n) * 9 / 1e6


def _ram_disponivel_mb() -> Optional[float]:
    try:
        for linha in Path('/proc/meminfo').read_text().splitlines():
            if linha.startswith('MemAvailable:'):
                return int(linha.split()[1]) / 1024
    except OSError:
        pass
    return None


def _ram_total_mb() -> Optional[float]:
    try:
        for linha in Path('/proc/meminfo').read_text().splitlines():
            if linha.startswith('MemTotal:'):
                return int(linha.split()[1]) / 1024
    except OSError:
        pass
    return None


def medir(n: int, funcao: str = 'held_karp') -> Dict:
    raiz = str(Path(__file__).resolve().parents[1])
    saida = subprocess.run([sys.executable, '-c', _FILHO.format(raiz=raiz, n=n, funcao=funcao)],
                           capture_output=True, text=True, check=True)
    return json.loads(saida.stdout.strip().splitlines()[-1])


def _fatorial(n: int) -> int:
    f = 1
    for i in range(2, n + 1):
        f *= i
    return f


def limite_forca_bruta(n_max: int, tempo_max_s: float, escrever=print) -> Dict:
    """Mede a força bruta enquanto o próximo N couber em ``tempo_max_s``; prevê o resto."""
    medidos: List[Dict] = []
    for n in range(3, n_max + 1):
        if medidos and medidos[-1]['tempo_s'] * n > tempo_max_s:
            break
        m = medir(n, 'forca_bruta')
        medidos.append(m)
        escrever(f'força bruta N={n:2d}: {m["tempo_s"]:9.3f} s   pico {m["pico_mb"]:6.1f} MB')
    # t = c · N! ajustado nos 3 maiores medidos (abaixo disso domina o custo fixo).
    ultimos = medidos[-3:]
    c = sum(m['tempo_s'] / _fatorial(m['n']) for m in ultimos) / len(ultimos)
    previsao = [{'n': n, 'tempo_s': c * _fatorial(n), 'ordens': _fatorial(n)}
                for n in range(medidos[-1]['n'] + 1, n_max + 1)]
    return {'medidos': medidos, 'previsao': previsao, 'c_ajustado': c,
            'ultimo_medido': medidos[-1]['n'],
            'primeiro_n_acima_do_tempo': next((p['n'] for p in previsao
                                               if p['tempo_s'] > tempo_max_s), None)}


def rodar(n_max: int = 30, tempo_max_s: float = 60.0, mem_max_mb: Optional[float] = None,
          saida: Optional[Path] = None, escrever=print,
          forca_tempo_max_s: Optional[float] = None) -> Dict:
    ram_livre = _ram_disponivel_mb()
    ram_total = _ram_total_mb()
    if mem_max_mb is None:
        mem_max_mb = (ram_livre or 1024.0) * 0.8
    medidos: List[Dict] = []
    parou, motivo = None, ''
    inicio = time.perf_counter()
    forca = limite_forca_bruta(n_max, tempo_max_s if forca_tempo_max_s is None else forca_tempo_max_s,
                               escrever)
    for n in range(3, n_max + 1):
        if len(medidos) >= 2:
            crescimento = medidos[-1]['tempo_s'] / max(medidos[-2]['tempo_s'], 1e-9)
            previsto = medidos[-1]['tempo_s'] * max(crescimento, 2.0)
        else:
            previsto = 0.0
        if memoria_estimada_mb(n) > mem_max_mb:
            parou, motivo = n, f'memória estimada {memoria_estimada_mb(n):.0f} MB > {mem_max_mb:.0f} MB'
            break
        if previsto > tempo_max_s * 2.5:
            parou, motivo = n, f'tempo previsto {previsto:.0f} s (limite {tempo_max_s:.0f} s)'
            break
        m = medir(n)
        medidos.append(m)
        escrever(f'N={n:2d}: {m["tempo_s"]:9.3f} s   pico {m["pico_mb"]:8.1f} MB')
        if m['tempo_s'] > tempo_max_s:
            parou, motivo = n + 1, f'N={n} já levou {m["tempo_s"]:.0f} s (> {tempo_max_s:.0f} s)'
            break
    # t = c · n² · 2ⁿ ajustado nos 5 maiores medidos.
    ultimos = medidos[-5:]
    c = sum(m['tempo_s'] / (m['n'] ** 2 * 2 ** m['n']) for m in ultimos) / len(ultimos)
    previsao = [{'n': n, 'tempo_s': c * n * n * 2 ** n, 'memoria_mb': memoria_estimada_mb(n)}
                for n in range((medidos[-1]['n'] + 1) if medidos else 3, n_max + 1)]
    primeiro_tempo = next((m['n'] for m in medidos if m['tempo_s'] > tempo_max_s),
                          next((p['n'] for p in previsao if p['tempo_s'] > tempo_max_s), None))
    primeiro_mem_livre = next(n for n in range(3, 64) if ram_livre and memoria_estimada_mb(n) > ram_livre)
    primeiro_mem_total = next(n for n in range(3, 64) if ram_total and memoria_estimada_mb(n) > ram_total)
    resumo = {'maquina': _maquina(), 'ram_livre_mb': round(ram_livre or 0), 'ram_total_mb': round(ram_total or 0),
              'tempo_max_s': tempo_max_s, 'mem_max_mb': round(mem_max_mb), 'parou_em': parou,
              'motivo': motivo, 'c_ajustado': c, 'medidos': medidos,
              'previsao': previsao,
              'primeiro_n_acima_do_tempo': primeiro_tempo,
              'primeiro_n_acima_da_ram_livre': primeiro_mem_livre,
              'primeiro_n_acima_da_ram_total': primeiro_mem_total,
              'pi_zero_ram_mb': 512, 'forca_bruta': forca,
              'primeiro_n_acima_da_metade_ram_pi': next(n for n in range(3, 64)
                                                         if memoria_estimada_mb(n) > 256),
              'tempo_total_s': round(time.perf_counter() - inicio, 1)}
    if saida is not None:
        saida.mkdir(parents=True, exist_ok=True)
        (saida / 'limite_exato.json').write_text(json.dumps(resumo, indent=2, ensure_ascii=False),
                                                 encoding='utf-8')
        (saida / 'limite_exato.md').write_text(markdown(resumo), encoding='utf-8')
    return resumo


def _tempo(s: float) -> str:
    if s < 1:
        return f'{s * 1000:.1f} ms'
    if s < 120:
        return f'{s:.1f} s'
    if s < 7200:
        return f'{s / 60:.0f} min'
    if s < 2 * 86400:
        return f'{s / 3600:.1f} h'
    if s < 2 * 365 * 86400:
        return f'{s / 86400:.0f} dias'
    return f'{s / (365.25 * 86400):.3g} anos'


def _mem(mb: float) -> str:
    return f'{mb:.1f} MB' if mb < 1000 else f'{mb / 1000:.1f} GB'


def markdown(r: Dict) -> str:
    m = r['maquina']
    fb = r['forca_bruta']
    out = [f"Máquina: {m.get('placa') or m.get('cpu', '?')} · Python {m['python']} · RAM total "
           f"{_mem(r['ram_total_mb'])}, livre {_mem(r['ram_livre_mb'])} · limite de tempo "
           f"{r['tempo_max_s']:.0f} s por instância",
           '',
           '## Força bruta (N! ordens)',
           '',
           '| N | ordens testadas | tempo | pico de RAM do processo |',
           '|---|---|---|---|']
    for x in fb['medidos']:
        out.append(f"| {x['n']} | {_fatorial(x['n']):,} | {_tempo(x['tempo_s'])} | "
                   f"{_mem(x['pico_mb'])} |".replace(',', '.'))
    out += ['', 'Sem rodar (previsão t = c·N! ajustada nos 3 maiores medidos):', '',
            '| N | ordens | tempo previsto |', '|---|---|---|']
    for p in fb['previsao']:
        out.append(f"| {p['n']} | {p['ordens']:.2e} | {_tempo(p['tempo_s'])} |")
    out += ['', f"- Primeiro N acima de {r['tempo_max_s']:.0f} s por instância: "
            f"**{fb['primeiro_n_acima_do_tempo']}**",
            '',
            '## Held-Karp (programação dinâmica, N²·2ᴺ)',
            '',
            f"Parou em N = {r['parou_em']}: {r['motivo']}.",
           '',
           '| N | tempo medido | pico de RAM do processo | memória do Held-Karp (N·2ᴺ·9 B) |',
           '|---|---|---|---|']
    for x in r['medidos']:
        out.append(f"| {x['n']} | {_tempo(x['tempo_s'])} | {_mem(x['pico_mb'])} | "
                   f"{_mem(memoria_estimada_mb(x['n']))} |")
    out += ['', 'Sem rodar (previsão t = c·N²·2ᴺ ajustada nos 5 maiores medidos):', '',
            '| N | tempo previsto | memória necessária | cabe na RAM livre? | cabe na RAM total? |',
            '|---|---|---|---|---|']
    for p in r['previsao']:
        out.append(f"| {p['n']} | {_tempo(p['tempo_s'])} | {_mem(p['memoria_mb'])} | "
                   f"{'sim' if p['memoria_mb'] <= r['ram_livre_mb'] else 'não'} | "
                   f"{'sim' if p['memoria_mb'] <= r['ram_total_mb'] else 'não'} |")
    out += ['',
            f"- Primeiro N acima de {r['tempo_max_s']:.0f} s por instância: **{r['primeiro_n_acima_do_tempo']}**",
            f"- Primeiro N que não cabe na RAM livre ({_mem(r['ram_livre_mb'])}): "
            f"**{r['primeiro_n_acima_da_ram_livre']}**",
            f"- Primeiro N que não cabe nem na RAM total ({_mem(r['ram_total_mb'])}): "
            f"**{r['primeiro_n_acima_da_ram_total']}**",
            f"- Raspberry Pi Zero (512 MB): passa da metade da RAM a partir de "
            f"**{r['primeiro_n_acima_da_metade_ram_pi']}** pontos"]
    return '\n'.join(out) + '\n'
