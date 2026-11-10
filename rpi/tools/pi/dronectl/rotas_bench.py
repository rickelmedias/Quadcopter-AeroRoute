"""Benchmark dos algoritmos de rota 3D (aleatório, guloso, 2-opt, 2-opt
multipartida, genético, força bruta e exato).

Para cada N (pontos além da origem) e cada instância (semente), gera pontos
(x, y, z), roda os algoritmos e registra comprimento do circuito, tempo de
cálculo e o gap: quanto a rota é mais longa que o
**ótimo global** (Held-Karp) — ou, quando o exato não cabe no tempo ou na
memória, que o **melhor encontrado** entre os algoritmos.

Os dois exatos saem do comparativo quando o tempo previsto por instância passa
do limite e não voltam mais: a força bruta (n! ordens) por volta de 10 pontos e
o Held-Karp (n² 2ⁿ) por volta de 22; daí em diante sobram as heurísticas.

Também registra, por instância, os ótimos locais do 2-opt multipartida
(quantos diferentes, o pior e o melhor) e, para alguns N, a curva de
convergência do genético.

    python -m dronectl rotas-bench                         # N = 3..30, 10 instâncias
    python -m dronectl rotas-bench --saida resultados/pc

Grava ``resultados.csv``, ``otimos_locais.csv``, ``resumo.md``,
``resumo.json`` e ``convergencia.json``. A **assinatura** (SHA-256 dos pontos e
das rotas de todos os algoritmos menos os exatos) tem de ser igual no PC e na
Raspberry Pi com os mesmos parâmetros: prova de que o algoritmo que roda na Pi
é o do benchmark.
"""
from __future__ import annotations

import csv
import hashlib
import json
import platform
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Dict, List, Optional

from . import rotas

#: Ordem das colunas.
NOMES = ['aleatorio', 'guloso', 'guloso+2opt', '2opt-multi', 'genetico', 'forca-bruta', 'exato']
#: Os que dão o ótimo global (ficam fora da assinatura: na Pi rodam até N menores).
EXATOS = ('forca-bruta', 'exato')
#: Crescimento do tempo do Held-Karp por ponto a mais (medido: ~2,2x).
CRESCIMENTO_EXATO = 2.3
#: Partidas do 2-opt multipartida.
PARTIDAS_2OPT = 20
#: N em que a convergência do genético é guardada (instância 1).
N_CONVERGENCIA = (10, 15, 20, 25, 30)


def _maquina() -> Dict[str, str]:
    info = {'python': sys.version.split()[0], 'plataforma': platform.platform(),
            'processador': platform.machine()}
    try:
        for linha in Path('/proc/cpuinfo').read_text().splitlines():
            chave, _, valor = linha.partition(':')
            if chave.strip().lower() in ('model name', 'hardware') and valor.strip():
                info.setdefault('cpu', valor.strip())
        modelo = Path('/proc/device-tree/model')
        if modelo.exists():
            info['placa'] = modelo.read_text(errors='ignore').strip('\x00 \n')
    except OSError:
        pass
    return info


def _memoria_exato_mb(n: int) -> float:
    """Memória do Held-Karp: n · 2ⁿ estados de 8 bytes + 1 byte do predecessor."""
    return n * (1 << n) * 9 / 1e6


def _rodar_algoritmo(nome: str, pontos) -> Dict:
    t0 = time.perf_counter()
    extra: Dict = {}
    if nome == 'genetico':
        ordem, info = rotas.genetico(pontos)
        extra = {'geracoes': info['geracoes'], 'historico': info['historico']}
    elif nome == '2opt-multi':
        ordem, otimos = rotas.dois_opt_multi(pontos, PARTIDAS_2OPT)
        extra = {'otimos_locais': otimos}
    else:
        ordem = rotas.ALGORITMOS[nome](pontos)
    dt = time.perf_counter() - t0
    return {'ordem': ordem, 'custo': rotas.custo(ordem, pontos), 'tempo_s': dt, **extra}


def rodar(n_min: int = 3, n_max: int = 30, instancias: int = 10, exato_max_s: float = 10.0,
          exato_max_mb: float = 600.0, saida: Optional[Path] = None,
          escrever=print, forca_max_s: float = 10.0) -> Dict:
    """Roda o benchmark e devolve o resumo (também grava em ``saida``)."""
    linhas: List[Dict] = []
    locais: List[Dict] = []
    convergencia: Dict[str, Dict] = {}
    assinatura = hashlib.sha256()
    tempo_exato: Dict[int, float] = {}
    exato_parou = False
    tempo_forca: Dict[int, float] = {}
    forca_parou = False
    inicio = time.perf_counter()
    for n in range(n_min, n_max + 1):
        # O exato entra se o tempo previsto (pelo N anterior) e a memória cabem;
        # depois que sai, não volta (sem a medida do N anterior não há previsão).
        previsto = tempo_exato.get(n - 1, 0.0) * CRESCIMENTO_EXATO
        roda_exato = (not exato_parou and previsto <= exato_max_s and
                      _memoria_exato_mb(n) <= exato_max_mb)
        exato_parou = not roda_exato
        # Força bruta: n vezes o tempo do N anterior (n! / (n-1)! = n).
        roda_forca = not forca_parou and tempo_forca.get(n - 1, 0.0) * n <= forca_max_s
        forca_parou = not roda_forca
        tempos_exato, tempos_forca = [], []
        for semente in range(1, instancias + 1):
            pontos = rotas.pontos_aleatorios(n, semente)
            assinatura.update(repr((n, semente, pontos)).encode())
            resultado = {nome: _rodar_algoritmo(nome, pontos) for nome in NOMES
                         if (nome != 'exato' or roda_exato) and (nome != 'forca-bruta' or roda_forca)}
            if 'exato' in resultado:
                tempos_exato.append(resultado['exato']['tempo_s'])
            if 'forca-bruta' in resultado:
                tempos_forca.append(resultado['forca-bruta']['tempo_s'])
                if 'exato' in resultado and abs(resultado['forca-bruta']['custo'] -
                                                resultado['exato']['custo']) > 1e-6:
                    raise RuntimeError(f'N={n} semente {semente}: força bruta e Held-Karp divergem')
            for nome, r in resultado.items():
                if nome not in EXATOS:
                    assinatura.update(repr((nome, r['ordem'])).encode())
            otimo = 'exato' in resultado
            referencia = (resultado['exato']['custo'] if otimo
                          else min(r['custo'] for r in resultado.values()))
            for nome, r in resultado.items():
                linhas.append({'n': n, 'instancia': semente, 'algoritmo': nome,
                               'custo_cm': round(r['custo'], 3),
                               'gap_pct': round(100.0 * (r['custo'] / referencia - 1.0), 4),
                               'referencia': 'otimo' if otimo else 'melhor',
                               'tempo_ms': round(r['tempo_s'] * 1000.0, 3),
                               'geracoes': r.get('geracoes', ''),
                               'ordem': ' '.join(map(str, r['ordem']))})
            ot = resultado['2opt-multi']['otimos_locais']
            locais.append({'n': n, 'instancia': semente, 'referencia': 'otimo' if otimo else 'melhor',
                           'partidas': len(ot),
                           'distintos': len({round(c, 3) for c in ot}),
                           'acharam_referencia': sum(1 for c in ot if c <= referencia + 1e-6),
                           'melhor_gap_pct': round(100 * (min(ot) / referencia - 1), 3),
                           'medio_gap_pct': round(100 * (sum(ot) / len(ot) / referencia - 1), 3),
                           'pior_gap_pct': round(100 * (max(ot) / referencia - 1), 3)})
            if n in N_CONVERGENCIA and semente == 1:
                convergencia[str(n)] = {'referencia_cm': referencia,
                                        'referencia': 'otimo' if otimo else 'melhor',
                                        'genetico': resultado['genetico']['historico'],
                                        'guloso_cm': resultado['guloso']['custo'],
                                        'guloso+2opt_cm': resultado['guloso+2opt']['custo'],
                                        'aleatorio_cm': resultado['aleatorio']['custo']}
        if tempos_exato:
            tempo_exato[n] = max(tempos_exato)
        if tempos_forca:
            tempo_forca[n] = max(tempos_forca)
        escrever(f'N={n:2d}: {"força bruta " if roda_forca else "            "}'
                 f'{"exato " if roda_exato else "      "}'
                 f'{time.perf_counter() - inicio:6.1f} s acumulados')
    resumo = {'maquina': _maquina(), 'parametros_ga': asdict(rotas.ParamsGA()),
              'partidas_2opt': PARTIDAS_2OPT, 'instancias_por_n': instancias,
              'pontos': {'xy_cm': 600, 'z_cm': [30, 150], 'origem': list(rotas.ORIGEM)},
              'limites_exato': {'tempo_previsto_s': exato_max_s, 'memoria_mb': exato_max_mb,
                                'forca_bruta_tempo_previsto_s': forca_max_s},
              'execucoes': len(linhas), 'instancias_total': len(locais),
              'tempo_total_s': round(time.perf_counter() - inicio, 1),
              'assinatura_sha256': assinatura.hexdigest(),
              'tabela': _resumir(linhas), 'otimos_locais': _resumir_locais(locais)}
    if saida is not None:
        saida.mkdir(parents=True, exist_ok=True)
        with open(saida / 'resultados.csv', 'w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=list(linhas[0]))
            w.writeheader()
            w.writerows(linhas)
        with open(saida / 'otimos_locais.csv', 'w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=list(locais[0]))
            w.writeheader()
            w.writerows(locais)
        (saida / 'convergencia.json').write_text(json.dumps(convergencia), encoding='utf-8')
        (saida / 'resumo.json').write_text(json.dumps(resumo, indent=2, ensure_ascii=False),
                                           encoding='utf-8')
        (saida / 'resumo.md').write_text(tabela_markdown(resumo), encoding='utf-8')
    return resumo


def _resumir(linhas: List[Dict]) -> List[Dict]:
    """Por N e algoritmo: gap médio e máximo, quantas vezes achou a referência, tempos."""
    grupos: Dict = {}
    for l in linhas:
        grupos.setdefault((l['n'], l['algoritmo']), []).append(l)
    tabela = []
    for (n, nome), ls in sorted(grupos.items(), key=lambda kv: (kv[0][0], NOMES.index(kv[0][1]))):
        tabela.append({'n': n, 'algoritmo': nome, 'referencia': ls[0]['referencia'],
                       'gap_medio_pct': round(sum(l['gap_pct'] for l in ls) / len(ls), 2),
                       'gap_max_pct': round(max(l['gap_pct'] for l in ls), 2),
                       'acertos': sum(1 for l in ls if l['gap_pct'] < 1e-6),
                       'instancias': len(ls),
                       'tempo_medio_ms': round(sum(l['tempo_ms'] for l in ls) / len(ls), 2),
                       'tempo_max_ms': round(max(l['tempo_ms'] for l in ls), 2)})
    return tabela


def _resumir_locais(locais: List[Dict]) -> List[Dict]:
    grupos: Dict = {}
    for l in locais:
        grupos.setdefault(l['n'], []).append(l)
    out = []
    for n, ls in sorted(grupos.items()):
        out.append({'n': n, 'referencia': ls[0]['referencia'],
                    'distintos_medio': round(sum(l['distintos'] for l in ls) / len(ls), 1),
                    'acharam_referencia_pct': round(100 * sum(l['acharam_referencia'] for l in ls)
                                                    / sum(l['partidas'] for l in ls), 1),
                    'medio_gap_pct': round(sum(l['medio_gap_pct'] for l in ls) / len(ls), 2),
                    'pior_gap_pct': round(max(l['pior_gap_pct'] for l in ls), 2)})
    return out


def tabela_markdown(resumo: Dict) -> str:
    """Tabelas: gap médio (máx) por N e algoritmo, tempos, acertos e ótimos locais."""
    por_n: Dict[int, Dict[str, Dict]] = {}
    for t in resumo['tabela']:
        por_n.setdefault(t['n'], {})[t['algoritmo']] = t
    m, p = resumo['maquina'], resumo['parametros_ga']
    out = [f"Máquina: {m.get('placa') or m.get('cpu', '?')} · Python {m['python']} · "
           f"{resumo['instancias_por_n']} instâncias por N ({resumo['instancias_total']} "
           f"instâncias, {resumo['execucoes']} execuções) · total {resumo['tempo_total_s']} s",
           '',
           f"Pontos: x, y em {resumo['pontos']['xy_cm']} cm × {resumo['pontos']['xy_cm']} cm, "
           f"z de {resumo['pontos']['z_cm'][0]} a {resumo['pontos']['z_cm'][1]} cm; origem "
           f"{tuple(resumo['pontos']['origem'])}. Genético: população {p['populacao']}, até "
           f"{p['geracoes']} gerações (para após {p['paciencia']} sem melhora), torneio "
           f"{p['torneio']}, OX {p['taxa_cruzamento']}, inversão {p['taxa_mutacao']}, elitismo "
           f"{p['elitismo']}, semente {p['semente']}. 2-opt multipartida: "
           f"{resumo['partidas_2opt']} partidas aleatórias.",
           '',
           f"Assinatura: `{resumo['assinatura_sha256']}`",
           '',
           '### Gap médio (máximo) em %, por algoritmo',
           '',
           '| N | ref. | ' + ' | '.join(NOMES) + ' |',
           '|---|---|' + '---|' * len(NOMES)]
    for n in sorted(por_n):
        linha = por_n[n]
        ref = next(iter(linha.values()))['referencia']
        cel = []
        for a in NOMES:
            t = linha.get(a)
            cel.append('—' if t is None else f"{t['gap_medio_pct']:.1f} ({t['gap_max_pct']:.1f})")
        out.append(f'| {n} | {ref} | ' + ' | '.join(cel) + ' |')
    out += ['', '### Tempo médio de cálculo (ms)', '',
            '| N | ' + ' | '.join(NOMES) + ' |', '|---|' + '---|' * len(NOMES)]
    for n in sorted(por_n):
        cel = [('—' if por_n[n].get(a) is None else f"{por_n[n][a]['tempo_medio_ms']:.1f}")
               for a in NOMES]
        out.append(f'| {n} | ' + ' | '.join(cel) + ' |')
    out += ['', '### Acertos da referência (instâncias em que o algoritmo achou o ótimo/melhor)', '',
            '| N | ' + ' | '.join(NOMES) + ' |', '|---|' + '---|' * len(NOMES)]
    for n in sorted(por_n):
        cel = [('—' if por_n[n].get(a) is None else
                f"{por_n[n][a]['acertos']}/{por_n[n][a]['instancias']}") for a in NOMES]
        out.append(f'| {n} | ' + ' | '.join(cel) + ' |')
    out += ['', f"### Ótimos locais do 2-opt ({resumo['partidas_2opt']} partidas aleatórias "
            'por instância)', '',
            '| N | ref. | ótimos locais distintos (média) | partidas que acharam a ref. | '
            'gap médio de um ótimo local | pior ótimo local |',
            '|---|---|---|---|---|---|']
    for l in resumo['otimos_locais']:
        out.append(f"| {l['n']} | {l['referencia']} | {l['distintos_medio']} | "
                   f"{l['acharam_referencia_pct']} % | {l['medio_gap_pct']} % | {l['pior_gap_pct']} % |")
    out += ['', 'gap = quanto o circuito é mais longo que a referência: o ótimo global do '
            'Held-Karp (ref. = otimo) ou, sem ele, o melhor dos algoritmos (ref. = melhor). '
            'Onde a força bruta roda, ela confirma o mesmo ótimo do Held-Karp.']
    return '\n'.join(out) + '\n'
