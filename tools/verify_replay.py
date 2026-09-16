#!/usr/bin/env python3
"""Verifica que o replay reproduz uma captura E que ele consegue falhar.

São duas metades, e a segunda é a que importa. Um verificador que sempre
informa sucesso é pior que nenhum: transforma "não verificamos" em
"verificamos e estava tudo certo". Por isso este script gera uma captura
sintética cujos STEP trazem as saídas verdadeiras dos estimadores, confirma
que o replay a reproduz exatamente, depois danifica a captura de vários
jeitos específicos e exige que o replay pegue cada um.

Os danos miram as distinções que o replay deve defender:

  - um valor de saída gravado               -> verificação numérica
  - o indicador de aceite do FUSED          -> verificação de decisão
  - o indicador de aceite do gate do filtro -> verificação de decisão
  - se a IMU estava online                  -> verificação de decisão (gravado, não deduzido)
  - um contador de amostra usada            -> verificação de pareamento
  - um registro BNO inteiro removido        -> verificação de pareamento, como uma perda real
  - NaN/infinito, perda por disputa, observação faltando ou com relógio errado

Também confere dois casos legítimos que precisam reproduzir (preempção e
frame não observado), o diagnóstico de perda do replay e a rejeição de
capturas vazias ou corrompidas por leitura em modo texto.

Usa só a biblioteca padrão, como tools/build.py. Precisa de gcc no PATH.

    python3 tools/verify_replay.py

Código de saída: 0 se tudo passou; 1 caso contrário.
"""
import importlib.util
import pathlib
import struct
import subprocess
import sys
import tempfile

#: Raiz do projeto.
ROOT = pathlib.Path(__file__).resolve().parent.parent
#: Fontes de estimação ligados ao gerador e ao replay.
ESTIMATOR_SRC = ['App/Src/disp_estimator.c', 'App/Src/flow_ekf.c', 'App/Src/attitude.c']
#: Opções de compilação (com ASan/UBSan).
CFLAGS = ['-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
          '-fno-omit-frame-pointer', '-IApp/Inc']

# Posições dentro da carga do STEP, pela ordem dos campos em capture_step().
STEP_FLAGS = 20      # depois de 4*u32 + f32
STEP_USED_ROT = 4
STEP_EKF_PX = 45     # flags(1) + raw(8) + fused(8) + ekf_v(8)

FLAG_FUSED, FLAG_EKF, FLAG_IMU = 0x01, 0x02, 0x04


def load_decoder():
    """Importa tools/decode_capture.py como módulo.

    Returns:
        O módulo do decoder (usado como ``dc`` no resto do script).
    """
    spec = importlib.util.spec_from_file_location('dc', ROOT / 'tools/decode_capture.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def build(out, source):
    """Compila um programa C junto com os fontes de estimação.

    Encerra o script se a compilação falhar.

    Args:
        out: caminho do executável.
        source: fonte principal, relativo à raiz.
    """
    cmd = ['gcc', *CFLAGS, source, *ESTIMATOR_SRC, '-lm', '-o', str(out)]
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    if r.returncode:
        print(r.stderr)
        sys.exit(f'failed to build {source}')


def walk(dc, b):
    """Percorre os registros válidos de uma captura.

    Args:
        dc: módulo do decoder.
        b: bytes da captura.

    Yields:
        Tuplas (posição, tipo, início da carga, tamanho da carga).
    """
    pos = 0
    while pos + dc.OVERHEAD <= len(b):
        if b[pos] != dc.SYNC:
            pos += 1
            continue
        rtype, length = b[pos + 1], b[pos + 2]
        size = length + dc.OVERHEAD
        if not dc.plausible(rtype, length) or pos + size > len(b):
            pos += 1
            continue
        if dc.crc8(b[pos:pos + size - 1]) != b[pos + size - 1]:
            pos += 1
            continue
        yield pos, rtype, pos + dc.HEADER_BYTES, length
        pos += size


def records_of(dc, b, rtype):
    """Lista os registros de um tipo.

    Args:
        dc: módulo do decoder.
        b: bytes da captura.
        rtype: tipo desejado.

    Returns:
        Lista de tuplas no formato de walk().
    """
    return [x for x in walk(dc, b) if x[1] == rtype]


def reseal(dc, b, pos, length):
    """Recalcula o checksum de um registro alterado.

    Assim o replay rejeita pelo conteúdo, e não pelo enquadramento.

    Args:
        dc: módulo do decoder.
        b: bytearray da captura (alterado no lugar).
        pos: posição do registro.
        length: tamanho da carga.
    """
    end = pos + length + dc.OVERHEAD - 1
    b[end] = dc.crc8(bytes(b[pos:end]))


def main():
    """Executa a verificação completa.

    Returns:
        0 se tudo passou; 1 se alguma sabotagem não foi detectada. Outras
        falhas encerram o script com mensagem.
    """
    dc = load_decoder()
    tmp = pathlib.Path(tempfile.mkdtemp(prefix='replay-verify-'))
    mksyn, replay = tmp / 'mksyn', tmp / 'replay'

    build(mksyn, 'tools/make_synthetic_capture.c')
    build(replay, 'tools/replay.c')

    clean = tmp / 'synthetic.bin'
    r = subprocess.run([str(mksyn)], capture_output=True, cwd=ROOT)
    if r.returncode:
        sys.exit('synthetic capture generation failed')
    clean.write_bytes(r.stdout)
    print(r.stderr.decode().strip())

    def run(path):
        """Roda o replay sobre uma captura e devolve o resultado do processo."""
        return subprocess.run([str(replay), str(path)], capture_output=True, text=True)

    out = run(clean)
    if out.returncode != 0:
        print(out.stdout)
        sys.exit('FAIL: replay could not reproduce an undamaged capture')
    print(f'\nclean capture: reproduced ({clean.stat().st_size} bytes)')

    # O escritor em C e o leitor em Python são mantidos separadamente e se
    # afastam em silêncio - um campo acrescentado num e não no outro ainda gera
    # um arquivo legível, só que com tudo depois dele deslocado. Decodificar a
    # mesma captura aqui pega isso na próxima execução, e não na próxima
    # sessão de bancada.
    dec = subprocess.run([sys.executable, 'tools/decode_capture.py', str(clean),
                          '--out-dir', str(tmp / 'decoded')],
                         capture_output=True, text=True, cwd=ROOT)
    if dec.returncode != 0:
        print(dec.stdout, dec.stderr)
        sys.exit('FAIL: the Python decoder could not read the capture')
    for must in ('sequence gaps: 0', 'records failing checksum: 0',
                 'false sync bytes rejected on shape: 0'):
        if must not in dec.stdout:
            print(dec.stdout)
            sys.exit(f'FAIL: decoder reported a problem - expected "{must}"')
    counts = dict(step=0, bno=0, mtf=0, observation=0)
    for line in dec.stdout.splitlines():
        for key in counts:
            if line.endswith(' records') and f'/{key}.csv:' in line:
                counts[key] = int(line.split(':')[-1].split()[0])
    # O gerador em C informou o que produziu; o decoder precisa achar o mesmo.
    if counts != dict(step=398, mtf=400, bno=1200, observation=400):
        print(dec.stdout)
        sys.exit(f'FAIL: decoder found {counts}, expected 398 step / 400 mtf / 1200 bno')
    print(f'python decoder agrees: {counts["mtf"]} mtf, {counts["bno"]} bno, '
          f'{counts["step"]} step\n')

    blob = clean.read_bytes()
    failures = []

    def check(name, mutate):
        """Aplica uma sabotagem e exige que o replay falhe.

        Args:
            name: nome da sabotagem (também nome do arquivo).
            mutate: função (dc, bytearray) que danifica a captura.
        """
        b = bytearray(blob)
        mutate(dc, b)
        path = tmp / f'{name}.bin'
        path.write_bytes(bytes(b))
        res = run(path)
        caught = res.returncode != 0
        print(f'  {name:24} {"caught" if caught else "MISSED"}')
        if not caught:
            failures.append(name)

    def step_flag(index, bit):
        """Sabotagem: inverte um indicador de um STEP."""
        def mutate(dc, b):
            pos, _, pay, length = records_of(dc, b, dc.TYPE_STEP)[index]
            b[pay + STEP_FLAGS] ^= bit
            reseal(dc, b, pos, length)
        return mutate

    def step_value(index):
        """Sabotagem: troca a posição X do EKF gravada num STEP por 99."""
        def mutate(dc, b):
            pos, _, pay, length = records_of(dc, b, dc.TYPE_STEP)[index]
            struct.pack_into('<f', b, pay + STEP_EKF_PX, 99.0)
            reseal(dc, b, pos, length)
        return mutate

    def nonfinite(value):
        """Sabotagem: grava NaN ou infinito num valor de STEP."""
        def mutate(dc, b):
            pos, _, pay, length = records_of(dc, b, dc.TYPE_STEP)[200]
            struct.pack_into('<f', b, pay + STEP_EKF_PX, value)
            reseal(dc, b, pos, length)
        return mutate

    def contention_loss(dc, b):
        """Sabotagem: um DIAG informa perda por disputa (sem buraco na sequência)."""
        pos, _, pay, length = records_of(dc, b, dc.TYPE_DIAG)[-1]
        struct.pack_into('<I', b, pay + 8, 1)
        reseal(dc, b, pos, length)

    def drop_observation(dc, b):
        """Sabotagem: remove um registro OBSERVATION."""
        pos, _, _, length = records_of(dc, b, dc.TYPE_OBSERVATION)[250]
        del b[pos:pos + length + dc.OVERHEAD]

    def change_observation_clock(dc, b):
        """Sabotagem: zera o carimbo anterior de uma OBSERVATION."""
        pos, _, pay, length = records_of(dc, b, dc.TYPE_OBSERVATION)[251]
        struct.pack_into('<I', b, pay + 4, 0)
        reseal(dc, b, pos, length)

    def resequence(b):
        """Renumera a sequência de todos os registros, para não haver buracos."""
        for seq, (pos, _, _, length) in enumerate(walk(dc, b)):
            struct.pack_into('<H', b, pos + 3, seq & 65535)
            reseal(dc, b, pos, length)

    # Intercalação legítima: eventos novos dos sensores chegam depois da
    # fotografia lida, mas antes de o STEP ser gravado. Os IDs antigos
    # continuam existindo.
    b = bytearray(blob)
    items = list(walk(dc, b))
    step_pos = records_of(dc, b, dc.TYPE_STEP)[100][0]
    following = [x for x in items if x[0] > step_pos and x[1] in (dc.TYPE_BNO, dc.TYPE_MTF)][:4]
    moved = b''.join(b[p:p+n+dc.OVERHEAD] for p, _, _, n in following)
    for p, _, _, n in reversed(following):
        del b[p:p+n+dc.OVERHEAD]
    b[step_pos:step_pos] = moved
    resequence(b)
    path = tmp / 'legal-preemption.bin'
    path.write_bytes(b)
    result = run(path)
    if result.returncode:
        print(result.stdout, result.stderr)
        sys.exit('FAIL: legal sensor preemption must reproduce')
    print('legal preemption: reproduced using recorded sample IDs')

    # Uma rajada tem um frame que o parser capturou mas a task nunca observou.
    # Os IDs são remapeados para números pares, com um frame ímpar inserido no
    # meio.
    b = bytearray(blob)
    for p, typ, pay, length in list(walk(dc, b)):
        off = 17 if typ == dc.TYPE_MTF else 0
        if typ in (dc.TYPE_MTF, dc.TYPE_STEP, dc.TYPE_OBSERVATION):
            old = struct.unpack_from('<I', b, pay + off)[0]
            struct.pack_into('<I', b, pay + off, old*2)
            reseal(dc, b, p, length)
    p, _, pay, length = records_of(dc, b, dc.TYPE_MTF)[100]
    extra = bytearray(b[p:p+length+dc.OVERHEAD])
    struct.pack_into('<I', extra, dc.HEADER_BYTES+17, 203)
    old_time = struct.unpack_from('<I', extra, dc.HEADER_BYTES+13)[0]
    struct.pack_into('<I', extra, dc.HEADER_BYTES+13, old_time+5)
    extra[-1] = dc.crc8(extra[:-1])
    insert_at = records_of(dc, b, dc.TYPE_STEP)[99][0]
    b[insert_at:insert_at] = extra
    resequence(b)
    path = tmp / 'unobserved-burst-frame.bin'
    path.write_bytes(b)
    result = run(path)
    if result.returncode:
        print(result.stdout, result.stderr)
        sys.exit('FAIL: extra unobserved MTF frame must not change dt')
    print('unobserved burst frame: reproduced without changing dt')

    def step_counter(index):
        """Sabotagem: troca o ID de rotação usado num STEP."""
        def mutate(dc, b):
            pos, _, pay, length = records_of(dc, b, dc.TYPE_STEP)[index]
            struct.pack_into('<I', b, pay + STEP_USED_ROT, 12345)
            reseal(dc, b, pos, length)
        return mutate

    def drop_bno(index):
        """Sabotagem: remove um registro BNO."""
        def mutate(dc, b):
            pos, _, _, length = records_of(dc, b, dc.TYPE_BNO)[index]
            del b[pos:pos + length + dc.OVERHEAD]
        return mutate

    def session_scale(dc, b):
        """Sabotagem: altera a escala X gravada na SESSION."""
        pos, _, pay, length = records_of(dc, b, dc.TYPE_SESSION)[0]
        name_len = b[pay + 1]
        struct.pack_into('<f', b, pay + 2 + name_len, 0.5)
        reseal(dc, b, pos, length)

    print('sabotage - every one of these must be caught:')
    check('recorded-value', step_value(200))
    check('fused-decision', step_flag(200, FLAG_FUSED))
    check('gate-decision', step_flag(200, FLAG_EKF))
    check('imu-online-flag', step_flag(160, FLAG_IMU))
    check('used-sample-counter', step_counter(200))
    check('dropped-bno-record', drop_bno(300))
    check('session-calibration', session_scale)
    check('recorded-nan', nonfinite(float('nan')))
    check('recorded-infinity', nonfinite(float('inf')))
    check('contention-without-gap', contention_loss)
    check('missing-observation', drop_observation)
    check('wrong-observation-clock', change_observation_clock)

    # Uma captura que perde um registro MTF referenciado precisa manter um
    # relatório útil do trecho anterior, em vez de parar sem contexto.
    b = bytearray(blob)
    p, _, _, length = records_of(dc, b, dc.TYPE_MTF)[100]
    del b[p:p+length+dc.OVERHEAD]
    path = tmp / 'missing-mtf-with-prefix.bin'
    path.write_bytes(b)
    result = run(path)
    for expected in ('STOPPED: offset=', 'seq=', 't_ms=', 'reason=',
                     'verified prefix steps: 99 ', 'steps replayed:', 'INCOMPLETE'):
        if result.returncode == 0 or expected not in result.stdout:
            print(result.stdout, result.stderr)
            sys.exit(f'FAIL: missing loss diagnostic {expected}')
    print('loss diagnostic: reports failing record and 99 verified prefix steps')

    # Uma gravação feita em modo texto troca o A5 por '?'; o decoder precisa
    # falhar, e não produzir um conjunto vazio de CSVs com cara de sucesso.
    for name, data in [('empty', b''), ('ascii-damaged', blob.decode('ascii', 'replace').encode('ascii', 'replace'))]:
        path = tmp / (name + '.bin')
        path.write_bytes(data)
        decoded = subprocess.run([sys.executable, 'tools/decode_capture.py', str(path),
                                  '--out-dir', str(tmp / name)], capture_output=True, text=True, cwd=ROOT)
        if decoded.returncode == 0 or 'no decodable records' not in decoded.stderr:
            sys.exit(f'FAIL: decoder accepted {name} capture')
    print('empty and text-corrupted captures: rejected')

    print()
    if failures:
        print('FAIL: replay did not catch ' + ', '.join(failures))
        return 1
    print('replay reproduces a clean capture and catches every sabotage')
    return 0


if __name__ == '__main__':
    sys.exit(main())
