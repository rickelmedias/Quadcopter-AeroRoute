#!/usr/bin/env python3
"""Decodifica uma captura binária gerada por App/Src/capture.c.

Lê a sequência de registros (layout em App/Inc/capture.h) e grava um CSV por
tipo de registro, além de um resumo. Depois de uma perda, volta a se
sincronizar pelo byte de sincronismo e informa os buracos de sequência, porque
o replay precisa distinguir dados faltando de um período parado.

Usa só a biblioteca padrão, como tools/build.py.

    python3 tools/decode_capture.py capture.bin --out-dir replay/

Código de saída: 0 se decodificou algo com SESSION; 1 se o arquivo está vazio,
não tem registros válidos (por exemplo, gravado por um leitor de texto) ou não
tem SESSION.
"""
import argparse
import csv
import struct
import sys
from pathlib import Path

#: Byte que abre todo registro.
SYNC = 0xA5
#: Bytes do cabeçalho: sync, tipo, tamanho, seq (2) e t_ms (4).
HEADER_BYTES = 9
#: Bytes do checksum.
CRC_BYTES = 1
#: Bytes fixos de todo registro.
OVERHEAD = HEADER_BYTES + CRC_BYTES

#: Versão de formato que este decoder sabe ler.
SUPPORTED_VERSION = 3

TYPE_SESSION, TYPE_MTF, TYPE_BNO, TYPE_STEP, TYPE_DIAG = 1, 2, 3, 4, 5
TYPE_OBSERVATION = 6
#: Nome de cada tipo, usado também como nome do CSV.
TYPE_NAMES = {TYPE_SESSION: 'session', TYPE_MTF: 'mtf', TYPE_BNO: 'bno',
              TYPE_STEP: 'step', TYPE_DIAG: 'diag', TYPE_OBSERVATION: 'observation'}
#: Nome de cada tipo de relatório do BNO.
BNO_REPORTS = {1: 'rotation', 2: 'gyro', 3: 'accel'}

# Tamanho da carga por tipo: exato onde o registro é fixo, mínimo onde ele
# termina numa string com prefixo de tamanho. É conferido antes de aceitar um
# registro, então um falso byte de sincronismo cujo checksum passe por acaso -
# uma chance em 256 - ainda é rejeitado por ter o formato errado.
PAYLOAD_EXACT = {
    TYPE_OBSERVATION: 9,
    TYPE_MTF: 21,   # i16 i16 u32 u8 u8 u8 u8 u8 u32 u32
    TYPE_BNO: 35,   # u8 u8 u8 4*f32 u32 u64 u32
    TYPE_STEP: 53,  # 4*u32 f32 u8 8*f32
}
PAYLOAD_MIN = {
    TYPE_SESSION: 50,  # u8 + str(>=1) + 6*f32 + 2*u32 + 3*f32
    TYPE_DIAG: 29,     # 7*u32 + str(>=1)
}


def crc8(data):
    """Calcula o CRC-8 (polinômio 0x07, início 0xFF).

    Precisa ser idêntico a cap_crc8() do firmware.

    Args:
        data: bytes a verificar.

    Returns:
        O CRC-8, de 0 a 255.
    """
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class Reader:
    """Cursor de leitura little-endian sobre a carga de um registro."""

    def __init__(self, payload):
        """Cria o cursor no início da carga.

        Args:
            payload: bytes da carga.
        """
        self.buf, self.pos = payload, 0

    def take(self, n):
        """Consome n bytes.

        Args:
            n: quantidade de bytes.

        Returns:
            Os bytes consumidos.

        Raises:
            ValueError: se a carga acabar antes.
        """
        if self.pos + n > len(self.buf):
            raise ValueError('record shorter than its fields')
        chunk = self.buf[self.pos:self.pos + n]
        self.pos += n
        return chunk

    def u8(self):
        """Lê um inteiro sem sinal de 8 bits."""
        return self.take(1)[0]

    def u16(self):
        """Lê um inteiro sem sinal de 16 bits."""
        return struct.unpack('<H', self.take(2))[0]

    def u32(self):
        """Lê um inteiro sem sinal de 32 bits."""
        return struct.unpack('<I', self.take(4))[0]

    def u64(self):
        """Lê um inteiro sem sinal de 64 bits."""
        return struct.unpack('<Q', self.take(8))[0]

    def i16(self):
        """Lê um inteiro com sinal de 16 bits."""
        return struct.unpack('<h', self.take(2))[0]

    def f32(self):
        """Lê um float IEEE-754 de precisão simples."""
        return struct.unpack('<f', self.take(4))[0]

    def string(self):
        """Lê uma string precedida de 1 byte de tamanho."""
        return self.take(self.u8()).decode('ascii', 'replace')


def plausible(rtype, length):
    """Diz se o par tipo/tamanho corresponde a um registro que o firmware grava.

    Args:
        rtype: tipo do registro.
        length: tamanho da carga.

    Returns:
        True se o tamanho é possível para o tipo; False para tipo desconhecido.
    """
    if rtype in PAYLOAD_EXACT:
        return length == PAYLOAD_EXACT[rtype]
    if rtype in PAYLOAD_MIN:
        return length >= PAYLOAD_MIN[rtype]
    return False  # tipo desconhecido - o firmware não grava isto


def records(blob):
    """Percorre os registros válidos da captura, pulando corrupção.

    Toda rejeição avança um único byte e continua procurando. Desistir do resto
    do arquivo deixaria um falso byte de sincronismo com um tamanho grande
    descartar todos os registros válidos seguintes - uma falha silenciosa, que
    parece exatamente uma captura que só terminou mais cedo.

    Args:
        blob: bytes da captura.

    Yields:
        Tuplas (tipo, seq, t_ms, carga).

    Returns:
        Ao terminar, um dicionário com skipped_bytes, bad_crc e malformed
        (entregue como valor do StopIteration).
    """
    pos, skipped, bad_crc, malformed = 0, 0, 0, 0
    while pos + OVERHEAD <= len(blob):
        if blob[pos] != SYNC:
            pos += 1
            skipped += 1
            continue
        rtype = blob[pos + 1]
        length = blob[pos + 2]
        size = length + OVERHEAD
        # Formato primeiro: é mais barato que o checksum e já elimina a maioria
        # dos falsos bytes de sincronismo.
        if not plausible(rtype, length) or pos + size > len(blob):
            pos += 1
            skipped += 1
            malformed += 1
            continue
        if crc8(blob[pos:pos + size - CRC_BYTES]) != blob[pos + size - 1]:
            pos += 1
            skipped += 1
            bad_crc += 1
            continue
        seq = struct.unpack('<H', blob[pos + 3:pos + 5])[0]
        t_ms = struct.unpack('<I', blob[pos + 5:pos + 9])[0]
        yield rtype, seq, t_ms, blob[pos + HEADER_BYTES:pos + size - CRC_BYTES]
        pos += size
    return {'skipped_bytes': skipped, 'bad_crc': bad_crc, 'malformed': malformed}


def decode(rtype, payload):
    """Decodifica a carga de um registro nos seus campos.

    Args:
        rtype: tipo do registro.
        payload: bytes da carga.

    Returns:
        Dicionário campo -> valor, ou None para tipo desconhecido.

    Raises:
        ValueError: se a carga for mais curta que os campos.
    """
    r = Reader(payload)
    if rtype == TYPE_OBSERVATION:
        row = {'frames': r.u32(), 'previous_sensor_ms': r.u32()}
        flags = r.u8()
        row.update(have_previous=bool(flags & 1), gap_usable=bool(flags & 2),
                   range_valid=bool(flags & 4), flow_valid=bool(flags & 8))
        return row
    if rtype == TYPE_SESSION:
        return {'format_version': r.u8(), 'firmware_id': r.string(),
                'flow_scale_x': r.f32(),
                'flow_scale_y': r.f32(), 'gyro_sign_x': r.f32(),
                'gyro_sign_y': r.f32(), 'deadband_rad_s': r.f32(),
                'flow_lsb_rad_s': r.f32(), 'max_frame_gap_ms': r.u32(),
                'max_correction_age_ms': r.u32(),
                'ekf_accel_noise': r.f32(), 'ekf_flow_noise_rad_s': r.f32(),
                'ekf_gate_sigma': r.f32()}
    if rtype == TYPE_MTF:
        row = {'flow_x': r.i16(), 'flow_y': r.i16(), 'distance_mm': r.u32(),
               'quality': r.u8(), 'distance_status': r.u8(), 'strength': r.u8(),
               'precision': r.u8()}
        flags = r.u8()
        row['range_valid'] = bool(flags & 1)
        row['flow_valid'] = bool(flags & 2)
        row['sensor_ms'] = r.u32()
        row['frames'] = r.u32()
        return row
    if rtype == TYPE_BNO:
        report = r.u8()
        row = {'report': BNO_REPORTS.get(report, str(report)),
               'accuracy': r.u8(), 'sequence': r.u8()}
        row['v0'], row['v1'], row['v2'], row['v3'] = r.f32(), r.f32(), r.f32(), r.f32()
        row['count'] = r.u32()
        # Instante da medição segundo o hub, contra o t_ms, que é a nossa
        # recepção. Relatórios entregues no mesmo lote têm o mesmo t_ms e
        # diferem aqui.
        row['sensor_us'] = r.u64()
        row['delay_us'] = r.u32()
        return row
    if rtype == TYPE_STEP:
        row = {'used_mtf_frames': r.u32(), 'used_bno_rotation': r.u32(),
               'used_bno_gyro': r.u32(), 'used_bno_accel': r.u32(),
               'dt_s': r.f32()}
        flags = r.u8()
        row['fused_accepted'] = bool(flags & 1)
        row['ekf_accepted'] = bool(flags & 2)
        # Gravado, não deduzido: uma IMU offline deixa para trás um relatório
        # velho que parece igual a um novo.
        row['imu_online'] = bool(flags & 4)
        row['raw_px'], row['raw_py'] = r.f32(), r.f32()
        row['fused_vx'], row['fused_vy'] = r.f32(), r.f32()
        row['ekf_vx'], row['ekf_vy'] = r.f32(), r.f32()
        row['ekf_px'], row['ekf_py'] = r.f32(), r.f32()
        return row
    if rtype == TYPE_DIAG:
        # dropped_full e dropped_oversize consumiram número de sequência e por
        # isso aparecem como buraco; dropped_contention não. Mantê-los
        # separados é o que permite ao leitor conciliar buracos e descartes.
        row = {'dropped_full': r.u32(), 'dropped_oversize': r.u32(),
                'dropped_contention': r.u32(), 'clock_resyncs': r.u32(),
                'min_ever_free_heap': r.u32(), 'free_heap': r.u32(),
                'stack_margin_bytes': r.u32(), 'task': r.string()}
        if row['clock_resyncs'] == 0xFFFFFFFF:
            row['clock_resyncs'] = None  # a task não tem esse contador; não é zero
        return row
    return None


def main():
    """Lê a captura, grava os CSVs e imprime o resumo.

    Returns:
        Código de saída (ver o topo do arquivo).
    """
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('capture', type=Path)
    ap.add_argument('--out-dir', type=Path, default=Path('.'))
    args = ap.parse_args()

    blob = args.capture.read_bytes()
    rows = {name: [] for name in TYPE_NAMES.values()}
    last_seq, seq_gaps, unknown = None, 0, 0

    gen = records(blob)
    stats = {}
    while True:
        try:
            rtype, seq, t_ms, payload = next(gen)
        except StopIteration as stop:
            stats = stop.value or {}
            break
        if last_seq is not None:
            step = (seq - last_seq) & 0xFFFF
            if step != 1:
                seq_gaps += 1
        last_seq = seq
        try:
            fields = decode(rtype, payload)
        except ValueError:
            # Passou pelas verificações de formato e checksum, mas ainda assim
            # não decodificou: tratar como corrupção em vez de abortar o
            # arquivo inteiro.
            unknown += 1
            continue
        if fields is None:
            unknown += 1
            continue
        if rtype == TYPE_SESSION and fields['format_version'] != SUPPORTED_VERSION:
            # Recusar é melhor que adivinhar: uma versão diferente tem outro
            # layout, e lê-la como esta produziria números que parecem certos
            # e não significam nada.
            sys.exit(f"capture is format version {fields['format_version']}, "
                     f"this decoder reads version {SUPPORTED_VERSION}")
        rows[TYPE_NAMES[rtype]].append({'seq': seq, 't_ms': t_ms, **fields})

    args.out_dir.mkdir(parents=True, exist_ok=True)
    for name, items in rows.items():
        if not items:
            continue
        path = args.out_dir / f'{name}.csv'
        with path.open('w', newline='') as fh:
            writer = csv.DictWriter(fh, fieldnames=list(items[0].keys()))
            writer.writeheader()
            writer.writerows(items)
        print(f'{path}: {len(items)} records')

    total = sum(len(v) for v in rows.values())
    print(f'\ntotal records: {total}')
    print(f'sequence gaps: {seq_gaps}  (each one means records were dropped, '
          f'not that the vehicle was still)')
    print(f'bytes skipped resynchronising: {stats.get("skipped_bytes", 0)}')
    print(f'records failing checksum: {stats.get("bad_crc", 0)}')
    print(f'false sync bytes rejected on shape: {stats.get("malformed", 0)}')
    if unknown:
        print(f'unknown record types: {unknown}')
    if rows['session']:
        print('\nsession:')
        for key, value in rows['session'][0].items():
            print(f'  {key} = {value}')
    else:
        print('\nWARNING: no session record - the capture may have started mid-stream, '
              'so the configuration behind these numbers is unknown.')
    if total == 0:
        print('ERROR: no decodable records; use binary capture, not a text serial logger.', file=sys.stderr)
        return 1
    if not rows['session']:
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
