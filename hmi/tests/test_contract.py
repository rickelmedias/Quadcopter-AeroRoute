"""Python and firmware agree on IDs, payload sizes and exact frame bytes."""

import re
import struct
import unittest
from pathlib import Path

from benchlink import protocol as p

from . import vectors

LINK = Path(__file__).resolve().parents[2] / 'stm32/App/Inc/link'
HEADERS = (LINK / 'bench_frame.h', LINK / 'bench_proto.h')


def c_defines() -> dict[str, int]:
    text = ''.join(h.read_text() for h in HEADERS)
    return {
        name: int(value, 0)
        for name, value in re.findall(r'#define\s+(BL_\w+)\s+(0x[0-9A-Fa-f]+|\d+)U', text)
    }


class ContractTest(unittest.TestCase):
    def test_ids_match_c_header(self):
        defines = c_defines()
        for name in dir(p):
            if re.fullmatch(r'(MSG|CMD)_[A-Z_]+', name):
                with self.subTest(name=name):
                    self.assertEqual(defines[f'BL_{name}'], getattr(p, name))
        self.assertEqual(defines['BL_MAX_PAYLOAD'], p.MAX_PAYLOAD)
        self.assertEqual(defines['BL_PROTO_MAJOR'], p.PROTO_MAJOR)
        self.assertEqual(defines['BL_TEXT_MAX'], p.TEXT_MAX)

    def test_sizes_match_c_header(self):
        defines = c_defines()
        for msg_id, fmt in p.TELEMETRY_FORMATS.items():
            name = p.MESSAGE_NAMES[msg_id].upper()
            with self.subTest(message=name):
                self.assertEqual(defines[f'BL_LEN_{name}'], struct.calcsize(fmt))
        for msg_id, fmt in p.COMMAND_FORMATS.items():
            name = p.COMMAND_NAMES[msg_id].upper()
            key = f'BL_LEN_CMD_{name}'
            if key in defines:
                with self.subTest(command=name):
                    self.assertEqual(defines[key], struct.calcsize(fmt))
            else:
                self.assertEqual(struct.calcsize(fmt), 0, name)
        self.assertEqual(defines['BL_LEN_TEXT_MAX'], struct.calcsize(p.TEXT_HEADER) + p.TEXT_MAX)

    def test_golden_vectors_are_current(self):
        """The committed vector file equals what the Python encoder produces."""
        self.assertEqual(vectors.VECTOR_FILE.read_text(), vectors.render())

    def test_golden_vectors_decode(self):
        frames = vectors.read()
        for name, seq, message in vectors.TELEMETRY:
            with self.subTest(vector=name):
                packets = p.FrameDecoder().feed(frames[name])
                self.assertEqual(len(packets), 1)
                self.assertEqual(packets[0].seq, seq)
                decoded = p.decode_message(packets[0].msg_id, packets[0].payload)
                for key, value in message.items():
                    self.assertEqual(decoded[key], value, key)


if __name__ == '__main__':
    unittest.main()
