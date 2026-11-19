"""Framing and command validation (host side)."""

import math
import random
import unittest

from benchlink import protocol as p


class FramingTest(unittest.TestCase):
    def test_crc_reference(self):
        self.assertEqual(p.crc16(b'123456789'), 0x29B1)

    def test_cobs_roundtrip(self):
        rng = random.Random(7)
        for length in (0, 1, 2, 253, 254, 255, 300):
            for zeros in (0.0, 0.1, 1.0):
                data = bytes(
                    0 if rng.random() < zeros else rng.randint(1, 255) for _ in range(length)
                )
                encoded = p.cobs_encode(data)
                self.assertNotIn(0, encoded)
                if length:
                    self.assertEqual(p.cobs_decode(encoded), data)

    def test_decoder_resync_and_errors(self):
        frame = p.encode_frame(p.MSG_ACK, 9, bytes([0x86, 1, 0, 0x90, 0x01]))
        decoder = p.FrameDecoder()
        packets = decoder.feed(b'\x13\x37\x00' + b'\x00' + frame[:3])
        self.assertEqual(packets, [])
        self.assertEqual(decoder.errors, 1)
        packets = decoder.feed(frame[3:])
        self.assertEqual(len(packets), 1)
        self.assertEqual((packets[0].msg_id, packets[0].seq), (p.MSG_ACK, 9))
        corrupt = bytearray(frame)
        corrupt[2] ^= 1
        self.assertEqual(decoder.feed(bytes(corrupt)), [])
        self.assertEqual(decoder.errors, 2)
        self.assertEqual(decoder.feed(b'\x55' * 300 + b'\x00'), [])
        self.assertEqual(decoder.errors, 3)
        self.assertEqual(len(decoder.feed(frame)), 1)

    def test_payload_limit(self):
        with self.assertRaises(ValueError):
            p.encode_frame(1, 0, bytes(p.MAX_PAYLOAD + 1))

    def test_message_roundtrip_with_nan(self):
        msg = {
            'type': 'power',
            't_ms': 1,
            'vbat_pin_v': 1.5,
            'current_pin_v': 0.5,
            'vbat_v': math.nan,
            'current_a': math.nan,
            'flags': 3,
        }
        decoded = p.decode_message(*p.encode_message(msg))
        self.assertTrue(math.isnan(decoded['vbat_v']))
        self.assertEqual(decoded['flags'], 3)

    def test_wrong_size_is_rejected(self):
        self.assertIsNone(p.decode_message(p.MSG_STATUS, b'\x00' * 10))
        self.assertIsNone(p.decode_message(0x7F, b''))


class CommandTest(unittest.TestCase):
    def test_friendly_arguments(self):
        msg_id, payload = p.command_payload('motor_set', values={2: 12.5, 4: 0})
        self.assertEqual(msg_id, p.CMD_MOTOR_SET)
        cmd = p.decode_command(msg_id, payload)
        self.assertEqual(cmd['mask'], 0b1010)
        self.assertEqual(cmd['values'], [0, 125, 0, 0])

    def test_rejections(self):
        bad = [
            ('motor_set', {'values': {5: 10}}),
            ('motor_set', {'values': {1: 101}}),
            ('cap', {'percent': -1}),
            ('rate', {'percent_per_s': 0.5}),
            ('direction', {'motors': [1], 'reversed': [2]}),
            ('sweep', {'motors': [1], 'max_percent': 30, 'step_percent': 5, 'hold_s': 0.05}),
            ('stream', {'bno_divider': 17}),
            ('power_cal', {'vbat_gain': math.inf}),
            ('power_cal', {'current_a_per_v': -1}),
            ('esc_telem', {'enabled': True, 'period_ms': 3}),
            ('fly', {}),
        ]
        for name, kwargs in bad:
            with self.subTest(name=name, kwargs=kwargs), self.assertRaises(ValueError):
                p.command_payload(name, **kwargs)

    def test_decode_command_matches_firmware_ranges(self):
        self.assertIsNone(p.decode_command(p.CMD_ARM, b'\x00'))
        self.assertIsNone(p.decode_command(p.CMD_MOTOR_SET, bytes([0x10, 0, 0, 0, 0, 0, 0, 0, 0])))
        self.assertIsNone(p.decode_command(0x85, b''))


if __name__ == '__main__':
    unittest.main()
