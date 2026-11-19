"""Aba Controle: mensagens, comandos e sequência armar/decolar/pairar/pousar (simulador)."""

import tempfile
import unittest
from pathlib import Path

from benchlink import protocol as p
from benchlink.hub import Hub

from .helpers import ManualSimSource

GOOD = {
    'height_m': 0.30,
    'hover_percent': 55.0,
    'cap_percent': 80.0,
    'climb_m_s': 0.10,
    'land_m_s': 0.10,
    'spool_percent_per_s': 4.0,
    'trim_ki': 1.2,
    'ki_xy': 0.05,
    'ki_yaw': 0.3,
    'autotrim': True,
    'use_ekf': True,
}


class ProtocolTest(unittest.TestCase):
    def test_ctrl_roundtrip(self):
        msg = {
            'type': 'ctrl',
            't_ms': 5,
            'state': 4,
            'flags': 0x0F,
            'fail': 0,
            'y': [-0.25, 0.5, -0.5, 1.0, 0.125, 0.0, 0.0, 0.25, 0.0],
            'pos': [1.0, 2.0, -0.25],
            'vel': [0.0, 0.5, -1.0],
            'h_ref': 0.25,
            'trim': 2.5,
            'base': 550.0,
            'ref_xy': [0.5, -0.25],
            'ref_psi': 0.125,
            'vbat_factor': 1.0,
        }
        msg_id, payload = p.encode_message(msg)
        self.assertEqual(len(payload), 96)
        self.assertEqual(p.decode_message(msg_id, payload), msg)

    def test_ctrl_commands(self):
        for action, code in (('arm', 1), ('takeoff', 2), ('land', 3)):
            msg_id, payload = p.command_payload('ctrl', action=action)
            self.assertEqual((msg_id, payload), (p.CMD_CTRL, bytes([code])))
            self.assertEqual(p.decode_command(msg_id, payload)['action'], action)
        with self.assertRaises(ValueError):
            p.command_payload('ctrl', action='fly')
        self.assertIsNone(p.decode_command(p.CMD_CTRL, b'\x00'))
        self.assertIsNone(p.decode_command(p.CMD_CTRL, b'\x04'))

    def test_params_units_and_limits(self):
        msg_id, payload = p.command_payload('ctrl_params', **GOOD)
        cmd = p.decode_command(msg_id, payload)
        self.assertAlmostEqual(cmd['hover_permille'], 550.0, places=3)
        self.assertAlmostEqual(cmd['cap_permille'], 800.0, places=3)
        self.assertAlmostEqual(cmd['spool_permille_s'], 40.0, places=3)
        self.assertAlmostEqual(cmd['trim_ki_permille'], 12.0, places=3)
        self.assertAlmostEqual(cmd['ki_xy'], 0.05, places=5)
        self.assertAlmostEqual(cmd['ki_yaw'], 0.3, places=5)
        self.assertTrue(cmd['autotrim'] and cmd['use_ekf'])
        for key, bad in (
            ('height_m', 0.05),
            ('height_m', 1.5),
            ('hover_percent', 95.0),
            ('cap_percent', 10.0),
            ('trim_ki', 25.0),
            ('ki_xy', 0.9),
            ('ki_yaw', -0.1),
            ('climb_m_s', float('nan')),
        ):
            with self.subTest(key=key, value=bad), self.assertRaises(ValueError):
                p.command_payload('ctrl_params', **{**GOOD, key: bad})
        # o firmware (e o simulador) também recusam o que passar pela IHM
        bad = bytearray(payload)
        bad[0:4] = b'\x00\x00\x00\x00'
        self.assertIsNone(p.decode_command(p.CMD_CTRL_PARAMS, bytes(bad)))


class ControlFlowTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.hub = Hub(Path(self.tmp.name))
        self.source = ManualSimSource()
        self.hub.set_source(self.source)
        self.source.step(300)
        self.assertTrue(self.hub.claim_control('ui')['controller'])
        self.hub.client_heartbeat('ui')
        self.sim = self.source.sim

    def tearDown(self):
        self.hub.close()
        self.tmp.cleanup()

    def alive(self, ms=0):
        self.hub.client_heartbeat('ui')
        self.hub._send_heartbeat()
        if ms:
            self.source.step(ms)

    def run_alive(self, seconds):
        for _ in range(int(seconds * 20)):
            self.alive(50)

    def test_full_sequence(self):
        self.alive()
        self.assertTrue(self.hub.command('ui', 'ctrl_params', dict(GOOD))['ok'])
        self.assertTrue(self.hub.command('ui', 'ctrl', {'action': 'arm'})['ok'])
        self.run_alive(0.2)
        self.assertEqual(self.sim.owner, 2)
        self.assertEqual(self.hub.snapshot()['ctrl']['state'], 1)
        self.assertEqual(max(self.sim.current), 0)  # armado, motores em zero
        # decolar só depois de armado; pousar só no ar
        self.assertFalse(self.hub.command('ui', 'ctrl', {'action': 'land'})['ok'])
        self.assertTrue(self.hub.command('ui', 'ctrl', {'action': 'takeoff'})['ok'])
        self.run_alive(20)
        ctrl = self.hub.snapshot()['ctrl']
        self.assertEqual(ctrl['state'], 4)
        self.assertAlmostEqual(-ctrl['pos'][2], 0.30, delta=0.03)
        self.assertLessEqual(max(self.sim.current), 800)  # limite do controle
        self.assertGreater(max(self.sim.current), 400)  # passa dos 40 % da bancada
        self.assertTrue(self.hub.command('ui', 'ctrl', {'action': 'land'})['ok'])
        self.run_alive(25)
        self.assertEqual(self.sim.owner, 0)
        self.assertEqual(self.sim.current, [0, 0, 0, 0])
        texts = [t['text'] for t in self.hub.snapshot()['texts']]
        self.assertTrue(any('pouso concluido' in t for t in texts))

    def test_stop_cuts_everything_in_flight(self):
        self.alive()
        self.hub.command('ui', 'ctrl_params', dict(GOOD))
        self.hub.command('ui', 'ctrl', {'action': 'arm'})
        self.run_alive(0.2)
        self.hub.command('ui', 'ctrl', {'action': 'takeoff'})
        self.run_alive(15)
        self.assertGreater(max(self.sim.current), 400)
        self.assertTrue(self.hub.command('ui', 'stop')['ok'])
        self.assertEqual(self.sim.owner, 0)
        self.assertEqual(self.sim.current, [0, 0, 0, 0])
        self.assertEqual(self.sim.ctrl_state, 0)

    def test_no_heartbeat_disarms_control(self):
        self.alive()
        self.hub.command('ui', 'ctrl', {'action': 'arm'})
        self.run_alive(0.2)
        self.hub.command('ui', 'ctrl', {'action': 'takeoff'})
        self.run_alive(5)
        self.source.step(350)  # a página parou: sem heartbeat
        self.assertEqual(self.sim.owner, 0)
        self.assertEqual(self.sim.current, [0, 0, 0, 0])

    def test_arm_needs_fresh_heartbeat_and_free_output(self):
        self.sim.heartbeat_ms = None
        self.assertEqual(self.hub._transact('ctrl', {'action': 'arm'})['result'], 'recusado')
        self.alive()
        self.assertTrue(self.hub.command('ui', 'arm')['ok'])  # bancada ocupa a saída
        self.assertFalse(self.hub._transact('ctrl', {'action': 'arm'})['ok'])

    def test_params_persist_and_resend_each_session(self):
        changed = {**GOOD, 'height_m': 0.45, 'hover_percent': 62.0}
        self.assertTrue(self.hub.command('ui', 'ctrl_params', changed)['ok'])
        self.assertAlmostEqual(self.hub.config['control']['height_m'], 0.45)
        self.assertAlmostEqual(self.sim.ctrl_params['hover_percent'], 62.0, places=3)
        self.sim.ctrl_params = dict(p.CTRL_DEFAULTS)  # firmware reiniciou
        self.hub._push_config()
        self.assertAlmostEqual(self.sim.ctrl_params['height_m'], 0.45, places=3)
        with self.assertRaises(ValueError):
            self.hub.command('ui', 'ctrl_params', {**GOOD, 'height_m': 5.0})
        self.assertAlmostEqual(self.hub.config['control']['height_m'], 0.45)

    def test_gains_persist_validate_and_resend(self):
        gains = {**p.GAIN_DEFAULTS, 'z': 8.0, 'w': 4.0}
        self.assertTrue(self.hub.command('ui', 'ctrl_gains', gains)['ok'])
        self.assertEqual(self.sim.ctrl_gains['z'], 8.0)
        self.assertEqual(self.hub.config['gains']['z'], 8.0)
        for key, bad in (('att', 0.1), ('z', 25.0), ('yaw', float('nan'))):
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.hub.command('ui', 'ctrl_gains', {**gains, key: bad})
        self.sim.ctrl_gains = dict(p.GAIN_DEFAULTS)
        self.hub._push_config()
        self.assertEqual(self.sim.ctrl_gains['w'], 4.0)

    def test_only_the_controlling_client_commands(self):
        with self.assertRaises(PermissionError):
            self.hub.command('other', 'ctrl', {'action': 'arm'})

    def test_signals_for_charts_and_recording(self):
        self.alive()
        self.hub.command('ui', 'ctrl', {'action': 'arm'})
        self.run_alive(0.5)
        values = self.hub.snapshot()['values']
        for name in (
            'ctrl.h_est',
            'ctrl.h_ref',
            'ctrl.phi',
            'ctrl.theta',
            'ctrl.trim',
            'ctrl.base',
        ):
            self.assertIn(name, values)


if __name__ == '__main__':
    unittest.main()
