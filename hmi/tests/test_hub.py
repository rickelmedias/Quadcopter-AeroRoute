"""Hub + simulated STM32: safety rules, control ownership, freezes, recording/replay."""

import csv
import json
import tempfile
import time
import unittest
from pathlib import Path

from benchlink import protocol as p
from benchlink.hub import Hub
from benchlink.recorder import read_raw
from benchlink.sources import ReplaySource

from .helpers import ManualSimSource


def wait_for(predicate, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class HubTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.hub = Hub(Path(self.tmp.name))
        self.source = ManualSimSource()
        self.hub.set_source(self.source)
        self.source.step(300)
        self.assertTrue(self.hub.claim_control('ui')['controller'])
        self.hub.client_heartbeat('ui')

    def tearDown(self):
        self.hub.close()
        self.tmp.cleanup()

    def alive(self, ms=0):
        """Browser heartbeat + simulated time; the hub forwards HEARTBEAT."""
        self.hub.client_heartbeat('ui')
        self.hub._send_heartbeat()
        if ms:
            self.source.step(ms)

    def test_hello_and_values(self):
        self.assertEqual(self.hub.hello['firmware_id'], 'simulador')
        snap = self.hub.snapshot()
        self.assertIn('imu.roll', snap['values'])
        self.assertIn('mtf.distance', snap['values'])
        self.assertIn('est.height', snap['values'])
        self.assertAlmostEqual(snap['values']['mtf.distance']['v'], 0.30, delta=0.02)

    def test_arm_requires_heartbeat_and_keepalive_disarms(self):
        sim = self.source.sim
        sim.heartbeat_ms = None
        result = self.hub._transact('arm', {})
        self.assertEqual(result['result'], 'recusado')
        self.alive()
        self.assertTrue(self.hub.command('ui', 'arm')['ok'])
        self.assertTrue(self.hub.command('ui', 'motor_set', {'values': {1: 30}})['ok'])
        for _ in range(20):
            self.alive(50)
        self.assertGreater(sim.current[0], 0)
        self.source.step(350)  # no heartbeat from the browser/hub
        self.assertEqual(sim.owner, 0)
        self.assertEqual(sim.current, [0, 0, 0, 0])

    def test_no_auto_stop_for_flight_armed_elsewhere(self):
        """Armado pela Pi (DroneLink): a IHM só grava; sem navegador ela não manda STOP."""
        self.hub.controller_beat = 0.0  # navegador "sumiu"
        self.hub.status = {'flags': p.STATUS_ARMED}  # armado por outro caminho
        sent = len(self.source.sent)
        time.sleep(0.25)  # algumas voltas do laço de heartbeat
        stops = [f for f in self.source.sent[sent:] if p.cobs_decode(f[:-1])[0] == p.CMD_STOP]
        self.assertEqual(stops, [])
        self.assertFalse(self.hub.stop_sent)
        # Armado por esta IHM, o STOP automático continua valendo.
        self.hub.armed_here = True
        time.sleep(0.25)
        self.assertTrue(self.hub.stop_sent)

    def test_cap_clamps_targets(self):
        self.alive()
        self.hub.command('ui', 'arm')
        self.hub.command('ui', 'motor_set', {'values': {2: 35}})
        r = self.hub.command('ui', 'cap', {'percent': 20})
        self.assertEqual(r['value'], 200)
        self.assertEqual(self.source.sim.target[1], 200)

    def test_only_controller_commands_but_anyone_stops(self):
        with self.assertRaises(PermissionError):
            self.hub.command('other', 'arm')
        self.assertTrue(self.hub.command('other', 'stop')['ok'])
        self.assertFalse(self.hub.claim_control('other')['controller'])
        self.assertTrue(self.hub.claim_control('other', force=True)['controller'])

    def test_invalid_frame_stops_motors(self):
        self.alive()
        self.hub.command('ui', 'arm')
        self.assertEqual(self.source.sim.owner, 1)
        self.source.send(b'\x05\x01\x02\x03\x04\x00')
        self.assertEqual(self.source.sim.owner, 0)

    def test_stale_browser_triggers_stop(self):
        self.alive()
        self.hub.command('ui', 'arm')
        self.source.step(100)
        self.assertTrue(self.hub.status['flags'] & p.STATUS_ARMED)
        # No browser heartbeat: the hub loop must send STOP within ~0.4 s.
        self.assertTrue(wait_for(lambda: self.source.sim.owner == 0, 1.5))

    def test_freeze_value_group_and_persistence(self):
        self.source.step(1200)
        single = self.hub.freeze(['mtf.distance'], 'altura parada')
        self.assertEqual(list(single['values']), ['mtf.distance'])
        item = single['values']['mtf.distance']
        self.assertEqual(item['unit'], 'm')
        self.assertGreater(item['n_1s'], 50)
        self.assertGreater(item['std_1s'], 0)
        group = self.hub.freeze(None, 'imu', group='IMU (BNO085)')
        self.assertIn('imu.gyro_x', group['values'])
        reopened = Hub(Path(self.tmp.name))
        self.assertEqual([f['label'] for f in reopened.freezes.all()], ['altura parada', 'imu'])
        reopened.close()

    def test_record_and_replay(self):
        self.hub.start_recording('teste', 'nota', {'surface': 'mdf'})
        self.alive()
        self.hub.command('ui', 'cap', {'percent': 25})
        self.hub.marker('inicio')
        self.source.step(500)
        self.hub.freeze(['imu.roll'], 'f1')
        path = Path(self.hub.stop_recording()['path'])
        meta = json.loads((path / 'meta.json').read_text())
        self.assertEqual(meta['conditions'], {'surface': 'mdf'})
        self.assertEqual(meta['hello']['firmware_id'], 'simulador')  # HELLO anterior à gravação
        self.assertGreater(meta['counts']['imu'], 150)
        with open(path / 'imu.csv') as handle:
            header = next(csv.reader(handle))
        self.assertIn('gyro_x_rad_s', header)
        events = [
            json.loads(line)['event'] for line in (path / 'events.jsonl').read_text().splitlines()
        ]
        self.assertIn('marcador', events)
        self.assertIn('comando', events)
        self.assertEqual(json.loads((path / 'freezes.json').read_text())[-1]['label'], 'f1')

        decoder = p.FrameDecoder()
        replayed = [
            p.decode_message(pk.msg_id, pk.payload)
            for _, chunk in read_raw(path / 'raw.blr')
            for pk in decoder.feed(chunk)
        ]
        imu = [m for m in replayed if m and m['type'] == 'imu']
        self.assertEqual(len(imu), meta['counts']['imu'])
        self.assertEqual(decoder.errors, 0)

        self.hub.set_source(ReplaySource(path))
        self.hub.source.control(speed=20)
        self.assertTrue(wait_for(lambda: self.hub.source.describe()['finished'], 3))
        with self.assertRaises(PermissionError):
            self.hub.command('ui', 'arm')
        with self.assertRaises(PermissionError):
            self.hub.start_recording('x')

    def test_config_is_pushed_on_new_session(self):
        self.hub.command('ui', 'power_cal', {'vbat_gain': 11.0, 'vbat_min_v': 0.0})
        self.assertEqual(self.source.sim.cal[0], 11.0)
        fresh = ManualSimSource()
        self.hub.set_source(fresh)
        self.assertTrue(wait_for(lambda: fresh.sim.cal[0] == 11.0, 2.0))
        fresh.step(200)
        self.assertIn('power.vbat', self.hub.snapshot()['values'])

    def test_low_battery_blocks_arm(self):
        self.hub.command('ui', 'power_cal', {'vbat_gain': 11.0, 'vbat_min_v': 30.0})
        self.alive()
        self.assertEqual(self.hub.command('ui', 'arm')['result'], 'recusado')
        self.source.step(150)
        self.assertTrue(self.hub.status['flags'] & p.STATUS_POWER_LOW)

    def test_spectrum_marks_measured_rotation(self):
        self.hub.command('ui', 'esc_telem', {'enabled': True, 'period_ms': 10})
        self.alive()
        self.hub.command('ui', 'arm')
        self.hub.command('ui', 'rate', {'percent_per_s': 100})
        self.hub.command('ui', 'motor_set', {'values': {1: 20}})
        for _ in range(60):
            self.alive(50)
        spec = self.hub.spectrum('imu.lacc_x', 2.0)
        markers = {m['motor']: m['hz'] for m in spec['markers']}
        self.assertIn(1, markers)
        expected = 0.2 * 160_000 / 7 / 60
        self.assertAlmostEqual(markers[1], expected, delta=1.0)
        freq, amp = spec['freq_hz'], spec['amplitude']
        if expected < freq[-1]:
            peak = freq[max(range(1, len(amp)), key=amp.__getitem__)]
            self.assertAlmostEqual(peak, expected, delta=3 * spec['fs_hz'] / spec['n'])


if __name__ == '__main__':
    unittest.main()
