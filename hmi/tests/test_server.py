"""HTTP API end to end against the hub and the simulated STM32 (localhost only)."""

import json
import tempfile
import unittest
import urllib.error
import urllib.request
from pathlib import Path

from benchlink.hub import Hub
from benchlink.server import start_background

from .helpers import ManualSimSource


class ServerTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.hub = Hub(Path(self.tmp.name))
        self.source = ManualSimSource()
        self.hub.set_source(self.source)
        self.source.step(1200)
        self.server, self.url = start_background(self.hub)

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.hub.close()
        self.tmp.cleanup()

    def get(self, path):
        with urllib.request.urlopen(self.url + path, timeout=5) as r:
            body = r.read()
            return (
                json.loads(body)
                if r.headers['Content-Type'].startswith('application/json')
                else body
            )

    def post(self, path, body):
        req = urllib.request.Request(
            self.url + path,
            data=json.dumps(body).encode(),
            headers={'Content-Type': 'application/json'},
        )
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read())

    def test_static_and_signals(self):
        self.assertIn(b'AeroRoute', self.get('/'))
        self.assertIn(b'drawSeries', self.get('/chart.js'))
        signals = self.get('/api/signals')
        self.assertTrue(any(s['name'] == 'imu.gyro_x' and s['unit'] == 'rad/s' for s in signals))
        with self.assertRaises(urllib.error.HTTPError):
            self.get('/../hub.py')

    def test_control_command_and_freeze(self):
        status, body = self.post('/api/command', {'client_id': 'b', 'name': 'arm'})
        self.assertEqual(status, 403)
        self.assertEqual(self.post('/api/control', {'client_id': 'a'})[1], {'controller': True})
        self.post('/api/heartbeat', {'client_id': 'a'})
        status, body = self.post(
            '/api/command', {'client_id': 'a', 'name': 'cap', 'args': {'percent': 30}}
        )
        self.assertEqual((status, body['ok'], body['value']), (200, True, 300))
        status, body = self.post(
            '/api/command', {'client_id': 'a', 'name': 'cap', 'args': {'percent': 300}}
        )
        self.assertEqual(status, 400)
        status, body = self.post('/api/freezes', {'names': ['mtf.distance'], 'label': 'h0'})
        self.assertEqual(status, 200)
        fid = body['id']
        self.assertEqual(self.post(f'/api/freezes/{fid}', {'label': 'novo'})[1]['label'], 'novo')
        self.assertEqual(len(self.get('/api/freezes')), 1)
        self.assertTrue(self.post(f'/api/freezes/{fid}', {'delete': True})[1]['deleted'])
        series = self.get('/api/series?names=imu.roll,mtf.distance&seconds=1')
        self.assertGreater(len(series['imu.roll']['t']), 100)
        spec = self.get('/api/spectrum?name=imu.gyro_x&seconds=1')
        self.assertGreater(len(spec['freq_hz']), 10)

    def test_replay_source_rejects_commands(self):
        self.post('/api/control', {'client_id': 'a'})
        status, _ = self.post('/api/record/start', {'client_id': 'a', 'name': 'r'})
        self.assertEqual(status, 200)
        self.source.step(300)
        path = self.post('/api/record/stop', {})[1]['path']
        sessions = self.get('/api/sessions')
        self.assertEqual(sessions[0]['id'], Path(path).name)
        status, body = self.post('/api/source', {'mode': 'replay', 'session': sessions[0]['id']})
        self.assertEqual((status, body['kind']), (200, 'replay'))
        status, body = self.post(
            '/api/command', {'client_id': 'a', 'name': 'cap', 'args': {'percent': 10}}
        )
        self.assertEqual(status, 403)
        status, _ = self.post('/api/source', {'mode': 'replay', 'session': '../../etc'})
        self.assertEqual(status, 400)


if __name__ == '__main__':
    unittest.main()
