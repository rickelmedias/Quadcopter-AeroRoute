"""Testes da ferramenta da Pi: projeto do LQG e enlace com um STM32 simulado.

Rodar a partir de tools/pi:  python3 -m unittest discover -s tests
"""
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dronectl.design import Weights, compare, design  # noqa: E402
from dronectl.gains import default_gains  # noqa: E402


class DesignTest(unittest.TestCase):
    """O projeto em numpy reproduz os ganhos exportados do Simulink."""

    def test_tcc_weights_reproduce_simulink(self):
        gains, info = design(Weights())
        for name, err in compare(gains, default_gains()).items():
            self.assertLess(err, 1e-9, name)
        self.assertLess(info['controle_maior_polo'], 1.0)
        self.assertLess(info['estimador_maior_polo'], 1.0)

    def test_heavier_altitude_weight_raises_altitude_gain(self):
        w = Weights()
        w.set('q', 'z', 2000.0)
        gains, _ = design(w)
        base = default_gains()
        self.assertGreater(abs(gains.Kd[0][2]), abs(base.Kd[0][2]))
        # O pré-compensador acompanha: ganho DC da referência continua unitário.
        self.assertAlmostEqual(gains.Kfd[0][2], gains.Kd[0][2], places=6)

    def test_noisier_sensor_lowers_its_kalman_gain(self):
        w = Weights()
        w.set('rn', 'vx', 1.28e-1)
        gains, _ = design(w)
        self.assertLess(abs(gains.M[6][4]), abs(default_gains().M[6][4]))

    def test_weights_validation(self):
        w = Weights()
        with self.assertRaises(ValueError):
            w.set('r', 'all', 0.0)
        self.assertIn('->', w.set('qn', 'vz', 2e-3))
        self.assertEqual(Weights.from_dict(w.to_dict()), w)


@unittest.skipUnless(sys.platform.startswith('linux'), 'pty só no Linux')
class LinkTest(unittest.TestCase):
    """Enlace completo contra o STM32 simulado (tests/fake_fc.py)."""

    def setUp(self):
        from fake_fc import FakeFC
        from dronectl.link import DroneLink

        self.texts = []
        self.fc = FakeFC(drop_first_row=True)
        self.link = DroneLink(self.fc.port, on_text=self.texts.append)
        self.link.wait_fc(timeout=5.0)

    def tearDown(self):
        self.link.close()
        self.fc.close()

    def test_params(self):
        params = self.link.param_list()
        self.assertEqual(params['CTL_W0'], 702.0)
        self.assertEqual(self.link.param_set('CTL_W0', 690.0), 690.0)
        self.assertEqual(self.link.param_get('ctl_w0'), 690.0)
        # Fora dos limites: continua o valor antigo e chega o motivo.
        self.assertEqual(self.link.param_set('CTL_W0', 5.0), 690.0)
        time.sleep(0.1)
        self.assertTrue(any('fora dos limites' in t for t in self.texts))

    def test_gain_upload_retries_lost_row_and_commits(self):
        w = Weights()
        w.set('q', 'z', 800.0)
        gains, _ = design(w)
        crc = self.link.upload_gains(gains)
        self.assertEqual(crc, gains.crc32())
        self.assertEqual(self.link.query_gains(), (crc, True))
        self.assertEqual(self.link.restore_gains(), 0x21F15F2A)

    def test_flight_sequence_and_debug_stream(self):
        from dronectl.monitor import FilterStats

        stats = FilterStats(window=50)
        self.link.subscribe(stats.add)
        self.link.param_set('DBG_RATE_HZ', 50)
        self.link.start_session()
        self.link.arm(True)
        command = self.link.takeoff(0.5)
        self.assertTrue(self.link.wait_leg(command, timeout=3.0))
        self.link.land()
        time.sleep(0.6)
        self.assertGreater(stats.samples, 5)
        self.assertIn('dp(y-x)', stats.table())
        self.assertEqual(self.link.flight_state, 2)


if __name__ == '__main__':
    unittest.main()
