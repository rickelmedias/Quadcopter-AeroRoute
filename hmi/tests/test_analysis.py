"""Statistics, spectrum and host estimate."""

import math
import unittest

from benchlink.estimator import EstimatorConfig, HostEstimator
from benchlink.signals import extract, quaternion_to_euler_deg
from benchlink.stats import SignalStore, spectrum


class StatsTest(unittest.TestCase):
    def test_window_stats_and_restart(self):
        store = SignalStore(window_s=5)
        for i in range(400):
            store.add('x', i / 100, 1.0 if i % 2 else 3.0)
        mean, std, n = store.stats('x', 1.0)
        self.assertAlmostEqual(mean, 2.0, delta=0.02)  # 101 amostras alternadas
        self.assertAlmostEqual(std, 1.0, delta=0.01)
        self.assertEqual(n, 101)
        self.assertAlmostEqual(store.rate('x'), 100.0, places=6)
        store.add('x', 0.0, 9.0)  # timeline restarted (reboot or replay)
        self.assertEqual(store.latest('x'), (0.0, 9.0))
        self.assertEqual(store.stats('x', 1.0)[2], 1)
        store.add('x', 0.1, math.nan)
        self.assertEqual(store.stats('x', 1.0)[2], 1)

    def test_spectrum_peak(self):
        fs = 400.0
        samples = [(i / fs, 0.5 * math.sin(2 * math.pi * 73.0 * i / fs) + 2.0) for i in range(1600)]
        spec = spectrum(samples)
        peak = max(range(1, len(spec['amplitude'])), key=spec['amplitude'].__getitem__)
        self.assertAlmostEqual(spec['freq_hz'][peak], 73.0, delta=fs / spec['n'])
        self.assertAlmostEqual(spec['amplitude'][peak], 0.5, delta=0.1)
        self.assertIsNone(spectrum(samples[:5]))


class EstimatorTest(unittest.TestCase):
    def test_euler(self):
        roll, pitch, yaw = quaternion_to_euler_deg([math.cos(0.25), math.sin(0.25), 0, 0])
        self.assertAlmostEqual(roll, math.degrees(0.5), places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)
        self.assertAlmostEqual(yaw, 0.0, places=6)

    def test_height_is_tilt_corrected_and_flow_scaled(self):
        est = HostEstimator(EstimatorConfig(height_tau_s=0.0, vz_tau_s=0.0, min_quality=10))
        out = est.on_range(0.0, 0.5, True, (0, 0, 100, True))
        self.assertEqual(out, {})  # sem IMU ainda: nada é inventado
        est.on_imu(60.0, 0.0, [0.0, 0.0, 0.0])
        out = est.on_range(0.01, 0.5, True, (20, -10, 100, True))
        self.assertAlmostEqual(out['est.height'], 0.25, places=6)
        self.assertAlmostEqual(out['est.vx'], 20 * 0.25 / 100, places=6)
        self.assertAlmostEqual(out['est.vy'], -10 * 0.25 / 100, places=6)
        out = est.on_range(0.02, 0.5, True, (20, 0, 5, True))  # baixa qualidade
        self.assertNotIn('est.vx', out)
        out = est.on_range(0.03, 0.5, False, (20, 0, 100, True))
        self.assertEqual(out, {})

    def test_gyro_compensation(self):
        est = HostEstimator(EstimatorConfig(height_tau_s=0.0, flow_gyro_kx=10.0))
        est.on_imu(0.0, 0.0, [0.0, 2.0, 0.0])
        out = est.on_range(0.0, 1.0, True, (20, 0, 200, True))
        self.assertAlmostEqual(out['est.vx'], 0.0, places=9)

    def test_extract_units(self):
        values, valid = extract(
            {
                'type': 'esc',
                'motor': 1,
                'erpm_100': 140,
                'voltage_cv': 1650,
                'current_ca': 250,
                'temperature_c': 30,
                'consumption_mah': 5,
            },
            14,
        )
        self.assertAlmostEqual(values['esc.m2.rpm'], 2000.0)
        self.assertAlmostEqual(values['esc.m2.voltage'], 16.5)
        self.assertAlmostEqual(values['esc.m2.current'], 2.5)
        self.assertTrue(valid)
        values, _ = extract(
            {
                'type': 'power',
                'vbat_pin_v': 1.5,
                'current_pin_v': 0.5,
                'vbat_v': math.nan,
                'current_a': math.nan,
                'flags': 3,
            }
        )
        self.assertNotIn('power.vbat', values)
        self.assertIn('power.vbat_pin', values)


if __name__ == '__main__':
    unittest.main()
