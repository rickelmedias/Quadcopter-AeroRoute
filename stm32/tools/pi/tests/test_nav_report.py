"""Testes da análise do EKF de navegação (dronectl/nav_report.py).

Rodar a partir de tools/pi:  python3 -m unittest tests.test_nav_report
"""
import csv
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dronectl.nav_report import (FLOW_FUSED, NEW_FRAME, XY_USABLE, Z_USABLE,  # noqa: E402
                                 NavSample, analyze_nav, nav_line, nav_notes, save_nav_log)


def nav(t_ms, z=-0.12, vz=0.0, x=0.0, range_rejected=0, flags=NEW_FRAME | FLOW_FUSED | XY_USABLE
        | Z_USABLE):
    return NavSample(t_ms, [0.0, 0.0], [0.0, 0.0], 0.12, 0.12, [x, 0.0, z], [0.0, 0.0, vz],
                     [0.01, -0.02, 0.0], [0.0, 0.0, 0.0], 0.1, 0.2, 0, 0, range_rejected, flags,
                     60)


def ctrl(t_ms, z=-0.12, vz=0.0, x=0.0, state=5):
    x_hat = [x, 0.0, z, 0, 0, 0, 0, 0, vz, 0, 0, 0]
    return SimpleNamespace(time_ms=t_ms, state=state, y=[z, 0, 0, 0, 0, 0, 0, 0, 0], x_hat=x_hat)


class NavReportTest(unittest.TestCase):

    def test_lqg_sees_a_climb_the_ekf_does_not(self):
        # Altura parada: o LQG acha que sobe 2,5 cm/s (vz NED negativo), o EKF não.
        n = [nav(1000 + 100 * k, range_rejected=65534 + k) for k in range(50)]
        c = [ctrl(1000 + 20 * k, vz=-0.025) for k in range(250)]
        m = analyze_nav(n, c)
        self.assertEqual(m['samples'], 50)
        self.assertAlmostEqual(m['rate_hz'], 10.0, places=1)
        self.assertAlmostEqual(m['climb_lqg_mean_cm_s'], 2.5, places=2)
        self.assertAlmostEqual(m['climb_ekf_mean_cm_s'], 0.0, places=2)
        self.assertAlmostEqual(m['height_trend_cm_s'], 0.0, places=2)
        self.assertEqual(m['range_rejected'], 49)  # contador de 16 bits deu a volta
        # Zeramento ao armar no meio da série: conta só o que veio depois.
        reset = [nav(1000 + 100 * k, range_rejected=(68 if k < 5 else k - 5)) for k in range(20)]
        self.assertEqual(analyze_nav(reset, c)['range_rejected'], 14)
        notes = nav_notes(m)
        self.assertTrue(any('vz:' in s for s in notes))
        self.assertIn('EKF nav', nav_line(m))

    def test_xy_difference_and_empty(self):
        n = [nav(1000 + 100 * k, x=0.01 * k) for k in range(20)]
        c = [ctrl(1000 + 100 * k, x=0.0) for k in range(20)]
        m = analyze_nav(n, c)
        self.assertAlmostEqual(m['xy_ekf_minus_lqg_max_cm'], 19.0, places=1)
        self.assertEqual(m['xy_ekf_final_cm'], [19.0, 0.0])
        self.assertEqual(analyze_nav([], c), {})
        self.assertEqual(nav_notes({}), [])
        self.assertIsNone(nav_line({}))

    def test_nav_log_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'nav_log.csv'
            save_nav_log(path, [(1.0, nav(1000)), (1.1, nav(1100))])
            with open(path, encoding='utf-8') as f:
                rows = list(csv.DictReader(f, delimiter=';'))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]['xy_usable'], '1')
            self.assertEqual(rows[1]['t_stm32_ms'], '1100')


if __name__ == '__main__':
    unittest.main()
