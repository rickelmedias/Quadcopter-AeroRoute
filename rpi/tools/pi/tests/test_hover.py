"""Testes do ensaio de hover da Pi: análise, perfil, ganhos e o ensaio completo
contra o STM32 simulado (tests/fake_fc.py com o modelo de voo).

Rodar a partir de tools/pi:  python3 -m unittest discover -s tests
"""

import csv
import json
import math
import os
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dronectl.gains import REPO_ROOT  # noqa: E402
from dronectl.hover_report import (
    ARMED,
    HOLD,
    LAND,
    STANDBY,
    TAKEOFF,
    Sample,  # noqa: E402
    analyze,
    format_report,
    parse_dist,
    report_folder,
)
from dronectl.hover_test import (
    Options,
    Profile,
    adjusted_gains,  # noqa: E402
    parse_assignments,
)

GROUND = -0.08
PARAMS = {
    'CTL_W0': 1050.0,
    'CTL_FF_TH': -50.0,
    'CTL_FF_PHI': -40.0,
    'CTL_FF_PSI': 50.0,
    'SNS_LVL_PHI': 0.0,
    'SNS_LVL_TH': 0.0,
    'CTL_XY': 1.0,
    'MOT_W_AT_0': 0.0,
    'MOT_W_AT_100': 1755.0,
}


def synthetic_flight(
    phi_deg=0.4, theta_deg=-0.3, xy=(0.05, -0.08), target=0.30, hover_s=20.0, rate=50.0
):
    """Amostras de um voo: chão, partida, rampa, hover parado e pouso.

    Returns:
        As amostras e o tempo da Pi (t_pi) no meio do hover e depois do pouso.
    """
    samples = []
    t = [0.0]
    dt = 1.0 / rate

    def add(state, h, x=0.0, y=0.0, phi=0.0, theta=0.0):
        z = GROUND - h
        att = [math.radians(phi), math.radians(theta)]
        samples.append(
            Sample(
                t=t[0],
                state=state,
                meas=15,
                flags=1,
                y=[z] + att + [0.0] * 6,
                x=[x, y, z] + att + [0.0] * 7,
                du=[0.0] * 4,
                ref=[0.0, 0.0, z, 0.0],
                thr=[0.62, 0.60, 0.53, 0.61],
                t_pi=1000.0 + t[0],
            )
        )
        t[0] += dt

    for _ in range(int(rate)):
        add(STANDBY, 0.0)
    for _ in range(int(rate * 0.5)):
        add(ARMED, 0.0)
    for _ in range(int(rate * 3)):
        add(TAKEOFF, 0.0)
    h = 0.0
    while h < target:
        h = min(target, h + 0.05 * dt)
        add(TAKEOFF, h)
    t_mid = None
    for i in range(int(rate * hover_s)):
        add(HOLD, target, xy[0], xy[1], phi_deg, theta_deg)
        if i == int(rate * hover_s / 2):
            t_mid = 1000.0 + t[0]
    while h > 0:
        h = max(0.0, h - 0.1 * dt)
        add(LAND, h)
    for _ in range(int(rate * 0.5)):
        add(STANDBY, 0.0)
    return samples, t_mid, 1000.0 + t[0]


def flight_texts(t_mid, t_end):
    # Durante o hover e depois do pouso (z já descontado pelo pouso).
    return [(t_mid, 'dist z=35 th=-12 phi=8 psi=20'), (t_end, 'dist z=5 th=-12 phi=8 psi=20 rad/s')]


class HoverReportTest(unittest.TestCase):
    """A análise mede o voo e sugere o próximo a partir dos números certos."""

    def test_ctrl_c_near_ground_cuts_without_waiting_for_land(self):
        from dronectl.hover_test import HoverTest

        commands = []
        test = object.__new__(HoverTest)
        sample = SimpleNamespace(state=TAKEOFF, meas_flags=4, y=[-0.164], x_hat=[0.0, 0.0, -0.165])

        def arm(armed):
            commands.append(('arm', armed))
            sample.state = STANDBY

        test.link = SimpleNamespace(arm=arm, land=lambda: commands.append(('land',)))
        test.opts = SimpleNamespace(mode='motors')
        test.say = lambda message: None
        test.stop_reason = 'Ctrl+C'
        test.ground = -0.08
        test.samples = [(time.monotonic(), sample)]
        test.land()
        self.assertEqual(commands, [('arm', False)])

    def test_ctrl_c_above_ground_still_lands(self):
        from dronectl.hover_test import HoverTest

        commands = []
        sample = SimpleNamespace(state=TAKEOFF, meas_flags=4, y=[-0.28], x_hat=[0.0, 0.0, -0.28])

        def land():
            commands.append(('land',))
            sample.state = STANDBY

        test = object.__new__(HoverTest)
        test.link = SimpleNamespace(arm=lambda armed: commands.append(('arm', armed)), land=land)
        test.opts = SimpleNamespace(mode='motors')
        test.say = lambda message: None
        test.stop_reason = 'Ctrl+C'
        test.ground = -0.08
        test.samples = [(time.monotonic(), sample)]
        test.land()
        self.assertEqual(commands, [('land',)])

    def test_metrics_and_suggestion(self):
        samples, t_mid, t_end = synthetic_flight()
        rep = analyze(
            samples,
            flight_texts(t_mid, t_end),
            {'height_m': 0.30, 'params': PARAMS, 'ground_y_z': GROUND},
        )
        m = rep.metrics
        self.assertIsNotNone(m['phases_s']['reached'])
        self.assertGreater(m['window_s'], 15.0)
        self.assertAlmostEqual(m['height']['mean_cm'], 30.0, places=1)
        self.assertEqual(m['xy']['mean_cm'], [5.0, -8.0])
        self.assertAlmostEqual(m['attitude']['phi_mean_deg'], 0.4, places=2)
        self.assertEqual(m['still_pct'], 100.0)
        # u1..u4 -> M4, M2, M1, M3.
        self.assertEqual(m['motors']['throttle_pct_M1_M4'], [53.0, 60.0, 61.0, 62.0])
        self.assertEqual(m['motors']['hover_equiv_w0'], round(0.59 * 1755))
        s = rep.suggestion
        self.assertAlmostEqual(s['SNS_LVL_PHI'], -0.4, places=2)
        self.assertAlmostEqual(s['SNS_LVL_TH'], 0.3, places=2)
        # O relato de antes do pouso (z=35), não o de depois (z=5).
        self.assertEqual(s['CTL_W0'], 1085)
        self.assertEqual((s['CTL_FF_TH'], s['CTL_FF_PHI'], s['CTL_FF_PSI']), (-62, -32, 70))
        text = format_report(rep)
        self.assertIn('sugestão para o próximo voo', text)
        self.assertIn('XY: deslocamento médio (+5.0, -8.0) cm', text)

    def test_lost_samples_are_counted(self):
        samples, t_mid, t_end = synthetic_flight()
        kept = [s for i, s in enumerate(samples) if not 100 <= i < 110]  # 10 perdidas
        rep = analyze(kept, [], {'height_m': 0.30, 'params': PARAMS})
        self.assertEqual(rep.metrics['link']['rate_hz'], 50.0)
        self.assertAlmostEqual(rep.metrics['link']['lost_pct'], 100 * 10 / len(samples), places=1)
        self.assertIn('perdidas', format_report(rep))

    def test_level_step_is_limited(self):
        samples, t_mid, t_end = synthetic_flight(phi_deg=3.0)
        params = dict(PARAMS, SNS_LVL_PHI=0.5)
        rep = analyze(samples, [], {'height_m': 0.30, 'params': params})
        self.assertAlmostEqual(rep.suggestion['SNS_LVL_PHI'], 0.5 - 1.5)
        self.assertNotIn('CTL_W0', rep.suggestion)  # sem relato dist

    def test_no_level_without_xy(self):
        samples, t_mid, t_end = synthetic_flight()
        rep = analyze(
            samples,
            flight_texts(t_mid, t_end),
            {'height_m': 0.30, 'params': dict(PARAMS, CTL_XY=0.0)},
        )
        self.assertNotIn('SNS_LVL_PHI', rep.suggestion)
        self.assertTrue(any('CTL_XY = 0' in n for n in rep.notes))

    def test_short_hover_suggests_nothing(self):
        samples, t_mid, t_end = synthetic_flight(hover_s=4.0)
        rep = analyze(samples, flight_texts(t_mid, t_end), {'height_m': 0.30, 'params': PARAMS})
        self.assertEqual(rep.suggestion, {})

    def test_never_lifted(self):
        samples = [
            Sample(
                t=i * 0.02,
                state=TAKEOFF if i > 50 else STANDBY,
                meas=15,
                flags=1,
                y=[GROUND] + [0.0] * 8,
                x=[0.0, 0.0, GROUND] + [0.0] * 9,
                du=[0.0] * 4,
                ref=[0.0] * 4,
                thr=[0.0] * 4,
            )
            for i in range(300)
        ]
        rep = analyze(samples, [], {'height_m': 0.30, 'params': PARAMS})
        self.assertTrue(any('não saiu do chão' in n for n in rep.notes))
        self.assertEqual(rep.suggestion, {})

    def test_parse_dist(self):
        texts = [
            (1.0, 'dist z=-12 th=3 phi=-4'),
            (2.0, 'voo: pousando'),
            (3.0, 'dist z=40 th=-1 phi=2 psi=-7 rad/s'),
        ]
        self.assertEqual(parse_dist(texts), {'z': 40.0, 'theta': -1.0, 'phi': 2.0, 'psi': -7.0})
        self.assertEqual(parse_dist(texts[:1])['psi'], 0.0)
        self.assertIsNone(parse_dist([(0.0, 'nada')]))


class ProfileTest(unittest.TestCase):
    """Perfil: gravar, ler, aplicar a sugestão; atribuições da linha de comando."""

    def test_round_trip_and_suggestion(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'perfil.json'
            p = Profile(path=path, params={'CTL_W0': 1060.0}, q={'x': 60.0}, height_cm=25.0)
            p.suggestion = {'CTL_W0': 1085.0, 'SNS_LVL_PHI': -0.4}
            p.save()
            back = Profile.load(path)
            self.assertEqual(back.params, {'CTL_W0': 1060.0})
            self.assertEqual(back.q, {'x': 60.0})
            self.assertEqual(back.height_cm, 25.0)
            self.assertEqual(back.apply_suggestion(['sns_lvl_phi']), {'SNS_LVL_PHI': -0.4})
            self.assertEqual(back.params['CTL_W0'], 1060.0)
            back.apply_suggestion()
            self.assertEqual(back.params['CTL_W0'], 1085.0)
            self.assertIn('CTL_W0=1085', back.text())

    def test_assignments(self):
        self.assertEqual(
            parse_assignments(['ctl_w0=1080,CTL_FF_TH=-40', 'CTL_XY=1']),
            {'CTL_W0': 1080.0, 'CTL_FF_TH': -40.0, 'CTL_XY': 1.0},
        )
        self.assertEqual(parse_assignments(['x=60,vx=25'], upper=False), {'x': 60.0, 'vx': 25.0})
        with self.assertRaises(ValueError):
            parse_assignments(['CTL_W0'])

    def test_options_refuse_bad_tests(self):
        with self.assertRaises(ValueError):
            Options(height_m=0.003, time_s=20, mode='dry').check()  # 0,3 cm: era em m
        with self.assertRaises(ValueError):
            Options(height_m=0.6, time_s=20, mode='dry').check()  # acima do teto
        with self.assertRaises(ValueError):
            Options(height_m=0.3, time_s=20, mode='dry', params={'FLT_SIM': 1}).check()
        with self.assertRaises(ValueError):
            Options(height_m=0.3, time_s=20, mode='dry', q={'altura': 1}).check()
        Options(height_m=0.3, time_s=20, mode='motors').check()


def _c_table(name, rows, cols):
    sys.path.insert(0, str(REPO_ROOT / 'tools'))
    import lqg_modelo

    text = (REPO_ROOT / 'App/Src/control/lqg_ajustado_gains.c').read_text(encoding='utf-8')
    return lqg_modelo._ler_tabela(text, name, rows, cols)


class AdjustedGainsTest(unittest.TestCase):
    """Os ganhos que a Pi projeta são os do firmware quando o Q é o de fábrica."""

    @staticmethod
    def _err(a, b):
        scale = max(abs(v) for row in b for v in row)
        return max(abs(x - y) for ra, rb in zip(a, b) for x, y in zip(ra, rb)) / scale

    def test_factory_q_matches_firmware_table(self):
        gains, q, r = adjusted_gains({}, None)
        self.assertEqual(r, 0.1)
        self.assertLess(self._err(gains.Kd, _c_table('LQG_AJ_KD', 4, 12)), 1e-9)
        self.assertLess(self._err(gains.Kfd, _c_table('LQG_AJ_KFD', 4, 4)), 1e-9)
        self.assertLess(self._err(gains.M, _c_table('LQG_AJ_M', 12, 9)), 1e-9)
        self.assertLess(self._err(gains.L, _c_table('LQG_AJ_L', 12, 9)), 1e-9)

    def test_heavier_position_weight_raises_position_gain(self):
        base, _, _ = adjusted_gains({}, None)
        firm, q, _ = adjusted_gains({'x': 100.0, 'y': 100.0}, None)
        self.assertEqual(q[0], 100.0)
        self.assertGreater(max(abs(r[0]) for r in firm.Kd), max(abs(r[0]) for r in base.Kd))
        with self.assertRaises(ValueError):
            adjusted_gains({'altura': 1.0}, None)
        with self.assertRaises(ValueError):
            adjusted_gains({}, 0.0)


@unittest.skipUnless(sys.platform.startswith('linux'), 'pty só no Linux')
class HoverTestRunTest(unittest.TestCase):
    """Ensaio completo contra o STM32 simulado com o modelo de voo (10x mais rápido)."""

    def setUp(self):
        from fake_fc import FakeFC
        from dronectl.link import DroneLink

        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.fc = FakeFC(hover=True, time_scale=10.0)
        self.fc.level_deg = (0.4, -0.3)
        self.link = DroneLink(self.fc.port, on_text=lambda t: None)
        self.link.wait_fc(timeout=5.0)
        self.profile = Profile(path=self.dir / 'perfil.json')
        self.lines = []

    def tearDown(self):
        self.link.close()
        self.fc.close()
        self.tmp.cleanup()

    def _run(self, **kw):
        from dronectl.hover_test import HoverTest

        opts = dict(
            height_m=0.30, time_s=1.0, mode='sim', countdown_s=0.0, out_dir=self.dir / 'ensaios'
        )
        opts.update(kw)
        self.lines.clear()
        test = HoverTest(self.link, Options(**opts), self.profile, self.lines.append)
        test.run()
        self.assertIsNone(test.error, '\n'.join(self.lines))
        return test

    def _when(self, cond, action, timeout=10.0):
        """Roda ``action`` numa thread assim que ``cond`` valer."""

        def watch():
            end = time.monotonic() + timeout
            while time.monotonic() < end and not cond():
                time.sleep(0.01)
            action()

        threading.Thread(target=watch, daemon=True).start()

    def test_sim_flight_records_analyzes_and_restores(self):
        test = self._run(label='primeiro')
        self.assertIsNone(test.error, self.lines)
        self.assertEqual(test.land_reason, 'fim do tempo de hover')
        for name in (
            'real_log.csv',
            'gains.json',
            'telemetria.csv',
            'textos.txt',
            'meta.json',
            'resumo.json',
            'linha_do_tempo.txt',
        ):
            self.assertTrue((test.folder / name).exists(), name)
        with open(test.folder / 'real_log.csv', newline='', encoding='utf-8') as f:
            rows = list(csv.DictReader(f, delimiter=';'))
        self.assertGreater(len(rows), 10)
        self.assertEqual(rows[0]['mode'], 'sim')
        self.assertIn('m1_pct', rows[0])
        self.assertIn('x_m', rows[0])
        self.assertIn('param_CTL_W0', rows[0])
        self.assertEqual(float(rows[0]['param_CTL_W0']), 1050.0)
        gains = json.loads((test.folder / 'gains.json').read_text(encoding='utf-8'))
        self.assertEqual(gains['crc'], '21F15F2A')
        self.assertEqual(len(gains['Kd']), 4)
        self.assertEqual(self.fc.state, 2)
        # Volta ao estado seguro: sem telemetria rápida, a seco, sem simulado.
        self.assertEqual(self.fc.params['DBG_RATE_HZ'][0], 0.0)
        self.assertEqual(self.fc.params['FLT_DRY_RUN'][0], 1.0)
        self.assertEqual(self.fc.params['FLT_SIM'][0], 0.0)
        m = test.report.metrics
        self.assertIsNotNone(m['phases_s']['reached'])
        self.assertAlmostEqual(m['height']['mean_cm'], 30.0, delta=1.0)
        s = test.report.suggestion
        self.assertAlmostEqual(s['SNS_LVL_PHI'], -0.4, delta=0.1)
        self.assertAlmostEqual(s['SNS_LVL_TH'], 0.3, delta=0.1)
        self.assertEqual(s['CTL_W0'], 1085)
        # A pasta reanalisada dá o mesmo resultado.
        self.assertEqual(report_folder(test.folder).suggestion, s)
        # O perfil guarda a sugestão, o histórico e os valores de fábrica deste boot.
        back = Profile.load(self.profile.path)
        self.assertEqual(back.suggestion, s)
        self.assertEqual(back.history[-1]['label'], 'primeiro')
        self.assertEqual(back.boot_values['CTL_W0'], 1050.0)
        self.assertEqual(back.boot_id, self.fc.boot_id)
        # Com a sugestão aplicada, o próximo voo fica no ponto.
        self.profile.apply_suggestion(['SNS_LVL_PHI', 'SNS_LVL_TH'])
        test = self._run()
        self.assertLess(test.report.metrics['xy']['rms_cm'], 3.0)
        self.assertAlmostEqual(test.applied['SNS_LVL_PHI'], -0.4, delta=0.1)

    def test_one_off_set_does_not_leak_into_next_test(self):
        test = self._run(params={'CTL_W0': 1100.0})
        self.assertEqual(test.applied['CTL_W0'], 1100.0)
        self.assertEqual(self.fc.params['CTL_W0'][0], 1100.0)
        test = self._run()
        self.assertEqual(test.applied['CTL_W0'], 1050.0)
        meta = json.loads((test.folder / 'meta.json').read_text(encoding='utf-8'))
        self.assertEqual(meta['params']['CTL_W0'], 1050.0)

    def test_q_weights_upload_gains_and_factory_restores(self):
        test = self._run(q={'x': 60.0})
        self.assertTrue(test.gains_info['custom'])
        gains, _, _ = adjusted_gains({'x': 60.0}, None)
        self.assertEqual(self.fc.active_crc, gains.crc32())
        self.assertEqual(self.fc.custom, 1)
        self._run()
        self.assertEqual(self.fc.custom, 0)

    def test_geofence_lands(self):
        self.fc.drift = (0.12, 0.0)
        test = self._run(time_s=5.0)
        self.assertIn('cerca', test.land_reason)
        self.assertEqual(self.fc.state, 2)

    def test_telemetry_loss_lands(self):
        self._when(lambda: self.fc.state == 6, lambda: setattr(self.fc, 'mute_debug', True))
        test = self._run(time_s=5.0)
        self.assertEqual(test.land_reason, 'telemetria parou')
        self.assertIn(self.fc.mod.DRONE_ACTION_LAND, [leg[0] for leg in self.fc.legs])
        self.assertEqual(self.fc.state, 2)

    def test_second_ctrl_c_cuts_motors(self):
        import signal

        from dronectl.hover_test import HoverTest

        self.fc.land_rate = 0.0  # o pouso nunca termina
        opts = Options(
            height_m=0.30, time_s=30.0, mode='sim', countdown_s=0.0, out_dir=self.dir / 'ensaios'
        )
        test = HoverTest(self.link, opts, self.profile, self.lines.append)
        self._when(lambda: self.fc.state == 6, lambda: test._request_stop(signal.SIGINT, None))
        self._when(lambda: self.fc.state == 8, lambda: test._request_stop(signal.SIGINT, None))
        test.run()
        self.assertEqual(test.stop_reason, 'Ctrl+C')
        self.assertEqual(test.land_reason, 'Ctrl+C')
        self.assertIn(False, self.fc.arm_commands)
        self.assertEqual(self.fc.state, 2)
        self.assertTrue(any('CORTANDO OS MOTORES' in line for line in self.lines))

    def test_dry_run_never_lifts_and_lands(self):
        test = self._run(mode='dry', time_s=0.3)
        self.assertEqual(test.land_reason, 'fim do ensaio a seco')
        self.assertTrue(any('não saiu do chão' in n for n in test.report.notes))
        self.assertEqual(self.fc.state, 2)


if __name__ == '__main__':
    unittest.main()
