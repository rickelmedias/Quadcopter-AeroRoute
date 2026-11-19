"""Missão passo a passo (dronectl/mission.py) e hover (dronectl/hover.py)."""

import math
import sys
import tempfile
import time
import unittest
from pathlib import Path

from dronectl.mission import heading, parse

ROOT = Path(__file__).resolve().parents[1]


class ParseTest(unittest.TestCase):
    def test_commands(self):
        self.assertIsNone(parse('   # comentário'))
        self.assertEqual(parse('takeoff 50').numbers, [50])
        self.assertEqual(parse('YAW -90').numbers, [-90])
        self.assertEqual(parse('goto 100 -200 # vai').numbers, [100, -200])
        self.assertEqual(
            parse('waypoints 100,0 150,-50 0,0').points, [(100, 0), (150, -50), (0, 0)]
        )
        v = parse('route 100,0 0,100,60')
        self.assertEqual((v.algorithm, v.points), ('ga', [(100, 0), (0, 100, 60)]))
        self.assertEqual((parse('route').algorithm, parse('route').points), ('ga', []))
        self.assertEqual(parse('route exact 1,1,30').algorithm, 'exact')
        self.assertEqual(parse('route 2optmulti 1,1,30').algorithm, '2optmulti')
        for bad in (
            'waypoints',
            'goto 1 2 3 4',
            'goto 1 2 120',
            'goto 400 0',
            'waypoints 1,2,5',
            'route dijkstra 1,1',
            'route 1,2,5',
            'fly 1',
            'takeoff',
            'takeoff -1',
            'takeoff 100',
            'takeoff 5',
            'forward 0',
            'goto 1',
            'forward 0.5',
            'route exact ' + ' '.join(f'{i},0' for i in range(1, 18)),
            'route bruteforce ' + ' '.join(f'{i},0' for i in range(1, 11)),
            'decolar 50',
        ):
            with self.assertRaises(ValueError, msg=bad):
                parse(bad)

    def test_heading(self):
        self.assertEqual(heading(0, 0, 0, 1, 0), (0.0, 1.0))
        turn, dist = heading(0, 0, 0, 0, 2)
        self.assertAlmostEqual(turn, 90.0)
        self.assertAlmostEqual(dist, 2.0)
        turn, dist = heading(1, 1, math.pi / 2, 0, 0)
        self.assertAlmostEqual(turn, 135.0)
        self.assertLess(abs(heading(0, 0, math.radians(170), -1, -0.1)[0]), 180.0)


class SequenceTest(unittest.TestCase):
    def test_reordered_is_not_lost(self):
        from dronectl.link import DroneLink

        link = DroneLink.__new__(DroneLink)  # só o contador, sem porta
        link._last_rx_seq = None
        link.rx_gaps = link.rx_repeats = link.rx_reordered = 0
        from collections import deque

        link._missing = deque(maxlen=32)
        link.gap_events = deque(maxlen=200)
        for seq in (252, 253, 255, 254, 0, 2):  # 254 atrasada; 1 perdida na volta do 255
            link._count_gap(seq)
        self.assertEqual((link.rx_gaps, link.rx_reordered, link.rx_repeats), (1, 1, 0))
        link._count_gap(2)
        self.assertEqual(link.rx_repeats, 1)


class AnalyzeTest(unittest.TestCase):
    def test_summary_uses_hold_steps_only(self):
        from dronectl.hover import analyze
        from dronectl.link import DebugSample

        def s(t, state, z, x=0.0, sat=0):
            return DebugSample(
                t,
                t,
                [0] * 9,
                [x, 0, z] + [0] * 9,
                [0] * 4,
                [0, 0, -0.30, 0],
                [0.55] * 4,
                state,
                15,
                sat,
            )

        samples = [s(0, 5, -0.10)] + [
            s(100 * i, 6, -0.32, 0.03, 8 if i % 4 == 0 else 0) for i in range(1, 11)
        ]
        r = analyze(samples)
        self.assertEqual(r['hold_samples'], 10)
        self.assertAlmostEqual(r['height_err_mean_m'], 0.02, places=6)
        self.assertAlmostEqual(r['xy_dev_rms_m'], 0.03, places=6)
        self.assertAlmostEqual(r['saturated_fraction'], 0.2)
        self.assertEqual(analyze([s(0, 5, 0.0)]), {'hold_samples': 0})


@unittest.skipUnless(sys.platform.startswith('linux'), 'pty só no Linux')
class MissionTest(unittest.TestCase):
    """Missão inteira contra o STM32 simulado (tests/fake_fc.py)."""

    def setUp(self):
        from dronectl.link import DroneLink
        from dronectl.mission import Mission

        from .fake_fc import FakeFC

        self.fc = FakeFC()
        self.out = []
        self.link = DroneLink(self.fc.port, on_text=self.out.append)
        self.link.wait_fc(timeout=5.0)
        self.mission = Mission(self.link, out=self.out.append)

    def tearDown(self):
        self.link.close()
        self.fc.close()

    def test_move_before_takeoff_rejected(self):
        from dronectl.link import LinkError

        with self.assertRaises(LinkError):
            self.mission.run(parse('forward 100'))
        self.assertEqual(self.fc.legs, [])  # a Pi nem mandou

    def test_arm_needs_lease(self):
        from dronectl.link import LinkError

        with self.assertRaises(LinkError):
            self.link.arm(True)
        self.assertEqual(self.fc.state, 2)

    def test_full_route(self):
        ok = self.mission.run_lines(
            [
                'takeoff 50',
                'yaw 90',
                'forward 100',
                'goto 100 100',
                'waypoints 100,0 0,0',
                'land',
            ]
        )
        self.assertTrue(ok, self.out)
        a = self.fc.mod
        actions = [x[0] for x in self.fc.legs]
        self.assertEqual(actions[0], a.DRONE_ACTION_TAKEOFF_RELATIVE)
        self.assertEqual(actions[-1], a.DRONE_ACTION_LAND)
        self.assertAlmostEqual(self.fc.x, 0.0, places=3)
        self.assertAlmostEqual(self.fc.y, 0.0, places=3)
        moves = [
            (x, y) for act, x, y, z, yaw in self.fc.legs if act == a.DRONE_ACTION_TRANSLATE_RELATIVE
        ]
        self.assertTrue(all(abs(y) < 1e-6 and x > 0 for x, y in moves), moves)
        time.sleep(0.5)
        self.assertEqual(self.fc.state, 2)

    def test_square_mission_file(self):
        lines = (ROOT / 'missions' / 'stage4_square.txt').read_text(encoding='utf-8').splitlines()
        self.mission.points_file = ROOT / 'missions' / 'points.txt'
        self.assertTrue(self.mission.run_lines(lines), self.out)
        self.assertTrue(any('genetic: ordem' in s for s in self.out), self.out)
        self.assertTrue(any('3 pontos de' in s for s in self.out), self.out)
        self.assertAlmostEqual(self.fc.x, 0.0, places=3)
        self.assertAlmostEqual(self.fc.y, 0.0, places=3)

    def test_stage_files(self):
        """Etapas 1 a 3: cada uma termina no chão, de volta a (0, 0) e com a guinada inicial."""
        import dronectl.mission as m

        self.addCleanup(setattr, m, 'MARGIN_S', m.MARGIN_S)
        for name in ('stage1_hover.txt', 'stage2_a_b_a.txt', 'stage3_yaw.txt'):
            lines = (ROOT / 'missions' / name).read_text(encoding='utf-8').splitlines()
            # "wait" de verdade deixaria o teste lento: troca por 1 s
            lines = ['wait 1' if ln.startswith('wait') else ln for ln in lines]
            self.assertTrue(self.mission.run_lines(lines), (name, self.out))
            self.assertAlmostEqual(self.fc.x, 0.0, places=3, msg=name)
            self.assertAlmostEqual(self.fc.y, 0.0, places=3, msg=name)
            self.assertAlmostEqual(self.fc.yaw, 0.0, places=3, msg=name)
            time.sleep(0.5)
            self.assertEqual(self.fc.state, 2, name)

    def test_ctrl_c_is_emergency_stop(self):
        """Ctrl+C no meio da missão corta os motores na hora e não tenta pousar."""
        real_wait = self.link.wait_leg
        calls = []

        def interrupt_on_second_leg(command_id, timeout):
            calls.append(command_id)
            if len(calls) == 2:
                raise KeyboardInterrupt
            return real_wait(command_id, timeout)

        self.link.wait_leg = interrupt_on_second_leg
        t0 = time.monotonic()
        self.assertFalse(self.mission.run_lines(['takeoff 30', 'forward 40', 'land']))
        self.assertLess(time.monotonic() - t0, 10.0)
        a = self.fc.mod
        self.assertEqual(self.fc.arm_commands[-1], False)  # desarmou
        self.assertNotEqual(self.fc.legs[-1][0], a.DRONE_ACTION_LAND)  # não pediu pouso
        self.assertEqual(self.fc.state, 2)
        self.assertTrue(any('PARADA DE EMERGÊNCIA' in s for s in self.out), self.out)
        self.assertFalse(any('sem confirmação' in s for s in self.out), self.out)

    def test_error_in_the_middle_lands(self):
        self.assertFalse(self.mission.run_lines(['takeoff 50', 'fly 3', 'forward 100']))
        time.sleep(0.5)
        self.assertEqual(self.fc.state, 2)
        self.assertTrue(any('erro na linha 2' in s for s in self.out), self.out)

    def test_hover_records(self):
        from dronectl.hover import run_hover

        with tempfile.TemporaryDirectory() as tmp:
            r = run_hover(self.link, 30, 1.5, root=Path(tmp), out=self.out.append)
            self.assertTrue(r['completed'], self.out)
            self.assertGreater(r['hold_samples'], 5)
            self.assertLess(abs(r['height_err_mean_m']), 0.01)
            folder = next(Path(tmp).iterdir())
            self.assertTrue((folder / 'telemetry.csv').exists())
            self.assertIn('"completed": true', (folder / 'summary.json').read_text())
        time.sleep(0.5)
        self.assertEqual(self.fc.state, 2)


if __name__ == '__main__':
    unittest.main()
