"""Testes da missão passo a passo (dronectl/missao.py).

Rodar a partir de tools/pi:  python3 -m unittest discover -s tests
"""

import math
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dronectl.missao import interpretar, rumo  # noqa: E402


class InterpretarTest(unittest.TestCase):
    def test_comandos(self):
        self.assertIsNone(interpretar('   # comentário'))
        self.assertEqual(interpretar('takeoff 50').numeros, [50])
        self.assertEqual(interpretar('YAW -90').numeros, [-90])
        self.assertEqual(interpretar('goto 100 -200 # vai').numeros, [100, -200])
        self.assertEqual(
            interpretar('waypoints 100,0 150,-50 0,0').pontos, [(100, 0), (150, -50), (0, 0)]
        )
        v = interpretar('route 100,0 0,100,80')  # sem algoritmo: o genético
        self.assertEqual((v.algoritmo, v.pontos), ('ga', [(100, 0), (0, 100, 80)]))
        v = interpretar('route')  # sem pontos: lidos do arquivo na execução
        self.assertEqual((v.algoritmo, v.pontos), ('ga', []))
        self.assertEqual(interpretar('route exact 1,1,30').algoritmo, 'exact')
        self.assertEqual(interpretar('goto 100 50 120').numeros, [100, 50, 120])
        self.assertEqual(interpretar('route 2optmulti 1,1,30').algoritmo, '2optmulti')
        self.assertEqual(interpretar('route bruteforce 1,1,30 2,2').algoritmo, 'bruteforce')
        for ruim in (
            'waypoints',
            'goto 1 2 3 4',
            'goto 1 2 200',
            'waypoints 1,2,5',
            'waypoints 1,2,3,4',
            'route dijkstra 1,1',
            'route 1,2,5',
            'route exact ' + ' '.join(f'{i},0' for i in range(1, 18)),
            'route bruteforce ' + ' '.join(f'{i},0' for i in range(1, 11)),
            'fly 1',
            'takeoff',
            'takeoff -1',
            'takeoff 200',
            'forward 0',
            'goto 1',
            'waypoints 1;2',
            'forward nan',
            'forward 0,5',
            'forward 0.5',
            'route 1.5,0',
            'decolar 50',
            'visitar ga 1,1',
            'mission ga 1,1',
        ):
            with self.assertRaises(ValueError, msg=ruim):
                interpretar(ruim)


class RumoTest(unittest.TestCase):
    """NED: x = frente inicial, y = direita inicial, guinada horária."""

    def test_rumo(self):
        self.assertEqual(rumo(0, 0, 0, 1, 0), (0.0, 1.0))
        giro, dist = rumo(0, 0, 0, 0, 2)  # à direita: gira +90°
        self.assertAlmostEqual(giro, 90.0)
        self.assertAlmostEqual(dist, 2.0)
        giro, _ = rumo(0, 0, 0, 0, -1)  # à esquerda: -90°
        self.assertAlmostEqual(giro, -90.0)
        giro, dist = rumo(1, 1, math.pi / 2, 0, 0)  # de frente para leste, voltar à origem
        self.assertAlmostEqual(giro, 135.0)
        self.assertAlmostEqual(dist, math.sqrt(2))
        giro, _ = rumo(0, 0, math.radians(170), -1, -0.1)  # giro pelo lado mais curto
        self.assertLess(abs(giro), 180.0)


@unittest.skipUnless(sys.platform.startswith('linux'), 'pty só no Linux')
class MissaoTest(unittest.TestCase):
    """Missão inteira contra o STM32 simulado (tests/fake_fc.py)."""

    def setUp(self):
        from fake_fc import FakeFC
        from dronectl.link import DroneLink
        from dronectl.missao import Missao

        self.fc = FakeFC()
        self.saida = []
        self.link = DroneLink(self.fc.port, on_text=self.saida.append)
        self.link.wait_fc(timeout=5.0)
        self.missao = Missao(self.link, saida=self.saida.append)

    def tearDown(self):
        self.link.close()
        self.fc.close()

    def _pose(self):
        time.sleep(0.5)  # LOCAL_POSITION_NED/ATTITUDE chegam a 5 Hz
        return self.missao.posicao()

    def test_mover_antes_de_decolar_e_recusado(self):
        from dronectl.link import LinkError

        with self.assertRaises(LinkError):
            self.missao.executar(interpretar('forward 100'))
        self.assertEqual(self.fc.legs, [])  # a Pi nem mandou

    def test_stm32_recusa_trecho_fora_de_hold(self):
        from dronectl.link import LinkError

        with self.assertRaises(LinkError):
            self.link.move(1.0, 0.0)  # direto, sem a checagem da Pi
        self.assertEqual(self.fc.state, 2)

    def test_rota_completa(self):
        ok = self.missao.executar_linhas(
            [
                'takeoff 50',
                'yaw 90',  # nariz para a direita inicial (+y)
                'forward 100',  # (0, 100) cm
                'goto 100 100',  # gira -90 e anda 100 cm
                'waypoints 100,0 0,0',  # dois waypoints, na ordem dada
                'land',
            ]
        )
        self.assertTrue(ok, self.saida)
        acoes = [a for a, *_ in self.fc.legs]
        a = self.fc.mod
        self.assertEqual(acoes[0], a.DRONE_ACTION_TAKEOFF_RELATIVE)
        self.assertEqual(acoes[-1], a.DRONE_ACTION_LAND)
        self.assertAlmostEqual(self.fc.x, 0.0, places=3)
        self.assertAlmostEqual(self.fc.y, 0.0, places=3)
        self.assertEqual(self.fc.state, 2)
        # Cada movimento foi "yaw" e depois "forward" (sem andar de lado).
        movimentos = [
            (act, x, y)
            for act, x, y, z, yaw in self.fc.legs
            if act == a.DRONE_ACTION_TRANSLATE_RELATIVE
        ]
        self.assertTrue(all(abs(y) < 1e-6 and x > 0 for _, x, y in movimentos), movimentos)

    def test_route_exact_volta_a_decolagem(self):
        ok = self.missao.executar_linhas(
            ['takeoff 50', 'route exact 200,0 0,200 200,200 100,-100', 'land']
        )
        self.assertTrue(ok, self.saida)
        self.assertAlmostEqual(self.fc.x, 0.0, places=3)
        self.assertAlmostEqual(self.fc.y, 0.0, places=3)
        self.assertTrue(any('exato: ordem' in s for s in self.saida), self.saida)

    def test_route_3d(self):
        ok = self.missao.executar_linhas(
            ['takeoff 50', 'route 2optmulti 200,0,100 0,200,30 200,200,150', 'goto 0 0 80', 'land']
        )
        self.assertTrue(ok, self.saida)
        self.assertAlmostEqual(self.fc.x, 0.0, places=3)
        self.assertAlmostEqual(self.fc.y, 0.0, places=3)
        # Subiu e desceu: o último "goto" deixou o drone a 80 cm antes do pouso.
        alturas = [
            -z
            for act, x, y, z, yaw in self.fc.legs
            if act == self.fc.mod.DRONE_ACTION_TRANSLATE_RELATIVE
        ]
        self.assertTrue(any(abs(dz) > 0.1 for dz in alturas), alturas)

    def test_route_do_arquivo_quadrado(self):
        """A missão de exemplo: decola, quadrado de 40 cm do arquivo, volta e pousa."""
        raiz = os.path.join(os.path.dirname(__file__), '..', 'missoes')
        with open(os.path.join(raiz, 'quadrado_40cm.txt'), encoding='utf-8') as f:
            linhas = f.read().splitlines()
        ok = self.missao.executar_linhas(linhas)
        self.assertTrue(ok, self.saida)
        self.assertTrue(any('genetico: ordem' in s for s in self.saida), self.saida)
        self.assertTrue(any('3 pontos de' in s for s in self.saida), self.saida)
        self.assertAlmostEqual(self.fc.x, 0.0, places=3)
        self.assertAlmostEqual(self.fc.y, 0.0, places=3)
        self.assertEqual(self.fc.state, 2)

    def test_resposta_antiga_de_outra_missao_e_ignorada(self):
        """Um DRONE_LEG_ACK antigo com o mesmo command_id, mas de outra missão
        (outro programa no mesmo enlace), não pode encerrar o trecho atual."""

        class AckAntigo:
            mission_id = self.link._mission_id ^ 1
            command_id = self.link._command_id + 1
            result = self.fc.mod.MAV_RESULT_DENIED
            reason = 11  # sem autorização do operador
            flight_state = 2

            def get_type(self):
                return 'DRONE_LEG_ACK'

        with self.link._cond:
            self.link._seq += 1
            self.link._inbox.append((self.link._seq, AckAntigo()))
        ok = self.missao.executar_linhas(['takeoff 50', 'land'])
        self.assertTrue(ok, self.saida)
        self.assertFalse(any('sem autorização' in s for s in self.saida), self.saida)

    def test_erro_no_meio_pousa(self):
        ok = self.missao.executar_linhas(['takeoff 50', 'fly 3', 'forward 100'])
        self.assertFalse(ok)
        self.assertEqual(self.fc.state, 2)  # pousou
        self.assertTrue(any('erro na linha 2' in s for s in self.saida), self.saida)


if __name__ == '__main__':
    unittest.main()
