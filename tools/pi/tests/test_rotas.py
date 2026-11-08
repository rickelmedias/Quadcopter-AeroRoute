"""Testes dos algoritmos de rota (dronectl/rotas.py). Não precisam do drone."""
import itertools
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dronectl import rotas  # noqa: E402


class RotasTest(unittest.TestCase):
    def test_exato_igual_a_forca_bruta(self):
        for n in range(1, 8):
            for semente in range(3):
                p = rotas.pontos_aleatorios(n, semente)
                bruta = min(rotas.custo(list(o), p) for o in itertools.permutations(range(n)))
                self.assertAlmostEqual(rotas.custo(rotas.held_karp(p), p), bruta, places=6)
                self.assertAlmostEqual(rotas.custo(rotas.forca_bruta(p), p), bruta, places=6)

    def test_forca_bruta_igual_ao_held_karp(self):
        for semente in range(1, 4):
            p = rotas.pontos_aleatorios(9, semente)
            self.assertAlmostEqual(rotas.custo(rotas.forca_bruta(p), p),
                                   rotas.custo(rotas.held_karp(p), p), places=6)

    def test_todos_dao_permutacao_e_nunca_abaixo_do_otimo(self):
        for n in (3, 6, 9, 12):
            p = rotas.pontos_aleatorios(n, 7)
            otimo = rotas.custo(rotas.held_karp(p), p)
            for nome, f in rotas.ALGORITMOS.items():
                if nome == 'forca-bruta' and n > 9:
                    continue
                ordem = f(p)
                self.assertEqual(sorted(ordem), list(range(n)), nome)
                self.assertGreaterEqual(rotas.custo(ordem, p), otimo - 1e-6, nome)

    def test_2opt_nao_piora_o_guloso(self):
        for semente in range(5):
            p = rotas.pontos_aleatorios(20, semente)
            g = rotas.guloso(p)
            self.assertLessEqual(rotas.custo(rotas.dois_opt(g, p), p), rotas.custo(g, p) + 1e-9)

    def test_genetico_deterministico(self):
        p = rotas.pontos_aleatorios(15, 3)
        a, _ = rotas.genetico(p)
        b, _ = rotas.genetico(p)
        self.assertEqual(a, b)
        c, _ = rotas.genetico(p, rotas.ParamsGA(semente=7))
        self.assertEqual(sorted(c), list(range(15)))

    def test_3d(self):
        self.assertAlmostEqual(rotas.distancia((0, 0, 0), (3, 4, 12)), 13.0)
        p = rotas.pontos_aleatorios(30, 1)
        self.assertTrue(all(30 <= z <= 150 for _, _, z in p))
        # Mesmo xy, alturas diferentes: a ordem ótima muda com o z.
        pts = [(100, 0, 50), (100, 0, 150), (200, 0, 50)]
        self.assertAlmostEqual(rotas.custo(rotas.held_karp(pts), pts),
                               min(rotas.custo(list(o), pts) for o in itertools.permutations(range(3))))

    def test_multipartida_e_ordenar(self):
        p = rotas.pontos_aleatorios(12, 4)
        melhor, otimos = rotas.dois_opt_multi(p, partidas=10)
        self.assertEqual(len(otimos), 10)
        self.assertAlmostEqual(rotas.custo(melhor, p), min(otimos))
        for nome in rotas.ALGORITMOS:
            if nome == 'forca-bruta':
                continue
            self.assertEqual(rotas.ordenar(nome, p), rotas.ALGORITMOS[nome](p), nome)
        origem = (50.0, -50.0, 120.0)
        self.assertLessEqual(rotas.custo(rotas.ordenar('exato', p, origem), p, origem),
                             rotas.custo(rotas.ordenar('guloso', p, origem), p, origem) + 1e-9)


class LimiteExatoTest(unittest.TestCase):
    def test_mede_em_processo_separado(self):
        from dronectl import rotas_limite
        m = rotas_limite.medir(7)
        p = rotas.pontos_aleatorios(7, 1)
        self.assertAlmostEqual(m['custo_cm'], rotas.custo(rotas.held_karp(p), p), places=6)
        self.assertGreater(m['pico_mb'], 0)

    def test_para_no_limite_de_memoria(self):
        from dronectl import rotas_limite
        # Limite minúsculo: com 9 pontos o Held-Karp já passaria de 0,04 MB.
        r = rotas_limite.rodar(n_max=12, mem_max_mb=0.04, escrever=lambda *_: None,
                              forca_tempo_max_s=0.05)
        self.assertGreaterEqual(r['forca_bruta']['ultimo_medido'], 5)
        self.assertIn('Força bruta', rotas_limite.markdown(r))
        self.assertEqual(r['parou_em'], 9)
        self.assertIn('memória', r['motivo'])
        self.assertIn('| N |', rotas_limite.markdown(r))


if __name__ == '__main__':
    unittest.main()
