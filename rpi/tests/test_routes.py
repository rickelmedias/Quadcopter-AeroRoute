"""Algoritmos de rota (dronectl/routes.py). Não precisam do drone."""

import itertools
import unittest

from dronectl import routes


class RoutesTest(unittest.TestCase):
    def test_exact_equals_brute_force(self):
        for n in range(1, 8):
            for seed in range(3):
                p = routes.random_points(n, seed)
                best = min(routes.cost(list(o), p) for o in itertools.permutations(range(n)))
                self.assertAlmostEqual(routes.cost(routes.held_karp(p), p), best, places=6)
                self.assertAlmostEqual(routes.cost(routes.brute_force(p), p), best, places=6)

    def test_all_return_permutations_never_below_optimum(self):
        for n in (3, 6, 9, 12):
            p = routes.random_points(n, 7)
            optimum = routes.cost(routes.held_karp(p), p)
            for name, f in routes.ALGORITHMS.items():
                if name == 'brute-force' and n > 9:
                    continue
                order = f(p)
                self.assertEqual(sorted(order), list(range(n)), name)
                self.assertGreaterEqual(routes.cost(order, p), optimum - 1e-6, name)

    def test_two_opt_does_not_worsen_greedy(self):
        for seed in range(5):
            p = routes.random_points(20, seed)
            g = routes.greedy(p)
            self.assertLessEqual(routes.cost(routes.two_opt(g, p), p), routes.cost(g, p) + 1e-9)

    def test_genetic_deterministic(self):
        p = routes.random_points(15, 3)
        self.assertEqual(routes.genetic(p)[0], routes.genetic(p)[0])
        c, _ = routes.genetic(p, routes.GAParams(seed=7))
        self.assertEqual(sorted(c), list(range(15)))

    def test_3d_and_origin(self):
        self.assertAlmostEqual(routes.distance((0, 0, 0), (3, 4, 12)), 13.0)
        p = routes.random_points(12, 4)
        best, optima = routes.two_opt_multi(p, starts=10)
        self.assertEqual(len(optima), 10)
        self.assertAlmostEqual(routes.cost(best, p), min(optima))
        origin = (50.0, -50.0, 60.0)
        self.assertLessEqual(
            routes.cost(routes.order('exact', p, origin), p, origin),
            routes.cost(routes.order('greedy', p, origin), p, origin) + 1e-9,
        )
        with self.assertRaises(ValueError):
            routes.order('dijkstra', p)


if __name__ == '__main__':
    unittest.main()
