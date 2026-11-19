"""Testes do projeto LQR em Python e do cálculo de inércia (rodar de stm32/, ambiente aeroroute).

conda run -n aeroroute python tests/test_lqr_design.py
"""

import json
import re
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import inertia_calc
import loop_margin
import lqr_design as d


class GeneratedModel(unittest.TestCase):
    """O modelo do Simulink Coder sai da física e dos pesos da submissão, sem MATLAB."""

    def test_physics_reproduces_generated_matrices(self):
        gen = d.load_generated()
        ac, bc = d.continuous(d.Params(xy_leak=1e-3))
        ad, bd = d.discretize(ac, bc)
        self.assertLess(np.abs(ad - gen['A']).max(), 1e-12)
        self.assertLess(np.abs(bd - gen['B']).max(), 1e-12)

    def test_riccati_reproduces_generated_gains(self):
        gen = d.load_generated()
        q = np.diag(d.Q_SUBMISSION).astype(float)
        k, kfd, pole = d.design(gen['A'], gen['B'], q, d.R_SUBMISSION * np.eye(4))
        self.assertLess(np.abs(k - gen['Kd']).max(), 1e-8)
        self.assertLess(np.abs(kfd - gen['Kfd']).max(), 1e-8)
        self.assertLess(pole, 1.0)

    def test_header_matches_design(self):
        gen = d.load_generated()
        text = d.format_header(gen['Kd'], gen['Kfd'], 'teste')
        self.assertIn('LQG_KD[48]', text)
        self.assertIn('LQG_KFD[16]', text)

    def test_header_carries_hover_trim(self):
        gen = d.load_generated()
        zero = d.format_header(gen['Kd'], gen['Kfd'], 'teste')
        self.assertIn('LQG_HOVER_TRIM_PERMILLE[4] = {0.0000F, 0.0000F, 0.0000F, 0.0000F}', zero)
        p = d.Params(cg_m=(0.010, 0.0))
        text = d.format_header(gen['Kd'], gen['Kfd'], 'teste', p.hover_trim_permille())
        values = re.search(r'LQG_HOVER_TRIM_PERMILLE\[4\] = \{(.*?)\}', text).group(1)
        parsed = [float(v.rstrip('F')) for v in values.split(',')]
        self.assertTrue(np.allclose(parsed, p.hover_trim_permille(), atol=1e-3))
        self.assertGreater(parsed[0], 0.0)  # CG à frente: u1, u2 (frente) pedem mais
        self.assertLess(parsed[2], 0.0)

    def test_params_only_emits_trim_with_generated_gains(self):
        with tempfile.TemporaryDirectory() as tmp:
            params = Path(tmp) / 'p.json'
            params.write_text(json.dumps({'cg_m': [0.008, 0.0]}))
            out = Path(tmp) / 'g.h'
            self.assertEqual(d.main(['emit', '--params', str(params), '--out', str(out)]), 0)
            text = out.read_text()
        self.assertIn('A e B do modelo gerado', text)
        values = re.search(r'LQG_HOVER_TRIM_PERMILLE\[4\] = \{(.*?)\}', text).group(1)
        parsed = [float(v.rstrip('F')) for v in values.split(',')]
        want = d.Params(cg_m=(0.008, 0.0)).hover_trim_permille()
        self.assertTrue(np.allclose(parsed, want, atol=1e-3))
        gen = d.load_generated()
        self.assertIn(f'{gen["Kd"].flatten(order="F")[0]:.9g}'[:6], text)  # ganhos do modelo gerado

    def test_trim_option_sets_the_table(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'g.h'
            args = ['emit', '--trim', '-15', '15', '-15', '15', '--out', str(out)]
            self.assertEqual(d.main(args), 0)
            text = out.read_text()
        self.assertIn('{-15.0000F, 15.0000F, -15.0000F, 15.0000F}', text)

    def test_identify_recovers_hover_speed(self):
        gen = d.load_generated()
        ident = d.identify(gen['A'], gen['B'])
        self.assertAlmostEqual(ident['w0'], d.Params().w0, places=3)
        self.assertAlmostEqual(ident['xy_leak'], 1e-3, places=9)


class MassDistribution(unittest.TestCase):
    def test_centered_cg_is_symmetric(self):
        p = d.Params()
        self.assertTrue(np.allclose(p.thrust_hover, p.mass * d.G / 4))
        self.assertTrue(np.allclose(p.hover_trim_permille(), 0.0))

    def test_cg_forward_shifts_thrust_to_the_front(self):
        # CG 10 mm à frente: os motores da frente ficam mais perto do CG, então empurram mais.
        p = d.Params(cg_m=(0.010, 0.0))
        t = p.thrust_hover
        front, rear = t[:2].sum(), t[2:].sum()
        self.assertGreater(front, rear)
        self.assertAlmostEqual(t.sum(), p.mass * d.G, places=9)
        xy = p.motor_xy
        self.assertAlmostEqual(float((xy[:, 0] * t).sum()), 0.0, places=9)
        self.assertAlmostEqual(float((xy[:, 1] * t).sum()), 0.0, places=9)
        self.assertGreater(np.abs(p.hover_trim_permille()).max(), 1.0)

    def test_cg_outside_motors_has_no_equilibrium(self):
        with self.assertRaises(ValueError):
            _ = d.Params(cg_m=(0.2, 0.0)).w_hover

    def test_products_of_inertia_couple_axes(self):
        base = d.Params()
        tensor = base.inertia_matrix.copy()
        tensor[0, 2] = tensor[2, 0] = 2e-4
        _, bc = d.continuous(d.Params(inertia_tensor=tuple(tensor.flatten())))
        _, b0 = d.continuous(base)
        # Ixz acopla o torque de rolagem à guinada: B[r] de u1 muda e deixa de ser simétrico.
        self.assertGreater(np.abs(bc[11] - b0[11]).max(), 1e-3)
        self.assertLess(np.abs(bc[8] - b0[8]).max(), 1e-15)

    def test_design_with_offset_cg_is_stable(self):
        p = d.Params(cg_m=(0.01, 0.004))
        ac, bc = d.continuous(p)
        ad, bd = d.discretize(ac, bc)
        k, _, pole = d.design(ad, bd, np.diag(d.Q_SUBMISSION).astype(float), 0.1 * np.eye(4))
        self.assertLess(pole, 1.0)
        self.assertEqual(k.shape, (4, 12))


class InertiaCalc(unittest.TestCase):
    def test_point_masses_parallel_axis(self):
        comps = [
            {'name': 'a', 'mass_g': 100, 'shape': 'point', 'pos_mm': [50, 0, 0]},
            {'name': 'b', 'mass_g': 100, 'shape': 'point', 'pos_mm': [-50, 0, 0]},
        ]
        r = inertia_calc.compose(comps)
        self.assertTrue(np.allclose(r['cg_m'], 0.0))
        self.assertAlmostEqual(r['tensor'][1, 1], 2 * 0.1 * 0.05**2, places=12)
        self.assertAlmostEqual(r['tensor'][0, 0], 0.0, places=12)

    def test_box_and_ring_own_inertia(self):
        box = {'mass_g': 120, 'shape': 'box', 'dims_mm': [100, 60, 20], 'pos_mm': [0, 0, 0]}
        ib = inertia_calc.own_inertia(box)
        self.assertAlmostEqual(ib[2, 2], 0.12 / 12 * (0.1**2 + 0.06**2), places=12)
        ring = {'mass_g': 10, 'shape': 'ring', 'radius_mm': 40, 'pos_mm': [0, 0, 0]}
        self.assertAlmostEqual(inertia_calc.own_inertia(ring)[2, 2], 0.01 * 0.04**2, places=12)

    def test_cad_tensor_part(self):
        tensor = [[1e-5, 0, 0], [0, 2e-5, 0], [0, 0, 3e-5]]
        part = {
            'name': 'placa',
            'mass_g': 50,
            'shape': 'tensor',
            'inertia_kgm2': tensor,
            'pos_mm': [0, 0, 0],
        }
        self.assertTrue(np.allclose(inertia_calc.own_inertia(part), tensor))
        r = inertia_calc.compose([part])
        self.assertTrue(np.allclose(r['tensor'], tensor))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'c.json'
            part['inertia_kgm2'] = [[None] * 3] * 3
            path.write_text(json.dumps({'components': [part]}))
            with self.assertRaises(SystemExit):
                inertia_calc.load(path)

    def test_refuses_null_fields(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'c.json'
            path.write_text(
                json.dumps(
                    {
                        'components': [
                            {'name': 'x', 'mass_g': 5, 'pos_mm': [None, 0, 0], 'shape': 'point'}
                        ]
                    }
                )
            )
            with self.assertRaises(SystemExit):
                inertia_calc.load(path)


class LoopMargin(unittest.TestCase):
    """Atraso e motor de 1ª ordem sobre a planta: os multiplicadores altos perdem a estabilidade."""

    GAINS = dict(z=10, w=5, att=1, rate=1, yaw=5, yaw_rate=5, xy=1, vel=1)

    def setUp(self):
        self.p = loop_margin.load_params(loop_margin.PARAMS)
        self.ad, self.bd = d.discretize(*d.continuous(self.p))
        self.kd = d.load_generated()['Kd']

    def radius(self, att, tau, delay):
        g = dict(self.GAINS, att=att, rate=att)
        k = loop_margin.scaled_gains(self.kd, g)
        return loop_margin.spectral_radius(self.ad, self.bd, k, tau, delay)

    def test_design_gains_robust_to_delay_and_lag(self):
        g = dict(self.GAINS)
        self.assertLess(loop_margin.worst_case(self.p, loop_margin.scaled_gains(self.kd, g)), 1.0)

    def test_attitude_5x_loses_stability_with_two_step_delay(self):
        self.assertLess(self.radius(2, 0.04, 2), 1.0)
        self.assertGreater(self.radius(5, 0.04, 2), 1.0)

    def test_matches_ideal_plant_without_delay_effects(self):
        k = loop_margin.scaled_gains(self.kd, self.GAINS)
        ideal = np.abs(np.linalg.eigvals(self.ad - self.bd @ k)).max()
        self.assertAlmostEqual(self.radius(1, 0.02, 1), ideal, places=3)


if __name__ == '__main__':
    unittest.main()
