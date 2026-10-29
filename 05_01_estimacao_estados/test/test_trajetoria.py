"""Trajetória em 8 e lei de controle com pose ruidosa (sem ROS nem Gazebo)."""

import math
from pathlib import Path
import random
from types import SimpleNamespace

import pytest

from sensores.trajetoria import (
    comando_realimentado, erro_no_corpo, erros, normaliza_angulo, referencia,
    yaw_from_quaternion)

RAIZ = Path(__file__).resolve().parents[1]
A, B, W = 1.0, 2.0, 0.2


def _ganhos():
    yaml = pytest.importorskip('yaml')
    with open(RAIZ / 'config' / 'trajetoria_8.yaml', encoding='utf-8') as f:
        return yaml.safe_load(f)['trajetoria_8']['ros__parameters']


def _pid(kp):
    # simple_pid.PID com setpoint 0: saída = kp * (0 - entrada)
    return lambda entrada, dt: -kp * entrada


def test_referencia_comeca_na_origem_e_fecha_a_volta():
    xd, yd, *_ = referencia(0.0, A, B, W)
    assert (xd, yd) == pytest.approx((0.0, 0.0))
    xd, yd, *_ = referencia(2 * math.pi / W, A, B, W)
    assert (xd, yd) == pytest.approx((0.0, 0.0), abs=1e-9)


def test_referencia_cabe_na_caixa_AxB():
    for i in range(1000):
        xd, yd, *_ = referencia(i * 0.05, A, B, W)
        assert abs(xd) <= A + 1e-9 and abs(yd) <= B + 1e-9


@pytest.mark.parametrize('t', [0.3, 2.0, 5.5, 11.0, 20.0])
def test_feedforward_e_derivada_da_trajetoria(t):
    """v = |velocidade|, theta = direção e w = dtheta/dt, por diferença finita."""
    h = 1e-5
    x0, y0, th0, v, w = referencia(t - h, A, B, W)
    x1, y1, th1, _, _ = referencia(t + h, A, B, W)
    _, _, th, v, w = referencia(t, A, B, W)
    assert math.hypot(x1 - x0, y1 - y0) / (2 * h) == pytest.approx(v, rel=1e-6)
    assert math.atan2(y1 - y0, x1 - x0) == pytest.approx(th, abs=1e-6)
    assert normaliza_angulo(th1 - th0) / (2 * h) == pytest.approx(w, rel=1e-4, abs=1e-6)


def test_normaliza_angulo():
    assert normaliza_angulo(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert normaliza_angulo(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
    assert normaliza_angulo(0.3) == pytest.approx(0.3)


def test_yaw_do_quaternion():
    q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.4), w=math.cos(0.4))
    assert yaw_from_quaternion(q) == pytest.approx(0.8)


def test_erro_de_yaw_toma_o_menor_caminho():
    ref = (0.0, 0.0, math.radians(170), 0.0, 0.0)
    assert erros(ref, (0.0, 0.0, math.radians(-170)))[2] == pytest.approx(math.radians(-20))


def test_erro_no_corpo():
    # robô olhando para +y; alvo 1 m à frente dele (+y no mundo) -> erro só "para frente"
    assert erro_no_corpo(0.0, 1.0, math.pi / 2) == pytest.approx((1.0, 0.0))
    # alvo à esquerda do robô (-x no mundo) -> erro lateral positivo
    assert erro_no_corpo(-1.0, 0.0, math.pi / 2) == pytest.approx((0.0, 1.0))


def test_sem_erro_o_comando_e_so_feedforward():
    ref = referencia(3.0, A, B, W)
    pose = ref[:3]
    v, w = comando_realimentado(ref, pose, 1.0, _pid(2.0), _pid(3.0), _pid(3.5), 0.02)
    assert (v, w) == pytest.approx((ref[3], ref[4]))


def test_robo_atras_da_referencia_acelera():
    ref = (1.0, 0.0, 0.0, 0.5, 0.0)
    v, _ = comando_realimentado(ref, (0.0, 0.0, 0.0), 1.0, _pid(2.0), _pid(3.0), _pid(3.5), 0.02)
    assert v == pytest.approx(0.5 + 2.0 * 1.0)


def test_robo_a_direita_da_referencia_vira_para_esquerda():
    ref = (0.0, 1.0, 0.0, 0.0, 0.0)
    _, w = comando_realimentado(ref, (0.0, 0.0, 0.0), 1.0, _pid(2.0), _pid(3.0), _pid(3.5), 0.02)
    assert w > 0.0


def _rmse(sigma_pos, sigma_yaw, seed=1):
    """Monociclo seguindo o 8 enquanto o controle enxerga a pose + ruído (estimativa)."""
    g = _ganhos()
    rng = random.Random(seed)
    ctrl = [_pid(g[k][0]) for k in ('pid_x', 'pid_y', 'pid_theta')]
    w_ref = g['omega']
    dt, T = 0.02, 2 * (2 * math.pi / w_ref)     # duas voltas; mede só a segunda
    x = y = th = t = soma = 0.0
    n = 0
    while t < T:
        ref = referencia(t, g['A'], g['B'], w_ref)
        medida = (x + rng.gauss(0, sigma_pos), y + rng.gauss(0, sigma_pos),
                  th + rng.gauss(0, sigma_yaw))
        v, w = comando_realimentado(ref, medida, g['k_ff'], *ctrl, dt)
        v, w = max(-2.5, min(2.5, v)), max(-7.0, min(7.0, w))   # limites do controllers.yaml
        x, y, th = x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th + w * dt
        if t > T / 2:
            soma += (ref[0] - x) ** 2 + (ref[1] - y) ** 2
            n += 1
        t += dt
    return math.sqrt(soma / n)


def test_malha_fechada_sem_ruido_segue_o_8():
    assert _rmse(0.0, 0.0) < 0.05


def test_malha_fechada_com_ruido_padrao_do_gps():
    assert _rmse(0.1, 0.01) < 0.1


def test_erro_cresce_com_o_ruido_mas_continua_estavel():
    erros_rmse = [_rmse(s, s / 10) for s in (0.0, 0.1, 0.5, 1.0)]
    assert erros_rmse == sorted(erros_rmse)
    assert erros_rmse[-1] < 0.4
