import math
import random

import pytest

from sensores.ruido import perturbar_quaternion, quaternion_para_rpy, rpy_para_quaternion


def _mesma_rotacao(q1, q2):
    # q e -q representam a mesma rotação
    return abs(sum(a * b for a, b in zip(q1, q2))) == pytest.approx(1.0, abs=1e-9)


def test_rpy_quaternion_ida_e_volta():
    rpy = (0.3, -0.2, 2.5)
    assert quaternion_para_rpy(*rpy_para_quaternion(*rpy)) == pytest.approx(rpy)


def test_identidade():
    assert rpy_para_quaternion(0.0, 0.0, 0.0) == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_yaw_de_90_graus():
    x, y, z, w = rpy_para_quaternion(0.0, 0.0, math.pi / 2)
    assert (z, w) == pytest.approx((math.sin(math.pi / 4), math.cos(math.pi / 4)))


def test_quaternion_perturbado_continua_unitario():
    rng = random.Random(1)
    q = rpy_para_quaternion(0.1, 0.2, 0.3)
    for _ in range(100):
        assert math.sqrt(sum(c * c for c in perturbar_quaternion(q, (0.05, 0.05, 0.05), rng))) \
            == pytest.approx(1.0)


def test_sem_ruido_nao_altera_a_orientacao():
    q = rpy_para_quaternion(0.1, -0.2, 1.0)
    assert _mesma_rotacao(perturbar_quaternion(q, (0.0, 0.0, 0.0), random.Random(0)), q)


def test_mesma_semente_reproduz_o_ruido():
    q = rpy_para_quaternion(0.0, 0.0, 0.5)
    a = perturbar_quaternion(q, (0.01, 0.01, 0.01), random.Random(42))
    b = perturbar_quaternion(q, (0.01, 0.01, 0.01), random.Random(42))
    assert a == b


def test_desvio_padrao_do_ruido_confere():
    rng = random.Random(7)
    desvios = (0.01, 0.02, 0.05)
    amostras = [quaternion_para_rpy(*perturbar_quaternion((0.0, 0.0, 0.0, 1.0), desvios, rng))
                for _ in range(5000)]
    for eixo, esperado in enumerate(desvios):
        valores = [a[eixo] for a in amostras]
        media = sum(valores) / len(valores)
        desvio = math.sqrt(sum((v - media) ** 2 for v in valores) / len(valores))
        assert media == pytest.approx(0.0, abs=3 * esperado / math.sqrt(len(valores)))
        assert desvio == pytest.approx(esperado, rel=0.05)
