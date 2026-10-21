import math

import pytest

from sensores.nuvem import scan_para_pontos


def test_angulos_cardeais():
    # leituras a -90°, 0° e +90°, todas a 2 m
    pontos = scan_para_pontos([2.0, 2.0, 2.0], -math.pi / 2, math.pi / 2, 0.1, 10.0)
    assert pontos[0] == pytest.approx((0.0, -2.0, 0.0), abs=1e-9)
    assert pontos[1] == pytest.approx((2.0, 0.0, 0.0), abs=1e-9)
    assert pontos[2] == pytest.approx((0.0, 2.0, 0.0), abs=1e-9)


def test_descarta_leituras_invalidas():
    ranges = [1.0, math.inf, math.nan, 0.05, 11.0, 3.0]
    pontos = scan_para_pontos(ranges, 0.0, 0.1, 0.12, 10.0)
    assert len(pontos) == 2
    assert math.hypot(pontos[0][0], pontos[0][1]) == pytest.approx(1.0)
    assert math.hypot(pontos[1][0], pontos[1][1]) == pytest.approx(3.0)


def test_indice_define_o_angulo_mesmo_com_descartes():
    pontos = scan_para_pontos([math.inf, 1.0], 0.0, math.pi / 2, 0.1, 10.0)
    assert pontos == [pytest.approx((0.0, 1.0, 0.0), abs=1e-9)]


def test_varredura_vazia():
    assert scan_para_pontos([], 0.0, 0.1, 0.1, 10.0) == []
