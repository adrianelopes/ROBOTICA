import math

import pytest

from estimacao_estados.geodesia import (
    ACHATAMENTO, ConversorGps, EXCENTRICIDADE_2, geodetica_para_ecef, SEMI_EIXO_MAIOR)

LAT, LON, ALT = -7.1396, -34.8450, 5.0


def test_origem_vira_zero():
    conversor = ConversorGps(LAT, LON, ALT)
    assert conversor.para_xyz(LAT, LON, ALT) == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)


def test_ecef_no_equador_e_no_polo():
    assert geodetica_para_ecef(0.0, 0.0, 0.0) == pytest.approx((SEMI_EIXO_MAIOR, 0.0, 0.0))
    semi_eixo_menor = SEMI_EIXO_MAIOR * (1.0 - ACHATAMENTO)
    assert geodetica_para_ecef(90.0, 0.0, 0.0) == pytest.approx(
        (0.0, 0.0, semi_eixo_menor), abs=1e-6)


def test_deslocamento_para_o_leste():
    # 0.001° de longitude no equador = 0.001° * (pi/180) * a
    conversor = ConversorGps(0.0, 0.0, 0.0)
    leste, norte, cima = conversor.para_xyz(0.0, 0.001, 0.0)
    assert leste == pytest.approx(math.radians(0.001) * SEMI_EIXO_MAIOR, abs=1e-3)
    assert norte == pytest.approx(0.0, abs=1e-3)
    assert cima == pytest.approx(0.0, abs=1e-3)


def test_deslocamento_para_o_norte_usa_raio_meridional():
    conversor = ConversorGps(LAT, LON, ALT)
    dlat = 0.0001
    leste, norte, cima = conversor.para_xyz(LAT + dlat, LON, ALT)
    seno = math.sin(math.radians(LAT))
    raio_meridional = (SEMI_EIXO_MAIOR * (1.0 - EXCENTRICIDADE_2)
                       / (1.0 - EXCENTRICIDADE_2 * seno ** 2) ** 1.5)
    assert norte == pytest.approx(math.radians(dlat) * raio_meridional, abs=1e-3)
    assert leste == pytest.approx(0.0, abs=1e-3)
    assert cima == pytest.approx(0.0, abs=1e-3)


def test_altitude_vira_z():
    conversor = ConversorGps(LAT, LON, ALT)
    leste, norte, cima = conversor.para_xyz(LAT, LON, ALT + 3.0)
    assert (leste, norte) == pytest.approx((0.0, 0.0), abs=1e-6)
    assert cima == pytest.approx(3.0, abs=1e-6)


def test_latitude_invalida():
    with pytest.raises(ValueError):
        ConversorGps(91.0, 0.0, 0.0)
