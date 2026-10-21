"""Conversão de coordenadas geodésicas (lat, lon, alt) para um referencial local (X, Y, Z)."""

import math

# Elipsoide WGS84
SEMI_EIXO_MAIOR = 6378137.0                 # m
ACHATAMENTO = 1.0 / 298.257223563
EXCENTRICIDADE_2 = ACHATAMENTO * (2.0 - ACHATAMENTO)


def geodetica_para_ecef(lat, lon, alt):
    """Conversão de (lat, lon em graus; alt em m) para ECEF (x, y, z em m)."""
    phi, lam = math.radians(lat), math.radians(lon)
    sin_phi = math.sin(phi)
    n = SEMI_EIXO_MAIOR / math.sqrt(1.0 - EXCENTRICIDADE_2 * sin_phi ** 2)
    x = (n + alt) * math.cos(phi) * math.cos(lam)
    y = (n + alt) * math.cos(phi) * math.sin(lam)
    z = (n * (1.0 - EXCENTRICIDADE_2) + alt) * sin_phi
    return x, y, z


class ConversorGps:
    """
    Conversão dos fixes do GPS em (X, Y, Z) no referencial local ENU de uma origem.

    X aponta para o leste, Y para o norte e Z para cima. É o mesmo referencial do mundo do
    Gazebo quando o `heading_deg` das coordenadas esféricas é 0.
    """

    def __init__(self, lat0, lon0, alt0):
        if not -90.0 <= lat0 <= 90.0:
            raise ValueError(f'Latitude da origem fora de [-90, 90]: {lat0}.')
        self._ecef0 = geodetica_para_ecef(lat0, lon0, alt0)
        phi, lam = math.radians(lat0), math.radians(lon0)
        self._sin_phi, self._cos_phi = math.sin(phi), math.cos(phi)
        self._sin_lam, self._cos_lam = math.sin(lam), math.cos(lam)

    def para_xyz(self, lat, lon, alt):
        """Devolve (x, y, z) em metros da posição geodésica em relação à origem."""
        x, y, z = geodetica_para_ecef(lat, lon, alt)
        dx = x - self._ecef0[0]
        dy = y - self._ecef0[1]
        dz = z - self._ecef0[2]
        leste = -self._sin_lam * dx + self._cos_lam * dy
        norte = (-self._sin_phi * self._cos_lam * dx
                 - self._sin_phi * self._sin_lam * dy
                 + self._cos_phi * dz)
        cima = (self._cos_phi * self._cos_lam * dx
                + self._cos_phi * self._sin_lam * dy
                + self._sin_phi * dz)
        return leste, norte, cima
