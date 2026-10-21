"""Conversão de uma varredura do lidar 2D em pontos 3D."""

import math


def scan_para_pontos(ranges, angle_min, angle_increment, range_min, range_max):
    """
    Conversão das distâncias do LaserScan em pontos (x, y, 0) no referencial do lidar.

    Leituras infinitas, NaN ou fora de [range_min, range_max] (raio que não acertou nada)
    são descartadas.
    """
    pontos = []
    for i, distancia in enumerate(ranges):
        if not math.isfinite(distancia) or not range_min <= distancia <= range_max:
            continue
        angulo = angle_min + i * angle_increment
        pontos.append((distancia * math.cos(angulo), distancia * math.sin(angulo), 0.0))
    return pontos
