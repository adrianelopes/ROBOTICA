"""Seguidor de pontos de passagem para um robô diferencial (sem ROS)."""

import math

from estimacao_estados.odometria import normalizar_angulo


def _limitar(valor, limite):
    return max(-limite, min(limite, valor))


def pontos_de_lista(valores):
    """Lista plana [x0, y0, x1, y1, ...] (formato de parâmetro ROS) -> [(x, y), ...]."""
    if len(valores) % 2:
        raise ValueError('a lista de pontos precisa ter um número par de valores')
    return [(float(valores[i]), float(valores[i + 1])) for i in range(0, len(valores), 2)]


class Percurso:
    """
    Visita os pontos em ordem: gira no lugar até mirar no ponto e então anda até ele.

    Girar parado antes de andar deixa o caminho reto entre os pontos, igual em todas as
    execuções, o que permite comparar os cenários de SLAM sobre o mesmo trajeto.
    """

    def __init__(self, pontos, v_max=0.3, w_max=0.8, tolerancia=0.08,
                 angulo_para_andar=0.15, ganho_v=1.0, ganho_w=2.0):
        if not pontos:
            raise ValueError('o percurso precisa de pelo menos um ponto')
        self.pontos = list(pontos)
        self.v_max = v_max
        self.w_max = w_max
        self.tolerancia = tolerancia
        self.angulo_para_andar = angulo_para_andar
        self.ganho_v = ganho_v
        self.ganho_w = ganho_w
        self.indice = 0

    @property
    def concluido(self):
        return self.indice >= len(self.pontos)

    def comando(self, x, y, yaw):
        """Pose atual -> (v, w). Devolve (0, 0) quando o último ponto foi alcançado."""
        while not self.concluido:
            alvo_x, alvo_y = self.pontos[self.indice]
            distancia = math.hypot(alvo_x - x, alvo_y - y)
            if distancia > self.tolerancia:
                break
            self.indice += 1
        if self.concluido:
            return 0.0, 0.0

        erro = normalizar_angulo(math.atan2(alvo_y - y, alvo_x - x) - yaw)
        w = _limitar(self.ganho_w * erro, self.w_max)
        if abs(erro) > self.angulo_para_andar:
            return 0.0, w
        return min(self.v_max, self.ganho_v * distancia), w
