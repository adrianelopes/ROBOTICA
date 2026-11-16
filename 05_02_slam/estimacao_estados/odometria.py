"""Odometria simulada: pose exata do Gazebo + erro de odometria controlado (sem ROS)."""

import math
import random


def normalizar_angulo(angulo):
    """Ângulo equivalente em (-pi, pi]."""
    return math.atan2(math.sin(angulo), math.cos(angulo))


class OdometriaRuidosa:
    """
    Integra a odometria a partir de poses exatas, somando erro a cada incremento.

    O incremento entre duas poses exatas é escrito no referencial do robô (avanço,
    deslocamento lateral e giro) e recebe dois tipos de erro, como uma odometria de rodas:

    * aleatório, com variância proporcional ao tamanho do incremento. É um passeio
      aleatório: o desvio-padrão acumulado cresce com a raiz da distância (ou do giro)
      e não depende da taxa com que as poses chegam;
    * sistemático (erro de escala), como um raio de roda ou uma bitola mal calibrados.

    Parâmetros (todos >= 0; zero em todos = odometria groundtruth, igual à pose exata):

    * ``sigma_avanco`` [m/sqrt(m)]: desvio do avanço após 1 m percorrido;
    * ``sigma_giro`` [rad/sqrt(rad)]: desvio do giro após 1 rad girado;
    * ``sigma_giro_avanco`` [rad/sqrt(m)]: desvio da orientação após 1 m em linha reta;
    * ``vies_avanco`` e ``vies_giro``: erro de escala (0.02 = mede 2 % a mais).
    """

    def __init__(self, sigma_avanco=0.0, sigma_giro=0.0, sigma_giro_avanco=0.0,
                 vies_avanco=0.0, vies_giro=0.0, rng=None):
        parametros = (sigma_avanco, sigma_giro, sigma_giro_avanco, vies_avanco, vies_giro)
        if any(p < 0.0 for p in parametros[:3]):
            raise ValueError('desvios-padrão da odometria devem ser >= 0')
        self.sigma_avanco = sigma_avanco
        self.sigma_giro = sigma_giro
        self.sigma_giro_avanco = sigma_giro_avanco
        self.vies_avanco = vies_avanco
        self.vies_giro = vies_giro
        self.sem_ruido = all(p == 0.0 for p in parametros)
        self.rng = rng if rng is not None else random.Random()
        self._real = None
        self.pose = None

    def atualizar(self, x, y, yaw):
        """Recebe a pose exata (x, y, yaw) e devolve a pose da odometria (x, y, yaw)."""
        if self.sem_ruido:
            self.pose = (x, y, normalizar_angulo(yaw))
            return self.pose
        if self._real is None:
            self._real = self.pose = (x, y, normalizar_angulo(yaw))
            return self.pose

        x0, y0, yaw0 = self._real
        cos0, sen0 = math.cos(yaw0), math.sin(yaw0)
        dx, dy = x - x0, y - y0
        avanco = cos0 * dx + sen0 * dy
        lateral = -sen0 * dx + cos0 * dy
        giro = normalizar_angulo(yaw - yaw0)
        self._real = (x, y, yaw)

        distancia = math.hypot(avanco, lateral)
        escala = 1.0 + self.vies_avanco
        avanco_medido = avanco * escala + self.rng.gauss(
            0.0, self.sigma_avanco * math.sqrt(distancia))
        lateral_medido = lateral * escala
        giro_medido = giro * (1.0 + self.vies_giro) + self.rng.gauss(
            0.0, math.sqrt(self.sigma_giro ** 2 * abs(giro)
                           + self.sigma_giro_avanco ** 2 * distancia))

        xe, ye, yawe = self.pose
        cose, sene = math.cos(yawe), math.sin(yawe)
        self.pose = (xe + cose * avanco_medido - sene * lateral_medido,
                     ye + sene * avanco_medido + cose * lateral_medido,
                     normalizar_angulo(yawe + giro_medido))
        return self.pose
