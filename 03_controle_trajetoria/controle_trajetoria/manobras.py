import logging
import math


def normalizar_angulo(angulo):
    return math.atan2(math.sin(angulo), math.cos(angulo))


def saturar(valor, limite):
    return max(-limite, min(limite, valor))


class ControlePose:
    ROTACAO_INICIAL = 'ROTACAO_INICIAL'
    TRANSLACAO = 'TRANSLACAO'
    ROTACAO_FINAL = 'ROTACAO_FINAL'
    CONCLUIDO = 'CONCLUIDO'

    def __init__(self, k_linear=0.5, k_angular=1.5, v_max=0.3, w_max=1.0,
                 tol_distancia=0.02, tol_angulo=0.02, logger=None):
        self.x_d = None
        self.y_d = None
        self.theta_d = None
        self.k_v = k_linear
        self.k_w = k_angular
        self.v_max = v_max
        self.w_max = w_max
        self.tol_d = tol_distancia
        self.tol_a = tol_angulo

        self.pose = None
        self.estado = self.ROTACAO_INICIAL
        self.logger = logger or logging.getLogger('controle_pose')

    # Interface usada pelo seguidor_waypoints

    def definir_objetivo(self, goal):
        self.x_d, self.y_d, self.theta_d = goal
        self.estado = self.ROTACAO_INICIAL
        self.logger.info(
            f'Nova pose desejada: x={self.x_d:.2f} m, y={self.y_d:.2f} m, '
            f'theta={math.degrees(self.theta_d):.1f}°')

    def calcular(self, pose, dt):
        return self.controle(pose)

    @property
    def chegou(self):
        return self.estado == self.CONCLUIDO

    # Lógica de controle

    def mudar_estado(self, novo):
        x, y, yaw = self.pose
        self.logger.info(
            f'{self.estado} -> {novo} | pose: x={x:.3f} y={y:.3f} '
            f'yaw={math.degrees(yaw):.1f}°')
        self.estado = novo

    def controle(self, pose):
        """Retorna (v, w) para a pose atual (x, y, yaw)."""
        self.pose = pose
        x, y, yaw = self.pose
        dx = self.x_d - x
        dy = self.y_d - y
        distancia = math.hypot(dx, dy)
        v = w = 0.0

        if self.estado == self.ROTACAO_INICIAL:
            if distancia < self.tol_d:
                self.mudar_estado(self.ROTACAO_FINAL)
                return v, w
            erro = normalizar_angulo(math.atan2(dy, dx) - yaw)
            if abs(erro) < self.tol_a:
                self.mudar_estado(self.TRANSLACAO)
            else:
                w = saturar(self.k_w * erro, self.w_max)

        elif self.estado == self.TRANSLACAO:
            erro_d = dx * math.cos(yaw) + dy * math.sin(yaw)
            if distancia < self.tol_d:
                self.mudar_estado(self.ROTACAO_FINAL)
            else:
                v = saturar(self.k_v * erro_d, self.v_max)
                if distancia > 0.1:
                    erro_a = normalizar_angulo(math.atan2(dy, dx) - yaw)
                    if erro_d < 0:
                        erro_a = normalizar_angulo(erro_a + math.pi)
                    w = saturar(self.k_w * erro_a, self.w_max)

        elif self.estado == self.ROTACAO_FINAL:
            erro = normalizar_angulo(self.theta_d - yaw)
            if abs(erro) < self.tol_a:
                self.mudar_estado(self.CONCLUIDO)
            else:
                w = saturar(self.k_w * erro, self.w_max)

        return v, w
