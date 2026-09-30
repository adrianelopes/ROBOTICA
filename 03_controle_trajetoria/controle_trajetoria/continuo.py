# Controle de pose contínuo

import logging
import math

from simple_pid import PID


def normalize_angle(angle):
    """Leva um ângulo para o intervalo [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    """Limita value ao intervalo [-limit, limit]."""
    return max(-limit, min(limit, value))


class PoseController:
    """Recebe a pose desejada e a atual e calcula as velocidades (v, w)."""

    def __init__(self, max_linear_vel=0.5, max_angular_vel=1.5,
                 position_tolerance=0.03, yaw_tolerance=0.05,
                 pid_x=(1.2, 0.0, 0.0), pid_y=(4.0, 1.0, 0.0),
                 pid_theta=(0.2, 0.0, 0.0), logger=None):
        self.max_lin = max_linear_vel
        self.max_ang = max_angular_vel
        self.pos_tol = position_tolerance
        self.yaw_tol = yaw_tolerance
        self.logger = logger or logging.getLogger('pose_controller')

        # Os três PIDs (9 ganhos).
        self.pid_x = self._make_pid(pid_x, self.max_lin)
        self.pid_y = self._make_pid(pid_y, self.max_ang)
        self.pid_theta = self._make_pid(pid_theta, self.max_ang)

        self.goal = None       # (x, y, yaw) desejado
        self.reached = False   # True enquanto está dentro da tolerância

    def _make_pid(self, gains, limit):
        """Cria um PID com os ganhos (kp, ki, kd).

        A biblioteca calcula (setpoint - entrada). Usamos setpoint 0 e
        passamos o NEGATIVO do erro, então a saída é Kp*e + Ki*int(e) + Kd*de/dt.
        sample_time=None: o dt é fornecido por nós (relógio do ROS/simulação).
        """
        kp, ki, kd = gains
        return PID(kp, ki, kd, setpoint=0.0, sample_time=None,
                   output_limits=(-limit, limit))

    def _reset_pids(self):
        for pid in (self.pid_x, self.pid_y, self.pid_theta):
            pid.reset()

    # Interface usada pelo seguidor_waypoints

    def definir_objetivo(self, goal):
        self.goal = goal
        self.reached = False
        self._reset_pids()
        x, y, yaw = goal
        self.logger.info(
            f'Nova pose desejada: x={x:.2f} y={y:.2f} yaw={yaw:.2f} rad')

    def calcular(self, pose, dt):
        return self.compute_command(pose, self.goal, dt)

    @property
    def chegou(self):
        return self.reached

    # Lógica de controle

    def compute_command(self, pose, goal, dt):
        """Retorna (v, w) a partir da pose atual, da desejada e do dt."""
        x, y, yaw = pose
        xd, yd, yaw_d = goal

        # 1) Erro no referencial do mundo {I}
        e_x = xd - x
        e_y = yd - y
        e_theta = normalize_angle(yaw_d - yaw)

        # Chegou: para e zera os integradores (evita "wind-up" parado)
        if math.hypot(e_x, e_y) < self.pos_tol and abs(e_theta) < self.yaw_tol:
            if not self.reached:
                self.reached = True
                self._reset_pids()
                self.logger.info('Pose desejada alcançada.')
            return 0.0, 0.0
        self.reached = False

        # Posição já atingida, falta só a orientação: gira no lugar
        # (evita o PID de e_y^b brigar com o de e_theta perto do objetivo).
        if math.hypot(e_x, e_y) < self.pos_tol:
            return 0.0, self.pid_theta(-e_theta, dt=dt)

        # 2) Erro no referencial do robô {B}: e^b = R^-1 * e
        c, s = math.cos(yaw), math.sin(yaw)
        e_x_b = c * e_x + s * e_y
        e_y_b = -s * e_x + c * e_y

        # 3) Três PIDs simultâneos
        v = self.pid_x(-e_x_b, dt=dt)                          # frente/trás
        w = (self.pid_y(-e_y_b, dt=dt)                         # erro lateral
             + self.pid_theta(-e_theta, dt=dt))                # erro de ângulo
        return v, clamp(w, self.max_ang)
