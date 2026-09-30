
#     v = PID_x(e_x^b)          
#     w = PID_y(e_y^b)          


import logging
import math

from simple_pid import PID


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    #Limita value ao intervalo [-limit, limit]
    return max(-limit, min(limit, value))


class PoseController:

    def __init__(self, max_linear_vel=0.5, max_angular_vel=1.5,
                 position_tolerance=0.03, yaw_tolerance=0.05,
                 pid_x=(1.2, 0.0, 0.0), pid_y=(8.0, 0.0, 0.0),
                 pid_theta=(1.5, 0.0, 0.0), alignment_hysteresis=2.0,
                 logger=None):
        self.max_lin = max_linear_vel
        self.max_ang = max_angular_vel
        self.pos_tol = position_tolerance
        self.yaw_tol = yaw_tolerance
        self.align_hyst = alignment_hysteresis
        self.logger = logger or logging.getLogger('pose_controller')

        # Os três PIDs (9 ganhos).
        self.pid_x = self._make_pid(pid_x, self.max_lin)
        self.pid_y = self._make_pid(pid_y, self.max_ang)
        self.pid_theta = self._make_pid(pid_theta, self.max_ang)

        self.goal = None       
        self.reached = False  
        self.aligning = False  

    def _make_pid(self, gains, limit):
        

        #A biblioteca calcula (setpoint - entrada).     
        kp, ki, kd = gains
        return PID(kp, ki, kd, setpoint=0.0, sample_time=None,
                   output_limits=(-limit, limit))

    def _reset_pids(self):
        for pid in (self.pid_x, self.pid_y, self.pid_theta):
            pid.reset()


    def definir_objetivo(self, goal):
        self.goal = goal
        self.reached = False
        self.aligning = False
        self._reset_pids()
        x, y, yaw = goal
        self.logger.info(
            f'Nova pose desejada: x={x:.2f} y={y:.2f} yaw={yaw:.2f} rad')

    def calcular(self, pose, dt):
        return self.compute_command(pose, self.goal, dt)

    @property
    def chegou(self):
        return self.reached


    def compute_command(self, pose, goal, dt):
        x, y, yaw = pose
        xd, yd, yaw_d = goal

        e_x = xd - x
        e_y = yd - y
        dist = math.hypot(e_x, e_y)
        e_theta = normalize_angle(yaw_d - yaw)

        if dist < self.pos_tol and abs(e_theta) < self.yaw_tol:
            if not self.reached:
                self.reached = True
                self._reset_pids()
                self.logger.info('Pose desejada alcançada.')
            return 0.0, 0.0
        self.reached = False

        
        if dist < self.pos_tol:
            if not self.aligning:
                self.aligning = True
                self.pid_theta.reset()
        elif dist > self.pos_tol * self.align_hyst or abs(e_theta) < self.yaw_tol:
            self.aligning = False

        if self.aligning:
            self._reset_pids_posicao()
            return 0.0, clamp(self.pid_theta(-e_theta, dt=dt), self.max_ang)

        c, s = math.cos(yaw), math.sin(yaw)
        e_x_b = c * e_x + s * e_y
        e_y_b = -s * e_x + c * e_y

        v = self.pid_x(-e_x_b, dt=dt)     
        w = self.pid_y(-e_y_b, dt=dt)      
        return v, clamp(w, self.max_ang)

    def _reset_pids_posicao(self):
        self.pid_x.reset()
        self.pid_y.reset()
