"""Trajetória em 8 e lei de controle (sem ROS, para testar sem simulador)."""

import math


def referencia(t, A, B, W):
    xd = A * math.sin(W * t)
    yd = B * math.sin(2 * W * t)

    dyd = 2 * B * W * math.cos(2 * W * t)
    dxd = A * W * math.cos(W * t)
    theta_d = math.atan2(dyd, dxd)

    v_f = math.sqrt(dxd*dxd + dyd*dyd)

    ddxd = -A * W*W * math.sin(W * t)
    ddyd = -4 * B * W*W * math.sin(2 * W * t)

    w_f = (dxd * ddyd - dyd * ddxd) / (dxd*dxd + dyd*dyd)

    return xd, yd, theta_d, v_f, w_f


def normaliza_angulo(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def erros(ref, pose):
    """Erro no referencial do mundo (ex, ey, etheta) entre a referência e a pose."""
    xd, yd, theta_d = ref[:3]
    x, y, theta = pose
    return xd - x, yd - y, normaliza_angulo(theta_d - theta)


def erro_no_corpo(ex, ey, theta):
    """Gira o erro de posição do mundo para o referencial do robô (frente, lado)."""
    return (math.cos(theta) * ex + math.sin(theta) * ey,
            -math.sin(theta) * ex + math.cos(theta) * ey)


def comando_realimentado(ref, pose, k_ff, ctrl_x, ctrl_y, ctrl_theta, dt):
    """
    Feedforward + realimentação sobre a pose recebida (real, bruta ou estimada).

    Cada ctrl_* é um callable ``f(erro_invertido, dt=dt)`` com setpoint 0, como o
    ``simple_pid.PID``. Retorna (v, w).
    """
    ex, ey, etheta = erros(ref, pose)
    exb, eyb = erro_no_corpo(ex, ey, pose[2])
    v = k_ff * ref[3] + ctrl_x(-exb, dt=dt)
    w = k_ff * ref[4] + ctrl_y(-eyb, dt=dt) + ctrl_theta(-etheta, dt=dt)
    return v, w
