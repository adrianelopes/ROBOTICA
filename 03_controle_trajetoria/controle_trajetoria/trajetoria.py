import math


def referencia(t, A, B, W):
    xd = A * math.sin(W * t)
    yd = B * math.sin(2 * W * t)

    dyd = 2 * B * W * math.cos(2 * W * t)
    dxd = A * W * math.cos(W * t)
    theta_d = math.atan2(dyd, dxd)

    v_f = math.sqrt(dxd*dxd + dyd*dyd)

    ddxd = -A * W*W * math.sin(W * t)
    ddyd = -4 * B * W*W * math.sin(2* W *t)

    w_f = (dxd * ddyd - dyd * ddxd) / (dxd*dxd + dyd*dyd)

    return xd, yd, theta_d, v_f, w_f


def normaliza_angulo(a):
    return math.atan2(math.sin(a), math.cos(a))

def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
