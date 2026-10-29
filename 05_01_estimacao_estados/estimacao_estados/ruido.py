"""Ruído gaussiano na orientação da IMU (o Gazebo só ruidifica velocidade e aceleração)."""

import math
import random


def quaternion_para_rpy(x, y, z, w):
    """Conversão do quaternion (x, y, z, w) em (roll, pitch, yaw) em rad."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    seno_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(seno_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def rpy_para_quaternion(roll, pitch, yaw):
    """Conversão de (roll, pitch, yaw) em rad no quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def perturbar_quaternion(quaternion, desvios, rng=random):
    """
    Soma de ruído gaussiano em (roll, pitch, yaw) do quaternion (x, y, z, w).

    `desvios` são os desvios-padrão (roll, pitch, yaw) em rad e `rng` é um `random.Random`
    (use uma semente para reproduzir o ruído).
    """
    roll, pitch, yaw = quaternion_para_rpy(*quaternion)
    return rpy_para_quaternion(roll + rng.gauss(0.0, desvios[0]),
                               pitch + rng.gauss(0.0, desvios[1]),
                               yaw + rng.gauss(0.0, desvios[2]))
