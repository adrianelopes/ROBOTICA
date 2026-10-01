"""Mede o RMSE de cada volta do 8 só escutando os tópicos.

Suba o robô normalmente (ros2 launch controle_trajetoria trajetoria_8.launch.py) e, em outro
terminal, rode:  python3 medir_rmse.py [--voltas 2]

Compara /trajetoria/desejada com a pose real (/ground_truth) e com a odometria (/odom).
Cada volta é acrescentada em resultados_rmse.csv, junto com omega, modo, k_ff e topico_pose do nó.
"""
import argparse
import csv
import math
import os
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter

CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resultados_rmse.csv')


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def tempo(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Medidor(Node):

    def __init__(self, eixo):
        super().__init__('medidor_rmse', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.eixo = eixo
        self.ref, self.odom, self.real = [], [], []
        self.create_subscription(PoseStamped, '/trajetoria/desejada', self.cb_ref, 100)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 100)
        self.create_subscription(Odometry, '/ground_truth', self.cb_real, 100)

    def cb_ref(self, m):
        self.ref.append((tempo(m), m.pose.position.x, m.pose.position.y, yaw(m.pose.orientation)))

    def cb_odom(self, m):
        p = m.pose.pose.position
        self.odom.append((tempo(m), p.x, p.y, yaw(m.pose.pose.orientation)))

    def cb_real(self, m):
        # /ground_truth é a origem do chassi; a referência é o ponto entre as rodas
        p, th = m.pose.pose.position, yaw(m.pose.pose.orientation)
        self.real.append((tempo(m), p.x + self.eixo * math.cos(th) - self.eixo,
                          p.y + self.eixo * math.sin(th), th))

    def parametros_do_no(self):
        cli = self.create_client(GetParameters, '/trajetoria_8/get_parameters')
        nomes = ['omega', 'modo', 'k_ff', 'topico_pose']
        if not cli.wait_for_service(timeout_sec=60):
            return {}
        fut = cli.call_async(GetParameters.Request(names=nomes))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)
        tipos = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value'}
        return {n: getattr(v, tipos[v.type]) for n, v in zip(nomes, fut.result().values) if v.type in tipos}

    def rmse(self, serie, t_ini, t_fim):
        ref = np.array(self.ref)
        ref = ref[(ref[:, 0] >= t_ini) & (ref[:, 0] < t_fim)]
        s = np.array(serie)
        if len(s) < 2:
            return None
        ref = ref[(ref[:, 0] >= s[0, 0]) & (ref[:, 0] <= s[-1, 0])]
        x = np.interp(ref[:, 0], s[:, 0], s[:, 1])
        y = np.interp(ref[:, 0], s[:, 0], s[:, 2])
        th = np.interp(ref[:, 0], s[:, 0], np.unwrap(s[:, 3]))
        ep = np.hypot(ref[:, 1] - x, ref[:, 2] - y)
        et = np.arctan2(np.sin(ref[:, 3] - th), np.cos(ref[:, 3] - th))
        return float(np.sqrt(np.mean(ep ** 2))), float(np.degrees(np.sqrt(np.mean(et ** 2))))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--voltas', type=int, default=2)
    ap.add_argument('--eixo', type=float, default=0.1, help='distância da origem do chassi ao eixo das rodas (m)')
    args = ap.parse_args()

    rclpy.init()
    no = Medidor(args.eixo)
    print('Esperando o nó trajetoria_8...', flush=True)
    par = no.parametros_do_no()
    if 'omega' not in par:
        raise SystemExit('Não achei o nó /trajetoria_8. O launch está rodando (com o mesmo source)?')
    periodo = 2 * math.pi / par['omega']
    print(f'omega={par["omega"]} modo={par.get("modo")} k_ff={par.get("k_ff")} '
          f'topico_pose={par.get("topico_pose", "/odom")} | uma volta = {periodo:.1f} s', flush=True)

    novo = not os.path.exists(CSV)
    with open(CSV, 'a', newline='') as f:
        w = csv.writer(f)
        if novo:
            w.writerow(['data', 'omega', 'modo', 'k_ff', 'topico_pose', 'volta',
                        'rmse_real_m', 'rmse_real_graus', 'rmse_odom_m', 'rmse_odom_graus'])
        volta = 0
        try:
            while rclpy.ok() and volta < args.voltas:
                rclpy.spin_once(no, timeout_sec=0.1)
                if not no.ref or no.ref[-1][0] - no.ref[0][0] < (volta + 1) * periodo:
                    continue
                t_ini = no.ref[0][0] + volta * periodo
                real = no.rmse(no.real, t_ini, t_ini + periodo)
                odom = no.rmse(no.odom, t_ini, t_ini + periodo)
                txt_real = f'{real[0]:.3f} m / {real[1]:.1f}°' if real else 'sem /ground_truth'
                print(f'volta {volta + 1}: real {txt_real} | odom {odom[0]:.3f} m / {odom[1]:.1f}°', flush=True)
                w.writerow([datetime.now().strftime('%Y-%m-%d %H:%M:%S'), par['omega'], par.get('modo'),
                            par.get('k_ff'), par.get('topico_pose', '/odom'), volta + 1,
                            *(real or ('', '')), *odom])
                f.flush()
                volta += 1
        except KeyboardInterrupt:
            pass
    print(f'Resultados acrescentados em {CSV}')
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
