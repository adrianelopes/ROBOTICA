import csv
import inspect
import json
import math
import os
from typing import NamedTuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .continuo import PoseController
from .manobras import ControlePose

MIN_WAYPOINTS = 3

CONTROLADORES = {
    'continuo': PoseController,
    'manobras': ControlePose,
}


class Pose2D(NamedTuple):

    x: float
    y: float
    theta: float

    def __str__(self):
        return (f'x={self.x:.2f} m, y={self.y:.2f} m, '
                f'theta={math.degrees(self.theta):.1f}°')


def yaw_do_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SeguidorWaypoints(Node):

    def __init__(self):
        super().__init__('seguidor_waypoints')
        taxa = float(self._declarar('control_rate', 50.0))
        odom_topic = str(self._declarar('odom_topic', '/odom'))
        cmd_vel_topic = str(self._declarar('cmd_vel_topic', '/cmd_vel'))
        self.repetir = bool(self._declarar('repetir', False))
        registro_csv = str(self._declarar('registro_csv', '') or '')

        self.controlador = self._criar_controlador()
        self.waypoints = self._carregar_waypoints()
        self.indice = 0
        self.concluido = False
        self.pose = None
        self._periodo = 1.0 / taxa
        self._t_anterior = None
        self._t0 = None
        self._registro = None
        self._csv = None
        if registro_csv:
            self._abrir_registro(registro_csv)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.create_timer(self._periodo, self._laco_controle)

        self.get_logger().info(
            f'{len(self.waypoints)} waypoints carregados '
            f'(repetir={self.repetir}):\n' + '\n'.join(
                f'  {i + 1}. {nome}: {pose}'
                for i, (nome, pose) in enumerate(self.waypoints)))
        self._ir_para(0)

    def _declarar(self, nome, padrao=None):
        descritor = ParameterDescriptor(dynamic_typing=True)
        return self.declare_parameter(nome, padrao, descritor).value

    def _ler_argumentos(self, prefixo, classe):
        argumentos = {}
        for nome, arg in inspect.signature(classe).parameters.items():
            if nome == 'logger':
                continue
            if isinstance(arg.default, tuple):
                argumentos[nome] = tuple(
                    float(self._declarar(f'{prefixo}.{nome}.{k}', padrao))
                    for k, padrao in zip(('kp', 'ki', 'kd'), arg.default))
            else:
                argumentos[nome] = float(
                    self._declarar(f'{prefixo}.{nome}', arg.default))
        return argumentos

    def _criar_controlador(self):
        tipo = str(self._declarar('controlador', 'continuo'))
        if tipo not in CONTROLADORES:
            raise ValueError(
                f"Parâmetro 'controlador' inválido: '{tipo}'. "
                f"Opções: {', '.join(CONTROLADORES)}.")
        classe = CONTROLADORES[tipo]
        argumentos = self._ler_argumentos(tipo, classe)
        self._tipo = tipo
        self._argumentos_controlador = argumentos
        self.get_logger().info(f'Controlador: {tipo} | {argumentos}')
        return classe(**argumentos, logger=self.get_logger())

    def _carregar_waypoints(self):
        nomes = self._declarar('waypoints')
        if (not isinstance(nomes, list) or not nomes
                or not all(isinstance(n, str) for n in nomes)):
            raise ValueError(
                "Parâmetro 'waypoints' ausente ou inválido: deve ser uma "
                'lista de nomes, ex.: waypoints: [p1, p2, p3].')
        if len(nomes) < MIN_WAYPOINTS:
            raise ValueError(
                f'São necessários ao menos {MIN_WAYPOINTS} waypoints '
                f'(recebidos {len(nomes)}).')
        if len(set(nomes)) != len(nomes):
            raise ValueError(f"Nomes repetidos em 'waypoints': {nomes}.")

        waypoints = []
        for nome in nomes:
            x = self._declarar(f'{nome}.x')
            y = self._declarar(f'{nome}.y')
            theta = self._declarar(f'{nome}.theta', 0.0)
            if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                       for v in (x, y, theta)):
                raise ValueError(
                    f"Waypoint '{nome}' precisa de x e y numéricos "
                    '(theta opcional, em rad).')
            theta = (theta + math.pi) % (2.0 * math.pi) - math.pi  # wrap
            waypoints.append((nome, Pose2D(float(x), float(y), theta)))
        return waypoints

    def _abrir_registro(self, caminho):
        #Abre o CSV da missão (usado por comparar_missoes)
        caminho = os.path.expanduser(caminho)
        pasta = os.path.dirname(caminho)
        if pasta:
            os.makedirs(pasta, exist_ok=True)
        self._registro = open(caminho, 'w', newline='', encoding='utf-8')
        self._registro.write(f'# controlador={self._tipo}\n')
        self._registro.write(
            f'# parametros={json.dumps(self._argumentos_controlador)}\n')
        self._registro.write('# waypoints=' + json.dumps(
            [[n, p.x, p.y, p.theta] for n, p in self.waypoints]) + '\n')
        self._csv = csv.writer(self._registro, lineterminator='\n')
        self._csv.writerow(['t', 'x', 'y', 'yaw', 'v_cmd', 'w_cmd',
                            'waypoint', 'evento'])
        self.get_logger().info(f'Registrando a missão em {caminho}')

    def _registrar(self, agora, v, w, evento):
        if self._csv is None:
            return
        if self._t0 is None:
            self._t0 = agora
        t = (agora - self._t0).nanoseconds * 1e-9
        p = self.pose
        self._csv.writerow([f'{t:.3f}', f'{p.x:.4f}', f'{p.y:.4f}',
                            f'{p.theta:.4f}', f'{v:.4f}', f'{w:.4f}',
                            self.waypoints[self.indice][0], evento])

    def fechar_registro(self):
        if self._registro is not None:
            self._registro.close()
            self._registro = None
            self._csv = None

    def _ir_para(self, indice):
        self.indice = indice
        nome, pose = self.waypoints[indice]
        ultimo = indice == len(self.waypoints) - 1 and not self.repetir
        self.controlador.definir_objetivo(pose, ultimo=ultimo)
        self.get_logger().info(
            f'-> Waypoint {indice + 1}/{len(self.waypoints)} ({nome}): {pose}')

    def _ao_chegar(self):
        if self.concluido:
            return
        self.get_logger().info(
            f'Waypoint {self.waypoints[self.indice][0]} alcançado.')
        proximo = self.indice + 1
        if proximo < len(self.waypoints):
            self._ir_para(proximo)
        elif self.repetir:
            self._ir_para(0)
        else:
            # Mantém o último objetivo: o robô segura a pose final.
            self.concluido = True
            self.fechar_registro()
            self.get_logger().info('Trajetória concluída.')

    def _odom_callback(self, msg):
        p = msg.pose.pose
        self.pose = Pose2D(p.position.x, p.position.y,
                           yaw_do_quaternion(p.orientation))

    def _laco_controle(self):
        if self.pose is None:
            self.get_logger().info('Aguardando odometria...', once=True)
            return

        agora = self.get_clock().now()
        if self._t_anterior is None:
            dt = self._periodo
        else:
            dt = (agora - self._t_anterior).nanoseconds * 1e-9
        self._t_anterior = agora
        if dt <= 0.0:   # relógio da simulação parado/reiniciado
            return

        v, w = self.controlador.calcular(self.pose, dt)
        self.publicar(v, w)
        chegou = self.controlador.chegou
        if not self.concluido:
            # 'passagem': entrou no raio de aceitação (sem exigir a orientação)
            tipo = 'passagem' if getattr(self.controlador, 'passagem', False) \
                else 'chegada'
            self._registrar(agora, v, w, tipo if chegou else '')
        if chegou:
            self._ao_chegar()

    def publicar(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    try:
        no = SeguidorWaypoints()
    except ValueError as erro:
        rclpy.logging.get_logger('seguidor_waypoints').fatal(str(erro))
        rclpy.try_shutdown()
        return 1
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            no.publicar(0.0, 0.0)
        except Exception:
            pass
        no.fechar_registro()
        no.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    main()
