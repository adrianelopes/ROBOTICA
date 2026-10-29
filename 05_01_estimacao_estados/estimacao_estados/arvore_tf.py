"""Imprime a árvore de transformações (tf2) e, opcionalmente, grava um PDF com ela."""

import os
import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import tf2_ros

from .arvore import faltando, grafo_de_yaml, para_dot, renderizar

# Frames que existem quando a simulação + a trajetória em 8 estão rodando.
FRAMES_ESPERADOS = [
    'world', 'odom', 'chassis',
    'imu_link', 'gps_link', 'camera_link', 'camera_optical_link', 'lidar_link',
    'left_wheel', 'right_wheel',
]


class ArvoreTf(Node):

    def __init__(self):
        super().__init__('arvore_tf')
        self.periodo = float(self.declare_parameter('periodo', 2.0).value)
        self.espera = float(self.declare_parameter('espera', 60.0).value)
        self.pdf = self.declare_parameter('pdf', '').value
        self.frames = list(self.declare_parameter('frames_esperados', FRAMES_ESPERADOS).value)

        # `Buffer(node=...)` anuncia o serviço tf2_frames (o que o view_frames consulta);
        # o listener alimenta o buffer com /tf e /tf_static.
        self.buffer = tf2_ros.Buffer(node=self)
        self.listener = tf2_ros.TransformListener(self.buffer, self, spin_thread=False)

        self.t_inicio = None
        self.publicada = False
        self.create_timer(self.periodo, self._verificar)
        self.get_logger().info(
            f'Aguardando os {len(self.frames)} frames da simulação '
            f'(checagem a cada {self.periodo:g} s)...')

    def _grafo(self):
        return grafo_de_yaml(self.buffer.all_frames_as_yaml())

    def _verificar(self):
        if self.publicada:
            return
        agora = self.get_clock().now().nanoseconds / 1e9
        # o /clock do Gazebo só anda depois do gzserver: não conta esse tempo na espera
        if agora == 0.0:
            return
        if self.t_inicio is None:
            self.t_inicio = agora

        grafo = self._grafo()
        resto = faltando(grafo, self.frames)
        if not resto:
            self._publicar(grafo)
        elif agora - self.t_inicio > self.espera:
            self.publicada = True
            self.get_logger().warn(
                f'{self.espera:g} s sem os frames {", ".join(resto)}. '
                'A árvore até agora é:\n' + renderizar(grafo))

    def _publicar(self, grafo):
        self.publicada = True

        def descricao(frame):
            dados = grafo.get(frame, {})
            return f'  [{dados.get("broadcaster", "?")}, {dados.get("rate", "?")} Hz]'

        self.get_logger().info(
            'Árvore de transformações (tf2):\n' + renderizar(grafo, descricao=descricao))
        if self.pdf:
            self._gravar_pdf(grafo)

    def _gravar_pdf(self, grafo):
        caminho = os.path.abspath(os.path.expanduser(self.pdf))
        base = caminho[:-4] if caminho.endswith('.pdf') else caminho
        gv = base + '.gv'
        with open(gv, 'w', encoding='utf-8') as f:
            f.write(para_dot(grafo))
        try:
            subprocess.run(['dot', '-Tpdf', gv, '-o', base + '.pdf'], check=True)
            self.get_logger().info(f'Árvore gravada em {base}.pdf')
        except (OSError, subprocess.CalledProcessError) as erro:
            self.get_logger().warn(f'Não foi possível gerar o PDF (graphviz): {erro}')


def main(args=None):
    rclpy.init(args=args)
    no = ArvoreTf()
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
