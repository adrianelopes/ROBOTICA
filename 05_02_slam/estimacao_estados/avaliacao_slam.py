"""
Registra uma execução do SLAM e compara com o groundtruth.

A cada ``1/taxa`` s grava, no mesmo instante:
groundtruth (/groundtruth/odom), pose do SLAM (TF map -> chassis) e pose da odometria
(TF odom -> chassis). Quando o percurso termina (/percurso/concluido) ou no Ctrl+C, grava
em ``saida``: ``trajetoria.csv``, ``mapa.pgm``/``mapa.yaml`` (último /map) e ``metricas.json``.
"""

from bisect import bisect_left
from collections import deque
import csv
import json
from pathlib import Path
import time

from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from estimacao_estados import mapa
from estimacao_estados.ruido import quaternion_para_rpy

COLUNAS = ['t', 'gt_x', 'gt_y', 'gt_yaw', 'slam_x', 'slam_y', 'slam_yaw',
           'odom_x', 'odom_y', 'odom_yaw']


def _pose_2d(posicao, orientacao):
    _, _, yaw = quaternion_para_rpy(orientacao.x, orientacao.y, orientacao.z, orientacao.w)
    return posicao.x, posicao.y, yaw


class AvaliacaoSlam(Node):

    def __init__(self):
        super().__init__('avaliacao_slam')
        self.saida = Path(self.declare_parameter('saida', 'resultados/slam').value).expanduser()
        self.mundo = self.declare_parameter('mundo', '').value
        self.cenario = self.declare_parameter('cenario', '').value
        self.descricao = self.declare_parameter('descricao', '').value
        self.frame_mapa = self.declare_parameter('frame_mapa', 'map').value
        self.frame_odom = self.declare_parameter('frame_odom', 'odom').value
        self.frame_base = self.declare_parameter('frame_base', 'chassis').value
        self.espera_final = float(self.declare_parameter('espera_final', 5.0).value)
        self.encerrar = bool(self.declare_parameter('encerrar_ao_concluir', False).value)
        taxa = float(self.declare_parameter('taxa', 5.0).value)
        self.atraso = float(self.declare_parameter('atraso', 0.2).value)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.groundtruth = deque(maxlen=500)   # (t, x, y, yaw), em ordem de chegada
        self.linhas = []
        self.mapa = None
        self.inicio_parede = time.monotonic()
        self.inicio_sim = None
        self.salvo = False
        self.terminou = False

        qos_mapa = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos_latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._mapa, qos_mapa)
        self.create_subscription(Odometry, '/groundtruth/odom', self._groundtruth, 50)
        self.create_subscription(Bool, '/percurso/concluido', self._concluido, qos_latch)
        self.create_timer(1.0 / taxa, self._registrar)
        self.get_logger().info(f'Resultados em {self.saida}')

    # ------------------------------------------------------------ entradas

    def _mapa(self, msg):
        self.mapa = msg

    def _groundtruth(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.groundtruth.append((t, *_pose_2d(msg.pose.pose.position, msg.pose.pose.orientation)))

    def _concluido(self, msg):
        if msg.data and not self.terminou:
            self.terminou = True
            self.get_logger().info(
                f'Percurso concluído: salvando em {self.espera_final:.0f} s '
                '(espera a última atualização do /map)')
            self._timer_fim = self.create_timer(self.espera_final, self._finalizar)

    # ------------------------------------------------------------ registro

    def _groundtruth_recente(self):
        """Amostra do groundtruth de ~``atraso`` s atrás (as TFs desse instante já chegaram)."""
        if not self.groundtruth:
            return None
        tempos = [g[0] for g in self.groundtruth]
        i = bisect_left(tempos, tempos[-1] - self.atraso)
        return self.groundtruth[min(i, len(tempos) - 1)]

    def _registrar(self):
        if self.terminou:
            return
        gt = self._groundtruth_recente()
        if gt is None:
            return
        # As três poses no MESMO instante, o do groundtruth (a TF interpola).
        instante = Time(nanoseconds=round(gt[0] * 1e9), clock_type=self.get_clock().clock_type)
        try:
            slam = self.buffer.lookup_transform(self.frame_mapa, self.frame_base, instante)
            odom = self.buffer.lookup_transform(self.frame_odom, self.frame_base, instante)
        except TransformException:
            return  # SLAM ainda não publicou map -> odom (ou TF fora do buffer)
        t = gt[0]
        if self.inicio_sim is None:
            self.inicio_sim = t
        tr_s, tr_o = slam.transform, odom.transform
        self.linhas.append([t - self.inicio_sim, *gt[1:],
                            *_pose_2d(tr_s.translation, tr_s.rotation),
                            *_pose_2d(tr_o.translation, tr_o.rotation)])

    # ------------------------------------------------------------ saída

    def _finalizar(self):
        self._timer_fim.cancel()
        self.salvar()
        if self.encerrar:
            self.get_logger().info('encerrar_ao_concluir: fim da execução')
            raise SystemExit(0)

    def salvar(self):
        if self.salvo:
            return
        self.salvo = True
        self.saida.mkdir(parents=True, exist_ok=True)
        with open(self.saida / 'trajetoria.csv', 'w', newline='', encoding='utf-8') as f:
            escritor = csv.writer(f)
            escritor.writerow(COLUNAS)
            escritor.writerows([f'{v:.4f}' for v in linha] for linha in self.linhas)

        metricas = {'cenario': self.cenario, 'descricao': self.descricao,
                    'mundo': self.mundo, 'amostras': len(self.linhas),
                    'duracao_sim_s': self.linhas[-1][0] if self.linhas else 0.0,
                    'duracao_real_s': time.monotonic() - self.inicio_parede}
        if self.linhas:
            gt = [linha[1:4] for linha in self.linhas]
            slam = [linha[4:7] for linha in self.linhas]
            odom = [linha[7:10] for linha in self.linhas]
            correcao = [mapa.compor(s, mapa.inversa(o)) for s, o in zip(slam, odom)]
            metricas['comprimento_percurso_m'] = mapa.comprimento(gt)
            metricas['slam'] = mapa.metricas_trajetoria(gt, slam)
            metricas['odometria'] = mapa.metricas_trajetoria(gt, odom)
            metricas['correcao_map_odom_max_m'] = max(
                (c[0] ** 2 + c[1] ** 2) ** 0.5 for c in correcao)

        if self.mapa is not None:
            info = self.mapa.info
            grade = mapa.grade_de_mensagem(self.mapa.data, info.width, info.height)
            q = info.origin.orientation
            origem = (info.origin.position.x, info.origin.position.y,
                      quaternion_para_rpy(q.x, q.y, q.z, q.w)[2])
            mapa.salvar_mapa(self.saida / 'mapa', grade, info.resolution, origem)
            if self.mundo:
                metricas['mapa'] = mapa.metricas_mapa(
                    grade, info.resolution, origem, mapa.ler_mundo(self.mundo))
        else:
            self.get_logger().warning('nenhum /map recebido: mapa não salvo')

        (self.saida / 'metricas.json').write_text(
            json.dumps(metricas, indent=2, ensure_ascii=False), encoding='utf-8')
        self._resumo(metricas)

    def _resumo(self, m):
        linhas = [f'=== {self.cenario or "SLAM"}: {m["amostras"]} amostras, '
                  f'{m["duracao_sim_s"]:.0f} s simulados ===']
        for chave, nome in (('slam', 'SLAM (map->chassis)'), ('odometria', 'Odometria')):
            if chave in m:
                e = m[chave]
                linhas.append(f'{nome:20s} RMSE {e["rmse_posicao_m"]:.3f} m / '
                              f'{e["rmse_yaw_graus"]:.2f} graus, final '
                              f'{e["erro_final_posicao_m"]:.3f} m')
        if 'mapa' in m and 'erro_medio_paredes_m' in m['mapa']:
            e = m['mapa']
            linhas.append(f'{"Mapa":20s} erro médio das paredes {e["erro_medio_paredes_m"]:.3f} m'
                          f', {100 * e["fracao_ocupadas_na_parede"]:.0f} % das células '
                          'ocupadas sobre a parede real')
        linhas.append(f'Arquivos em {self.saida}')
        self.get_logger().info('\n'.join(linhas))


def main(args=None):
    rclpy.init(args=args)
    no = AvaliacaoSlam()
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        try:
            no.salvar()
        except Exception as erro:  # noqa: B902 - não perder o Ctrl+C por erro na gravação
            print(f'avaliacao_slam: falha ao salvar: {erro}')
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
