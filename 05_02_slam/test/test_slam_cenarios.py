"""Odometria com erro, percurso, métricas e launch dos cenários do SLAM (05_02), sem ROS."""

import ast
import math
from pathlib import Path
import random
import sys

import pytest

np = pytest.importorskip('numpy')
yaml = pytest.importorskip('yaml')

RAIZ = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ))

from estimacao_estados import mapa  # noqa: E402
from estimacao_estados.odometria import OdometriaRuidosa  # noqa: E402
from estimacao_estados.percurso import Percurso, pontos_de_lista  # noqa: E402

MUNDO = RAIZ / 'worlds' / 'slam_simples.world'


@pytest.fixture(scope='module')
def cenarios():
    with open(RAIZ / 'config' / 'slam_cenarios.yaml', encoding='utf-8') as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def formas():
    return mapa.ler_mundo(MUNDO)


def _arco(n=500, raio=2.0):
    """Poses exatas de meia volta num círculo."""
    for i in range(n + 1):
        a = math.pi * i / n
        yield raio * math.sin(a), raio * (1 - math.cos(a)), a


# ------------------------------------------------------------------- odometria

def test_odometria_sem_ruido_e_o_groundtruth():
    odom = OdometriaRuidosa()
    for pose in _arco():
        assert odom.atualizar(*pose) == pytest.approx(pose)


def test_so_vies_reproduz_erro_de_escala_sem_aleatoriedade():
    odom = OdometriaRuidosa(vies_avanco=0.1)
    for x in np.linspace(0.0, 2.0, 101):
        estimada = odom.atualizar(x, 0.0, 0.0)
    assert estimada == pytest.approx((2.2, 0.0, 0.0))


def test_erro_aleatorio_nao_depende_da_taxa_de_atualizacao():
    """sigma_avanco é por raiz de metro: o desvio após 4 m é 2 * sigma, a 10 ou 100 Hz."""
    for passos in (40, 400):
        finais = []
        for semente in range(300):
            odom = OdometriaRuidosa(sigma_avanco=0.05, rng=random.Random(semente))
            for x in np.linspace(0.0, 4.0, passos + 1):
                estimada = odom.atualizar(x, 0.0, 0.0)
            finais.append(estimada[0])
        assert np.std(finais) == pytest.approx(0.05 * 2.0, rel=0.2)


def test_mesma_semente_mesmo_ruido():
    def rodar():
        odom = OdometriaRuidosa(0.05, 0.05, 0.03, rng=random.Random(7))
        return [odom.atualizar(*p) for p in _arco()][-1]
    assert rodar() == rodar()


def test_teletransporte_nao_entra_na_odometria():
    """Exercício 3: arrastar o robô no Gazebo não gira as rodas; andar normalmente não é salto."""
    odom = OdometriaRuidosa(salto_maximo=0.25, giro_maximo=0.5)
    for x in np.linspace(0.0, 1.0, 51):          # 1 m a 50 amostras/m
        odom.atualizar(x, 0.0, 0.0)
    assert odom.saltos == 0 and odom.pose == pytest.approx((1.0, 0.0, 0.0))
    odom.atualizar(-1.0, 2.0, 0.0)               # sequestro
    assert odom.saltos == 1 and odom.pose == pytest.approx((1.0, 0.0, 0.0))


def test_desvio_negativo_e_rejeitado():
    with pytest.raises(ValueError):
        OdometriaRuidosa(sigma_avanco=-0.1)


# ------------------------------------------------------------------- percurso

def _simular(percurso, dt=0.05, passos=20000):
    x = y = yaw = 0.0
    caminho = [(x, y)]
    for _ in range(passos):
        v, w = percurso.comando(x, y, yaw)
        if percurso.concluido:
            break
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
        caminho.append((x, y))
    return np.array(caminho)


def test_percurso_visita_todos_os_pontos(cenarios):
    p = cenarios['percurso_sala']['ros__parameters']
    pontos = pontos_de_lista(p['pontos'])
    percurso = Percurso(pontos, v_max=p['v_max'], w_max=p['w_max'],
                        tolerancia=p['tolerancia'])
    caminho = _simular(percurso)
    assert percurso.concluido
    assert np.hypot(*(caminho[-1] - pontos[-1])) <= p['tolerancia']


def test_percurso_nao_encosta_em_parede_nem_obstaculo(cenarios, formas):
    """Raio do robô ~0,25 m (chassi 0,4 x 0,26 m): exige 0,35 m de folga."""
    p = cenarios['percurso_sala']['ros__parameters']
    percurso = Percurso(pontos_de_lista(p['pontos']), v_max=p['v_max'], w_max=p['w_max'],
                        tolerancia=p['tolerancia'])
    caminho = _simular(percurso)
    assert np.min(mapa.distancia_superficie(caminho, formas)) > 0.35


def test_pontos_do_yaml_sao_reais(cenarios):
    """Parâmetro double[] do ROS: inteiros no YAML virariam integer[] e o nó falharia."""
    valores = cenarios['percurso_sala']['ros__parameters']['pontos']
    assert len(valores) % 2 == 0
    assert all(isinstance(v, float) for v in valores)


# ------------------------------------------------------------------- mapa e métricas

def test_mundo_tem_paredes_e_obstaculos(formas):
    tipos = sorted(f['tipo'] for f in formas)
    assert tipos == ['caixa'] * 7 + ['cilindro']


def test_distancia_a_caixa_e_cilindro():
    caixa = {'tipo': 'caixa', 'x': 0.0, 'y': 0.0, 'yaw': math.pi / 2,
             'meio_x': 1.0, 'meio_y': 0.5}
    cilindro = {'tipo': 'cilindro', 'x': 5.0, 'y': 0.0, 'raio': 0.3}
    pontos = [(0.0, 0.0), (0.0, 1.5), (0.8, 0.0), (5.0, 0.0), (5.5, 0.0)]
    d = mapa.distancia_superficie(pontos, [caixa, cilindro])
    assert d == pytest.approx([0.5, 0.5, 0.3, 0.3, 0.2])


def test_mapa_ida_e_volta_pelo_disco(tmp_path):
    grade = np.full((4, 6), -1, dtype=np.int8)
    grade[0, :] = 0
    grade[3, 5] = 100
    mapa.salvar_mapa(tmp_path / 'm', grade, 0.05, (-1.0, -2.0, 0.0))
    lida, resolucao, origem = mapa.ler_mapa(tmp_path / 'm.yaml')
    assert np.array_equal(lida, grade)
    assert resolucao == 0.05 and tuple(origem) == (-1.0, -2.0, 0.0)
    centros = mapa.centros_ocupados(lida, resolucao, origem)
    assert centros.shape == (1, 2)
    assert centros[0] == pytest.approx([-1.0 + 5.5 * 0.05, -2.0 + 3.5 * 0.05])


def test_mapa_perfeito_tem_erro_pequeno(formas):
    """Rasteriza as paredes reais: o erro tem que ficar na ordem da discretização."""
    resolucao, origem = 0.05, (-2.5, -3.5, 0.0)
    xs = origem[0] + (np.arange(180) + 0.5) * resolucao
    ys = origem[1] + (np.arange(140) + 0.5) * resolucao
    gx, gy = np.meshgrid(xs, ys)
    d = mapa.distancia_superficie(np.column_stack([gx.ravel(), gy.ravel()]), formas)
    grade = np.where(d.reshape(gx.shape) < resolucao / 2, 100, 0).astype(np.int8)
    m = mapa.metricas_mapa(grade, resolucao, origem, formas)
    assert m['erro_medio_paredes_m'] < resolucao / 2
    assert m['fracao_ocupadas_na_parede'] == 1.0


def test_compor_com_inversa_e_identidade():
    a = (1.0, -2.0, 0.7)
    assert mapa.compor(a, mapa.inversa(a)) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_metricas_de_trajetoria():
    real = [(0, 0, 0), (1, 0, 0)]
    estimada = [(0, 0.3, 0), (1, 0.4, math.radians(10))]
    m = mapa.metricas_trajetoria(real, estimada)
    assert m['rmse_posicao_m'] == pytest.approx(math.sqrt((0.09 + 0.16) / 2))
    assert m['erro_final_posicao_m'] == pytest.approx(0.4)
    assert m['erro_final_yaw_graus'] == pytest.approx(10.0)


# ------------------------------------------------------------------- launch e configuração

def test_launch_define_os_tres_cenarios():
    texto = (RAIZ / 'launch' / 'slam.launch.py').read_text(encoding='utf-8')
    arvore = ast.parse(texto)
    cenarios = next(no.value for no in ast.walk(arvore) if isinstance(no, ast.Assign)
                    and getattr(no.targets[0], 'id', '') == 'CENARIOS')
    tabela = ast.literal_eval(cenarios)
    assert tabela['sem_ruido'] == ('0.0', '0.0', '0.0')     # tudo groundtruth
    assert float(tabela['lidar'][1]) > 0 and float(tabela['lidar'][2]) == 0
    assert float(tabela['lidar_odom'][1]) > 0 and float(tabela['lidar_odom'][2]) > 0


def test_odometria_slam_substitui_a_tf_das_rodas():
    """Dois nós publicando odom -> chassis quebrariam a árvore TF."""
    extra = yaml.safe_load((RAIZ / 'config' / 'controllers_sem_tf_odom.yaml').read_text())
    assert extra['diff_drive_controller']['ros__parameters'] == {'enable_odom_tf': False}
    xacro = (RAIZ / 'models' / 'my_robot' / 'model.urdf.xacro').read_text(encoding='utf-8')
    assert '<xacro:unless value="$(arg odom_tf_rodas)">' in xacro
    assert 'controllers_sem_tf_odom.yaml' in xacro
    launch = (RAIZ / 'launch' / 'slam.launch.py').read_text(encoding='utf-8')
    assert "'odom_tf_rodas': 'true' if rodas else 'false'" in launch


def test_nos_novos_estao_no_setup():
    setup = (RAIZ / 'setup.py').read_text(encoding='utf-8')
    for no in ('odometria_slam', 'percurso_sala', 'avaliacao_slam', 'comparar_slam'):
        assert f"'{no} = estimacao_estados.{no}:main'" in setup


def test_frames_da_odometria_iguais_aos_do_slam(cenarios):
    with open(RAIZ / 'config' / 'slam_toolbox.yaml', encoding='utf-8') as f:
        slam = yaml.safe_load(f)['slam_toolbox']['ros__parameters']
    fonte = (RAIZ / 'estimacao_estados' / 'odometria_slam.py').read_text(encoding='utf-8')
    assert f"'frame_odom', '{slam['odom_frame']}'" in fonte
    assert f"'frame_base', '{slam['base_frame']}'" in fonte


def test_relogio_da_simulacao_rapido_no_slam():
    """/clock a 10 Hz (padrão) dessincroniza a TF da odometria do /scan em até 100 ms."""
    launch = (RAIZ / 'launch' / 'slam.launch.py').read_text(encoding='utf-8')
    assert "'gazebo_params_slam.yaml'" in launch
    params = yaml.safe_load((RAIZ / 'config' / 'gazebo_params_slam.yaml').read_text())
    assert params['gazebo']['ros__parameters']['publish_rate'] >= 1000.0
