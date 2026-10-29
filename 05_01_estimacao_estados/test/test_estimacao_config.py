"""Consistência EKF x nós x launch x modelo (exercícios 1.1 e 2.1), sem ROS."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

yaml = pytest.importorskip('yaml')

RAIZ = Path(__file__).resolve().parents[1]
CORPO = ['x', 'y', 'z', 'roll', 'pitch', 'yaw', 'vx', 'vy', 'vz',
         'vroll', 'vpitch', 'vyaw', 'ax', 'ay', 'az']


def _yaml(nome):
    with open(RAIZ / 'config' / nome, encoding='utf-8') as f:
        return yaml.safe_load(f)


def _texto(caminho):
    return (RAIZ / caminho).read_text(encoding='utf-8')


def _usadas(config):
    return [nome for nome, ligado in zip(CORPO, config) if ligado]


@pytest.fixture(scope='module')
def ekf():
    return _yaml('ekf.yaml')['ekf_filter_node']['ros__parameters']


@pytest.fixture(scope='module')
def traj():
    return _yaml('trajetoria_8.yaml')


def test_python_dos_pacotes_compila():
    """Pega erros de sintaxe/indentação nos nós, que o pytest só acha ao importar."""
    for arquivo in [*RAIZ.glob('sensores/*.py'), *RAIZ.glob('launch/*.py'), RAIZ / 'setup.py']:
        ast.parse(arquivo.read_text(encoding='utf-8'), filename=str(arquivo))


def test_matrizes_do_ekf_tem_15_variaveis(ekf):
    for chave in ('odom0_config', 'odom1_config', 'imu0_config'):
        assert len(ekf[chave]) == 15, chave


def test_fontes_de_cada_sensor_no_ekf(ekf):
    assert _usadas(ekf['odom0_config']) == ['vx', 'vyaw']        # rodas: só velocidades
    assert _usadas(ekf['odom1_config']) == ['x', 'y']            # GPS: posição
    assert _usadas(ekf['imu0_config']) == ['yaw', 'vyaw']        # IMU: yaw absoluto + giro


def test_entradas_do_ekf_sao_topicos_publicados(ekf, traj):
    sensores = _yaml('sensores.yaml')
    assert ekf['odom0'] == '/odom'
    assert ekf['odom1'] == traj['estimador_pose']['ros__parameters']['topico_saida']
    assert ekf['imu0'] == sensores['imu_orientacao']['ros__parameters']['topico_saida']


def test_frames_do_ekf_existem_no_robo(ekf, traj):
    ctrl = _yaml('controllers.yaml')['diff_drive_controller']['ros__parameters']
    robo = ET.parse(RAIZ / 'models' / 'my_robot' / 'model.urdf.xacro').getroot()
    links = {no.get('name') for no in robo.iter('link')}
    assert ekf['base_link_frame'] in links
    assert ekf['base_link_frame'] == ctrl['base_frame_id']
    assert ekf['odom_frame'] == ekf['world_frame'] == ctrl['odom_frame_id']
    assert traj['estimador_pose']['ros__parameters']['frame_id'] == ekf['odom_frame']


def test_ekf_nao_briga_com_o_controlador_pelo_tf(ekf):
    # o diff_drive_controller já publica odom -> chassis
    ctrl = _yaml('controllers.yaml')['diff_drive_controller']['ros__parameters']
    assert ctrl['enable_odom_tf'] is True
    assert ekf['publish_tf'] is False


def test_ekf_em_2d_e_com_tempo_de_simulacao(ekf):
    assert ekf['two_d_mode'] is True
    assert ekf['use_sim_time'] is True
    assert ekf['frequency'] >= 30.0       # o controle roda a 50 Hz


def test_controle_usa_o_filtro_de_kalman(traj):
    p = traj['trajetoria_8']['ros__parameters']
    assert p['topico_pose'] == '/odometry/filtered'
    assert p['topico_real'] == '/odom'
    assert p['modo'] == 'realimentado'


def test_default_do_no_trajetoria_e_o_filtro():
    fonte = _texto('sensores/trajetoria_8.py')
    assert "'topico_pose', '/odometry/filtered'" in fonte
    assert "super().__init__('trajetoria_8')" in fonte


def test_launch_da_trajetoria_sobe_o_ekf_e_usa_o_filtro_por_padrao():
    traj_launch = _texto('launch/trajetoria_8.launch.py')
    est_launch = _texto('launch/estimacao.launch.py')
    assert "'estimacao.launch.py'" in traj_launch
    assert "executable='ekf_node'" in est_launch
    assert "executable='estimador_pose'" in est_launch
    assert "DeclareLaunchArgument('topico_pose', default_value='/odometry/filtered'" \
        in traj_launch


def test_entry_points_apontam_para_modulos_existentes():
    fonte = _texto('setup.py')
    for nome in ('trajetoria_8', 'estimador_pose'):
        assert f"'{nome} = sensores.{nome}:main'" in fonte
        assert (RAIZ / 'sensores' / f'{nome}.py').is_file()


def test_estimador_pose_inicializa_yaw_antes_do_primeiro_gps():
    """O callback do GPS testa self.yaw: ele precisa existir desde o __init__."""
    arvore = ast.parse(_texto('sensores/estimador_pose.py'))
    init = next(f for c in arvore.body if isinstance(c, ast.ClassDef)
                for f in c.body if isinstance(f, ast.FunctionDef) and f.name == '__init__')
    atribuidos = {t.attr for n in ast.walk(init) if isinstance(n, ast.Assign)
                  for t in n.targets if isinstance(t, ast.Attribute)}
    assert 'yaw' in atribuidos


def test_ekf_declarado_no_package_xml():
    assert '<exec_depend>robot_localization</exec_depend>' in _texto('package.xml')
