"""Consistência slam_toolbox x robô x mundo x launch (exercício 05_02), sem ROS."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

yaml = pytest.importorskip('yaml')

RAIZ = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def slam():
    with open(RAIZ / 'config' / 'slam_toolbox.yaml', encoding='utf-8') as f:
        return yaml.safe_load(f)['slam_toolbox']['ros__parameters']


@pytest.fixture(scope='module')
def lidar():
    """Bloco <sensor type="ray"> do sensores.xacro (lido como XML puro)."""
    raiz = ET.parse(RAIZ / 'models' / 'my_robot' / 'sensores.xacro').getroot()
    for sensor in raiz.iter('sensor'):
        if sensor.get('type') == 'ray':
            return sensor
    pytest.fail('sensor ray (lidar) não encontrado em sensores.xacro')


def test_frames_seguem_rep105(slam):
    assert slam['map_frame'] == 'map'
    assert slam['odom_frame'] == 'odom'
    # mesmo link base usado pelo diff_drive_controller (odom -> chassis)
    with open(RAIZ / 'config' / 'controllers.yaml', encoding='utf-8') as f:
        texto = f.read()
    assert 'base_frame_id: chassis' in texto
    assert slam['base_frame'] == 'chassis'


def test_usa_relogio_da_simulacao(slam):
    assert slam['use_sim_time'] is True


def test_topico_do_scan_e_o_do_lidar(slam, lidar):
    remaps = [r.text for r in lidar.iter('remapping')]
    assert '~/out:=scan' in remaps
    assert slam['scan_topic'] == '/scan'


def test_alcance_igual_ao_do_lidar(slam, lidar):
    alcance = lidar.find('ray').find('range')
    assert slam['min_laser_range'] == pytest.approx(float(alcance.findtext('min')))
    assert slam['max_laser_range'] == pytest.approx(float(alcance.findtext('max')))


def test_modo_mapeamento(slam):
    assert slam['mode'] == 'mapping'
    assert slam['resolution'] > 0


def test_mundo_e_uma_sala_fechada():
    raiz = ET.parse(RAIZ / 'worlds' / 'slam_simples.world').getroot()
    nomes = {m.get('name') for m in raiz.iter('model')}
    assert {'parede_norte', 'parede_sul', 'parede_leste', 'parede_oeste'} <= nomes
    # GPS do robô continua precisando da origem geodésica
    assert raiz.find('world').find('spherical_coordinates') is not None


def test_launch_slam_desliga_world_odom():
    """A TF map -> odom (slam_toolbox) e a world -> odom (estática) não coexistem."""
    texto = (RAIZ / 'launch' / 'slam.launch.py').read_text(encoding='utf-8')
    ast.parse(texto)
    assert "'publicar_world_odom': 'false'" in texto
    assert "executable='async_slam_toolbox_node'" in texto
    robo = (RAIZ / 'launch' / 'robo_gazebo.launch.py').read_text(encoding='utf-8')
    assert "IfCondition(LaunchConfiguration('publicar_world_odom'))" in robo
