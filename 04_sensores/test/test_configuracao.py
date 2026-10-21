"""Confere se a origem do GPS no YAML é a mesma do mundo do Gazebo e se os arquivos existem."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

RAIZ = Path(__file__).resolve().parents[1]


def test_origem_do_gps_igual_a_do_mundo():
    yaml = pytest.importorskip('yaml')
    with open(RAIZ / 'config' / 'sensores.yaml', encoding='utf-8') as f:
        gps = yaml.safe_load(f)['gps_xyz']['ros__parameters']
    mundo = ET.parse(RAIZ / 'worlds' / 'sensores.world').getroot()
    esferico = mundo.find('world/spherical_coordinates')

    assert esferico is not None
    assert gps['origem_lat'] == pytest.approx(float(esferico.findtext('latitude_deg')))
    assert gps['origem_lon'] == pytest.approx(float(esferico.findtext('longitude_deg')))
    assert gps['origem_alt'] == pytest.approx(float(esferico.findtext('elevation')))
    # 180 compensa o bug do Gazebo Classic e deixa o mundo em ENU (ver worlds/sensores.world)
    assert float(esferico.findtext('heading_deg')) == 180.0


@pytest.mark.parametrize('arquivo', [
    'config/controllers.yaml',
    'config/sensores.yaml',
    'config/sensores.rviz',
    'launch/robo_gazebo.launch.py',
    'launch/sensores.launch.py',
    'models/my_robot/model.urdf.xacro',
    'models/my_robot/sensores.xacro',
    'models/my_robot/meshes/chassis.dae',
    'worlds/sensores.world',
])
def test_arquivo_existe(arquivo):
    assert (RAIZ / arquivo).is_file()
