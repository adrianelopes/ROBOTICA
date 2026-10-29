"""Confere a árvore de transformações montada pela simulação (sem precisar do Gazebo)."""

from pathlib import Path

import pytest

xacro = pytest.importorskip('xacro')

RAIZ = Path(__file__).resolve().parents[1]
LAUNCH = RAIZ / 'launch' / 'robo_gazebo.launch.py'


@pytest.fixture
def robo(monkeypatch):
    # $(find estimacao_estados) aponta para a pasta do código-fonte (não exige o pacote instalado)
    from xacro import substitution_args
    monkeypatch.setattr(substitution_args, '_eval_find', lambda pacote: str(RAIZ))
    doc = xacro.process_file(str(RAIZ / 'models' / 'my_robot' / 'model.urdf.xacro'))
    return doc.toprettyxml()


def _pais_e_filhos(urdf):
    import xml.etree.ElementTree as ET
    raiz = ET.fromstring(urdf)
    return [(j.find('parent').get('link'), j.find('child').get('link'))
            for j in raiz.findall('joint')]


def test_robot_state_publisher_cobre_os_frames_dos_sensores(robo):
    pais = {filho: pai for pai, filho in _pais_e_filhos(robo)}
    for link in ['imu_link', 'gps_link', 'camera_link', 'camera_optical_link', 'lidar_link']:
        assert link in pais
    assert pais['camera_optical_link'] == 'camera_link'


def test_todos_os_links_chegam_ao_chassis(robo):
    pais = {filho: pai for pai, filho in _pais_e_filhos(robo)}
    for link in pais:
        atual, passos = link, 0
        while atual != 'chassis':
            atual = pais[atual]
            passos += 1
            assert passos < 20, f'{link} não chega ao chassis'
    # a raiz do URDF é o chassis; o odom vem do diff_drive_controller
    assert 'chassis' not in pais


def test_launch_publica_a_raiz_world():
    texto = LAUNCH.read_text(encoding='utf-8')
    assert "'world'" in texto and "'odom'" in texto
    assert '--frame-id' in texto and '--child-frame-id' in texto


def test_diff_drive_publica_odom_para_o_chassis():
    yaml = pytest.importorskip('yaml')
    with open(RAIZ / 'config' / 'controllers.yaml', encoding='utf-8') as f:
        ctrl = yaml.safe_load(f)['diff_drive_controller']['ros__parameters']
    assert ctrl['enable_odom_tf'] is True
    assert ctrl['odom_frame_id'] == 'odom'
    assert ctrl['base_frame_id'] == 'chassis'
