"""Confere o modelo do robô: sensores, tópicos, ruído e corpo visual (sem precisar do Gazebo)."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

xacro = pytest.importorskip('xacro')

RAIZ = Path(__file__).resolve().parents[1]
MODELO = RAIZ / 'models' / 'my_robot' / 'model.urdf.xacro'

# tipo do sensor -> (link, plugin, tópicos esperados pelo plugin)
SENSORES = {
    'imu': ('imu_link', 'libgazebo_ros_imu_sensor.so', ['~/out:=data_bruto']),
    'gps': ('gps_link', 'libgazebo_ros_gps_sensor.so', ['~/out:=fix']),
    'camera': ('camera_link', 'libgazebo_ros_camera.so', []),
    'ray': ('lidar_link', 'libgazebo_ros_ray_sensor.so', ['~/out:=scan']),
}


@pytest.fixture
def expandir(monkeypatch):
    # $(find sensores) aponta para a pasta do código-fonte (não exige o pacote instalado)
    from xacro import substitution_args
    monkeypatch.setattr(substitution_args, '_eval_find', lambda pacote: str(RAIZ))

    def _expandir(escala=1.0):
        doc = xacro.process_file(str(MODELO), mappings={'escala_ruido': str(escala)})
        return ET.fromstring(doc.toxml())
    return _expandir


def _desvios(sensor):
    return [float(e.text) for e in sensor.iter('stddev')]


def test_modelo_tem_os_quatro_sensores(expandir):
    robo = expandir()
    tipos = {s.get('type') for s in robo.iter('sensor')}
    assert tipos == set(SENSORES)


@pytest.mark.parametrize('tipo', SENSORES)
def test_sensor_tem_plugin_e_topicos(expandir, tipo):
    link, plugin, remapeamentos = SENSORES[tipo]
    sensor = next(s for s in expandir().iter('sensor') if s.get('type') == tipo)
    assert sensor.find('plugin').get('filename') == plugin
    publicados = [r.text for r in sensor.find('plugin').iter('remapping')]
    for esperado in remapeamentos:
        assert esperado in publicados


@pytest.mark.parametrize('tipo', SENSORES)
def test_sensor_usa_ruido_gaussiano(expandir, tipo):
    sensor = next(s for s in expandir().iter('sensor') if s.get('type') == tipo)
    desvios = _desvios(sensor)
    assert desvios and all(d > 0.0 for d in desvios)
    for ruido in sensor.iter('noise'):
        tipo_ruido = ruido.get('type') or ruido.findtext('type')
        assert tipo_ruido == 'gaussian'


@pytest.mark.parametrize('tipo', SENSORES)
def test_escala_do_ruido_multiplica_os_desvios(expandir, tipo):
    def desvios(escala):
        robo = expandir(escala)
        return _desvios(next(s for s in robo.iter('sensor') if s.get('type') == tipo))

    base, dobro, zero = desvios(1.0), desvios(2.0), desvios(0.0)
    assert dobro == pytest.approx([2 * d for d in base])
    assert all(d == 0.0 for d in zero)


@pytest.mark.parametrize('tipo', SENSORES)
def test_sensor_esta_visivel_no_robo(expandir, tipo):
    robo = expandir()
    nome_link = SENSORES[tipo][0]
    link = next(link for link in robo.findall('link') if link.get('name') == nome_link)
    assert link.find('visual') is not None


def test_arvore_de_links_e_consistente(expandir):
    robo = expandir()
    links = [link.get('name') for link in robo.findall('link')]
    assert len(links) == len(set(links))
    for junta in robo.findall('joint'):  # as do ros2_control ficam dentro de <ros2_control>
        assert junta.find('parent').get('link') in links
        assert junta.find('child').get('link') in links
    for sensor_link, _, _ in SENSORES.values():
        assert sensor_link in links


def test_camera_usa_frame_optico(expandir):
    robo = expandir()
    camera = next(s for s in robo.iter('sensor') if s.get('type') == 'camera')
    assert camera.find('plugin').findtext('frame_name') == 'camera_optical_link'
    assert 'camera_optical_link' in [link.get('name') for link in robo.findall('link')]
