"""Confere a leitura e o desenho da árvore de transformações (sem ROS)."""

from estimacao_estados.arvore import (
    faltando, filhos_de, grafo_de_yaml, para_dot, raiz_de, renderizar)

# Trecho real do `Buffer.all_frames_as_yaml()` (Humble), com a indentação do ROS
YAML = """chassis:
  parent: 'odom'
  broadcaster: 'default_authority'
  rate: 50.228
camera_link:
  parent: 'chassis'
  broadcaster: 'default_authority'
  rate: 10000.0
lidar_link:
  parent: 'chassis'
  broadcaster: 'default_authority'
  rate: 10000.0
odom:
  parent: 'world'
  broadcaster: 'default_authority'
  rate: 10000.0
world:
  parent: ''
  broadcaster: 'default_authority'
  rate: 10000.0
"""


def test_le_o_yaml_do_tf():
    grafo = grafo_de_yaml(YAML)
    assert set(grafo) == {'chassis', 'camera_link', 'lidar_link', 'odom', 'world'}
    assert grafo['chassis']['parent'] == 'odom'
    assert grafo['camera_link']['rate'] == '10000.0'


def test_raiz_e_world():
    # a raiz aparece só como parent; o desenho a descobre por isso
    grafo = grafo_de_yaml(YAML)
    grafo.pop('world')
    assert raiz_de(grafo) == 'world'


def test_filhos_em_ordem_alfabetica():
    grafo = grafo_de_yaml(YAML)
    assert filhos_de(grafo, 'chassis') == ['camera_link', 'lidar_link']


def test_desenha_a_arvore():
    texto = renderizar(grafo_de_yaml(YAML))
    assert texto.splitlines() == [
        'world',
        '└── odom',
        '    └── chassis',
        '        ├── camera_link',
        '        └── lidar_link',
    ]


def test_descricao_entra_na_linha():
    grafo = grafo_de_yaml(YAML)
    texto = renderizar(grafo, descricao=lambda f: f' [{grafo[f]["rate"]} Hz]')
    assert '        └── lidar_link [10000.0 Hz]' in texto


def test_faltando_lista_so_os_ausentes():
    grafo = grafo_de_yaml(YAML)
    assert faltando(grafo, ['world', 'odom', 'imu_link']) == ['imu_link']
    assert faltando(grafo, ['world', 'odom']) == []


def test_raiz_publicada_so_como_parent_conta_como_presente():
    # o ROS não lista a raiz como chave; ela aparece só como parent de odom
    grafo = {'odom': {'parent': 'world'}, 'chassis': {'parent': 'odom'}}
    assert 'world' not in grafo
    assert faltando(grafo, ['world', 'odom', 'chassis']) == []


def test_arvore_vazia_nao_quebra():
    assert renderizar({}) == ''
    assert raiz_de({}) is None
    assert 'No tf data received' in para_dot({})


def test_dot_tem_uma_aresta_por_filho():
    dot = para_dot(grafo_de_yaml(YAML))
    assert '"odom" -> "chassis"[label=" Broadcaster: default_authority' in dot
    assert dot.count('->') == 4  # world e odom não têm aresta de entrada
