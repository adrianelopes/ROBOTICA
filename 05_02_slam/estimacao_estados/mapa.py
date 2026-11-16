"""
Avaliação do SLAM contra o groundtruth (sem ROS).

* geometria real do mundo, lida do .world (caixas e cilindros estáticos);
* mapa de ocupação em disco no formato do map_saver (PGM + YAML);
* erros de trajetória e de mapa.
"""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from estimacao_estados.odometria import normalizar_angulo

# Valores de pixel do map_saver (modo trinary)
PIXEL_OCUPADO, PIXEL_LIVRE, PIXEL_DESCONHECIDO = 0, 254, 205


# ---------------------------------------------------------------- geometria do mundo

def _pose(elemento):
    texto = elemento.findtext('pose') if elemento is not None else None
    valores = [float(v) for v in texto.split()] if texto else [0.0] * 6
    return valores[0], valores[1], valores[5]


def ler_mundo(caminho):
    """
    Formas 2D dos modelos estáticos do .world (as que o lidar enxerga).

    Cada forma é um dict: ``{'tipo': 'caixa', 'x', 'y', 'yaw', 'meio_x', 'meio_y'}`` ou
    ``{'tipo': 'cilindro', 'x', 'y', 'raio'}``. Considera só a pose do modelo (o link e a
    colisão ficam na origem dele, como em ``slam_simples.world``).
    """
    formas = []
    for modelo in ET.parse(caminho).getroot().iter('model'):
        if modelo.findtext('static', '').strip() != 'true':
            continue
        x, y, yaw = _pose(modelo)
        for geometria in modelo.iter('geometry'):
            if geometria.find('box') is not None:
                sx, sy = (float(v) for v in geometria.find('box').findtext('size').split()[:2])
                formas.append({'tipo': 'caixa', 'x': x, 'y': y, 'yaw': yaw,
                               'meio_x': sx / 2.0, 'meio_y': sy / 2.0})
            elif geometria.find('cylinder') is not None:
                raio = float(geometria.find('cylinder').findtext('radius'))
                formas.append({'tipo': 'cilindro', 'x': x, 'y': y, 'raio': raio})
            break  # só a primeira geometria (colisão) de cada modelo
    return formas


def distancia_superficie(pontos, formas):
    """Distância (m) de cada ponto Nx2 à superfície mais próxima das formas."""
    pontos = np.asarray(pontos, dtype=float).reshape(-1, 2)
    melhor = np.full(len(pontos), np.inf)
    for forma in formas:
        dx = pontos[:, 0] - forma['x']
        dy = pontos[:, 1] - forma['y']
        if forma['tipo'] == 'cilindro':
            d = np.abs(np.hypot(dx, dy) - forma['raio'])
        else:
            c, s = math.cos(forma['yaw']), math.sin(forma['yaw'])
            qx = np.abs(c * dx + s * dy) - forma['meio_x']
            qy = np.abs(-s * dx + c * dy) - forma['meio_y']
            fora = np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0))
            dentro = np.minimum(np.maximum(qx, qy), 0.0)
            d = np.abs(fora + dentro)
        melhor = np.minimum(melhor, d)
    return melhor


def contornos(forma, n=64):
    """Polígono (Nx2) do contorno da forma, para desenhar."""
    if forma['tipo'] == 'cilindro':
        a = np.linspace(0.0, 2.0 * math.pi, n)
        return np.column_stack([forma['x'] + forma['raio'] * np.cos(a),
                                forma['y'] + forma['raio'] * np.sin(a)])
    mx, my = forma['meio_x'], forma['meio_y']
    cantos = np.array([[-mx, -my], [mx, -my], [mx, my], [-mx, my], [-mx, -my]])
    c, s = math.cos(forma['yaw']), math.sin(forma['yaw'])
    return cantos @ np.array([[c, s], [-s, c]]) + [forma['x'], forma['y']]


# ---------------------------------------------------------------- mapa em disco

def grade_de_mensagem(dados, largura, altura):
    """``OccupancyGrid.data`` (lista, linha 0 embaixo) -> matriz altura x largura int8."""
    return np.asarray(dados, dtype=np.int8).reshape(altura, largura)


def salvar_mapa(prefixo, grade, resolucao, origem):
    """Grava ``prefixo.pgm`` e ``prefixo.yaml`` como o ``map_saver_cli`` (modo trinary)."""
    prefixo = Path(prefixo)
    prefixo.parent.mkdir(parents=True, exist_ok=True)
    imagem = np.full(grade.shape, PIXEL_DESCONHECIDO, dtype=np.uint8)
    imagem[(grade >= 0) & (grade <= 25)] = PIXEL_LIVRE
    imagem[grade >= 65] = PIXEL_OCUPADO
    imagem = np.flipud(imagem)  # imagem: linha 0 em cima; grade: linha 0 embaixo
    altura, largura = imagem.shape
    with open(prefixo.with_suffix('.pgm'), 'wb') as f:
        f.write(f'P5\n# estimacao_estados avaliacao_slam\n{largura} {altura}\n255\n'.encode())
        f.write(imagem.tobytes())
    ox, oy, oyaw = origem
    prefixo.with_suffix('.yaml').write_text(
        f'image: {prefixo.with_suffix(".pgm").name}\n'
        'mode: trinary\n'
        f'resolution: {resolucao}\n'
        f'origin: [{ox}, {oy}, {oyaw}]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n',
        encoding='utf-8')


def _ler_pgm(caminho):
    dados = Path(caminho).read_bytes()
    campos, pos = [], 0
    while len(campos) < 4:
        while dados[pos:pos + 1].isspace():
            pos += 1
        if dados[pos:pos + 1] == b'#':
            pos = dados.index(b'\n', pos) + 1
            continue
        fim = pos
        while not dados[fim:fim + 1].isspace():
            fim += 1
        campos.append(dados[pos:fim])
        pos = fim
    if campos[0] != b'P5':
        raise ValueError(f'{caminho}: só PGM binário (P5) é suportado')
    largura, altura = int(campos[1]), int(campos[2])
    pixels = np.frombuffer(dados[pos + 1:pos + 1 + largura * altura], dtype=np.uint8)
    return pixels.reshape(altura, largura)


def ler_mapa(caminho_yaml):
    """``mapa.yaml`` -> (grade int8 com -1/0/100, linha 0 embaixo; resolução; origem)."""
    import yaml

    caminho_yaml = Path(caminho_yaml)
    meta = yaml.safe_load(caminho_yaml.read_text(encoding='utf-8'))
    imagem = _ler_pgm(caminho_yaml.parent / meta['image'])
    p = (255.0 - imagem) / 255.0
    if meta.get('negate', 0):
        p = 1.0 - p
    grade = np.full(imagem.shape, -1, dtype=np.int8)
    grade[p > meta['occupied_thresh']] = 100
    grade[p < meta['free_thresh']] = 0
    return np.flipud(grade), float(meta['resolution']), tuple(meta['origin'])


def centros_ocupados(grade, resolucao, origem):
    """Centros (Nx2, em m no frame do mapa) das células ocupadas."""
    linhas, colunas = np.nonzero(grade >= 65)
    return np.column_stack([origem[0] + (colunas + 0.5) * resolucao,
                            origem[1] + (linhas + 0.5) * resolucao])


def metricas_mapa(grade, resolucao, origem, formas):
    """Erro das paredes do mapa em relação à geometria real do mundo."""
    ocupados = centros_ocupados(grade, resolucao, origem)
    if len(ocupados) == 0:
        return {'celulas_ocupadas': 0}
    d = distancia_superficie(ocupados, formas)
    return {
        'celulas_ocupadas': int(len(ocupados)),
        'celulas_livres': int(np.count_nonzero((grade >= 0) & (grade <= 25))),
        'erro_medio_paredes_m': float(np.mean(d)),
        'erro_p95_paredes_m': float(np.percentile(d, 95)),
        # células a até 1 célula (meia célula de discretização + meia de folga) da parede
        'fracao_ocupadas_na_parede': float(np.mean(d <= resolucao)),
    }


# ---------------------------------------------------------------- trajetórias

def compor(a, b):
    """Pose a (x, y, yaw) seguida da pose b, escrita no referencial de a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            normalizar_angulo(a[2] + b[2]))


def inversa(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], normalizar_angulo(-a[2]))


def metricas_trajetoria(real, estimada):
    """Erros de posição e orientação entre duas trajetórias Nx3 (x, y, yaw) alinhadas no tempo."""
    real = np.asarray(real, dtype=float).reshape(-1, 3)
    estimada = np.asarray(estimada, dtype=float).reshape(-1, 3)
    if len(real) == 0:
        return {}
    erro_pos = np.hypot(*(estimada[:, :2] - real[:, :2]).T)
    erro_yaw = np.abs(np.arctan2(np.sin(estimada[:, 2] - real[:, 2]),
                                 np.cos(estimada[:, 2] - real[:, 2])))
    return {
        'rmse_posicao_m': float(np.sqrt(np.mean(erro_pos ** 2))),
        'erro_max_posicao_m': float(np.max(erro_pos)),
        'erro_final_posicao_m': float(erro_pos[-1]),
        'rmse_yaw_graus': float(np.degrees(np.sqrt(np.mean(erro_yaw ** 2)))),
        'erro_final_yaw_graus': float(np.degrees(erro_yaw[-1])),
    }


def comprimento(trajetoria):
    xy = np.asarray(trajetoria, dtype=float).reshape(-1, 3)[:, :2]
    return float(np.sum(np.hypot(*np.diff(xy, axis=0).T))) if len(xy) > 1 else 0.0
