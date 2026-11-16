"""
Compara execuções do SLAM gravadas pelo avaliacao_slam (figura + tabela).

    ros2 run estimacao_estados comparar_slam                     # resultados/{3 cenários}
    ros2 run estimacao_estados comparar_slam pasta1 pasta2 ... -o comparacao.png

Gera ``comparacao.png`` (mapas com a geometria real e as trajetórias; erro de posição no
tempo) e ``comparacao.md`` (tabela de métricas), e imprime a tabela.
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from estimacao_estados import mapa

PASTA_PADRAO = Path('~/ros2_ws/src/05_02_slam/resultados').expanduser()
CENARIOS = ('sem_ruido', 'lidar', 'lidar_odom')

# Paleta categórica (ordem fixa) + tinta neutra para a referência
COR_SLAM, COR_ODOM, COR_MUNDO = '#2a78d6', '#eb6834', '#1baf7a'
TINTA, TINTA_2, SUPERFICIE = '#0b0b0b', '#52514e', '#fcfcfb'


def mundo_existente(caminho=None):
    """
    Caminho de um .world que existe neste computador.

    O .world gravado no metricas.json pode ser o caminho de outro computador (os
    resultados foram gerados por outra pessoa do grupo): nesse caso usa o do pacote.
    """
    if caminho and Path(caminho).exists():
        return Path(caminho)
    local = Path(__file__).resolve().parents[1] / 'worlds' / 'slam_simples.world'
    if local.exists():
        return local
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('estimacao_estados')) / 'worlds' / \
        'slam_simples.world'


def ler_execucao(pasta):
    pasta = Path(pasta)
    metricas = json.loads((pasta / 'metricas.json').read_text(encoding='utf-8'))
    with open(pasta / 'trajetoria.csv', encoding='utf-8') as f:
        linhas = list(csv.DictReader(f))
    dados = {c: np.array([float(linha[c]) for linha in linhas]) for c in linhas[0]} \
        if linhas else {}
    grade = None
    if (pasta / 'mapa.yaml').exists():
        grade = mapa.ler_mapa(pasta / 'mapa.yaml')
    return {'pasta': pasta, 'metricas': metricas, 'dados': dados, 'mapa': grade}


def _valor(m, *chaves, fmt='{:.3f}'):
    for chave in chaves:
        if not isinstance(m, dict) or chave not in m:
            return '-'
        m = m[chave]
    return fmt.format(m)


def tabela(execucoes):
    cabecalho = ['Métrica'] + [e['metricas'].get('descricao') or e['pasta'].name
                               for e in execucoes]
    linhas = [
        ('Duração simulada (s)', ('duracao_sim_s',), '{:.0f}'),
        ('Percurso (m)', ('comprimento_percurso_m',), '{:.1f}'),
        ('SLAM: RMSE posição (m)', ('slam', 'rmse_posicao_m'), '{:.3f}'),
        ('SLAM: erro máx. posição (m)', ('slam', 'erro_max_posicao_m'), '{:.3f}'),
        ('SLAM: RMSE yaw (graus)', ('slam', 'rmse_yaw_graus'), '{:.2f}'),
        ('Odometria: RMSE posição (m)', ('odometria', 'rmse_posicao_m'), '{:.3f}'),
        ('Odometria: erro final (m)', ('odometria', 'erro_final_posicao_m'), '{:.3f}'),
        ('Odometria: erro final yaw (graus)', ('odometria', 'erro_final_yaw_graus'), '{:.1f}'),
        ('Correção máx. map->odom (m)', ('correcao_map_odom_max_m',), '{:.3f}'),
        ('Mapa: células ocupadas', ('mapa', 'celulas_ocupadas'), '{:d}'),
        ('Mapa: erro médio das paredes (m)', ('mapa', 'erro_medio_paredes_m'), '{:.3f}'),
        ('Mapa: erro p95 das paredes (m)', ('mapa', 'erro_p95_paredes_m'), '{:.3f}'),
        ('Mapa: % ocupadas sobre a parede', ('mapa', 'fracao_ocupadas_na_parede'), '{:.0%}'),
    ]
    texto = ['| ' + ' | '.join(cabecalho) + ' |',
             '|' + '|'.join(['---'] + ['---:'] * len(execucoes)) + '|']
    for nome, chaves, fmt in linhas:
        valores = [_valor(e['metricas'], *chaves, fmt=fmt) for e in execucoes]
        texto.append('| ' + ' | '.join([nome] + valores) + ' |')
    return '\n'.join(texto)


def figura(execucoes, formas, caminho):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    plt.rcParams.update({
        'font.size': 9, 'axes.edgecolor': TINTA_2, 'axes.labelcolor': TINTA_2,
        'xtick.color': TINTA_2, 'ytick.color': TINTA_2, 'axes.titlecolor': TINTA,
        'figure.facecolor': SUPERFICIE, 'axes.facecolor': SUPERFICIE,
        'axes.spines.top': False, 'axes.spines.right': False})
    n = len(execucoes)
    fig, eixos = plt.subplots(2, n, figsize=(4.6 * n, 8.2), squeeze=False,
                              gridspec_kw={'height_ratios': [1.35, 1]})
    erros_max = []
    for coluna, e in enumerate(execucoes):
        ax = eixos[0][coluna]
        d = e['dados']
        if e['mapa'] is not None:
            grade, resolucao, origem = e['mapa']
            imagem = np.full(grade.shape, 0.80)       # desconhecido: cinza
            imagem[grade == 0] = 1.0                    # livre: branco
            imagem[grade >= 65] = 0.0                   # ocupado: preto
            altura, largura = grade.shape
            ax.imshow(imagem, cmap='gray', vmin=0, vmax=1, origin='lower',
                      interpolation='nearest',
                      extent=[origem[0], origem[0] + largura * resolucao,
                              origem[1], origem[1] + altura * resolucao])
        for i, forma in enumerate(formas):
            contorno = mapa.contornos(forma)
            # fino e translúcido: num mapa perfeito a parede (1 célula) fica sob o contorno
            ax.plot(contorno[:, 0], contorno[:, 1], color=COR_MUNDO, lw=0.7, alpha=0.8,
                    label='geometria real' if i == 0 else None)
        if d:
            ax.plot(d['gt_x'], d['gt_y'], color=TINTA, lw=1.2, ls='--',
                    label='groundtruth')
            ax.plot(d['odom_x'], d['odom_y'], color=COR_ODOM, lw=1.5, label='odometria')
            ax.plot(d['slam_x'], d['slam_y'], color=COR_SLAM, lw=1.5, label='SLAM')
        ax.set_xlim(-2.6, 6.6)
        ax.set_ylim(-3.6, 3.6)
        ax.set_aspect('equal')
        ax.set_xlabel('x (m)')
        if coluna == 0:
            ax.set_ylabel('y (m)')
        ax.set_title(e['metricas'].get('descricao') or e['pasta'].name, fontsize=10)

        ax = eixos[1][coluna]
        if d:
            erro_slam = np.hypot(d['slam_x'] - d['gt_x'], d['slam_y'] - d['gt_y'])
            erro_odom = np.hypot(d['odom_x'] - d['gt_x'], d['odom_y'] - d['gt_y'])
            ax.plot(d['t'], np.maximum(erro_slam, 1e-3), color=COR_SLAM, lw=1.5, label='SLAM')
            if np.max(erro_odom) > 1e-6:
                ax.plot(d['t'], np.maximum(erro_odom, 1e-3), color=COR_ODOM, lw=1.5,
                        label='odometria')
            else:
                ax.text(0.02, 0.95, 'odometria = groundtruth (erro 0)', color=TINTA_2,
                        transform=ax.transAxes, va='top')
            erros_max.append(max(np.max(erro_slam), np.max(erro_odom)))
        ax.set_yscale('log')
        ax.grid(True, which='major', color='#e4e3df', lw=0.6)
        ax.set_xlabel('tempo simulado (s)')
        if coluna == 0:
            ax.set_ylabel('erro de posição (m, escala log)')
        ax.legend(loc='lower right', frameon=False)

    topo = max(erros_max + [0.1]) * 1.5
    for ax in eixos[1]:
        ax.set_ylim(1e-3, topo)
    manipuladores, rotulos = eixos[0][0].get_legend_handles_labels()
    fig.legend(manipuladores, rotulos, loc='upper center', ncol=4, frameon=False,
               bbox_to_anchor=(0.5, 0.995))
    fig.text(0.5, 0.955, 'Mapa: branco = livre, preto = ocupado, cinza = desconhecido.  '
             'Erro: valores abaixo de 1 mm aparecem em 1 mm (escala log).',
             ha='center', color=TINTA_2, fontsize=8)
    fig.tight_layout(rect=(0, 0, 1, 0.945), h_pad=2.5)
    fig.savefig(caminho, dpi=150)
    plt.close(fig)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('pastas', nargs='*', help='pastas de resultados (uma por cenário)')
    parser.add_argument('-o', '--saida', help='PNG de saída (padrão: comparacao.png ao lado)')
    parser.add_argument('-w', '--mundo', help='.world com a geometria real (padrão: o do SLAM)')
    opcoes = parser.parse_args(args)

    pastas = [Path(p) for p in opcoes.pastas] or [
        PASTA_PADRAO / c for c in CENARIOS if (PASTA_PADRAO / c / 'metricas.json').exists()]
    if not pastas:
        parser.error(f'nenhum resultado em {PASTA_PADRAO}; rode o slam.launch.py antes')
    execucoes = [ler_execucao(p) for p in pastas]

    mundo = opcoes.mundo or next(
        (e['metricas'].get('mundo') for e in execucoes if e['metricas'].get('mundo')), None)
    formas = mapa.ler_mundo(mundo_existente(mundo))

    png = Path(opcoes.saida) if opcoes.saida else pastas[0].parent / 'comparacao.png'
    figura(execucoes, formas, png)
    texto = tabela(execucoes)
    png.with_suffix('.md').write_text(texto + '\n', encoding='utf-8')
    print(texto)
    print(f'\nFigura: {png}\nTabela: {png.with_suffix(".md")}')


if __name__ == '__main__':
    main()
