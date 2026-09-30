
    #ros2 run controle_trajetoria comparar_missoes continuo.csv manobras.csv
    #ros2 run controle_trajetoria comparar_missoes a.csv b.csv --saida resultados

import argparse
import csv
import json
import math
import os
import sys

# Nome do parâmetro de limite em cada controlador (contínuo / manobras)
CHAVES_V = ('max_linear_vel', 'v_max')
CHAVES_W = ('max_angular_vel', 'w_max')
LIMIAR_PARADO = 0.01     # m/s: abaixo disso o robô está "sem andar"
LIMIAR_GIRO = 0.05       # rad/s: acima disso o robô está girando


def normalizar(angulo):
    return math.atan2(math.sin(angulo), math.cos(angulo))


def distancia_ao_segmento(px, py, ax, ay, bx, by):
    """Menor distância do ponto P ao segmento AB."""
    dx, dy = bx - ax, by - ay
    comprimento2 = dx * dx + dy * dy
    if comprimento2 == 0.0:
        return math.hypot(px - ax, py - ay)
    u = ((px - ax) * dx + (py - ay) * dy) / comprimento2
    u = max(0.0, min(1.0, u))
    return math.hypot(px - (ax + u * dx), py - (ay + u * dy))


class Missao:
    """Uma missão gravada: metadados do cabeçalho e as amostras do CSV."""

    def __init__(self, caminho):
        self.caminho = caminho
        meta, corpo = {}, []
        with open(caminho, encoding='utf-8') as arquivo:
            for linha in arquivo:
                if linha.startswith('#'):
                    chave, _, valor = linha[1:].strip().partition('=')
                    meta[chave.strip()] = valor
                elif linha.strip():
                    corpo.append(linha)
        self.nome = meta.get('controlador', os.path.basename(caminho))
        self.parametros = json.loads(meta.get('parametros', '{}'))
        self.waypoints = {
            n: (x, y, th) for n, x, y, th in json.loads(
                meta.get('waypoints', '[]'))}
        self.ordem = list(self.waypoints)

        self.linhas = []
        for r in csv.DictReader(corpo):
            self.linhas.append({
                't': float(r['t']), 'x': float(r['x']), 'y': float(r['y']),
                'yaw': float(r['yaw']), 'v': float(r['v_cmd']),
                'w': float(r['w_cmd']), 'wp': r['waypoint'],
                'evento': r['evento']})
        if len(self.linhas) < 2:
            raise ValueError(f'{caminho}: poucas amostras para analisar.')

    def limite(self, chaves):
        for chave in chaves:
            if chave in self.parametros:
                return self.parametros[chave]
        return None

    def calcular_metricas(self):
        L = self.linhas
        distancia = rotacao = t_girando = t_parado_zero = 0.0
        soma_v2 = soma_w2 = 0.0
        for a, b in zip(L, L[1:]):
            dt = b['t'] - a['t']
            distancia += math.hypot(b['x'] - a['x'], b['y'] - a['y'])
            rotacao += abs(normalizar(b['yaw'] - a['yaw']))
            soma_v2 += b['v'] ** 2 * dt
            soma_w2 += b['w'] ** 2 * dt
            if abs(b['v']) < LIMIAR_PARADO and abs(b['w']) > LIMIAR_GIRO:
                t_girando += dt
        duracao = L[-1]['t'] - L[0]['t']

        # Por waypoint: tempo, erro na chegada e desvio do segmento reto
        chegadas = [(i, l) for i, l in enumerate(L) if l['evento']]
        por_wp = []
        inicio_idx, inicio_t = 0, L[0]['t']
        for i_chegada, l in chegadas:
            nome = l['wp']
            wx, wy, wth = self.waypoints.get(nome, (l['x'], l['y'], l['yaw']))
            ax, ay = L[inicio_idx]['x'], L[inicio_idx]['y']
            desvio = max(
                distancia_ao_segmento(p['x'], p['y'], ax, ay, wx, wy)
                for p in L[inicio_idx:i_chegada + 1])
            por_wp.append({
                'nome': nome,
                'tempo': l['t'] - inicio_t,
                'erro_pos': math.hypot(l['x'] - wx, l['y'] - wy),
                'erro_ang': math.degrees(abs(normalizar(l['yaw'] - wth))),
                'desvio': desvio})
            inicio_idx, inicio_t = i_chegada, l['t']

        concluiu = len(chegadas) == len(self.ordem) and len(chegadas) > 0
        tempo_total = chegadas[-1][1]['t'] - L[0]['t'] if chegadas else duracao
        return {
            'concluiu': concluiu,
            'tempo_total': tempo_total,
            'distancia': distancia,
            'rotacao': math.degrees(rotacao),
            't_girando': t_girando,
            'v_rms': math.sqrt(soma_v2 / duracao) if duracao else 0.0,
            'w_rms': math.sqrt(soma_w2 / duracao) if duracao else 0.0,
            'erro_pos_medio': (sum(w['erro_pos'] for w in por_wp) / len(por_wp)
                               if por_wp else float('nan')),
            'erro_ang_medio': (sum(w['erro_ang'] for w in por_wp) / len(por_wp)
                               if por_wp else float('nan')),
            'desvio_medio': (sum(w['desvio'] for w in por_wp) / len(por_wp)
                             if por_wp else float('nan')),
            'por_wp': por_wp,
        }


def tabelas_markdown(missoes, metricas):
    nomes = [m.nome for m in missoes]
    cab = '| Métrica | ' + ' | '.join(nomes) + ' |\n'
    cab += '|---|' + '---|' * len(nomes) + '\n'
    linhas = [
        ('Missão concluída', lambda k: 'sim' if k['concluiu'] else 'NÃO'),
        ('Tempo total (s)', lambda k: f"{k['tempo_total']:.1f}"),
        ('Distância percorrida (m)', lambda k: f"{k['distancia']:.2f}"),
        ('Rotação acumulada (°)', lambda k: f"{k['rotacao']:.0f}"),
        ('Tempo girando parado (s)', lambda k: f"{k['t_girando']:.1f}"),
        ('Velocidade linear RMS (m/s)', lambda k: f"{k['v_rms']:.3f}"),
        ('Velocidade angular RMS (rad/s)', lambda k: f"{k['w_rms']:.3f}"),
        ('Erro de posição na chegada, média (m)',
         lambda k: f"{k['erro_pos_medio']:.3f}"),
        ('Erro de orientação na chegada, média (°)',
         lambda k: f"{k['erro_ang_medio']:.1f}"),
        ('Desvio máx. da reta entre waypoints, média (m)',
         lambda k: f"{k['desvio_medio']:.3f}"),
    ]
    texto = '## Resumo da missão\n\n' + cab
    for rotulo, fmt in linhas:
        texto += f'| {rotulo} | ' + ' | '.join(fmt(k) for k in metricas) + ' |\n'

    texto += '\n## Por waypoint\n\n'
    texto += ('| Waypoint | ' + ' | '.join(
        f'{n}: tempo (s) | {n}: erro pos (m) | {n}: erro ang (°) | '
        f'{n}: desvio (m)' for n in nomes) + ' |\n')
    texto += '|---|' + '---|' * (4 * len(nomes)) + '\n'
    todos = []
    for k in metricas:
        for w in k['por_wp']:
            if w['nome'] not in todos:
                todos.append(w['nome'])
    for nome in todos:
        celulas = []
        for k in metricas:
            w = next((x for x in k['por_wp'] if x['nome'] == nome), None)
            if w is None:
                celulas += ['-'] * 4
            else:
                celulas += [f"{w['tempo']:.1f}", f"{w['erro_pos']:.3f}",
                            f"{w['erro_ang']:.1f}", f"{w['desvio']:.3f}"]
        texto += f'| {nome} | ' + ' | '.join(celulas) + ' |\n'
    return texto


def avisos(missoes):
    """Alerta se as condições dos experimentos não são comparáveis."""
    saida = []
    for rotulo, chaves in (('velocidade linear', CHAVES_V),
                           ('velocidade angular', CHAVES_W)):
        valores = {m.nome: m.limite(chaves) for m in missoes}
        if len({v for v in valores.values() if v is not None}) > 1:
            saida.append(
                f'Os limites de {rotulo} são diferentes ({valores}): '
                'o tempo total não é uma comparação justa.')
    ordens = {tuple(m.ordem) for m in missoes}
    if len(ordens) > 1:
        saida.append('As missões têm waypoints diferentes.')
    return saida


def plotar(missoes, pasta):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib não encontrado: gráficos não gerados '
              '(sudo apt install python3-matplotlib).')
        return []
    cores = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red']
    gerados = []

    # 1) Trajetórias
    fig, ax = plt.subplots(figsize=(7, 7))
    for m, cor in zip(missoes, cores):
        ax.plot([l['x'] for l in m.linhas], [l['y'] for l in m.linhas],
                color=cor, linewidth=1.8, label=m.nome)
        ax.plot(m.linhas[0]['x'], m.linhas[0]['y'], 'o', color=cor,
                markersize=7)
    ref = missoes[0]
    for nome, (x, y, th) in ref.waypoints.items():
        ax.plot(x, y, 'k*', markersize=13)
        ax.annotate(nome, (x, y), textcoords='offset points', xytext=(7, 7))
        ax.arrow(x, y, 0.12 * math.cos(th), 0.12 * math.sin(th),
                 head_width=0.035, color='k', length_includes_head=True)
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Trajetória percorrida (estrela: waypoint e orientação '
                 'desejada; círculo: início)')
    ax.grid(True, alpha=0.3)
    ax.legend()
    caminho = os.path.join(pasta, 'comparacao_trajetorias.png')
    fig.savefig(caminho, dpi=150, bbox_inches='tight')
    gerados.append(caminho)

    # 2) Velocidades comandadas
    fig, (ax_v, ax_w) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for m, cor in zip(missoes, cores):
        t = [l['t'] for l in m.linhas]
        ax_v.plot(t, [l['v'] for l in m.linhas], color=cor, label=m.nome)
        ax_w.plot(t, [l['w'] for l in m.linhas], color=cor, label=m.nome)
        for l in m.linhas:
            if l['evento']:
                ax_v.axvline(l['t'], color=cor, linestyle=':', alpha=0.6)
                ax_w.axvline(l['t'], color=cor, linestyle=':', alpha=0.6)
    ax_v.set_ylabel('v comandada (m/s)')
    ax_w.set_ylabel('ω comandada (rad/s)')
    ax_w.set_xlabel('tempo (s)')
    ax_v.set_title('Comandos em /cmd_vel (linhas pontilhadas: chegadas)')
    for eixo in (ax_v, ax_w):
        eixo.grid(True, alpha=0.3)
        eixo.legend()
    caminho = os.path.join(pasta, 'comparacao_velocidades.png')
    fig.savefig(caminho, dpi=150, bbox_inches='tight')
    gerados.append(caminho)
    return gerados


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Compara missões gravadas pelo seguidor_waypoints.')
    parser.add_argument('csv', nargs='+', help='CSVs das missões (2 ou mais)')
    parser.add_argument('--saida', default='.',
                        help='pasta onde salvar tabela e gráficos')
    args = parser.parse_args(argv)
    if len(args.csv) < 2:
        parser.error('informe ao menos dois CSVs para comparar.')

    missoes = [Missao(c) for c in args.csv]
    metricas = [m.calcular_metricas() for m in missoes]
    for aviso in avisos(missoes):
        print(f'ATENÇÃO: {aviso}')

    texto = tabelas_markdown(missoes, metricas)
    print(texto)
    os.makedirs(args.saida, exist_ok=True)
    caminho_md = os.path.join(args.saida, 'comparacao.md')
    with open(caminho_md, 'w', encoding='utf-8') as arquivo:
        arquivo.write(texto)
    print(f'Tabelas salvas em {caminho_md}')
    for caminho in plotar(missoes, args.saida):
        print(f'Gráfico salvo em {caminho}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
