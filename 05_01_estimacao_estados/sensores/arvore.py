"""Leitura e desenho da árvore de transformações (sem ROS, para testar sem simulador)."""


def grafo_de_yaml(texto):
    """Converte o YAML de `Buffer.all_frames_as_yaml()` em {filho: {parent, ...}}."""
    pai, grafo = None, {}
    for linha in texto.splitlines():
        if not linha.strip():
            continue
        if not linha.startswith(' '):
            pai = linha.split(':', 1)[0].strip()
            grafo[pai] = {}
        elif pai is not None:
            chave, _, valor = linha.strip().partition(':')
            grafo[pai][chave.strip()] = valor.strip().strip("'\"")
    return grafo


def filhos_de(grafo, frame):
    """Frames cujo parent é `frame`, em ordem alfabética."""
    return sorted(f for f, dados in grafo.items() if dados.get('parent') == frame)


def raiz_de(grafo):
    """
    Frame sem parent dentro do grafo (a raiz da árvore), ou None.

    A raiz pode ainda não ter sido publicada como chave: aí ela só aparece como
    `parent` de algum frame e é devolvida mesmo assim.
    """
    if not grafo:
        return None
    for frame, dados in grafo.items():
        pai = dados.get('parent') or ''
        if pai not in grafo:
            return pai or frame
    return None


def renderizar(grafo, raiz=None, descricao=None):
    """
    Desenha a árvore em texto.

    `descricao(frame)` devolve um sufixo opcional para a linha (ex.: a taxa).
    Devolve '' quando o grafo está vazio.
    """
    if not grafo:
        return ''
    raiz = raiz or raiz_de(grafo)
    linhas = []

    def visita(frame, prefixo, ultimo, primeiro):
        conector = '' if primeiro else ('└── ' if ultimo else '├── ')
        # a raiz costuma vir só como `parent`, sem dados de broadcaster/taxa
        sufixo = descricao(frame) if descricao and frame in grafo else ''
        linhas.append(f'{prefixo}{conector}{frame}{sufixo}')
        filhos = filhos_de(grafo, frame)
        for i, filho in enumerate(filhos):
            if primeiro:
                base = ''
            elif ultimo:
                base = prefixo + '    '
            else:
                base = prefixo + '│   '
            visita(filho, base, i == len(filhos) - 1, False)

    visita(raiz, '', True, True)
    return '\n'.join(linhas)


def faltando(grafo, esperados):
    """
    Frames esperados que ainda não apareceram no grafo.

    A raiz da árvore é publicada só como `parent` dos outros frames (o ROS não a
    lista como chave), então ela conta como presente.
    """
    presentes = set(grafo) | {d.get('parent') for d in grafo.values()}
    return [f for f in esperados if f not in presentes]


def para_dot(grafo):
    """Grafo no formato DOT (o mesmo que o `view_frames` desenha)."""
    if not grafo:
        return 'digraph G { "No tf data received" }'
    linhas = ['digraph G {']
    for filho, dados in sorted(grafo.items()):
        pai = dados.get('parent') or ''
        if not pai:
            continue  # a raiz não tem aresta de entrada
        rotulo = (f"Broadcaster: {dados.get('broadcaster', '?')}\\n"
                  f"Average rate: {dados.get('rate', '?')}\\n"
                  f"Buffer length: {dados.get('buffer_length', '?')}")
        linhas.append(f'"{pai}" -> "{filho}"[label=" {rotulo}"];')
    linhas.append('}')
    return '\n'.join(linhas)
