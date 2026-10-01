"""Mostra o resultados_rmse.csv como tabelas dos bônus 2.1 e 2.2.

Uso:  python3 tabelas.py [--limiar 0.15] [--csv resultados_rmse.csv]
Se uma configuração foi rodada mais de uma vez, usa a rodada mais recente.
"""
import argparse
import csv
import math
import os


def rodadas(caminho):
    """Agrupa as linhas por configuração, ficando só com a rodada mais recente de cada uma."""
    ultimas = {}
    with open(caminho) as f:
        for l in csv.DictReader(f):
            chave = (float(l['omega']), l['modo'], float(l['k_ff']))
            if l['volta'] == '1':
                ultimas[chave] = []
            ultimas.setdefault(chave, []).append(l)
    return ultimas


def resumo(voltas, fonte):
    """RMSE de todas as voltas juntas (voltas têm a mesma duração) e da última volta."""
    vals = [(float(v[f'rmse_{fonte}_m']), float(v[f'rmse_{fonte}_graus'])) for v in voltas if v[f'rmse_{fonte}_m']]
    if not vals:
        return None
    total = tuple(math.sqrt(sum(x[i] ** 2 for x in vals) / len(vals)) for i in (0, 1))
    return total, vals[-1]


def fmt(r, qual):
    return f'{"-":>8}   {"-":>6} ' if r is None else f'{r[qual][0]:>6.3f} m {r[qual][1]:>5.1f}°'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--csv', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resultados_rmse.csv'))
    ap.add_argument('--limiar', type=float, default=0.15, help='RMSE real da última volta para dizer que segue (m)')
    args = ap.parse_args()
    r = rodadas(args.csv)

    print('\nBônus 2.2: erro de pose (RMSE) por tipo de controle')
    print(f'{"Ω":>5}  {"caso":<22} {"voltas":>6} | {"real (todas)":^16} | {"real (última)":^16} | {"odom (todas)":^16}')
    casos = [('malha_aberta', None, 'malha aberta'), ('realimentado', 0.0, 'realimentado K_ff=0'),
             ('realimentado', 1.0, 'realimentado K_ff=1')]
    for w in sorted({k[0] for k in r if k[1] == 'malha_aberta'}):
        for modo, kff, nome in casos:
            chave = next((k for k in r if k[0] == w and k[1] == modo and (kff is None or k[2] == kff)), None)
            if chave:
                real, odom = resumo(r[chave], 'real'), resumo(r[chave], 'odom')
                print(f'{w:>5.2f}  {nome:<22} {len(r[chave]):>6} | {fmt(real, 0)} | {fmt(real, 1)} | {fmt(odom, 0)}')

    print(f'\nBônus 2.1: maior Ω seguido (realimentado, K_ff=1; segue se RMSE real da última volta ≤ {args.limiar} m)')
    print(f'{"Ω":>5} {"voltas":>6} | {"real (última)":^16} | {"odom (última)":^16} | segue')
    maior, quebrou = None, False
    for w, modo, kff in sorted(k for k in r if k[1] == 'realimentado' and k[2] == 1.0):
        real, odom = resumo(r[(w, modo, kff)], 'real'), resumo(r[(w, modo, kff)], 'odom')
        segue = real is not None and real[1][0] <= args.limiar
        quebrou = quebrou or not segue
        if not quebrou:
            maior = w
        print(f'{w:>5.2f} {len(r[(w, modo, kff)]):>6} | {fmt(real, 1)} | {fmt(odom, 1)} | {"sim" if segue else "não"}')
    print(f'\nMaior Ω seguido: {maior if maior is not None else "nenhum"}\n')


if __name__ == '__main__':
    main()
