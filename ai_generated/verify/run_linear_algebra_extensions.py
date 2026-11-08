#!/usr/bin/env python3
"""L: 11 items / 12 exact algebra APIs: independent oracles, examples, and header integration."""
import argparse
import json
import os
from pathlib import Path
import re
import random
import shlex
import subprocess
import tempfile
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]


def run(command, **kwargs):
    result = subprocess.run(list(map(str, command)), capture_output=True, text=True, timeout=300, **kwargs)
    if result.returncode:
        raise RuntimeError(f'{shlex.join(map(str, command))}\n{result.stdout}{result.stderr}')
    return result.stdout


def links(entries):
    names = ['README.md', 'docs/README.md', 'docs/linear_algebra/README.md',
             'docs/linear_algebra_extensions/README.md', 'docs/linear_algebra_extensions/references.md',
             'visualizations/README.md', 'visualizations/linear_algebra_extensions/README.md']
    names += ['docs/' + e['doc'] for e in entries]
    count = 0
    for name in names:
        path = ROOT / name
        source = re.sub(r'```.*?```', '', path.read_text(), flags=re.S)
        source = re.sub(r'`[^`]*`', '', source)
        for target in re.findall(r'\]\(([^)]+)\)', source):
            if '://' in target or target.startswith(('#', 'mailto:')):
                continue
            assert (path.parent / unquote(target.split('#', 1)[0])).exists(), (name, target)
            count += 1
    print(f'{count} local document links passed', flush=True)


def rational_determinant(a):
    from fractions import Fraction
    a = [list(map(Fraction,row)) for row in a]
    n, result = len(a), Fraction(1)
    for k in range(n):
        pivot = next((i for i in range(k,n) if a[i][k]), None)
        if pivot is None: return 0
        if pivot != k:
            a[k],a[pivot] = a[pivot],a[k]
            result = -result
        result *= a[k][k]
        for i in range(k+1,n):
            ratio = a[i][k] / a[k][k]
            for j in range(k,n): a[i][j] -= ratio*a[k][j]
    assert result.denominator == 1
    return int(result)


def characteristic_subset_dp(a):
    n, p = len(a), 998244353
    dp = [[0]*(n+1) for _ in range(1<<n)]
    dp[0][0] = 1
    for mask in range(1<<n):
        row = mask.bit_count()
        for j in range(n):
            if mask >> j & 1: continue
            sign = -1 if (mask >> (j+1)).bit_count() % 2 else 1
            nxt = dp[mask | 1<<j]
            for k in range(n+1):
                nxt[k] = (nxt[k]-sign*a[row][j]*dp[mask][k]) % p
                if row == j and k < n: nxt[k+1] = (nxt[k+1]+sign*dp[mask][k]) % p
    return dp[-1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    parser.add_argument('--links-only', action='store_true')
    args = parser.parse_args()
    entries = json.loads((ROOT / 'verify/linear_algebra_extensions/manifest.json').read_text())['algorithms']
    if args.links_only:
        links(entries)
        return
    flags = shlex.split(args.compiler) + ['-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-I', ROOT]
    runtime = ['-O1', '-g', '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer', '-fno-pie', '-no-pie'] if args.sanitize else ['-O2']
    headers = [e['header'] for e in entries]
    for header in headers:
        for extra in [[], ['-DNDEBUG']]:
            run(flags + extra + ['-x', 'c++', '-fsyntax-only', '-'], input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    print(f'{len(headers)} standalone/repeated headers in debug/release passed', flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-linear-algebra-extensions-') as folder:
        temp = Path(folder)
        def compile(source, name, extra=()):
            binary = temp / name
            run(flags + runtime + list(extra) + [source, '-o', binary])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes += ''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in sorted(ROOT.glob('*/all.hpp')))
        (temp / 'a.cpp').write_text(includes + 'int first(){return cp::determinant_any_mod(cp::Matrix<int>{{2,1},{3,4}},12); }\n')
        (temp / 'b.cpp').write_text(includes + 'int first();int main(){return first()!=5;}\n')
        run(flags + runtime + [temp / 'a.cpp', temp / 'b.cpp', '-o', temp / 'multi'])
        run([temp / 'multi'])
        observed = {}
        for e in entries:
            source = ROOT / 'examples' / e['example']
            doc = (ROOT / 'docs' / e['doc']).read_text()
            assert 'documentation_of: //' + e['header'] in doc
            assert re.search(r'```cpp\n(.*?)```', doc, re.S).group(1) == source.read_text(), e['doc']
            output = run([compile(source, e['mode'])])
            assert output == e['output'], (e['mode'], output)
            observed[e['mode']] = output.split()
        assert observed == json.loads((ROOT / 'verify/linear_algebra_extensions/visualization_expected.json').read_text())
        print('all-category translation units and 12 documented/visualization examples passed', flush=True)
        for extra in [[], ['-DNDEBUG']]:
            for name in ['integer_test', 'finite_test', 'matrix_test']:
                print(run([compile(ROOT / f'verify/linear_algebra_extensions/{name}.cpp', name, extra)]).strip(), flush=True)
            rng = random.Random(20261027)
            determinant = compile(ROOT / 'verify/linear_algebra_extensions/determinant_query.cpp', 'det-query', extra)
            characteristic = compile(ROOT / 'verify/linear_algebra_extensions/characteristic_polynomial.test.cpp', 'char-judge', extra)
            queries, expected = ['48'], []
            for trial in range(48):
                n = rng.randrange(10)
                modulus = rng.choice([1, 2, 4, 6, 1000, 998244353, 2**63, 2**64-1])
                a = [[rng.randrange(-10**30, 10**30) for _ in range(n)] for _ in range(n)]
                if trial % 7 == 0 and n > 1: a[1] = a[0][:]
                d = rational_determinant(a)
                queries.append(f'{n} {modulus}')
                queries += [' '.join(map(str, row)) for row in a]
                expected += [d, d % modulus]
            assert list(map(int, run([determinant], input='\n'.join(queries)+'\n').split())) == expected
            for trial in range(24):
                n = rng.randrange(9)
                a = [[rng.randrange(998244353) for _ in range(n)] for _ in range(n)]
                want = characteristic_subset_dp(a)
                data = str(n)+'\n'+'\n'.join(' '.join(map(str,row)) for row in a)+'\n'
                assert list(map(int,run([characteristic],input=data).split())) == want
            print(('release' if extra else 'debug') + ' and Python 48 rational determinants / 24 subset-DP characteristic cases passed', flush=True)
    links(entries)


if __name__ == '__main__':
    main()
