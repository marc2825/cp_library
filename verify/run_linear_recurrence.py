#!/usr/bin/env python3
"""3.18 matrix recurrences, Kitamasa, Berlekamp-Massey and shared Bostan-Mori."""
import argparse
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
MOD = 998244353
EXAMPLES = {'matrix_recurrence': '55\n5 1\n', 'kitamasa': '55\n0 4 \n', 'berlekamp_massey': '1 1 \n55\n', 'bostan_mori': '55\n55\n'}


def run(command, **kwargs):
    result = subprocess.run(command, text=True, capture_output=True, **kwargs)
    if result.returncode:
        raise RuntimeError(f'{shlex.join(map(str, command))}\n{result.stdout}{result.stderr}')
    return result.stdout


def matrix_multiply(a, b, p):
    d = len(a)
    return [[sum(a[i][k]*b[k][j] for k in range(d)) % p for j in range(d)] for i in range(d)]


def matrix_power(a, n, p):
    if not n:
        return [[int(i == j) % p for j in range(len(a))] for i in range(len(a))]
    half = matrix_power(a, n//2, p)
    result = matrix_multiply(half, half, p)
    return matrix_multiply(result, a, p) if n % 2 else result


def weights(c, n, p):
    # Ascending-state companion matrix; independent of polynomial reduction.
    d = len(c)
    a = [[0]*d for _ in range(d)]
    for i in range(d-1): a[i][i+1] = 1
    a[-1] = [x % p for x in reversed(c)]
    return matrix_power(a, n, p)[0]


def nth(a, c, n, p):
    return sum(x*y for x, y in zip(a, weights(c, n, p))) % p


def minimum_order(s, p):
    for d in range(len(s)+1):
        a = [[s[i-1-j] % p for j in range(d)]+[s[i] % p] for i in range(d, len(s))]
        rank = 0
        for col in range(d):
            pivot = next((i for i in range(rank, len(a)) if a[i][col]), None)
            if pivot is None: continue
            a[rank], a[pivot] = a[pivot], a[rank]
            inv = pow(a[rank][col], -1, p)
            a[rank] = [x*inv % p for x in a[rank]]
            for i in range(rank+1, len(a)):
                scale = a[i][col]
                a[i] = [(x-scale*y) % p for x, y in zip(a[i], a[rank])]
            rank += 1
        if all(any(row[:-1]) or not row[-1] for row in a): return d
    raise AssertionError('no candidate order')


def fits(s, c, p):
    return all(s[i] % p == sum(x*s[i-1-j] for j, x in enumerate(c)) % p for i in range(len(c), len(s)))


def rational(a, b, n, p):
    s = []
    for i in range(n+1):
        x = a[i] if i < len(a) else 0
        s.append((x-sum(b[j]*s[i-j] for j in range(1, min(i+1, len(b)))))*pow(b[0], -1, p) % p)
    return s[n]


def oracle(op, p, a, b, d, n):
    if op in ('K', 'M', 'R'):
        if op == 'R' and p not in (17, 97, MOD): return 'NTT'
        if not a or len(a) != len(b): return 'INVALID'
        if op == 'R' and n >= len(a) and p in (17, 97) and len(a) >= (8 if p == 17 else 16): return 'LENGTH'
        return [nth(a, b, n, p)]
    if op == 'C': return [*weights(b, n, p)] if b else 'INVALID'
    if op in ('F', 'E'):
        if p in (1, 12, 2000000000): return 'PRIME'
        return ('BM', a, p, op, n)
    if op == 'B':
        if p not in (17, 97, MOD): return 'NTT'
        if not b or b[0] % p == 0: return 'DOMAIN'
        trim = lambda x: next((i+1 for i in range(len(x)-1, -1, -1) if x[i] % p), 0)
        if p in (17, 97) and max(trim(a), trim(b)) > (8 if p == 17 else 16): return 'LENGTH'
        return [rational(a, b, n, p)]
    if op in ('P', 'A'):
        if len(a) != d*d or (op == 'A' and len(b) != d): return 'INVALID'
        power = matrix_power([a[i*d:(i+1)*d] for i in range(d)], n, p)
        return [x for row in power for x in row] if op == 'P' else [sum(x*y for x, y in zip(row, b)) % p for row in power]
    raise AssertionError(op)


def arbitrary_precision(binary):
    rng = random.Random(318018)
    commands, expected = [], []
    def append(op, p, a, b, d, n):
        commands.append(f'{op} {p} {len(a)} {len(b)} {d} {n}\n'+' '.join(map(str, a))+'\n'+' '.join(map(str, b))+'\n')
        expected.append(oracle(op, p, a, b, d, n))
    for p in [1, 2, 12, 17, 97, MOD, 1000000007, 2000000000]:
        for trial in range(48):
            d = rng.randrange(1, 11)
            a = [rng.choice([-2**31, 2**31-1, rng.randrange(-p, p)]) for _ in range(d)]
            b = [rng.choice([-2**31, 2**31-1, rng.randrange(-p, p)]) for _ in range(d)]
            n = rng.choice([0, d-1, d, rng.randrange(150), 2**63, 2**64-1])
            for op in 'KMCR': append(op, p, a, b, d, n)
            s = [rng.randrange(-p, p) for _ in range(rng.randrange(14))]
            for op in 'FE': append(op, p, s, [], 0, n)
            aa, bb = a[:], b[:]
            if trial % 3 == 0: bb[0] = 1
            append('B', p, aa, bb, d, rng.randrange(60))
            dimension = rng.randrange(6)
            matrix = [rng.randrange(-p, p) for _ in range(dimension**2)]
            state = [rng.randrange(-p, p) for _ in range(dimension)]
            for op in 'PA': append(op, p, matrix, state, dimension, n)
        for op, a, b, d, n in [('K', [], [], 0, 0), ('M', [1], [], 0, 0), ('C', [], [], 0, 0), ('P', [1], [], 2, 0), ('A', [1], [], 1, 0), ('B', [1], [0], 0, 0)]:
            append(op, p, a, b, d, n)
    lines = run([str(binary)], input=''.join(commands)).splitlines()
    assert len(lines) == len(expected), (len(lines), len(expected))
    for i, (line, want) in enumerate(zip(lines, expected)):
        if isinstance(want, str):
            assert line == want, (i, line, want)
            continue
        tokens = list(map(int, line.split())); size = tokens[0]
        result, rest = tokens[1:1+size], tokens[1+size:]
        assert len(result) == size
        if isinstance(want, tuple):
            _, s, p, op, n = want
            assert size == minimum_order(s, p) and fits(s, result, p), (i, line, want)
            assert all(0 <= x < p for x in result)
            if op == 'E':
                target = s[n] % p if n < len(s) else nth(s[:size], result, n, p) if size else 0
                assert rest == [target], (i, line, target)
            else: assert not rest
        else:
            assert result == want and not rest, (i, line, want)
    return len(expected)


def protocol(name, binary):
    rng = random.Random(318018)
    for trial in range(64):
        d, n = rng.randrange(1, 10), rng.choice([0, rng.randrange(180), 2**63-1])
        a = [rng.randrange(MOD) for _ in range(d)]
        b = [rng.randrange(MOD) for _ in range(d)]
        if name == 'berlekamp_massey':
            s = [rng.randrange(MOD) for _ in range(rng.randrange(20))]
            data = f'{len(s)}\n'+' '.join(map(str, s))+'\n'
            out = list(map(int, run([str(binary)], input=data).split()))
            assert out[0] == len(out)-1 == minimum_order(s, MOD) and fits(s, out[1:], MOD)
        elif name == 'matrix_power_mod':
            matrix = [[rng.randrange(MOD) for _ in range(d)] for _ in range(d)]
            data = f'{d} {n}\n'+''.join(' '.join(map(str, row))+'\n' for row in matrix)
            out = list(map(int, run([str(binary)], input=data).split()))
            assert out == [x for row in matrix_power(matrix, n, MOD) for x in row]
        else:
            data = f'{d} {n}\n'+' '.join(map(str, a))+'\n'+' '.join(map(str, b))+'\n'
            assert int(run([str(binary)], input=data)) == nth(a, b, n, MOD)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    args = parser.parse_args()
    compiler = shlex.split(args.compiler)
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-I', str(ROOT)]
    runtime = ['-O2', '-g'] if not args.sanitize else ['-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-sanitize-recover=all', '-fno-pie', '-no-pie']
    headers = sorted(str(p.relative_to(ROOT)) for p in (ROOT/'linear_recurrence').rglob('*.hpp'))
    for header in headers:
        for extra in [[], ['-DNDEBUG']]:
            run(compiler+flags+extra+['-fsyntax-only', '-x', 'c++', '-'], input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    for declaration in ['cp::berlekamp_massey<12>({});', 'cp::berlekamp_massey<1>({});', 'cp::kitamasa<0>({1},{1},0);', 'cp::matrix_power_mod<2000000001>({{1}},0);']:
        result = subprocess.run(compiler+flags+['-fsyntax-only', '-x', 'c++', '-'], input='#include "linear_recurrence/all.hpp"\nint main(){'+declaration+'}\n', text=True, capture_output=True)
        assert result.returncode and 'static assertion failed' in result.stderr
    print(f'{len(headers)} standalone headers (debug/release); 4 invalid modulus configurations rejected', flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-linear-recurrence-') as directory:
        temp = Path(directory)
        def compile(source, suffix='', extra=()):
            binary = temp/(source.stem+suffix)
            run(compiler+flags+list(extra)+runtime+[str(source), '-o', str(binary)])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))+'#include "fps/all.hpp"\n#include "combinatorics/all.hpp"\n#include "tree/all.hpp"\n'
        (temp/'first.cpp').write_text(includes+'int first(){return cp::linear_recurrence_nth({1},{1},1)-1;}\n')
        (temp/'second.cpp').write_text(includes+'int first();int main(){return first()+cp::kitamasa({1},{1},1)-1;}\n')
        run(compiler+flags+runtime+[str(temp/'first.cpp'), str(temp/'second.cpp'), '-o', str(temp/'multi')])
        run([str(temp/'multi')])
        for name, want in EXAMPLES.items():
            actual = run([str(compile(ROOT/f'examples/linear_recurrence/{name}.cpp'))])
            assert actual == want, (name, actual, want)
        for source in sorted((ROOT/'verify/linear_recurrence').glob('*.test.cpp')):
            protocol(source.name.removesuffix('.test.cpp'), compile(source))
        print('mixed/reverse includes, multiple TUs, 4 examples, 5 judge wrappers / 320 cases passed', flush=True)
        for suffix, extra in [('_debug', []), ('_release', ['-DNDEBUG'])]:
            count = arbitrary_precision(compile(ROOT/'verify/linear_recurrence/query_driver.cpp', suffix, extra))
            print(f'{suffix[1:]}: {count} arbitrary-precision queries passed', flush=True)
            print(run([str(compile(ROOT/'verify/linear_recurrence/unit_test.cpp', suffix, extra))]), end='', flush=True)


if __name__ == '__main__':
    main()
