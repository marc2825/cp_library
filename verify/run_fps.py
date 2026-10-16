#!/usr/bin/env python3
"""3.17 polynomial/FPS: standalone headers, independent algebra, protocols and large inputs."""
import argparse
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
MOD = 998244353
EXAMPLES = {
    'polynomial': '0 3 3 \n2 1 3 \n16 16 16 3 \n2 6 \n0 1 1 1 \n1 2 3 0 0 \n',
    'inv': '1 1 1 1 1 1 \n', 'log': '0 1 8 6 4 \n', 'exp': '1 1 9 3 5 \n',
    'pow': '0 0 0 8 12 6 1 \n', 'sqrt': '0 2 1 0 0 0 \n',
    'division': '3 16 4 \n15 \n', 'multipoint_evaluation': '1 6 0 \n',
    'interpolation': '1 2 3 \n', 'taylor_shift': '0 14 3 \n', 'bostan_mori': '4 \n4 \n',
}


def run(command, **kwargs):
    result = subprocess.run(command, text=True, capture_output=True, **kwargs)
    if result.returncode:
        raise RuntimeError(f'{shlex.join(map(str, command))}\n{result.stdout}{result.stderr}')
    return result.stdout


def product(a, b, p, n=None):
    if n is None:
        n = len(a) + len(b) - 1 if a and b else 0
    c = [0] * n
    for i, x in enumerate(a[:n]):
        for j, y in enumerate(b[:n-i]):
            c[i+j] = (c[i+j] + x*y) % p
    return c


def prefix(a, n, p):
    return [x % p for x in a[:n]] + [0] * max(0, n-len(a))


def trim(a, p):
    a = [x % p for x in a]
    while a and not a[-1]:
        a.pop()
    return a


def inverse(a, n, p):
    if not n:
        return []
    a = prefix(a, n, p)
    b = [pow(a[0], -1, p)]
    for i in range(1, n):
        b.append(-sum(a[j]*b[i-j] for j in range(1, i+1))*b[0] % p)
    return b


def exponential(a, n, p):
    if not n:
        return []
    a = prefix(a, n, p)
    b = [1]
    for i in range(1, n):
        b.append(sum(j*a[j]*b[i-j] for j in range(1, i+1))*pow(i, -1, p) % p)
    return b


def logarithm(a, n, p):
    if not n:
        return []
    d = [(i*x) % p for i, x in enumerate(a[:n])][1:]
    quotient = product(d, inverse(a, n, p), p, n-1)
    return [0]+[x*pow(i+1, -1, p) % p for i, x in enumerate(quotient)]


def power(a, k, n, p):
    result = prefix([1], n, p)
    while k:
        if k & 1:
            result = product(result, a, p, n)
        a = product(a, a, p, n)
        k >>= 1
    return result


def divide(a, b, p):
    a, b = trim(a, p), trim(b, p)
    q = [0]*max(0, len(a)-len(b)+1)
    while a and len(a) >= len(b):
        index = len(a)-len(b)
        q[index] = a[-1]*pow(b[-1], -1, p) % p
        for j, x in enumerate(b):
            a[index+j] = (a[index+j]-q[index]*x) % p
        a = trim(a, p)
    return trim(q, p), a


def evaluate(a, x, p):
    value = 0
    for c in reversed(a):
        value = (value*x+c) % p
    return value


def interpolate(xs, ys, p):
    result = [0]*len(xs)
    for i, (x, y) in enumerate(zip(xs, ys)):
        basis, denominator = [1], 1
        for j, z in enumerate(xs):
            if i != j:
                basis = product(basis, [-z, 1], p)
                denominator = denominator*(x-z) % p
        scale = y*pow(denominator, -1, p) % p
        result = [(v+scale*b) % p for v, b in zip(result, basis)]
    return result


def shift(a, c, p):
    # Horner in the polynomial ring, without factorials or convolution.
    result = []
    for x in reversed(a):
        new = [0]*(len(result)+1)
        for i, y in enumerate(result):
            new[i] = (new[i]+c*y) % p
            new[i+1] = (new[i+1]+y) % p
        new[0] = (new[0]+x) % p
        result = new
    return result


def recurrence(initial, c, k, p):
    # x^k modulo the characteristic polynomial (independent of Bostan-Mori).
    d = len(c)
    def combine(a, b):
        r = product(a, b, p, 2*d-1)
        for i in range(2*d-2, d-1, -1):
            for j, x in enumerate(c):
                r[i-1-j] = (r[i-1-j]+r[i]*x) % p
        return r[:d]
    result, base = [1]+[0]*(d-1), ([0, 1]+[0]*(d-2) if d > 1 else [c[0] % p])
    while k:
        if k & 1:
            result = combine(result, base)
        base = combine(base, base)
        k >>= 1
    return sum(x*y for x, y in zip(result, initial)) % p


def root_exists(a, n, p):
    a = prefix(a, n, p)
    v = next((i for i, x in enumerate(a) if x), n)
    return v == n or (v % 2 == 0 and pow(a[v], (p-1)//2, p) == 1)


def oracle(op, a, b, k, c, n, p):
    limits = {17: 8, 97: 16, MOD: 2**22, 1224736769: 2**23}
    limit = limits[p]
    if op in 'FLEPQ' and n > limit:
        return 'LENGTH'
    if op == 'A': return [(x+y) % p for x, y in zip(prefix(a, max(len(a), len(b)), p), prefix(b, max(len(a), len(b)), p))]
    if op == 'S': return [(x-y) % p for x, y in zip(prefix(a, max(len(a), len(b)), p), prefix(b, max(len(a), len(b)), p))]
    if op == 'M': return 'LENGTH' if a and b and len(a)+len(b)-1 > 2*limit else product(a, b, p)
    if op == 'V': return [i*x % p for i, x in enumerate(a)][1:]
    if op == 'I': return 'DOMAIN' if len(a) >= p else [0]+[x*pow(i+1, -1, p) % p for i, x in enumerate(a)]
    if op == 'T': return prefix(a, n, p)
    if op == 'F': return 'DOMAIN' if n and (not a or not a[0] % p) else inverse(a, n, p)
    if op == 'L': return 'DOMAIN' if n and (not a or a[0] % p != 1) else logarithm(a, n, p)
    if op == 'E': return 'DOMAIN' if n and a and a[0] % p else exponential(a, n, p)
    if op == 'P': return power(a, k, n, p)
    if op == 'Q': return ('ROOT', a, n, p)
    if op == 'D':
        if not trim(b, p): return 'DOMAIN'
        if max(len(trim(a, p)), len(trim(b, p))) > limit: return 'LENGTH'
        return ('DIV', *divide(a, b, p))
    if op == 'X':
        if not b: return []
        if len(b) >= limit or len(trim(a, p)) > limit: return 'LENGTH'
        return [evaluate(a, x, p) for x in b]
    if op == 'H':
        if len(a) != len(b): return 'INVALID'
        if not a: return []
        if len(a) >= limit: return 'LENGTH'
        if len({x % p for x in a}) != len(a): return 'DOMAIN'
        return interpolate(a, b, p)
    if op == 'C': return 'LENGTH' if len(a) > limit else shift(a, c, p)
    if op == 'B':
        if not b or not b[0] % p: return 'DOMAIN'
        if max(len(trim(a, p)), len(trim(b, p))) > limit: return 'LENGTH'
        return [product(a, inverse(b, k+1, p), p, k+1)[k]]
    if op == 'R':
        if not a or len(a) != len(b): return 'INVALID'
        if k < len(a): return [a[k] % p]
        if len(a) >= limit: return 'LENGTH'
        return [recurrence(a, b, k, p)]
    raise AssertionError(op)


def parse_poly(tokens):
    count = tokens[0]
    return tokens[1:1+count], tokens[1+count:]


def arbitrary_precision(binary):
    rng = random.Random(317017)
    commands, expected = [], []
    for p in [17, 97, MOD, 1224736769]:
        maximum = 8 if p == 17 else 16 if p == 97 else 75
        for trial in range(64):
            a = [rng.choice([-2**31, 2**31-1, rng.randrange(-p, p)]) for _ in range(rng.randrange(maximum+2))]
            b = [rng.randrange(-p, p) for _ in range(rng.randrange(maximum+2))]
            n, k, c = rng.randrange(maximum+2), rng.choice([0, 1, 2, 5, 2**64-1, 2**63]), rng.choice([-2**63, 2**63-1, -2, 0, 1])
            for op in 'ASM VITFLEPQDXHCBR'.replace(' ', ''):
                aa, bb, kk = a[:], b[:], k
                if op in 'FLE' and trial % 3 and aa:
                    aa[0] = {'F': 2, 'L': 1, 'E': 0}[op]
                if op == 'Q' and trial % 2:
                    aa = product(aa, aa, p, n)
                    if trial % 4 == 1: aa = [0, 0]+aa
                if op == 'H' and trial % 2:
                    aa = list(range(min(len(aa), maximum-1)))
                    rng.shuffle(aa); bb = [rng.randrange(p) for _ in aa]
                if op == 'B':
                    kk = rng.randrange(80)
                    if bb and trial % 2: bb[0] = 1
                if op == 'R' and trial % 2:
                    aa, bb = aa[:8], bb[:8]
                    size = min(len(aa), len(bb)); aa, bb = aa[:size], bb[:size]
                commands.append(f'{op} {p} {len(aa)} {len(bb)} {kk} {c} {n}\n'+ ' '.join(map(str, aa))+'\n'+' '.join(map(str, bb))+'\n')
                expected.append(oracle(op, aa, bb, kk, c, n, p))
    lines = run([str(binary)], input=''.join(commands)).splitlines()
    assert len(lines) == len(expected), (len(lines), len(expected))
    for i, (line, want) in enumerate(zip(lines, expected)):
        if isinstance(want, str):
            assert line == want, (i, line, want)
            continue
        if isinstance(want, tuple) and want[0] == 'ROOT':
            _, a, n, p = want
            exists = root_exists(a, n, p)
            if line == 'NONE':
                assert not exists, (i, a, n, p)
            else:
                got, rest = parse_poly(list(map(int, line.split())))
                assert exists and not rest and len(got) == n and product(got, got, p, n) == prefix(a, n, p), (i, line, want)
                v = next((j for j, x in enumerate(prefix(a, n, p)) if x), n)
                if v < n: assert 0 < got[v//2] <= p//2
            continue
        tokens = list(map(int, line.split()))
        got, rest = parse_poly(tokens)
        if isinstance(want, tuple):
            r, rest = parse_poly(rest)
            assert (got, r) == want[1:] and not rest, (i, line, want)
        else:
            assert got == want and not rest, (i, line, want)
    return len(expected)


def protocol(name, binary):
    rng = random.Random(317017)
    for trial in range(48):
        n, m, k = rng.randrange(1, 85), rng.randrange(1, 80), rng.choice([rng.randrange(180), 2**63-1])
        a = [rng.randrange(MOD) for _ in range(n)]
        if name in ('inv', 'log', 'exp', 'pow', 'sqrt'):
            if name in ('inv', 'log', 'exp'): a[0] = {'inv': 2, 'log': 1, 'exp': 0}[name]
            if name == 'sqrt':
                a = product(a, a, MOD, n) if trial % 3 else ([0, 1]+[0]*(n-2) if n > 1 else [3])
            data = f'{n}'+(f' {k}' if name == 'pow' else '')+'\n'+' '.join(map(str, a))+'\n'
            op = {'inv': 'F', 'log': 'L', 'exp': 'E', 'pow': 'P', 'sqrt': 'Q'}[name]
            want = oracle(op, a, [], k, 0, n, MOD)
            line = run([str(binary)], input=data).strip()
            if name == 'sqrt':
                if line == '-1': assert not root_exists(a, n, MOD)
                else:
                    got = list(map(int, line.split()))
                    assert len(got) == n and product(got, got, MOD, n) == a
                continue
        elif name == 'division':
            b = [rng.randrange(MOD) for _ in range(m)]; b[-1] = 1
            data = f'{n} {m}\n'+' '.join(map(str, a))+'\n'+' '.join(map(str, b))+'\n'
            q, r = divide(a, b, MOD)
            actual = list(map(int, run([str(binary)], input=data).split()))
            assert actual == [len(q), len(r)]+q+r
            continue
        elif name == 'multipoint_evaluation':
            xs = [rng.randrange(MOD) for _ in range(m)]
            data = f'{n} {m}\n'+' '.join(map(str, a))+'\n'+' '.join(map(str, xs))+'\n'
            want = [evaluate(a, x, MOD) for x in xs]
        elif name == 'interpolation':
            xs = rng.sample(range(1000000), n)
            ys = [evaluate(a, x, MOD) for x in xs]
            data = f'{n}\n'+' '.join(map(str, xs))+'\n'+' '.join(map(str, ys))+'\n'
            want = a
        elif name == 'taylor_shift':
            c = rng.randrange(MOD)
            data = f'{n} {c}\n'+' '.join(map(str, a))+'\n'
            want = shift(a, c, MOD)
        else:
            a = a[:8]; n = len(a)
            c = [rng.randrange(MOD) for _ in a]
            data = f'{n} {k}\n'+' '.join(map(str, a))+'\n'+' '.join(map(str, c))+'\n'
            want = [recurrence(a, c, k, MOD)]
        actual = list(map(int, run([str(binary)], input=data).split()))
        assert actual == want, (name, trial, actual, want)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    args = parser.parse_args()
    compiler = shlex.split(args.compiler)
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-I', str(ROOT)]
    runtime = ['-O2', '-g'] if not args.sanitize else ['-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-sanitize-recover=all', '-fno-pie', '-no-pie']
    headers = sorted(str(p.relative_to(ROOT)) for p in (ROOT/'fps').rglob('*.hpp'))
    for h in headers:
        for extra in [[], ['-DNDEBUG']]:
            run(compiler+flags+extra+['-fsyntax-only', '-x', 'c++', '-'], input=f'#include "{h}"\n#include "{h}"\nint main(){{}}\n')
    print(f'{len(headers)} standalone headers in debug/release passed', flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-fps-') as directory:
        temp = Path(directory)
        def compile(source, suffix='', extra=()):
            binary = temp/(source.stem+suffix)
            run(compiler+flags+list(extra)+runtime+[str(source), '-o', str(binary)])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))+'#include "convolution/all.hpp"\n#include "combinatorics/all.hpp"\n#include "number_theory_convolution/all.hpp"\n#include "tree/all.hpp"\n'
        (temp/'first.cpp').write_text(includes+'int first(){return cp::fps_inv({1},1)[0]-1;}\n')
        (temp/'second.cpp').write_text(includes+'int first(); int main(){return first()+cp::fps_inv({1},1)[0]-1;}\n')
        run(compiler+flags+runtime+[str(temp/'first.cpp'), str(temp/'second.cpp'), '-o', str(temp/'multi')])
        run([str(temp/'multi')])
        print('reverse/mixed includes and multiple translation units passed', flush=True)
        for name, want in EXAMPLES.items():
            actual = run([str(compile(ROOT/f'examples/fps/{name}.cpp'))])
            assert actual == want, (name, actual, want)
        for source in sorted((ROOT/'verify/fps').glob('*.test.cpp')):
            protocol(source.name.removesuffix('.test.cpp'), compile(source))
        print('11 examples; 10 judge wrappers / 480 independent protocol cases passed', flush=True)
        for suffix, extra in [('_debug', []), ('_release', ['-DNDEBUG'])]:
            count = arbitrary_precision(compile(ROOT/'verify/fps/query_driver.cpp', suffix, extra))
            print(f'{suffix[1:]}: {count} arbitrary-precision protocol queries passed', flush=True)
            print(run([str(compile(ROOT/'verify/fps/unit_test.cpp', suffix, extra))]), end='', flush=True)


if __name__ == '__main__':
    main()
