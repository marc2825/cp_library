#!/usr/bin/env python3
"""3.13: basic integer arithmetic, independent arbitrary-precision oracles."""
import argparse
import math
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = {'gcd_lcm': '6\n72\n', 'extended_gcd': '6 1 2\n6\n',
            'mod_pow': '24\n' + str(9000000000000000000 ** 2 % (2**63 - 1)) + '\n',
            'mod_inverse': '3\nnone\n', 'prime_sieve': '2 3 5 7 11 13 17 19 \n0 1\n',
            'smallest_prime_factor': '2\n2^2 3^1 7^1 \n', 'prime_factorization': '2^3 3^2 5^1 \n',
            'divisors': '1 2 3 4 6 9 12 18 36 \n'}


def run(command, **kwargs):
    result = subprocess.run(command, text=True, capture_output=True, **kwargs)
    if result.returncode:
        raise RuntimeError(f"{shlex.join(map(str, command))}\n{result.stdout}{result.stderr}")
    return result.stdout


def bigint_cases(binary):
    rng = random.Random(313016)
    lo, hi = -2**63, 2**63 - 1
    edge = [lo, lo+1, lo+2, -2**62, -1000000007, -2, -1, 0, 1, 2, 1000000007, 2**62, hi-2, hi-1, hi]
    pairs = [(a, b) for a in edge for b in edge]
    pairs += [(rng.randint(lo, hi), rng.randint(lo, hi)) for _ in range(8000)]
    # Euclid 最悪近傍: 連続 Fibonacci とその符号。
    a, b = 1, 1
    while a + b <= hi:
        a, b = b, a + b
        pairs += [(sa*a, sb*b) for sa in [-1, 1] for sb in [-1, 1]]
    commands, expected = [], []
    for a, b in pairs:
        commands.append(f'g {a} {b}\n'); expected.append(('g', a, b))
    for a in edge + [rng.randint(lo, hi) for _ in range(4000)]:
        modulus = rng.choice([1, 2, 3, 998244353, 2**32, 2**62, hi, rng.randrange(1, hi)])
        b = rng.randint(lo, hi)
        exponent = rng.choice([0, 1, 2, hi, rng.randrange(hi)])
        commands += [f'm {a} {b} {modulus}\n', f'p {a} {exponent} {modulus}\n', f'i {a} {modulus}\n']
        expected += [('m', a*b % modulus), ('p', pow(a, exponent, modulus)),
                     ('i', pow(a, -1, modulus) if math.gcd(a, modulus) == 1 else -1)]
    output = run([str(binary)], input=''.join(commands)).splitlines()
    assert len(output) == len(expected)
    for actual, want in zip(output, expected):
        if want[0] == 'g':
            _, a, b = want
            g, eg, x, y, l = map(int, actual.split())
            assert g == eg == math.gcd(a, b) == a*x + b*y, (a, b, actual)
            value = math.lcm(a, b)
            assert l == (value if value < 2**64 else -1), (a, b, l, value)
            assert lo <= x <= hi and lo <= y <= hi
        else: assert int(actual) == want[1], (actual, want)
    return len(expected)


def protocol(name, binary):
    rng = random.Random(313016)
    for trial in range(48):
        if name in ('gcd', 'extended_gcd'):
            a, b = rng.randrange(1, 201), rng.randrange(1, 201)
            data = f'{a} {b}\n'
            if name == 'gcd': expected = [math.gcd(a, b)]
            else:
                g = math.gcd(a, b)
                candidates = [(x, (g-a*x)//b) for x in range(-b, b+1) if (g-a*x) % b == 0]
                expected = list(min(candidates, key=lambda p: (abs(p[0])+abs(p[1]), p[0])))
        elif name == 'lcm':
            values = [rng.randrange(1, 31) for _ in range(rng.randrange(1, 8))]
            data = f'{len(values)}\n'+' '.join(map(str, values))+'\n'; expected = [math.lcm(*values)]
        elif name == 'mod_pow':
            a, b = rng.randrange(1, 10**9), rng.randrange(10**9)
            data = f'{a} {b}\n'; expected = [pow(a, b, 1000000007)]
        elif name == 'mod_inverse':
            triples = [(rng.randrange(2, 81), rng.randrange(1, 81), rng.randrange(1, 81)) for _ in range(10)]
            triples = [(n, s % (n-1)+1, k) for n, s, k in triples]
            data = f'{len(triples)}\n'+''.join(f'{n} {s} {k}\n' for n, s, k in triples)
            expected = [next((x for x in range(n) if (s+k*x) % n == 0), -1) for n, s, k in triples]
        elif name == 'prime_sieve':
            values = [rng.randrange(2, 3001) for _ in range(30)]
            data = f'{len(values)}\n'+' '.join(map(str, values))+'\n'
            expected = [sum(all(n % d for d in range(2, n)) for n in values)]
        elif name == 'smallest_prime_factor':
            n = rng.randrange(2, 201); data = f'{n}\n'
            expected = [sum(a*b < n for a in range(1, n) for b in range(1, n))]
        elif name == 'prime_factorization':
            n = rng.randrange(2, 100001); data = f'{n}\n'
            remaining, factors = n, []
            for p in range(2, n+1):
                while remaining % p == 0: factors.append(p); remaining //= p
                if remaining == 1: break
            expected = [n, *factors]
        elif name == 'divisors':
            a, b = sorted([rng.randrange(1, 101), rng.randrange(1, 101)])
            c = rng.randrange(1, 10001); data = f'{a} {b} {c}\n'
            expected = [sum(c % d == 0 for d in range(a, b+1))]
        else: raise AssertionError(name)
        actual = list(map(int, run([str(binary)], input=data).replace(':', '').split()))
        assert actual == expected, (name, data, actual, expected)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    args = parser.parse_args()
    compiler = shlex.split(args.compiler)
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-I', str(ROOT)]
    runtime = ['-O2', '-g'] if not args.sanitize else ['-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-sanitize-recover=all', '-fno-pie', '-no-pie']
    headers = sorted(str(p.relative_to(ROOT)) for p in (ROOT/'number_theory_basic').rglob('*.hpp'))
    for header in headers:
        for extra in [[], ['-DNDEBUG']]:
            run(compiler+flags+extra+['-fsyntax-only', '-x', 'c++', '-'], input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    print(f'standalone headers (debug/release): {len(headers)} passed', flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-number-theory-') as directory:
        temp = Path(directory)
        def compile(source, suffix='', extra=()):
            binary = temp/(source.stem+suffix)
            run(compiler+flags+list(extra)+runtime+[str(source), '-o', str(binary)])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))+'#include "dp_optimization/all.hpp"\n#include "tree/all.hpp"\n#include "graph_combinatorics/all.hpp"\n#include "range_query/all.hpp"\n'
        (temp/'first.cpp').write_text(includes+'int first(){return static_cast<int>(cp::gcd(0,0));}\n')
        (temp/'second.cpp').write_text(includes+'int first();int main(){return first();}\n')
        run(compiler+flags+runtime+[str(temp/'first.cpp'), str(temp/'second.cpp'), '-o', str(temp/'multi')]); run([str(temp/'multi')])
        print('reverse includes / existing categories / multiple translation units: passed', flush=True)
        for name, expected in EXAMPLES.items():
            assert run([str(compile(ROOT/f'examples/number_theory_basic/{name}.cpp'))]) == expected, name
        for source in sorted((ROOT/'verify/number_theory_basic').glob('*.test.cpp')):
            protocol(source.name.removesuffix('.test.cpp'), compile(source))
        print('8 examples; 9 judge wrappers / 432 independent protocol cases passed', flush=True)
        for suffix, extra in [('_debug', []), ('_release', ['-DNDEBUG'])]:
            count = bigint_cases(compile(ROOT/'verify/number_theory_basic/arithmetic_driver.cpp', suffix, extra))
            print(f'{suffix[1:]}: {count} Python arbitrary-precision cases passed', flush=True)
            print(run([str(compile(ROOT/'verify/number_theory_basic/unit_test.cpp', suffix, extra))]), end='', flush=True)
if __name__ == '__main__': main()
