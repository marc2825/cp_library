#!/usr/bin/env python3
"""3.19 sub/supermask enumeration, SOS DP and Boolean-lattice transforms."""
import argparse
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = {
    'submask_enumeration': '10 8 2 0 \n10 8 2 \n',
    'supermask_enumeration': '2 3 6 7 10 11 14 15 \n',
    'sos_dp': '3 4 7 10 \n3 3 4 4 \n',
    'subset_zeta_transform': '1 3 4 10 \n',
    'subset_mobius_transform': '1 2 3 4 \n',
    'superset_zeta_transform': '10 6 7 4 \n',
    'superset_mobius_transform': '1 2 3 4 \n',
}


def run(command, **kwargs):
    result = subprocess.run(command, text=True, capture_output=True, **kwargs)
    if result.returncode:
        raise RuntimeError(f'{shlex.join(map(str, command))}\n{result.stdout}{result.stderr}')
    return result.stdout


def profile(a, subset, inverse, processed):
    n = len(a)
    return [sum((-1 if inverse and (s ^ t).bit_count() % 2 else 1)*x
                for t, x in enumerate(a)
                if ((s & t) == t if subset else (s & t) == s) and ((s ^ t) & ~processed) == 0)
            for s in range(n)]


def oracle(op, kind, a):
    n = len(a)
    if not n or n & (n-1): return 'INVALID'
    if kind == 'm' and op in 'LHO': return 'INVALID'
    if op in 'LHO':
        result = []
        for s in range(n):
            terms = [x for t, x in enumerate(a) if (s & t) == t]
            if op == 'L': result.append(min(terms))
            elif op == 'H': result.append(max(terms))
            else:
                value = 0
                for x in terms: value |= x
                result.append(value)
        return result
    subset, inverse = op in 'ZMS', op in 'MV'
    bits = [1 << i for i in range(n.bit_length()-1)]
    if inverse: bits.reverse()
    if kind == 'm': a = [x % 12 for x in a]
    bounds = {'s': (-2**63, 2**63-1), 'u': (0, 2**64-1), 'b': (-128, 127)}
    processed, result = 0, a
    for bit in bits:
        processed |= bit
        result = profile(a, subset, inverse, processed)
        if kind == 'm': result = [x % 12 for x in result]
        elif any(x < bounds[kind][0] or x > bounds[kind][1] for x in result): return 'OVERFLOW'
    return result


def enumerate_masks(mask, bits, sub, include):
    if not sub and (bits < 0 or bits > 64 or mask >> bits): return 'INVALID'
    full = (1 << bits)-1
    free = mask if sub else full ^ mask
    positions = [i for i in range(64) if free >> i & 1]
    result = []
    for code in range(1 << len(positions)):
        value = sum(1 << position for j, position in enumerate(positions) if code >> j & 1)
        result.append(value if sub else mask | value)
    result.sort(reverse=sub)
    return [x for x in result if x or include] if sub else result


def arbitrary_precision(binary):
    rng = random.Random(319019); commands, expected = [], []
    for kind in 'subm':
        lo, hi = {'s': (-2**63, 2**63-1), 'u': (0, 2**64-1), 'b': (-128, 127), 'm': (-2**63, 2**63-1)}[kind]
        for trial in range(64):
            n = rng.choice([0, 1, 2, 3, 4, 8, 16, 32, 64])
            a = [rng.choice([lo, hi, 0, rng.randrange(max(lo, -1000), min(hi, 1000)+1)]) for _ in range(n)]
            for op in 'ZMUVSLHO':
                commands.append(f'{op} {kind} {n}\n'+' '.join(map(str, a))+'\n')
                expected.append(oracle(op, kind, a))
    enumeration_cases = [(0, 0), (0, 1), (1, 1), (1 << 63, 64), ((1 << 63) | 1, 64)]
    enumeration_cases += [(rng.randrange(1 << bits), bits) for bits in range(1, 11) for _ in range(12)]
    for mask, bits in enumeration_cases:
        for include in [False, True]:
            commands.append(f'e {mask} {bits} {int(include)}\n')
            expected.append(enumerate_masks(mask, bits, True, include))
        # A sparse required mask in 64 bits would enumerate almost the whole universe.
        if bits <= 10:
            commands.append(f'g {mask} {bits} 1\n')
            expected.append(enumerate_masks(mask, bits, False, True))
    for mask, bits in [((1 << 64)-1, 64), ((1 << 64)-4, 64), ((1 << 63)-1, 63), (1, 0), (8, 3), (0, -1), (0, 65)]:
        commands.append(f'g {mask} {bits} 1\n')
        expected.append(enumerate_masks(mask, bits, False, True))
    actual = run([str(binary)], input=''.join(commands)).splitlines()
    assert len(actual) == len(expected), (len(actual), len(expected))
    for i, (line, want) in enumerate(zip(actual, expected)):
        if isinstance(want, str): assert line == want, (i, line, want)
        else:
            tokens = list(map(int, line.split()))
            assert tokens == [len(want)]+want, (i, tokens, want)
    return len(expected)


def protocol(name, binary):
    rng = random.Random(319019)
    for trial in range(64):
        bits = rng.randrange(7)
        if name in ('subset_transforms', 'superset_transforms'):
            n = 1 << bits; p = 998244353
            a = [rng.randrange(p) for _ in range(n)]; b = [rng.randrange(p) for _ in range(n)]
            data = f'{bits}\n'+' '.join(map(str, a))+'\n'+' '.join(map(str, b))+'\n'
            want = [0]*n
            for i, x in enumerate(a):
                for j, y in enumerate(b): want[i & j] = (want[i & j]+x*y) % p
        else:
            a = [rng.randrange(1, 1 << max(bits, 1)) for _ in range(rng.randrange(1, 90))]
            data = f'{len(a)}\n'+' '.join(map(str, a))+'\n'
            if name == 'sos_dp':
                want = [value for x in a for value in (sum((x | y) == x for y in a), sum((x & y) == x for y in a), sum((x & y) != 0 for y in a))]
            else: want = [max((y for y in a if (x & y) == 0), default=-1) for x in a]
        got = list(map(int, run([str(binary)], input=data).split()))
        assert got == want, (name, trial, got, want)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true'); args = parser.parse_args()
    compiler = shlex.split(args.compiler)
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-I', str(ROOT)]
    runtime = ['-O2', '-g'] if not args.sanitize else ['-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-sanitize-recover=all', '-fno-pie', '-no-pie']
    headers = sorted(str(p.relative_to(ROOT)) for p in (ROOT/'set_function').rglob('*.hpp'))
    for header in headers:
        for extra in [[], ['-DNDEBUG']]:
            run(compiler+flags+extra+['-fsyntax-only', '-x', 'c++', '-'], input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    print(f'{len(headers)} standalone headers (debug/release) passed', flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-set-function-') as directory:
        temp = Path(directory)
        def compile(source, suffix='', extra=()):
            binary = temp/(source.stem+suffix)
            run(compiler+flags+list(extra)+runtime+[str(source), '-o', str(binary)])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))+'#include "dp_optimization/all.hpp"\n#include "number_theory_convolution/all.hpp"\n#include "linear_recurrence/all.hpp"\n'
        (temp/'first.cpp').write_text(includes+'int first(){std::vector<int>f{1,2};cp::subset_zeta_transform(f);return f[1]-3;}\n')
        (temp/'second.cpp').write_text('#include "dp_optimization/subset_dp.hpp"\n'+includes+'int first();int main(){return first();}\n')
        run(compiler+flags+runtime+[str(temp/'first.cpp'), str(temp/'second.cpp'), '-o', str(temp/'multi')]); run([str(temp/'multi')])
        for name, want in EXAMPLES.items():
            got = run([str(compile(ROOT/f'examples/set_function/{name}.cpp'))]); assert got == want, (name, got, want)
        for source in sorted((ROOT/'verify/set_function').glob('*.test.cpp')):
            protocol(source.name.removesuffix('.test.cpp'), compile(source))
        print('mixed includes, old DP API, multiple TUs, 7 examples, 4 judge wrappers / 256 cases passed', flush=True)
        for suffix, extra in [('_debug', []), ('_release', ['-DNDEBUG'])]:
            count = arbitrary_precision(compile(ROOT/'verify/set_function/query_driver.cpp', suffix, extra))
            print(f'{suffix[1:]}: {count} Python arbitrary-precision protocol cases passed', flush=True)
            print(run([str(compile(ROOT/'verify/set_function/unit_test.cpp', suffix, extra))]), end='', flush=True)


if __name__ == '__main__': main()
