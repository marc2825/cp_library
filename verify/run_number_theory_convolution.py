#!/usr/bin/env python3
"""3.14: divisor/multiple transforms and arithmetic convolutions."""
import argparse
import math
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = {
    'divisor_zeta_transform': '1 3 4 7 6 12 \n99\n',
    'divisor_mobius_transform': '1 1 2 2 4 2 \n',
    'multiple_zeta_transform': '21 12 9 4 5 6 \n',
    'multiple_mobius_transform': '1 2 3 4 5 6 \n',
    'dirichlet_convolution': '1 2 2 3 2 4 \n',
    'gcd_convolution': '284 90 39 12 10 6 \n',
    'lcm_convolution': '6 27 34 65 42 125 \n',
}


def run(command, **kwargs):
    result = subprocess.run(command, text=True, capture_output=True, **kwargs)
    if result.returncode:
        raise RuntimeError(f"{shlex.join(map(str, command))}\n{result.stdout}{result.stderr}")
    return result.stdout


def protocol(name, binary):
    rng = random.Random(314016)
    for trial in range(96):
        if name == 'dirichlet_convolution':
            n = rng.randrange(2, 251)
            data = f'{n}\n'
            expected = [sum(a*b < n for a in range(1, n) for b in range(1, n))]
        else:
            n, mod = rng.randrange(1, 51), 998244353
            def value(): return rng.choice([0, 1, mod-1, rng.randrange(mod)])
            a, b = [value() for _ in range(n)], [value() for _ in range(n)]
            data = f'{n}\n'+' '.join(map(str, a))+'\n'+' '.join(map(str, b))+'\n'
            expected = [0]*(n+1)
            operation = math.gcd if name == 'gcd_convolution' else math.lcm
            for i, x in enumerate(a, 1):
                for j, y in enumerate(b, 1):
                    k = operation(i, j)
                    if k <= n: expected[k] = (expected[k]+x*y) % mod
            expected = expected[1:]
        actual = list(map(int, run([str(binary)], input=data).split()))
        assert actual == expected, (name, data, actual, expected)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    args = parser.parse_args()
    compiler = shlex.split(args.compiler)
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-I', str(ROOT)]
    runtime = ['-O2', '-g'] if not args.sanitize else ['-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-sanitize-recover=all', '-fno-pie', '-no-pie']
    headers = sorted(str(p.relative_to(ROOT)) for p in (ROOT/'number_theory_convolution').rglob('*.hpp'))
    for header in headers:
        for extra in [[], ['-DNDEBUG']]:
            run(compiler+flags+extra+['-fsyntax-only', '-x', 'c++', '-'], input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    print(f'standalone headers (debug/release): {len(headers)} passed', flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-arithmetic-convolution-') as directory:
        temp = Path(directory)
        def compile(source, suffix='', extra=()):
            binary = temp/(source.stem+suffix)
            run(compiler+flags+list(extra)+runtime+[str(source), '-o', str(binary)])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))+'#include "number_theory_basic/all.hpp"\n#include "dp_optimization/all.hpp"\n#include "tree/all.hpp"\n#include "graph_combinatorics/all.hpp"\n#include "range_query/all.hpp"\n'
        (temp/'first.cpp').write_text(includes+'int first(){std::vector<int> f;cp::divisor_zeta_transform(f);return static_cast<int>(f.size());}\n')
        (temp/'second.cpp').write_text(includes+'int first();int main(){return first();}\n')
        run(compiler+flags+runtime+[str(temp/'first.cpp'), str(temp/'second.cpp'), '-o', str(temp/'multi')]); run([str(temp/'multi')])
        print('reverse includes / existing categories / multiple translation units: passed', flush=True)
        for name, expected in EXAMPLES.items():
            actual = run([str(compile(ROOT/f'examples/number_theory_convolution/{name}.cpp'))])
            assert actual == expected, (name, actual, expected)
        for source in sorted((ROOT/'verify/number_theory_convolution').glob('*.test.cpp')):
            protocol(source.name.removesuffix('.test.cpp'), compile(source))
        print('7 examples; 3 judge wrappers / 288 independent protocol cases passed', flush=True)
        for suffix, extra in [('_debug', []), ('_release', ['-DNDEBUG'])]:
            print(suffix[1:]+':\n'+run([str(compile(ROOT/'verify/number_theory_convolution/unit_test.cpp', suffix, extra))]), end='', flush=True)
if __name__ == '__main__': main()
