#!/usr/bin/env python3
"""F-01..F-07: independent integer oracle, I/O, headers, examples and links."""
import argparse
import json
import math
import os
from pathlib import Path
import random
import re
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


def compare(binary):
    rng = random.Random(20261019)
    inputs, expected = [], []

    def add(values, result):
        inputs.append(' '.join(map(str, values)))
        expected.append(' '.join(map(str, result)))

    bounds = [-(1 << 127), -(1 << 63), -1, 0, 1, (1 << 64) - 1, (1 << 127) - 1]
    for kind, fixed in [('D', None), ('S', (1 << 32) - 1), ('M', (1 << 63) - 1), ('H', (1 << 61) - 1)]:
        for i in range(1000):
            m = fixed or ([1, 2, 12, 1 << 31, (1 << 32) - 1][i % 5] if i < 100 else rng.randrange(1, 1 << 32))
            a = bounds[i % len(bounds)] if i < 100 else rng.randrange(-(1 << 127), 1 << 127)
            b = bounds[(i // len(bounds)) % len(bounds)] if i < 100 else rng.randrange(-(1 << 127), 1 << 127)
            e = [0, 1, 2, (1 << 64) - 1][i % 4] if i < 100 else rng.randrange(1 << 64)
            inverse = pow(b, -1, m) if math.gcd(b, m) == 1 else 'none'
            add([kind] + ([m] if kind == 'D' else []) + [a, b, e],
                [a % m, b % m, (a+b) % m, (a-b) % m, a*b % m, pow(a,e,m), inverse])
    for i in range(2000):
        m = rng.randrange(1, 1 << 32)
        x, a, b = rng.randrange(1 << 64), rng.randrange(1 << 32), rng.randrange(1 << 32)
        add(['B',m,x,a,b], [x % m, a*b % m])
        m = 1 if i == 0 else rng.randrange(1, 1 << 63) | 1
        x = rng.randrange(m << 64)
        a, b = rng.randrange(1 << 64), rng.randrange(1 << 64)
        add(['R',m,x,a,b], [x * pow(1 << 64,-1,m) % m, (a << 64) % m, a*b % m])
    for bits in [64, 128]:
        for signed in [True, False]:
            low, high = (-(1 << (bits-1)), (1 << (bits-1))-1) if signed else (0,(1 << bits)-1)
            limits = [low, low+1, -1 if signed else 0, 0, 1, high-1, high]
            pairs = [(a,b) for a in limits for b in limits]
            pairs += [(rng.randrange(low,high+1),rng.randrange(low,high+1)) for _ in range(1000)]
            for a,b in pairs:
                exact = [a+b,a-b,a*b]
                checked = [v if low <= v <= high else 'none' for v in exact]
                saturated = [min(high,max(low,v)) for v in exact]
                if b == 0:
                    divided = ['invalid'] * 2
                elif signed and a == low and b == -1:
                    divided = ['overflow'] * 2
                else:
                    divided = [a//b, -((-a)//b)]
                add([('I' if signed else 'U')+str(bits),a,b], checked+saturated+divided)
    actual = run([binary], input='\n'.join(inputs)).splitlines()  # Deliberately no final newline.
    assert len(actual) == len(expected), (len(actual),len(expected))
    for data,want,got in zip(inputs,expected,actual):
        assert want == got, (data,want,got)
    print(f'{len(inputs)} Python arbitrary-precision comparisons passed', flush=True)


def links(entries):
    paths = [ROOT / p for p in ['README.md','docs/README.md','docs/basic_utility/README.md','docs/basic_utility/foundation.md','docs/basic_utility/references.md',
                                'visualizations/README.md','visualizations/foundation/README.md']]
    paths += [ROOT / 'docs' / e['doc'] for e in entries]
    checked = 0
    for path in paths:
        content = re.sub(r'```.*?```','',path.read_text(),flags=re.S)
        content = re.sub(r'`[^`]*`','',content)
        for target in re.findall(r'\]\(([^)]+)\)',content):
            if '://' in target or target.startswith(('#','mailto:')):
                continue
            assert (path.parent / unquote(target.split('#',1)[0])).exists(), (path,target)
            checked += 1
    print(f'{checked} document links passed',flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler',default=os.environ.get('CXX','g++'))
    parser.add_argument('--sanitize',action='store_true')
    parser.add_argument('--links-only',action='store_true')
    args = parser.parse_args()
    entries = json.loads((ROOT / 'verify/foundation/manifest.json').read_text())['algorithms']
    if args.links_only:
        links(entries)
        return
    flags = shlex.split(args.compiler)+['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',ROOT]
    runtime = ['-O1','-g','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-fno-pie','-no-pie'] if args.sanitize else ['-O2']
    headers = [e['header'] for e in entries]
    headers += [str(p.relative_to(ROOT)) for p in (ROOT / 'basic_utility/detail').glob('*.hpp')]
    for header in headers:
        for release in [[],['-DNDEBUG']]:
            run(flags+release+['-x','c++','-fsyntax-only','-'], input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    print(f'{len(headers)} standalone/repeated headers in debug/release passed',flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-foundation-') as folder:
        temp = Path(folder)

        def compile(source,name,extra=()):
            binary = temp / name
            run(flags+runtime+list(extra)+[source,'-o',binary])
            return binary

        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes += ''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in sorted(ROOT.glob('*/all.hpp')))
        (temp/'a.cpp').write_text(includes+'void first(){cp::DynamicModInt<71>::set_mod(11);}\n')
        (temp/'b.cpp').write_text(includes+'void first();int main(){first();return cp::DynamicModInt<71>(12).val()!=1;}\n')
        run(flags+runtime+[temp/'a.cpp',temp/'b.cpp','-o',temp/'multi'])
        run([temp/'multi'])
        print('all-category includes and shared dynamic modulus across translation units passed',flush=True)
        for e in entries:
            source = ROOT/'examples'/e['example']
            doc = (ROOT/'docs'/e['doc']).read_text()
            assert re.search(r'```cpp\n(.*?)```',doc,re.S).group(1) == source.read_text(), e['doc']
            assert run([compile(source,e['mode'])]) == e['output'], e['mode']
        print('7 documented examples passed',flush=True)
        observed = {}
        for line in run([compile(ROOT/'verify/foundation/visualization_bridge.cpp','bridge')]).splitlines():
            mode, value = line.split(' ',1)
            observed[mode] = json.loads(value)
        assert observed == json.loads((ROOT/'verify/foundation/visualization_expected.json').read_text())
        print('7 C++/visualization reference cases passed',flush=True)
        for extra in [[],['-DNDEBUG']]:
            print(run([compile(ROOT/'verify/foundation/unit_test.cpp','unit',extra)]).strip(),flush=True)
            compare(compile(ROOT/'verify/foundation/query_driver.cpp','driver',extra))
            rng = random.Random(19)
            for name in ['aplusb','mod_pow']:
                binary = compile(ROOT/f'verify/foundation/{name}.test.cpp',name,extra)
                for _ in range(32):
                    a,b = rng.randrange(10**9+1),rng.randrange(10**9+1)
                    expected = a+b if name == 'aplusb' else pow(a,b,1000000007)
                    assert int(run([binary],input=f'{a} {b}')) == expected
            print('2 judge formats x 32 local inputs passed '+('release' if extra else 'debug'),flush=True)
    links(entries)


if __name__ == '__main__':
    main()
