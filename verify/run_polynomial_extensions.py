#!/usr/bin/env python3
"""Q: 31 polynomial/transform/sequence/set APIs: independent oracles, examples, and header integration."""
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
    names = ['README.md', 'HANDOFF.md', 'docs/README.md', 'docs/remaining_work.md',
             'docs/implementation_plan.md', 'docs/convolution/README.md', 'docs/fps/README.md', 'docs/linear_recurrence/README.md', 'docs/set_function/README.md',
             'docs/polynomial_extensions/README.md', 'docs/polynomial_extensions/references.md',
             'visualizations/README.md', 'visualizations/polynomial_extensions/README.md']
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    parser.add_argument('--links-only', action='store_true')
    args = parser.parse_args()
    entries = json.loads((ROOT / 'verify/polynomial_extensions/manifest.json').read_text())['algorithms']
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
    with tempfile.TemporaryDirectory(prefix='cp-polynomial-extensions-') as folder:
        temp = Path(folder)
        def compile(source, name, extra=()):
            binary = temp / name
            run(flags + runtime + list(extra) + [source, '-o', binary])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes += ''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in sorted(ROOT.glob('*/all.hpp')))
        (temp / 'a.cpp').write_text(includes + 'int first(){return cp::polynomial_resultant({2,-3,1},{-3,1});}\n')
        (temp / 'b.cpp').write_text(includes + 'int first();int main(){return first()!=2;}\n')
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
        assert observed == json.loads((ROOT / 'verify/polynomial_extensions/visualization_expected.json').read_text())
        print('all-category translation units and 31 documented/visualization examples passed', flush=True)
        for extra in [[], ['-DNDEBUG']]:
            for name in ['algebra_test', 'transform_test', 'fps_test', 'sequence_set_test']:
                print(run([compile(ROOT / f'verify/polynomial_extensions/{name}.cpp', name, extra)]).strip(), flush=True)
            rng = random.Random(20261026)
            wrap = compile(ROOT / 'verify/polynomial_extensions/convolution_u64.test.cpp', 'wrap-judge', extra)
            composition = compile(ROOT / 'verify/polynomial_extensions/composition.test.cpp', 'compose-judge', extra)
            for trial in range(24):
                n,m = rng.randrange(1,70),rng.randrange(1,70)
                a,b = [rng.getrandbits(64) for _ in range(n)],[rng.getrandbits(64) for _ in range(m)]
                expected = [0]*(n+m-1)
                for i,x in enumerate(a):
                    for j,y in enumerate(b): expected[i+j] = (expected[i+j]+x*y) % (1<<64)
                data = f'{n} {m}\n'+' '.join(map(str,a))+'\n'+' '.join(map(str,b))+'\n'
                assert list(map(int,run([wrap],input=data).split())) == expected
                n = rng.randrange(1,65)
                a,b = [rng.randrange(998244353) for _ in range(n)],[0]+[rng.randrange(998244353) for _ in range(n-1)]
                expected = [0]*n
                for coefficient in reversed(a):
                    nxt = [0]*n
                    for i,x in enumerate(expected):
                        for j,y in enumerate(b[:n-i]): nxt[i+j] = (nxt[i+j]+x*y) % 998244353
                    nxt[0] = (nxt[0]+coefficient) % 998244353
                    expected = nxt
                data = str(n)+'\n'+' '.join(map(str,a))+'\n'+' '.join(map(str,b))+'\n'
                assert list(map(int,run([composition],input=data).split())) == expected
            print(('release' if extra else 'debug') + ' and 2 judge formats x 24 cases passed', flush=True)
    links(entries)


if __name__ == '__main__':
    main()
