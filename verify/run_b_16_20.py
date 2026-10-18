#!/usr/bin/env python3
"""3.16--3.20 B: headers, documented examples, independent comparisons, judge I/O."""
import argparse
import json
import os
from pathlib import Path
import random
import re
import shlex
import subprocess
import tempfile
from urllib.parse import unquote
ROOT = Path(__file__).resolve().parents[1]
MOD = 998244353

def run(command, **kwargs):
    result = subprocess.run(list(map(str, command)), capture_output=True, text=True, timeout=300, **kwargs)
    if result.returncode:
        raise RuntimeError(f'{shlex.join(map(str, command))}\n{result.stdout}{result.stderr}')
    return result.stdout

def multiply(a, b, n=None):
    c = [0] * (n if n is not None else len(a) + len(b) - 1)
    for i, x in enumerate(a):
        for j, y in enumerate(b):
            if i + j < len(c):
                c[i + j] = (c[i + j] + x * y) % MOD
    return c

def judge(binary, name):
    rng = random.Random(316202026)
    for trial in range(24):
        if name == 'convolution':
            n, m = rng.randrange(1, 180), rng.randrange(1, 180)
            a = [rng.randrange(MOD) for _ in range(n)]
            b = [rng.randrange(MOD) for _ in range(m)]
            expected = multiply(a, b)
            data = f'{n} {m}\n' + ' '.join(map(str, a)) + '\n' + ' '.join(map(str, b)) + '\n'
        elif name == 'composition':
            n = rng.randrange(1, 65)
            a = [rng.randrange(MOD) for _ in range(n)]
            b = [0] + [rng.randrange(MOD) for _ in range(n - 1)]
            expected, power = [0] * n, [1] + [0] * (n - 1)
            for x in a:
                expected = [(u + x * v) % MOD for u, v in zip(expected, power)]
                power = multiply(power, b, n)
            data = str(n) + '\n' + ' '.join(map(str, a)) + '\n' + ' '.join(map(str, b)) + '\n'
        elif name == 'sps_exp':
            bits = rng.randrange(7)
            n = 1 << bits
            a = [0] + [rng.randrange(MOD) for _ in range(n - 1)]
            expected = [1] + [0] * (n - 1)
            for mask in range(1, n):
                bit = mask & -mask
                sub = mask
                while sub:
                    if sub & bit:
                        expected[mask] = (expected[mask] + a[sub] * expected[mask ^ sub]) % MOD
                    sub = (sub - 1) & mask
            data = str(bits) + '\n' + ' '.join(map(str, a)) + '\n'
        else:
            n = rng.randrange(1, 180)
            a = [0] + [rng.randrange(MOD) for _ in range(n - 1)]
            expected = [1] + [0] * (n - 1)
            for i in range(1, n):
                expected[i] = sum(j * a[j] * expected[i - j] for j in range(1, i + 1)) * pow(i, MOD - 2, MOD) % MOD
            data = str(n) + '\n' + ' '.join(map(str, a)) + '\n'
        assert list(map(int, run([binary], input=data).split())) == expected, (name, trial)

def check_links(entries):
    paths = [ROOT / 'README.md', ROOT / 'HANDOFF.md', ROOT / 'docs/README.md', ROOT / 'docs/implementation_plan.md', ROOT / 'visualizations/README.md', ROOT / 'visualizations/b_16_20/README.md']
    paths += list((ROOT / 'docs/b_16_20').glob('*.md'))
    paths += [ROOT / 'docs' / entry['doc'] for entry in entries]
    paths += [ROOT / f'docs/{cat}/README.md' for cat in ['convolution', 'fps', 'linear_recurrence', 'set_function', 'linear_algebra']]
    checked = 0
    for path in paths:
        content = re.sub(r'```.*?```', '', path.read_text(), flags=re.S)
        content = re.sub(r'`[^`]*`', '', content)
        for link in re.findall(r'\]\(([^)]+)\)', content):
            if '://' in link or link.startswith(('#', 'mailto:')):
                continue
            assert (path.parent / unquote(link.split('#', 1)[0])).exists(), (path, link)
            checked += 1
    print(f'{checked} local document links passed', flush=True)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    parser.add_argument('--links-only', action='store_true')
    args = parser.parse_args()
    entries = json.loads((ROOT / 'verify/b_16_20/manifest.json').read_text())['algorithms']
    if args.links_only:
        check_links(entries)
        return
    compiler = shlex.split(args.compiler)
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-I', ROOT]
    runtime = ['-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-sanitize-recover=all', '-fno-pie', '-no-pie'] if args.sanitize else ['-O2']
    headers = [entry['header'] for entry in entries]
    for header in headers:
        for release in [[], ['-DNDEBUG']]:
            run(compiler + flags + release + ['-fsyntax-only', '-x', 'c++', '-'], input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    print(f'{len(headers)} standalone/repeated headers debug/release passed', flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-b-16-20-') as folder:
        temporary = Path(folder)
        def compile(source, name, extra=()):
            binary = temporary / name
            run(compiler + flags + runtime + list(extra) + [source, '-o', binary])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes += ''.join(f'#include "{h.relative_to(ROOT)}"\n' for h in sorted(ROOT.glob('*/all.hpp')))
        (temporary / 'a.cpp').write_text(includes + 'int first(){return cp::sps_exp(std::vector<int>{0})[0]-1;}\n')
        (temporary / 'b.cpp').write_text(includes + 'int first();int main(){return first();}\n')
        run(compiler + flags + runtime + [temporary / 'a.cpp', temporary / 'b.cpp', '-o', temporary / 'multi'])
        run([temporary / 'multi'])
        for entry in entries:
            source = ROOT / 'examples' / entry['example']
            doc = (ROOT / 'docs' / entry['doc']).read_text()
            assert re.search(r'```cpp\n(.*?)```', doc, re.S).group(1) == source.read_text(), entry['doc']
            assert run([compile(source, Path(entry['header']).stem + '_example')]) == entry['output'], entry['header']
        print('mixed-category translation units and 31 documented examples passed', flush=True)
        observed = {}
        for line in run([compile(ROOT / 'verify/b_16_20/visualization_bridge.cpp', 'bridge')]).splitlines():
            mode, value = line.split(' ', 1)
            observed[mode] = json.loads(value)
        assert observed == json.loads((ROOT / 'verify/b_16_20/visualization_expected.json').read_text())
        print('31 C++/visualization reference cases passed', flush=True)
        for release in [[], ['-DNDEBUG']]:
            for name in ['convolution', 'fps_recurrence', 'set_algebra']:
                print(run([compile(ROOT / f'verify/b_16_20/{name}_test.cpp', name, release)]).strip(), flush=True)
            for name in ['convolution', 'composition', 'sps_exp', 'online_exp']:
                judge(compile(ROOT / f'verify/b_16_20/{name}.test.cpp', name, release), name)
            print('4 judge formats x 24 independent Python inputs passed ' + ('release' if release else 'debug'), flush=True)
    check_links(entries)

if __name__ == '__main__':
    main()
