#!/usr/bin/env python3
"""3.21--3.25 B: headers, documented examples, independent comparisons, judge I/O."""
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

def judge(binary, name):
    import math
    rng = random.Random(321252026)
    for trial in range(24):
        if name == 'runs':
            s = ''.join(rng.choice('abc') for _ in range(rng.randrange(1, 65)))
            expected = []
            for period in range(1, len(s) // 2 + 1):
                for left in range(len(s) - 2 * period + 1):
                    right = left + period
                    while right < len(s) and s[right] == s[right - period]:
                        right += 1
                    if right - left < 2 * period or left and s[left - 1] == s[left + period - 1]:
                        continue
                    if any(all(s[i] == s[i - q] for i in range(left + q, right)) for q in range(1, period)):
                        continue
                    expected.extend([period, left, right])
            expected = [len(expected) // 3] + expected
            assert list(map(int, run([binary], input=s + '\n').split())) == expected
        elif name == 'wildcard':
            n = rng.randrange(1, 180)
            m = rng.randrange(1, n + 1)
            s = ''.join(rng.choice('abc*') for _ in range(n))
            t = ''.join(rng.choice('abc*') for _ in range(m))
            expected = ''.join('1' if all(x == y or x == '*' or y == '*' for x, y in zip(s[i:i+m], t)) else '0' for i in range(n-m+1))
            assert run([binary], input=f'{s} {t}\n').strip() == expected
        elif name == 'circle_line':
            x, y, radius = rng.randrange(-20, 20), rng.randrange(-20, 20), rng.randrange(1, 15)
            lines, expected = [], []
            for i in range(10):
                dx, dy = rng.randrange(1, 10), rng.randrange(-10, 10)
                lines.append((x-dx, y-dy, x+dx, y+dy))
                scale = radius / math.hypot(dx, dy)
                points = sorted([(x-dx*scale, y-dy*scale), (x+dx*scale, y+dy*scale)])
                expected.extend(v for point in points for v in point)
            data = f'{x} {y} {radius}\n{len(lines)}\n' + ''.join(' '.join(map(str, line))+'\n' for line in lines)
            observed = list(map(float, run([binary], input=data).split()))
            assert len(observed) == len(expected) and all(math.isclose(a, b, abs_tol=1e-7, rel_tol=1e-7) for a, b in zip(observed, expected))
        else:
            radius = rng.randrange(1, 15)
            other = rng.randrange(1, 15)
            distance = rng.uniform(abs(radius-other)+0.1, radius+other-0.1)
            along = (radius*radius-other*other+distance*distance)/(2*distance)
            height = math.sqrt(max(0, radius*radius-along*along))
            expected = [along, -height, along, height]
            observed = list(map(float, run([binary], input=f'0 0 {radius} {distance:.18g} 0 {other}\n').split()))
            assert len(observed) == 4 and all(math.isclose(a, b, abs_tol=1e-7, rel_tol=1e-7) for a, b in zip(observed, expected))

def check_links(entries):
    paths = [ROOT / 'README.md', ROOT / 'HANDOFF.md', ROOT / 'docs/README.md', ROOT / 'docs/implementation_plan.md', ROOT / 'visualizations/README.md', ROOT / 'visualizations/b_21_25/README.md']
    paths += list((ROOT / 'docs/b_21_25').glob('*.md'))
    paths += [ROOT / 'docs' / entry['doc'] for entry in entries]
    paths += [ROOT / f'docs/{cat}/README.md' for cat in ['string_algorithm', 'computational_geometry', 'combinatorial_optimization', 'game_theory', 'heuristic_search']]
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
    entries = json.loads((ROOT / 'verify/b_21_25/manifest.json').read_text())['algorithms']
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
    with tempfile.TemporaryDirectory(prefix='cp-b-21-25-') as folder:
        temporary = Path(folder)
        def compile(source, name, extra=()):
            binary = temporary / name
            run(compiler + flags + runtime + list(extra) + [source, '-o', binary])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes += ''.join(f'#include "{h.relative_to(ROOT)}"\n' for h in sorted(ROOT.glob('*/all.hpp')))
        (temporary / 'a.cpp').write_text(includes + 'int first(){return cp::minimum_rotation("a");}\n')
        (temporary / 'b.cpp').write_text(includes + 'int first();int main(){return first();}\n')
        run(compiler + flags + runtime + [temporary / 'a.cpp', temporary / 'b.cpp', '-o', temporary / 'multi'])
        run([temporary / 'multi'])
        for entry in entries:
            source = ROOT / 'examples' / entry['example']
            doc = (ROOT / 'docs' / entry['doc']).read_text()
            assert re.search(r'```cpp\n(.*?)```', doc, re.S).group(1) == source.read_text(), entry['doc']
            assert run([compile(source, Path(entry['header']).stem + '_example')]) == entry['output'], entry['header']
        print('mixed-category translation units and 28 documented examples passed', flush=True)
        observed = {}
        for line in run([compile(ROOT / 'verify/b_21_25/visualization_bridge.cpp', 'bridge')]).splitlines():
            if line.strip():
                mode, value = line.split(' ', 1)
                observed[mode] = json.loads(value)
        expected = json.loads((ROOT / 'verify/b_21_25/visualization_expected.json').read_text())
        def close(a, b):
            if isinstance(a, list):
                return isinstance(b, list) and len(a) == len(b) and all(close(x, y) for x, y in zip(a, b))
            if isinstance(a, dict):
                return isinstance(b, dict) and a.keys() == b.keys() and all(close(a[k], b[k]) for k in a)
            if isinstance(a, (int, float)) and not isinstance(a, bool):
                return abs(a-b) <= 1e-10 * max(1, abs(a), abs(b))
            return a == b
        assert close(observed, expected), (observed, expected)
        print('28 C++/visualization reference cases passed', flush=True)
        for release in [[], ['-DNDEBUG']]:
            for name in ['string', 'geometry', 'matroid_game', 'heuristic']:
                print(run([compile(ROOT / f'verify/b_21_25/{name}_test.cpp', name, release)]).strip(), flush=True)
            for name in ['runs', 'wildcard', 'circle_line', 'circle_circle']:
                judge(compile(ROOT / f'verify/b_21_25/{name}.test.cpp', name, release), name)
            print('4 judge formats x 24 independent Python inputs passed ' + ('release' if release else 'debug'), flush=True)
    check_links(entries)

if __name__ == '__main__':
    main()
