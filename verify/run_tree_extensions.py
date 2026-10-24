#!/usr/bin/env python3
"""T-01..03/T-20..23: independent oracles, examples, and header integration."""
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


def run(command, **kwargs):
    result = subprocess.run(list(map(str, command)), capture_output=True, text=True, timeout=300, **kwargs)
    if result.returncode:
        raise RuntimeError(f'{shlex.join(map(str, command))}\n{result.stdout}{result.stderr}')
    return result.stdout


def links(entries):
    names = ['README.md', 'HANDOFF.md', 'docs/README.md', 'docs/remaining_work.md',
             'docs/implementation_plan.md', 'docs/tree/README.md', 'docs/tree/references.md',
             'docs/tree_extensions/README.md', 'docs/tree_extensions/references.md',
             'visualizations/README.md', 'visualizations/tree_extensions/README.md']
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


def judge_checks(compile):
    for name in ['dynamic_tree_vertex_add_path_sum', 'dynamic_tree_subtree_add_subtree_sum']:
        binary = compile(ROOT / f'verify/tree_extensions/{name}.test.cpp', 'judge')
        for seed in range(12):
            rng = random.Random(seed)
            n, q = 35, 450
            values = [rng.randrange(100) for _ in range(n)]
            edges = {(i, rng.randrange(i)) for i in range(1, n)}
            initial = f'{n} {q}\n' + ' '.join(map(str, values)) + '\n'
            initial += ''.join(f'{u} {v}\n' for u, v in sorted(edges))
            operations, expected = [], []

            def parents(root):
                adj = [[] for _ in range(n)]
                for u, v in edges:
                    adj[u].append(v)
                    adj[v].append(u)
                parent, queue = {root: -1}, [root]
                for u in queue:
                    for v in adj[u]:
                        if v not in parent:
                            parent[v] = u
                            queue.append(v)
                return parent

            def subtree(v, p):
                parent = parents(p)
                answer = []
                for u in parent:
                    x = u
                    while x not in (-1, v):
                        x = parent[x]
                    if x == v:
                        answer.append(u)
                return answer

            for _ in range(q):
                mode = rng.randrange(3)
                if mode == 0:
                    u, v = rng.choice(sorted(edges))
                    edges.remove((u, v))
                    left = parents(u)
                    w = rng.choice(sorted(left))
                    x = rng.choice(sorted(set(range(n)) - left.keys()))
                    edges.add((w, x))
                    operations.append(f'0 {u} {v} {w} {x}')
                elif name == 'dynamic_tree_vertex_add_path_sum':
                    u, v = rng.randrange(n), rng.randrange(n)
                    if mode == 1:
                        x = rng.randrange(100000)
                        values[u] += x
                        operations.append(f'1 {u} {x}')
                    else:
                        parent, total, x = parents(u), 0, v
                        while x != -1:
                            total += values[x]
                            x = parent[x]
                        operations.append(f'2 {u} {v}')
                        expected.append(total)
                else:
                    v, p = rng.choice(sorted(edges))
                    if rng.randrange(2):
                        v, p = p, v
                    target = subtree(v, p)
                    if mode == 1:
                        x = rng.randrange(100000)
                        for u in target:
                            values[u] += x
                        operations.append(f'1 {v} {p} {x}')
                    else:
                        operations.append(f'2 {v} {p}')
                        expected.append(sum(values[u] for u in target))
            data = initial + '\n'.join(operations) + '\n'
            assert list(map(int, run([binary], input=data).split())) == expected, (name, seed)
    print('2 judge protocols x 12 independent Python inputs passed', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    parser.add_argument('--links-only', action='store_true')
    args = parser.parse_args()
    entries = json.loads((ROOT / 'verify/tree_extensions/manifest.json').read_text())['algorithms']
    if args.links_only:
        links(entries)
        return
    flags = shlex.split(args.compiler) + ['-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-I', ROOT]
    runtime = ['-O1', '-g', '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer', '-fno-pie', '-no-pie'] if args.sanitize else ['-O2']
    headers = [e['header'] for e in entries] + ['tree/detail/top_tree_core.hpp']
    for header in headers:
        for extra in [[], ['-DNDEBUG']]:
            run(flags + extra + ['-x', 'c++', '-fsyntax-only', '-'], input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    print(f'{len(headers)} standalone/repeated headers in debug/release passed', flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-tree-extensions-') as folder:
        temp = Path(folder)
        def compile(source, name, extra=()):
            binary = temp / name
            run(flags + runtime + list(extra) + [source, '-o', binary])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes += ''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in sorted(ROOT.glob('*/all.hpp')))
        (temp / 'a.cpp').write_text(includes + 'int first(){return cp::prufer_decode(2,{}).edge_count();}\n')
        (temp / 'b.cpp').write_text(includes + 'int first();int main(){return first()!=1;}\n')
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
        assert observed == json.loads((ROOT / 'verify/tree_extensions/visualization_expected.json').read_text())
        print('all-category translation units and 7 documented/visualization examples passed', flush=True)
        for extra in [[], ['-DNDEBUG']]:
            for name in ['static_test', 'dynamic_test']:
                print(run([compile(ROOT / f'verify/tree_extensions/{name}.cpp', name, extra)]).strip(), flush=True)
            print(('release' if extra else 'debug') + ' passed', flush=True)
        judge_checks(compile)
    links(entries)


if __name__ == '__main__':
    main()
