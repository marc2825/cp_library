#!/usr/bin/env python3
"""3.11: rooted forests, LCA/Euler/diameter, successors, Tree DP and rerooting."""
import argparse
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[1]

def run(command, **kwargs):
    result = subprocess.run(command, text=True, capture_output=True, **kwargs)
    if result.returncode:
        raise RuntimeError(f"{shlex.join(map(str, command))}\n{result.stdout}{result.stderr}")
    return result.stdout

def verify_protocol(name, binary):
    rng = random.Random(311016)
    for trial in range(48):
        n = rng.randrange(2, 11)
        parents = [-1] + [rng.randrange(v) for v in range(1, n)]
        edges = [(parents[v], v, rng.randrange(1, 12)) for v in range(1, n)]
        adjacency = [[] for _ in range(n)]
        for u, v, w in edges:
            adjacency[u].append((v, w)); adjacency[v].append((u, w))
        def path(s, t):
            queue = [(s, [s], 0)]
            for v, vs, cost in queue:
                if v == t: return vs, cost
                for to, w in adjacency[v]:
                    if to not in vs: queue.append((to, vs + [to], cost + w))
            raise AssertionError('disconnected tree')
        queries = [(rng.randrange(n), rng.randrange(n), rng.randrange(n + 1)) for _ in range(40)]
        want = []
        if name == 'tree_diameter':
            data = f'{n}\n' + ''.join(f'{u} {v} {w}\n' for u, v, w in edges)
        elif name in ('independent_set', 'subtree'):
            mod = rng.choice([2, 6, 100, 1000000000])
            data = f'{n}' + (f' {mod}' if name == 'subtree' else '') + '\n' + ''.join(f'{u+1} {v+1}\n' for u, v, _ in edges)
            if name == 'independent_set':
                want = [sum(all(not (mask >> u & 1 and mask >> v & 1) for u, v, _ in edges) for mask in range(1 << n))]
            else:
                want = [0] * n
                for mask in range(1, 1 << n):
                    selected = [v for v in range(n) if mask >> v & 1]
                    if all(all(mask >> v & 1 for v in path(selected[0], t)[0]) for t in selected):
                        for v in selected: want[v] = (want[v] + 1) % mod
        elif name == 'planets_queries_i':
            nexts = [rng.randrange(n) for _ in range(n)]
            data = f'{n} {len(queries)}\n' + ' '.join(str(v + 1) for v in nexts) + '\n'
            for v, _, _ in queries:
                k = rng.randrange(1000000001); data += f'{v+1} {k}\n'
                visited, trail = {}, []
                while v not in visited:
                    visited[v] = len(trail); trail.append(v); v = nexts[v]
                start = visited[v]
                want.append(trail[k if k < len(trail) else start + (k - start) % (len(trail) - start)] + 1)
        elif name == 'jump_on_tree':
            data = f'{n} {len(queries)}\n' + ''.join(f'{u} {v}\n' for u, v, _ in edges)
            for u, v, k in queries:
                data += f'{u} {v} {k}\n'; vs, _ = path(u, v); want.append(vs[k] if k < len(vs) else -1)
        else:
            data = f'{n} {len(queries)}\n'
            values = [rng.randrange(100) for _ in range(n)]
            if name == 'vertex_add_subtree_sum': data += ' '.join(map(str, values)) + '\n'
            data += ' '.join(map(str, parents[1:])) + '\n'
            for u, v, k in queries:
                if name == 'lca':
                    data += f'{u} {v}\n'; ancestors = path(0, u)[0]; want.append(next(x for x in reversed(path(0, v)[0]) if x in ancestors))
                elif k % 2:
                    data += f'0 {v} {u}\n'; values[v] += u
                else:
                    data += f'1 {v}\n'; want.append(sum(values[x] for x in range(n) if v in path(0, x)[0]))
        actual = list(map(int, run([str(binary)], input=data).split()))
        if name == 'tree_diameter':
            length, count, *vertices = actual
            assert count == len(vertices) and len(set(vertices)) == count
            assert all(0 <= v < n for v in vertices)
            assert vertices == path(vertices[0], vertices[-1])[0]
            assert length == path(vertices[0], vertices[-1])[1] == max(path(u, v)[1] for u in range(n) for v in range(n))
        else: assert actual == want, (name, data, actual, want)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    args = parser.parse_args()
    compiler = shlex.split(args.compiler)
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-I', str(ROOT)]
    runtime = ['-O2', '-g'] if not args.sanitize else ['-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-sanitize-recover=all', '-fno-pie', '-no-pie']
    headers = sorted(str(p.relative_to(ROOT)) for p in (ROOT / 'tree').glob('*.hpp'))
    for h in headers:
        for extra in [[], ['-DNDEBUG']]:
            run(compiler + flags + extra + ['-fsyntax-only', '-x', 'c++', '-'], input=f'#include "{h}"\n#include "{h}"\nint main(){{}}\n')
    print(f'standalone headers (debug/release): {len(headers)} passed', flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-tree-') as directory:
        temp = Path(directory)
        def compile(source, suffix='', extra=()):
            binary = temp / (source.stem + suffix)
            run(compiler + flags + list(extra) + runtime + [str(source), '-o', str(binary)])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers)) + '#include "graph/all.hpp"\n#include "graph_combinatorics/all.hpp"\n#include "graph_flow/all.hpp"\n#include "range_query/all.hpp"\n#include "graph_theory/Tree/tree_template.hpp"\n'
        (temp / 'first.cpp').write_text(includes + 'int first(){return cp::BinaryLifting({0}).jump(0,0);}\n')
        (temp / 'second.cpp').write_text(includes + 'int first();int main(){return first();}\n')
        run(compiler + flags + runtime + [str(temp / 'first.cpp'), str(temp / 'second.cpp'), '-o', str(temp / 'multi')]); run([str(temp / 'multi')])
        print('reverse includes / existing categories / legacy API / multiple translation units: passed', flush=True)
        expected = {'rooted_tree': '2 3 1 4\n2 3 1 0 \n', 'doubling_lca': '1 8 1\n', 'euler_tour': '9\n0 1 2 1 3 1 0 \n', 'diameter': '8 3\n', 'binary_lifting': '1 -1\n', 'tree_dp': '9\n', 'rerooting': '5 3 5 5 \n'}
        for name, want in expected.items(): assert run([str(compile(ROOT / f'examples/tree/{name}.cpp'))]) == want, name
        for source in sorted((ROOT / 'verify/tree').glob('*.test.cpp')): verify_protocol(source.name.removesuffix('.test.cpp'), compile(source))
        print('7 examples; 7 judge wrappers / 336 independent protocol cases passed', flush=True)
        for suffix, extra in [('_debug', []), ('_release', ['-DNDEBUG'])]:
            print(suffix[1:] + ':\n' + run([str(compile(ROOT / 'verify/tree/unit_test.cpp', suffix, extra))]), end='', flush=True)
if __name__ == '__main__': main()
