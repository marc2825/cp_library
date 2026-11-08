#!/usr/bin/env python3
"""3.12: sequence DP, knapsacks, interval/digit/subset DP, largest rectangles."""
import argparse
from collections import deque
from functools import lru_cache
import itertools
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


def lcs(a, b):
    best = 0
    for mask in range(1 << len(a)):
        sub = [x for i, x in enumerate(a) if mask >> i & 1]
        iterator = iter(b)
        if all(any(y == x for y in iterator) for x in sub): best = max(best, len(sub))
    return best


def edits(a, b):
    queue = deque([(a, 0)]); seen = {a}
    while queue:
        word, distance = queue.popleft()
        if word == b: return distance
        neighbors = [word[:i] + word[i + 1:] for i in range(len(word))]
        neighbors += [word[:i] + c + word[i + 1:] for i in range(len(word)) for c in 'ab']
        if len(word) < max(len(a), len(b)):
            neighbors += [word[:i] + c + word[i:] for i in range(len(word) + 1) for c in 'ab']
        for next_word in neighbors:
            if next_word not in seen:
                seen.add(next_word); queue.append((next_word, distance + 1))
    raise AssertionError('edit search failed')


def verify_protocol(name, binary):
    rng = random.Random(312016)
    for trial in range(48):
        expected = []
        if name == 'lis':
            a = [rng.randrange(10) for _ in range(rng.randrange(1, 11))]
            data = f'{len(a)}\n' + ' '.join(map(str, a)) + '\n'
            candidates = ([v for i, v in enumerate(a) if mask >> i & 1] for mask in range(1 << len(a)))
            expected = max(len(sub) for sub in candidates if all(sub[i] < sub[i + 1] for i in range(len(sub) - 1)))
        elif name in ('lcs', 'edit_distance'):
            length = 8 if name == 'lcs' else 3
            def word(): return ''.join(rng.choice('ab') for _ in range(rng.randrange(1, length + 1)))
            pairs = [(word(), word()) for _ in range(3 if name == 'lcs' else 1)]
            data = (f'{len(pairs)}\n' if name == 'lcs' else '') + ''.join(f'{a}\n{b}\n' for a, b in pairs)
            expected = [(lcs if name == 'lcs' else edits)(a, b) for a, b in pairs]
        elif name.endswith('knapsack') and name != 'bitset_knapsack':
            n, capacity = rng.randrange(1, 6), rng.randrange(1, 21)
            items = [(rng.randrange(1, 8), rng.randrange(1, 10), rng.randrange(1, 5)) for _ in range(n)]
            counts = [1 if name == 'zero_one_knapsack' else capacity // w if name == 'unbounded_knapsack' else c for w, _, c in items]
            data = f'{n} {capacity}\n' + ''.join(f'{v} {w}' + (f' {c}' if name == 'bounded_knapsack' else '') + '\n' for w, v, c in items)
            expected = [max(sum(k * v for k, (_, v, _) in zip(choice, items)) for choice in itertools.product(*(range(c + 1) for c in counts)) if sum(k * w for k, (w, _, _) in zip(choice, items)) <= capacity)]
        elif name == 'bitset_knapsack':
            a = [rng.randrange(1, 25) for _ in range(rng.randrange(1, 11))]
            data = f'{len(a)}\n' + ' '.join(map(str, a)) + '\n'
            sums = sorted({sum(v for i, v in enumerate(a) if mask >> i & 1) for mask in range(1, 1 << len(a))})
            expected = [len(sums), *sums]
        elif name == 'interval_dp':
            a = tuple(rng.randrange(1, 20) for _ in range(rng.randrange(1, 8)))
            data = f'{len(a)}\n' + ' '.join(map(str, a)) + '\n'
            @lru_cache(None)
            def solve(values):
                if len(values) <= 1: return 0
                return min(values[i] + values[i + 1] + solve(values[:i] + (values[i] + values[i + 1],) + values[i + 2:]) for i in range(len(values) - 1))
            expected = [solve(a)]
        elif name == 'digit_dp':
            upper, mod = rng.randrange(1, 5001), rng.randrange(1, 10)
            data = f'{upper}\n{mod}\n'
            expected = [sum(sum(map(int, str(x))) % mod == 0 for x in range(1, upper + 1))]
        elif name == 'subset_dp':
            n = rng.randrange(2, 8); edges = {(u, v): rng.randrange(15) for u in range(n) for v in range(n) if u != v and rng.randrange(4)}
            data = f'{n} {len(edges)}\n' + ''.join(f'{u} {v} {w}\n' for (u, v), w in edges.items())
            values = []
            for order in itertools.permutations(range(1, n)):
                vs = (0, *order, 0)
                if all((u, v) in edges for u, v in zip(vs, vs[1:])): values.append(sum(edges[u, v] for u, v in zip(vs, vs[1:])))
            expected = [min(values) if values else -1]
        elif name == 'largest_rectangle_histogram':
            a = [rng.randrange(10) for _ in range(rng.randrange(1, 13))]
            data = f'{len(a)}\n' + ' '.join(map(str, a)) + '\n'
            expected = [max(min(a[l:r]) * (r - l) for l in range(len(a)) for r in range(l + 1, len(a) + 1))]
        elif name == 'largest_rectangle_grid':
            h, w = rng.randrange(1, 6), rng.randrange(1, 6)
            a = [[rng.randrange(2) for _ in range(w)] for _ in range(h)]
            data = f'{h} {w}\n' + ''.join(' '.join(map(str, row)) + '\n' for row in a)
            expected = [max([0] + [(bottom - top) * (right - left) for top in range(h) for bottom in range(top + 1, h + 1) for left in range(w) for right in range(left + 1, w + 1) if all(a[i][j] == 0 for i in range(top, bottom) for j in range(left, right))])]
        else: raise AssertionError(name)
        actual = list(map(int, run([str(binary)], input=data).split()))
        if name == 'lis':
            size, *indices = actual
            assert size == expected == len(indices)
            assert all(0 <= i < len(a) for i in indices)
            assert all(indices[i] < indices[i + 1] and a[indices[i]] < a[indices[i + 1]] for i in range(size - 1))
        else: assert actual == expected, (name, data, actual, expected)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    args = parser.parse_args()
    compiler = shlex.split(args.compiler)
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-I', str(ROOT)]
    runtime = ['-O2', '-g'] if not args.sanitize else ['-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-sanitize-recover=all', '-fno-pie', '-no-pie']
    headers = sorted(str(p.relative_to(ROOT)) for p in (ROOT / 'dp_optimization').rglob('*.hpp'))
    for header in headers:
        for extra in [[], ['-DNDEBUG']]:
            run(compiler + flags + extra + ['-fsyntax-only', '-x', 'c++', '-'], input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    print(f'standalone headers (debug/release): {len(headers)} passed', flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-dp-optimization-') as directory:
        temp = Path(directory)
        def compile(source, suffix='', extra=()):
            binary = temp / (source.stem + suffix)
            run(compiler + flags + list(extra) + runtime + [str(source), '-o', str(binary)])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers)) + '#include "tree/all.hpp"\n#include "graph_combinatorics/all.hpp"\n#include "range_query/all.hpp"\n#include "static_query/all.hpp"\n'
        (temp / 'first.cpp').write_text(includes + 'int first(){return cp::lis(std::vector<int>{}).length;}\n')
        (temp / 'second.cpp').write_text(includes + 'int first();int main(){return first();}\n')
        run(compiler + flags + runtime + [str(temp / 'first.cpp'), str(temp / 'second.cpp'), '-o', str(temp / 'multi')]); run([str(temp / 'multi')])
        print('reverse includes / existing categories / multiple translation units: passed', flush=True)
        expected = {'lis':'3\n1 2 4 \n', 'lcs':'3\nace\n', 'edit_distance':'3\nsitting\n', 'zero_one_knapsack':'9\n9\n', 'unbounded_knapsack':'10\n', 'bounded_knapsack':'10\n', 'bitset_knapsack':'0 3 5 6 8 9 11 \n1\n', 'interval_dp':'90\n0 1 2\n0 2 3\n', 'digit_dp':'333\n', 'subset_dp':'6\n0 1 2 0 \n1 3 4 10 \n', 'largest_rectangle':'8 2 4\n4 0 0 2 2\n'}
        for name, want in expected.items(): assert run([str(compile(ROOT / f'examples/dp_optimization/{name}.cpp'))]) == want, name
        for source in sorted((ROOT / 'verify/dp_optimization').glob('*.test.cpp')):
            verify_protocol(source.name.removesuffix('.test.cpp'), compile(source))
        print('11 examples; 12 judge wrappers / 576 independent protocol cases passed', flush=True)
        for suffix, extra in [('_debug', []), ('_release', ['-DNDEBUG'])]:
            print(suffix[1:] + ':\n' + run([str(compile(ROOT / 'verify/dp_optimization/unit_test.cpp', suffix, extra))]), end='', flush=True)
if __name__ == '__main__': main()
