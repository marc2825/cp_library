#!/usr/bin/env python3
"""D-20..D-35: independent oracles, examples, header integration."""
import argparse
import json
import os
from pathlib import Path
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
    paths = [ROOT / p for p in ['README.md', 'docs/README.md', 'docs/static_query/README.md', 'docs/ordered_structure/README.md', 'docs/range_query/README.md', 'docs/offline_query/README.md', 'docs/union_find/README.md', 'docs/data_structure_advanced/README.md',
        'docs/data_structure_advanced/references.md', 'visualizations/README.md', 'visualizations/data_structure_advanced/README.md']]
    paths += [ROOT / 'docs' / e['doc'] for e in entries]
    count = 0
    for path in paths:
        source = re.sub(r'```.*?```', '', path.read_text(), flags=re.S)
        source = re.sub(r'`[^`]*`', '', source)
        for target in re.findall(r'\]\(([^)]+)\)', source):
            if '://' in target or target.startswith(('#', 'mailto:')):
                continue
            if not (path.parent / unquote(target.split('#', 1)[0])).exists():
                raise AssertionError((path, target))
            count += 1
    print(f'{count} local document links passed', flush=True)


def judge_checks(compile):
    import random
    import bisect
    for mode in ['static_range_lis', 'range_inversions']:
        source = ROOT / ('verify/data_structure_advanced/static_range_lis.test.cpp' if mode == 'static_range_lis' else 'verify/offline_query/range_inversions.test.cpp')
        binary = compile(source, 'judge')
        for seed in range(12):
            rng = random.Random(seed)
            values = [rng.randrange(-20, 21) for _ in range(50)]
            queries = [tuple(sorted([rng.randrange(51), rng.randrange(51)])) for _ in range(150)]
            expected = []
            for l, r in queries:
                if mode == 'range_inversions':
                    expected.append(sum(values[i] > values[j] for i in range(l, r) for j in range(i+1, r)))
                else:
                    tails = []
                    for value in values[l:r]:
                        p = bisect.bisect_left(tails, value)
                        if p == len(tails): tails.append(value)
                        else: tails[p] = value
                    expected.append(len(tails))
            data = f'{len(values)} {len(queries)}\n' + ' '.join(map(str, values)) + '\n' + '\n'.join(f'{l} {r}' for l, r in queries) + '\n'
            assert list(map(int, run([binary], input=data).split())) == expected
    print('2 judge protocols x 12 independent Python inputs passed', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    parser.add_argument('--links-only', action='store_true')
    args = parser.parse_args()
    entries = json.loads((ROOT / 'verify/data_structure_advanced/manifest.json').read_text())['algorithms']
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
    with tempfile.TemporaryDirectory(prefix='cp-data-structure-advanced-') as folder:
        temp = Path(folder)
        def compile(source, name, extra=()):
            binary = temp / name
            run(flags + runtime + list(extra) + [source, '-o', binary])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes += ''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in sorted(ROOT.glob('*/all.hpp')))
        (temp / 'a.cpp').write_text(includes + 'int first(){return cp::DynamicWaveletMatrix(3).bit_width();}\n')
        (temp / 'b.cpp').write_text(includes + 'int first();int main(){return first()!=3;}\n')
        run(flags + runtime + [temp / 'a.cpp', temp / 'b.cpp', '-o', temp / 'multi'])
        run([temp / 'multi'])
        observed = {}
        for e in entries:
            source = ROOT / 'examples' / e['example']
            doc = (ROOT / 'docs' / e['doc']).read_text()
            assert 'documentation_of: //' + e['header'] in doc
            assert re.search(r'```cpp\n(.*?)```', doc, re.S).group(1) == source.read_text(), e['doc']
            output = run([compile(source, e['mode'])])
            assert output == e['output'], e['mode']
            observed[e['mode']] = output.split()
        print(f'all-category translation units and {len(entries)} documented examples passed', flush=True)
        assert observed == json.loads((ROOT / 'verify/data_structure_advanced/visualization_expected.json').read_text())
        print(f'{len(entries)} C++/visualization reference cases passed', flush=True)
        for extra in [[], ['-DNDEBUG']]:
            for name in ['query_test', 'sequence_test', 'retroactive_spatial_test']:
                print(run([compile(ROOT / f'verify/data_structure_advanced/{name}.cpp', name, extra)]).strip(), flush=True)
            print(('release' if extra else 'debug') + ' passed', flush=True)
        judge_checks(compile)

    links(entries)


if __name__ == '__main__':
    main()
