#!/usr/bin/env python3
"""D-02..D-10: independent oracles, examples, header integration."""
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
    paths = [ROOT / p for p in ['README.md', 'docs/README.md', 'docs/static_query/README.md', 'docs/ordered_structure/README.md', 'docs/range_query/README.md', 'docs/offline_query/README.md', 'docs/union_find/README.md', 'docs/data_structure_extensions/README.md',
        'docs/data_structure_extensions/references.md', 'visualizations/README.md', 'visualizations/data_structure_extensions/README.md']]
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
    for mode in ['double_ended_priority_queue', 'point_set_range_frequency']:
        binary = compile(ROOT / f'verify/data_structure_extensions/{mode}.test.cpp', 'judge')
        for seed in range(12):
            rng = random.Random(seed)
            initial = [rng.randrange(20) for _ in range(40)]
            values = sorted(initial) if mode == 'double_ended_priority_queue' else initial[:]
            queries, expected = [], []
            for _ in range(1500):
                if mode == 'double_ended_priority_queue':
                    t = rng.randrange(3) if values else 0
                    if t == 0:
                        x = rng.randrange(1000000); bisect.insort(values,x); queries.append(f'0 {x}')
                    else:
                        queries.append(str(t)); expected.append(values.pop(0 if t == 1 else -1))
                elif rng.randrange(2):
                    p, x = rng.randrange(len(values)), rng.randrange(100)
                    values[p] = x; queries.append(f'0 {p} {x}')
                else:
                    l, r = sorted([rng.randrange(len(values)+1), rng.randrange(len(values)+1)])
                    x = rng.randrange(100); queries.append(f'1 {l} {r} {x}'); expected.append(values[l:r].count(x))
            source = f'{len(initial)} {len(queries)}\n' + ' '.join(map(str,initial)) + '\n' + '\n'.join(queries) + '\n'
            assert list(map(int,run([binary],input=source).split())) == expected
    print('2 judge protocols x 12 independent Python inputs passed',flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    parser.add_argument('--links-only', action='store_true')
    args = parser.parse_args()
    entries = json.loads((ROOT / 'verify/data_structure_extensions/manifest.json').read_text())['algorithms']
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
    with tempfile.TemporaryDirectory(prefix='cp-data-structure-extensions-') as folder:
        temp = Path(folder)
        def compile(source, name, extra=()):
            binary = temp / name
            run(flags + runtime + list(extra) + [source, '-o', binary])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes += ''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in sorted(ROOT.glob('*/all.hpp')))
        (temp / 'a.cpp').write_text(includes + 'int first(){return cp::SegmentTreeCover(3).size();}\n')
        (temp / 'b.cpp').write_text(includes + 'int first();int main(){return first()!=3;}\n')
        run(flags + runtime + [temp / 'a.cpp', temp / 'b.cpp', '-o', temp / 'multi'])
        run([temp / 'multi'])
        for e in entries:
            source = ROOT / 'examples' / e['example']
            doc = (ROOT / 'docs' / e['doc']).read_text()
            assert 'documentation_of: //' + e['header'] in doc
            assert re.search(r'```cpp\n(.*?)```', doc, re.S).group(1) == source.read_text(), e['doc']
            assert run([compile(source, e['mode'])]) == e['output'], e['mode']
        print(f'all-category translation units and {len(entries)} documented examples passed', flush=True)
        observed = {}
        for line in run([compile(ROOT / 'verify/data_structure_extensions/visualization_bridge.cpp', 'bridge')]).splitlines():
            name, value = line.split(' ', 1)
            observed[name] = json.loads(value)
        assert observed == json.loads((ROOT / 'verify/data_structure_extensions/visualization_expected.json').read_text())
        print(f'{len(entries)} C++/visualization reference cases passed', flush=True)
        for extra in [[], ['-DNDEBUG']]:
            for name in ['query_test', 'persistent_test', 'rollback_test']:
                print(run([compile(ROOT / f'verify/data_structure_extensions/{name}.cpp', name, extra)]).strip(), flush=True)
            for source in ['verify/graph_a/paths_test.cpp', 'verify/a_priority/range_test.cpp', 'verify/a_priority/dsu_test.cpp', 'verify/b_1_6/spatial_test.cpp']:
                print(run([compile(ROOT / source, 'regression', extra)]).strip(), flush=True)
            print(('release' if extra else 'debug') + ' passed', flush=True)
        judge_checks(compile)

    links(entries)


if __name__ == '__main__':
    main()
