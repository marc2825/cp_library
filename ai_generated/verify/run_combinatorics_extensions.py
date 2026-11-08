#!/usr/bin/env python3
"""C-01..04, C-20..23: independent oracles, examples, and header integration."""
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
    names = ['README.md', 'docs/README.md', 'docs/number_theory_basic/README.md', 'docs/number_theory_convolution/README.md', 'docs/combinatorics/README.md',
             'docs/combinatorics_extensions/README.md', 'docs/combinatorics_extensions/references.md',
             'visualizations/README.md', 'visualizations/combinatorics_extensions/README.md']
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
    entries = json.loads((ROOT / 'verify/combinatorics_extensions/manifest.json').read_text())['algorithms']
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
    with tempfile.TemporaryDirectory(prefix='cp-combinatorics-extensions-') as folder:
        temp = Path(folder)
        def compile(source, name, extra=()):
            binary = temp / name
            run(flags + runtime + list(extra) + [source, '-o', binary])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes += ''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in sorted(ROOT.glob('*/all.hpp')))
        (temp / 'a.cpp').write_text(includes + 'int first(){return cp::standard_young_tableaux_count({3,2}).to_integral<int>();}\n')
        (temp / 'b.cpp').write_text(includes + 'int first();int main(){return first()!=5;}\n')
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
        assert observed == json.loads((ROOT / 'verify/combinatorics_extensions/visualization_expected.json').read_text())
        print('all-category translation units and 8 documented/visualization examples passed', flush=True)
        for extra in [[], ['-DNDEBUG']]:
            for name in ['counting_test', 'corridor_test', 'tableaux_test']:
                print(run([compile(ROOT / f'verify/combinatorics_extensions/{name}.cpp', name, extra)]).strip(), flush=True)
            print(('release' if extra else 'debug') + ' passed', flush=True)
    links(entries)


if __name__ == '__main__':
    main()
