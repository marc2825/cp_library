#!/usr/bin/env python3
"""U-01..U-11 and U-20..U-22: independent oracles, examples, header integration."""
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
    paths = [ROOT / p for p in ['README.md', 'HANDOFF.md', 'docs/README.md', 'docs/remaining_work.md',
        'docs/implementation_plan.md', 'docs/basic_utility/README.md', 'docs/utility_search/README.md',
        'docs/utility_search/references.md', 'visualizations/README.md', 'visualizations/utility_search/README.md']]
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    parser.add_argument('--links-only', action='store_true')
    args = parser.parse_args()
    entries = json.loads((ROOT / 'verify/utility_search/manifest.json').read_text())['algorithms']
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
    with tempfile.TemporaryDirectory(prefix='cp-utility-search-') as folder:
        temp = Path(folder)
        def compile(source, name, extra=()):
            binary = temp / name
            run(flags + runtime + list(extra) + [source, '-o', binary])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes += ''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in sorted(ROOT.glob('*/all.hpp')))
        (temp / 'a.cpp').write_text(includes + 'int first(){return cp::PermutationTree({1,0}).common_interval_count();}\n')
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
        for line in run([compile(ROOT / 'verify/utility_search/visualization_bridge.cpp', 'bridge')]).splitlines():
            name, value = line.split(' ', 1)
            observed[name] = json.loads(value)
        assert observed == json.loads((ROOT / 'verify/utility_search/visualization_expected.json').read_text())
        print(f'{len(entries)} C++/visualization reference cases passed', flush=True)
        for extra in [[], ['-DNDEBUG']]:
            for name in ['unit_test', 'retroactive_enumeration_test']:
                print(run([compile(ROOT / f'verify/utility_search/{name}.cpp', name, extra)]).strip(), flush=True)
            print(('release' if extra else 'debug') + ' passed', flush=True)
    links(entries)


if __name__ == '__main__':
    main()
