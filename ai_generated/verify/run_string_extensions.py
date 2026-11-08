#!/usr/bin/env python3
"""S: 11 string/index APIs: independent oracles, examples, and header integration."""
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
    names = ['README.md', 'docs/README.md', 'docs/string_algorithm/README.md',
             'docs/string_extensions/README.md', 'docs/string_extensions/references.md',
             'visualizations/README.md', 'visualizations/string_extensions/README.md']
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
    entries = json.loads((ROOT / 'verify/string_extensions/manifest.json').read_text())['algorithms']
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
    with tempfile.TemporaryDirectory(prefix='cp-string-extensions-') as folder:
        temp = Path(folder)
        def compile(source, name, extra=()):
            binary = temp / name
            run(flags + runtime + list(extra) + [source, '-o', binary])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes += ''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in sorted(ROOT.glob('*/all.hpp')))
        (temp / 'a.cpp').write_text(includes + 'int first(){return cp::FMIndex("banana").count("ana"); }\n')
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
        assert observed == json.loads((ROOT / 'verify/string_extensions/visualization_expected.json').read_text())
        print('all-category translation units and 11 documented/visualization examples passed', flush=True)
        for extra in [[], ['-DNDEBUG']]:
            for name in ['index_test', 'structure_test', 'deque_test']:
                print(run([compile(ROOT / f'verify/string_extensions/{name}.cpp', name, extra)]).strip(), flush=True)
            rng = random.Random(20261027)
            driver = compile(ROOT / 'verify/string_extensions/query_driver.cpp', 'query-driver', extra)
            judge = compile(ROOT / 'verify/string_extensions/number_of_substrings.test.cpp', 'judge', extra)
            data, expected = ['96'], []
            for trial in range(96):
                n = rng.randrange(120)
                text = bytes(rng.randrange(256 if trial % 2 else 3) for _ in range(n))
                if trial % 3 and n:
                    l, r = sorted([rng.randrange(n + 1), rng.randrange(n + 1)])
                    pattern = text[l:r]
                else:
                    pattern = bytes(rng.randrange(256) for _ in range(rng.randrange(8)))
                rate = rng.choice([1, 3, 32, 1000000])
                modulus = rng.choice([1, 2, 6, 998244353, 2**64-1])
                position = rng.randrange(n + 1)
                total, previous = 1, {}
                for c in text:
                    before = total
                    total = 2 * total - previous.get(c, 0)
                    previous[c] = before
                z = 0
                while position + z < n and text[z] == text[position + z]: z += 1
                found = [i for i in range(n + 1) if text[i:i+len(pattern)] == pattern]
                distinct = len({text[l:r] for l in range(n) for r in range(l+1,n+1)})
                data.append(f'{text.hex() or "-"} {pattern.hex() or "-"} {rate} {modulus} {position}')
                expected.append([len(found), distinct, total % modulus, z, *found])
            got = run([driver], input='\n'.join(data)+'\n')
            assert [list(map(int, line.split())) for line in got.splitlines()] == expected
            for trial in range(24):
                text = ''.join(rng.choice('abcde') for _ in range(1+rng.randrange(80)))
                want = len({text[l:r] for l in range(len(text)) for r in range(l+1,len(text)+1)})
                assert int(run([judge], input=text+'\n')) == want
            print(('release' if extra else 'debug') + ' and Python 96 byte/index/count cases / 24 judge cases passed', flush=True)
    links(entries)


if __name__ == '__main__':
    main()
