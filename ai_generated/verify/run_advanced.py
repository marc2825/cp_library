#!/usr/bin/env python3
"""Advanced additions: standalone headers, mixed TUs, examples, judge protocols, differential tests."""
import argparse
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

def run(command, **kwargs):
    result = subprocess.run(list(map(str, command)), text=True, capture_output=True, **kwargs)
    if result.returncode:
        raise RuntimeError(f"{shlex.join(map(str, command))}\n{result.stdout}{result.stderr}")
    return result.stdout

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    args = parser.parse_args()
    manifest = json.loads((ROOT / 'verify/advanced/manifest.json').read_text())
    compiler = shlex.split(args.compiler)
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-I', str(ROOT)]
    runtime = ['-O2'] if not args.sanitize else ['-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-sanitize-recover=all', '-fno-pie', '-no-pie']
    headers = [entry['header'] for entry in manifest['algorithms']]
    categories = sorted({h.split('/')[0] for h in headers})
    for h in headers:
        for extra in [[], ['-DNDEBUG']]:
            run(compiler + flags + extra + ['-fsyntax-only', '-x', 'c++', '-'], input=f'#include "{h}"\n#include "{h}"\nint main(){{}}\n')
    for h in ['data_structure/segtree_beats.hpp', 'data_structure/DynamicConectivity.hpp', 'graph_theory/Tree/AuxiliraryTree.hpp']:
        run(compiler + flags + ['-fsyntax-only', '-x', 'c++', '-'], input=f'#include "{h}"\nint main(){{}}\n')
    print(f'{len(headers)} standalone/repeated headers (debug/release): passed', flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-advanced-') as directory:
        temp = Path(directory)
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers)) + ''.join(f'#include "{c}/all.hpp"\n' for c in categories)
        includes += '#include "fps/all.hpp"\n#include "linear_recurrence/all.hpp"\n#include "combinatorial_optimization/all.hpp"\n'
        (temp / 'a.cpp').write_text(includes + 'int first(){return cp::is_prime_64(97)?0:1;}\n')
        (temp / 'b.cpp').write_text(includes + 'int first();int main(){return first();}\n')
        run(compiler + flags + runtime + [temp / 'a.cpp', temp / 'b.cpp', '-o', temp / 'multi'])
        run([temp / 'multi'])
        print('mixed all.hpp, reverse includes and multiple translation units: passed', flush=True)
        def compile(source, name, extra=()):
            binary = temp / name
            run(compiler + flags + runtime + list(extra) + [ROOT / source, '-o', binary])
            return binary
        for entry in manifest['algorithms']:
            example = f"examples/{entry['header'][:-4]}.cpp"
            binary = compile(example, entry['header'].replace('/', '_'))
            assert run([binary]) == entry['output'], example
            document = ROOT / f"docs/{entry['header'][:-4]}.md"
            content = document.read_text()
            assert f"documentation_of: //{entry['header']}" in content
            usage = re.search(r'```cpp\n(.*?)```', content, re.S).group(1)
            assert usage == (ROOT / example).read_text(), f'usage drift: {document}'
        print(f"{len(headers)} examples and matching documentation usages: passed", flush=True)
        for entry in manifest['wrappers']:
            binary = compile(entry['source'], Path(entry['source']).stem)
            assert run([binary], input=entry['input']) == entry['output'], entry['source']
        print(f"{len(manifest['wrappers'])} judge protocol samples: passed", flush=True)
        for source in sorted((ROOT / 'verify/advanced').glob('*_test.cpp')):
            for suffix, extra in [('debug', []), ('release', ['-DNDEBUG'])]:
                binary = compile(source, source.stem + '_' + suffix, extra)
                print(suffix + ' ' + run([binary]), end='', flush=True)

if __name__ == '__main__':
    main()
