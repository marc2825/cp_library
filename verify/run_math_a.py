#!/usr/bin/env python3
"""3.12--3.15 A additions: headers, examples, judge protocols and independent oracles."""
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
    result = subprocess.run(list(map(str, command)), text=True, capture_output=True, timeout=180, **kwargs)
    if result.returncode:
        raise RuntimeError(f"{shlex.join(map(str, command))}\n{result.stdout}{result.stderr}")
    return result.stdout

import sys
sys.dont_write_bytecode=True
sys.path.insert(0,str(ROOT/'verify/math_a'))
from protocols import verify

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler',default=os.environ.get('CXX','g++'))
    parser.add_argument('--sanitize',action='store_true')
    args=parser.parse_args(); compiler=shlex.split(args.compiler)
    algorithms=json.loads((ROOT/'verify/math_a/manifest.json').read_text())['algorithms']
    headers=json.loads((ROOT/'verify/math_a/headers.json').read_text())
    flags=['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',str(ROOT)]
    runtime=['-O2'] if not args.sanitize else ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all','-fno-pie','-no-pie']
    for h in headers:
        for extra in [[],['-DNDEBUG']]:
            run(compiler+flags+extra+['-fsyntax-only','-x','c++','-'],input=f'#include "{h}"\n#include "{h}"\nint main(){{}}\n')
    print(f'{len(headers)} standalone/repeated headers debug/release passed',flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-math-a-') as d:
        temp=Path(d)
        def compile(source,name,extra=()):
            binary=temp/name;run(compiler+flags+runtime+list(extra)+[source,'-o',binary]);return binary
        includes=''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes+=''.join(f'#include "{h.relative_to(ROOT)}"\n' for h in sorted(ROOT.glob('*/all.hpp')))
        (temp/'a.cpp').write_text(includes+'int first(){return cp::euler_phi(1)==1?0:1;}\n')
        (temp/'b.cpp').write_text(includes+'int first();int main(){return first();}\n')
        run(compiler+flags+runtime+[temp/'a.cpp',temp/'b.cpp','-o',temp/'multi']);run([temp/'multi'])
        for e in algorithms:
            example=ROOT/'examples'/e['example'];doc=ROOT/'docs'/e['doc']
            content=doc.read_text();assert f"documentation_of: //{e['header']}" in content
            assert re.search(r'```cpp\n(.*?)```',content,re.S).group(1)==example.read_text(),doc
            binary=compile(example,e['header'].replace('/','_'));assert run([binary])==e['output'],example
        print(f'mixed TUs, {len(algorithms)} examples and exact documentation usage passed',flush=True)
        for source in sorted((ROOT/'verify/math_a').glob('*.test.cpp')):
            binary=compile(source,source.stem);rng=random.Random(91823)
            for _ in range(24):
                verify(run,binary,source.name.split('.')[0],rng)
        print('8 judge wrappers x 24 cases: passed (local only)',flush=True)
        probe=compile(ROOT/'verify/math_a/number_probe.cpp','number_probe')
        from protocols import verify_big_integers
        verify_big_integers(run,probe)
        for source in sorted((ROOT/'verify/math_a').glob('*_test.cpp')):
            for name,extra in [('debug',[]),('release',['-DNDEBUG'])]:
                binary=compile(source,source.stem+name,extra);print(name+' '+run([binary]),end='',flush=True)
    documents=[ROOT/'docs'/e['doc'] for e in algorithms]
    documents+=[ROOT/'docs/math_a/README.md',ROOT/'docs/math_a/references.md',ROOT/'visualizations/math_a/index.html',ROOT/'visualizations/math_a/README.md']
    documents+=[ROOT/p for p in ['README.md','HANDOFF.md','docs/README.md','docs/implementation_plan.md','visualizations/index.html','visualizations/README.md']]
    for category in ['dp_optimization','number_theory_basic','combinatorics']:
        documents.extend([ROOT/'docs'/category/'README.md',ROOT/'docs'/category/'references.md'])
    count=0
    for doc in documents:
        prose=re.sub(r'```.*?```','',doc.read_text(),flags=re.S)
        for href in re.findall(r'\]\(([^)]+)\)|href="([^"]+)"',prose):
            target=next(x for x in href if x).split('#')[0]
            if not target or re.match(r'[a-z]+:',target): continue
            assert (doc.parent/unquote(target)).exists(),(doc,target);count+=1
    print(f'{count} local documentation links passed',flush=True)

if __name__=='__main__': main()
