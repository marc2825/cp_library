#!/usr/bin/env python3
"""3.1--3.6 A additions: headers, examples, judge protocols and independent oracles."""
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

def protocols(name, rng):
    n, q = 45, 300
    output, lines = [], []
    if name == 'persistent_unionfind':
        versions = [list(range(n))]
        lines = [f'{n} {q}']
        for i in range(q):
            base = rng.randrange(i+1); u, v = rng.randrange(n), rng.randrange(n); kind = rng.randrange(2)
            a = versions[base].copy(); lines.append(f'{kind} {base-1} {u} {v}')
            if kind: output.append(int(a[u] == a[v]))
            else:
                before, after = a[v], a[u]; a = [after if x == before else x for x in a]
            versions.append(a)
    elif name == 'persistent_queue':
        versions = [[]]; lines = [str(q)]
        for i in range(q):
            base = rng.randrange(i+1); a = versions[base].copy()
            if not a or rng.randrange(2):
                x = rng.randrange(10**9); lines.append(f'0 {base-1} {x}'); a.append(x)
            else:
                lines.append(f'1 {base-1}'); output.append(a.pop(0))
            versions.append(a)
    elif name == 'static_rmq':
        a = [rng.randrange(10**9) for _ in range(n)]; lines = [f'{n} {q}', ' '.join(map(str,a))]
        for _ in range(q):
            l = rng.randrange(n); r = rng.randrange(l+1,n+1); lines.append(f'{l} {r}'); output.append(min(a[l:r]))
    elif name == 'predecessor':
        a = {i for i in range(n) if rng.randrange(2)}; lines = [f'{n} {q}', ''.join('1' if i in a else '0' for i in range(n))]
        for _ in range(q):
            kind, x = rng.randrange(5), rng.randrange(n); lines.append(f'{kind} {x}')
            if kind == 0: a.add(x)
            elif kind == 1: a.discard(x)
            elif kind == 2: output.append(int(x in a))
            elif kind == 3: output.append(min((v for v in a if v>=x),default=-1))
            else: output.append(max((v for v in a if v<=x),default=-1))
    elif name == 'set_xor_min':
        a = set(); lines = [str(q)]
        for _ in range(q):
            kind, x = rng.randrange(3), rng.randrange(1<<30)
            if not a: kind=0
            if kind == 0: a.add(x)
            elif kind == 1:
                if rng.randrange(2): x=rng.choice(sorted(a))
                a.discard(x)
            else: output.append(min(v^x for v in a))
            lines.append(f'{kind} {x}')
    elif name == 'point_add_rectangle_sum':
        points = [(rng.randrange(20),rng.randrange(20),rng.randrange(1000)) for _ in range(n)]
        lines = [f'{n} {q}'] + [' '.join(map(str,p)) for p in points]
        for _ in range(q):
            if rng.randrange(2):
                p=(rng.randrange(20),rng.randrange(20),rng.randrange(1000));points.append(p);lines.append('0 '+' '.join(map(str,p)))
            else:
                l,r=sorted(rng.sample(range(22),2));d,u=sorted(rng.sample(range(22),2));lines.append(f'1 {l} {d} {r} {u}')
                output.append(sum(w for x,y,w in points if l<=x<r and d<=y<u))
    return '\n'.join(lines)+'\n', ''.join(f'{x}\n' for x in output)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler',default=os.environ.get('CXX','g++'))
    parser.add_argument('--sanitize',action='store_true')
    args=parser.parse_args(); compiler=shlex.split(args.compiler)
    algorithms=json.loads((ROOT/'verify/a_priority/manifest.json').read_text())['algorithms']
    headers=json.loads((ROOT/'verify/a_priority/headers.json').read_text())
    flags=['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',str(ROOT)]
    runtime=['-O2'] if not args.sanitize else ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all','-fno-pie','-no-pie']
    for h in headers:
        for extra in [[],['-DNDEBUG']]:
            run(compiler+flags+extra+['-fsyntax-only','-x','c++','-'],input=f'#include "{h}"\n#include "{h}"\nint main(){{}}\n')
    print(f'{len(headers)} standalone/repeated headers debug/release passed',flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-a-priority-') as d:
        temp=Path(d)
        def compile(source,name,extra=()):
            binary=temp/name;run(compiler+flags+runtime+list(extra)+[source,'-o',binary]);return binary
        includes=''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes+=''.join(f'#include "{c}/all.hpp"\n' for c in sorted({h.split('/')[0] for h in headers}))
        (temp/'a.cpp').write_text(includes+'int first(){cp::FastSet s(1);s.insert(0);return s.next(0);}\n')
        (temp/'b.cpp').write_text(includes+'int first();int main(){return first();}\n')
        run(compiler+flags+runtime+[temp/'a.cpp',temp/'b.cpp','-o',temp/'multi']);run([temp/'multi'])
        for e in algorithms:
            example=ROOT/'examples'/e['header'].replace('.hpp','.cpp');doc=ROOT/'docs'/e['header'].replace('.hpp','.md')
            content=doc.read_text();assert f"documentation_of: //{e['header']}" in content
            assert re.search(r'```cpp\n(.*?)```',content,re.S).group(1)==example.read_text(),doc
            binary=compile(example,e['header'].replace('/','_'));assert run([binary])==e['output'],example
        print(f'mixed TUs, {len(algorithms)} examples and exact documentation usage passed',flush=True)
        for source in sorted((ROOT/'verify/a_priority').glob('*.test.cpp')):
            binary=compile(source,source.stem);rng=random.Random(91823)
            for _ in range(24):
                inp,out=protocols(source.name.split('.')[0],rng);assert run([binary],input=inp)==out,source
        print('6 judge protocols x 24 batches x 300 operations: passed (local only)',flush=True)
        for source in sorted((ROOT/'verify/a_priority').glob('*_test.cpp')):
            for name,extra in [('debug',[]),('release',['-DNDEBUG'])]:
                binary=compile(source,source.stem+name,extra);print(name+' '+run([binary]),end='',flush=True)
    documents=[ROOT/'docs'/e['header'].replace('.hpp','.md') for e in algorithms]
    documents+=[ROOT/'docs/a_priority/README.md',ROOT/'docs/a_priority/references.md',ROOT/'visualizations/a_priority/index.html',ROOT/'visualizations/a_priority/README.md']
    count=0
    for doc in documents:
        prose=re.sub(r'```.*?```','',doc.read_text(),flags=re.S)
        for href in re.findall(r'\]\(([^)]+)\)|href="([^"]+)"',prose):
            target=next(x for x in href if x).split('#')[0]
            if not target or re.match(r'[a-z]+:',target): continue
            assert (doc.parent/unquote(target)).exists(),(doc,target);count+=1
    print(f'{count} local documentation links passed',flush=True)

if __name__=='__main__': main()
