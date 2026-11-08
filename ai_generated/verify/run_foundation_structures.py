#!/usr/bin/env python3
"""F-08..F-10: hash map, CSR graph, monoids and polynomial actions."""
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
MASK = (1 << 64) - 1
MOD = 998244353


def run(command, **kwargs):
    result = subprocess.run(list(map(str,command)),capture_output=True,text=True,timeout=300,**kwargs)
    if result.returncode:
        raise RuntimeError(f'{shlex.join(map(str,command))}\n{result.stdout}{result.stderr}')
    return result.stdout


def mix(x):
    x = (x + 0x9e3779b97f4a7c15) & MASK
    x = ((x ^ (x >> 30)) * 0xbf58476d1ce4e5b9) & MASK
    x = ((x ^ (x >> 27)) * 0x94d049bb133111eb) & MASK
    return x ^ (x >> 31)


def oracle(binary):
    rng = random.Random(810192026)
    inputs, expected = [], []
    for i in range(1500):
        seed = rng.getrandbits(64)
        kind = ['I','W','T','V','S'][i % 5]
        if kind == 'I':
            x = rng.randrange(-(1<<63),1<<63)
            inputs.append(f'I {seed} {x}')
            expected.append(str(mix((x & MASK) ^ seed)))
        elif kind == 'W':
            x = rng.getrandbits(128)
            inputs.append(f'W {seed} {x}')
            expected.append(str(mix(mix((x & MASK) ^ seed) ^ (x >> 64))))
        else:
            n = 2 if kind == 'T' else rng.randrange(10)
            a = [rng.randrange(256) if kind == 'S' else rng.randrange(-(1<<63),1<<63) for _ in range(n)]
            inputs.append(' '.join(map(str,[kind,seed]+([] if kind=='T' else [n])+a)))
            state = mix(seed ^ (0x7465706c65 if kind == 'T' else 0x736571))
            for x in a:
                state = mix(state ^ mix((x & MASK) ^ seed))
            expected.append(str(mix(state ^ n)))
    for _ in range(300):
        n,directed = rng.randrange(20),rng.randrange(2)
        edges = [(rng.randrange(n),rng.randrange(n),rng.randrange(-100,101)) for _ in range(rng.randrange(90))] if n else []
        inputs.append(' '.join(map(str,['G',n,len(edges),directed]+[x for e in edges for x in e])))
        adjacency = [[] for _ in range(n)]
        for idx,(u,v,w) in enumerate(edges):
            adjacency[u].append((u,v,w,idx,0))
            if not directed:
                adjacency[v].append((v,u,w,idx,1))
        arcs = [e for row in adjacency for e in row]
        locations = {(e[3],e[4]):i for i,e in enumerate(arcs)}
        offsets = [0]
        for row in adjacency:
            offsets.append(offsets[-1]+len(row))
        result = [len(arcs)]+offsets
        for u,v,w,idx,direction in arcs:
            result += [u,v,w,idx,-1 if directed else locations[idx,1-direction]]
        expected.append(' '.join(map(str,result)))
    for _ in range(100):
        n,q = rng.randrange(31),80
        a = [rng.randrange(MOD) for _ in range(n)]
        inputs.append(' '.join(map(str,['P',n,q]+a)))
        for j in range(q):
            op = rng.randrange(3) if n else rng.randrange(2)
            l,r = sorted([rng.randrange(n+1),rng.randrange(n+1)])
            if op == 0:
                c = [rng.randrange(-10**9,10**9) for _ in range(3)]
                inputs.append(' '.join(map(str,[0,l,r]+c)))
                for i in range(l,r):
                    a[i]=(a[i]+c[0]+c[1]*i+c[2]*i*i)%MOD
            elif op == 1:
                inputs.append(f'1 {l} {r}')
                expected.append(str(sum(a[l:r])%MOD))
            else:
                p,x = rng.randrange(n),rng.randrange(MOD)
                inputs.append(f'2 {p} {x}')
                a[p]=x
    actual = run([binary],input='\n'.join(inputs)).splitlines()
    assert actual == expected, next(((i,a,b) for i,(a,b) in enumerate(zip(actual,expected)) if a!=b),(len(actual),len(expected)))
    print(f'1500 hash inputs, 300 CSR graphs, 8000 polynomial operations / {len(expected)} outputs passed',flush=True)


def judges(binary,name):
    rng=random.Random(81010)
    for trial in range(24):
        lines,expected=[],[]
        if name=='associative_array':
            q=500;lines=[str(q)];values={}
            for _ in range(q):
                op,key=rng.randrange(2),rng.randrange(100)*1000000007
                if op==0:
                    value=rng.randrange(10**18+1);values[key]=value;lines.append(f'0 {key} {value}')
                else:
                    lines.append(f'1 {key}');expected.append(values.get(key,0))
        else:
            n,q=12,200;values=[(rng.randrange(MOD),rng.randrange(MOD)) for _ in range(n)]
            lines=[f'{n} {q}']+[' '.join(map(str,v)) for v in values]
            for _ in range(q):
                if rng.randrange(2):
                    p=rng.randrange(n);a,b=rng.randrange(MOD),rng.randrange(MOD)
                    values[p]=(a,b);lines.append(f'0 {p} {a} {b}')
                else:
                    l,r=sorted([rng.randrange(n+1),rng.randrange(n+1)]);x=rng.randrange(MOD)
                    lines.append(f'1 {l} {r} {x}')
                    for a,b in values[l:r]:
                        x=(a*x+b)%MOD
                    expected.append(x)
        assert list(map(int,run([binary],input='\n'.join(lines)).split()))==expected,name


def links(entries):
    paths=[ROOT/p for p in ['README.md','HANDOFF.md','docs/README.md','docs/remaining_work.md','docs/implementation_plan.md',
        'docs/basic_utility/README.md','docs/graph_basic/README.md','docs/range_query/README.md',
        'docs/foundation_structures/README.md','docs/foundation_structures/references.md',
        'visualizations/README.md','visualizations/foundation_structures/README.md']]
    paths += [ROOT/'docs'/e['doc'] for e in entries]
    count=0
    for path in paths:
        text=re.sub(r'```.*?```','',path.read_text(),flags=re.S)
        text=re.sub(r'`[^`]*`','',text)
        for target in re.findall(r'\]\(([^)]+)\)',text):
            if '://' in target or target.startswith(('#','mailto:')):
                continue
            assert (path.parent/unquote(target.split('#',1)[0])).exists(),(path,target)
            count+=1
    print(f'{count} local document links passed',flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler',default=os.environ.get('CXX','g++'))
    parser.add_argument('--sanitize',action='store_true')
    parser.add_argument('--links-only',action='store_true')
    args=parser.parse_args()
    manifest=json.loads((ROOT/'verify/foundation_structures/manifest.json').read_text())
    entries=manifest['algorithms']
    if args.links_only:
        links(entries);return
    flags=shlex.split(args.compiler)+['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',ROOT]
    runtime=['-O1','-g','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-fno-pie','-no-pie'] if args.sanitize else ['-O2']
    headers=[e['header'] for e in entries]+manifest['extra_headers']
    for header in headers:
        for extra in [[],['-DNDEBUG']]:
            run(flags+extra+['-x','c++','-fsyntax-only','-'],input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    print('6 standalone/repeated headers in debug/release passed',flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-foundation-structures-') as folder:
        temp=Path(folder)
        def compile(source,name,extra=()):
            binary=temp/name;run(flags+runtime+list(extra)+[source,'-o',binary]);return binary
        includes=''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes+=''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in sorted(ROOT.glob('*/all.hpp')))
        (temp/'a.cpp').write_text(includes+'unsigned long long first(){return cp::SeededHash(42).hash64(12);}\n')
        (temp/'b.cpp').write_text(includes+'unsigned long long first();int main(){return first()!=cp::SeededHash(42).hash64(12);}\n')
        run(flags+runtime+[temp/'a.cpp',temp/'b.cpp','-o',temp/'multi']);run([temp/'multi'])
        for e in entries:
            source=ROOT/'examples'/e['example'];doc=(ROOT/'docs'/e['doc']).read_text()
            assert re.search(r'```cpp\n(.*?)```',doc,re.S).group(1)==source.read_text(),e['doc']
            assert run([compile(source,e['mode'])])==e['output'],e['mode']
        print('all-category translation units and 5 documented examples passed',flush=True)
        observed={}
        for line in run([compile(ROOT/'verify/foundation_structures/visualization_bridge.cpp','bridge')]).splitlines():
            name,value=line.split(' ',1);observed[name]=json.loads(value)
        assert observed==json.loads((ROOT/'verify/foundation_structures/visualization_expected.json').read_text())
        print('5 C++/visualization reference cases passed',flush=True)
        for extra in [[],['-DNDEBUG']]:
            print(run([compile(ROOT/'verify/foundation_structures/unit_test.cpp','unit',extra)]).strip(),flush=True)
            oracle(compile(ROOT/'verify/foundation_structures/query_driver.cpp','driver',extra))
            for name in ['associative_array','point_set_range_composite']:
                judges(compile(ROOT/f'verify/foundation_structures/{name}.test.cpp',name,extra),name)
            print('2 judge formats x 24 local inputs passed '+('release' if extra else 'debug'),flush=True)
    links(entries)


if __name__=='__main__':
    main()
