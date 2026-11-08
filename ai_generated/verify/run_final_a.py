#!/usr/bin/env python3
"""3.21--3.25 A additions: independent oracles and public entry points."""
import argparse
import json
import os
from pathlib import Path
import random
import re
import shlex
import subprocess
import sys
import tempfile
from urllib.parse import unquote

ROOT=Path(__file__).resolve().parents[1]
sys.dont_write_bytecode=True
sys.path.insert(0,str(ROOT/'verify/final_a'))
from oracles import verify,common,factorization,nim_reference

def run(command,**kwargs):
    r=subprocess.run(list(map(str,command)),text=True,capture_output=True,timeout=180,**kwargs)
    if r.returncode:raise RuntimeError(f'{shlex.join(map(str,command))}\n{r.stdout}{r.stderr}')
    return r.stdout

def judge(binary,name):
    rng=random.Random(321325)
    for trial in range(24):
        text=lambda n:''.join(rng.choice('abc')for _ in range(n))
        if name=='nim_product':
            pairs=[(rng.getrandbits(64),rng.getrandbits(64))for _ in range(10)]
            data='10\n'+'\n'.join(f'{a} {b}'for a,b in pairs)+'\n'
            assert list(map(int,run([binary],input=data).split()))==[nim_reference(a,b)for a,b in pairs]
        elif name=='hash_2d':
            h,w=rng.randrange(1,9),rng.randrange(1,9);n,m=rng.randrange(1,6),rng.randrange(1,6);a=[text(w)for _ in range(h)];b=[text(m)for _ in range(n)]
            if trial%2 and n<=h and m<=w:b=[row[:m]for row in a[:n]]
            data=f'{h} {w}\n'+'\n'.join(a)+f'\n{n} {m}\n'+'\n'.join(b)+'\n'
            want=[v for i in range(h-n+1)for j in range(w-m+1)if all(a[i+k][j:j+m]==b[k]for k in range(n))for v in [i,j]]
            assert list(map(int,run([binary],input=data).split()))==want
        else:
            s=text(1+rng.randrange(10 if name=='lyndon' else 40));data=s+'\n'
            if name=='suffix_automaton':want=[len({s[i:j]for i in range(len(s))for j in range(i+1,len(s)+1)})]
            elif name=='lyndon':want=[0]+[r for l,r in factorization(s.encode())]
            else:
                t=text(1+rng.randrange(40));data+=t+'\n';i,j,n=common(s.encode(),t.encode());want=[i,i+n,j,j+n]
            assert list(map(int,run([binary],input=data).split()))==want

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--compiler',default=os.environ.get('CXX','g++'));parser.add_argument('--sanitize',action='store_true');args=parser.parse_args()
    compiler=shlex.split(args.compiler);entries=json.loads((ROOT/'verify/final_a/manifest.json').read_text())['algorithms'];headers=[e['header']for e in entries]
    flags=['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',str(ROOT)]
    runtime=['-O2']if not args.sanitize else ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all','-fno-pie','-no-pie']
    for h in headers:
        for extra in [[],['-DNDEBUG']]:run(compiler+flags+extra+['-fsyntax-only','-x','c++','-'],input=f'#include "{h}"\n#include "{h}"\nint main(){{}}\n')
    print(f'{len(headers)} standalone/repeated headers debug/release passed',flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-final-a-')as d:
        temp=Path(d)
        def compile(source,name,extra=()):
            binary=temp/name;run(compiler+flags+runtime+list(extra)+[source,'-o',binary]);return binary
        includes=''.join(f'#include "{h}"\n'for h in reversed(headers))+''.join(f'#include "{h.relative_to(ROOT)}"\n'for h in sorted(ROOT.glob('*/all.hpp')))
        (temp/'a.cpp').write_text(includes+'int first(){return cp::SuffixAutomaton().distinct_substrings()==0?0:1;}\n');(temp/'b.cpp').write_text(includes+'int first();int main(){return first();}\n')
        run(compiler+flags+runtime+[temp/'a.cpp',temp/'b.cpp','-o',temp/'multi']);run([temp/'multi'])
        for e in entries:
            example=ROOT/'examples'/e['example'];doc=ROOT/'docs'/e['doc'];content=doc.read_text();assert f"documentation_of: //{e['header']}"in content
            assert re.search(r'```cpp\n(.*?)```',content,re.S).group(1)==example.read_text(),doc
            assert run([compile(example,e['header'].replace('/','_'))])==e['output'],example
        print('mixed category TUs, 10 examples and exact documentation usage passed',flush=True)
        for source in sorted((ROOT/'verify/final_a').glob('*.test.cpp')):judge(compile(source,source.stem),source.name.split('.')[0])
        print('5 judge wrappers x 24 cases passed (local only)',flush=True)
        for name,extra in [('debug',[]),('release',['-DNDEBUG'])]:
            print(name+' '+run([compile(ROOT/'verify/final_a/unit_test.cpp','unit'+name,extra)]),end='',flush=True)
            count=verify(run,compile(ROOT/'verify/final_a/query_driver.cpp','probe'+name,extra));print(f'{name}: {count} Python byte/partition/nim/window/hash/search oracles passed',flush=True)
    docs=[ROOT/'docs'/e['doc']for e in entries]+[ROOT/p for p in ['README.md','docs/README.md','docs/final_a/README.md','docs/final_a/references.md','visualizations/index.html','visualizations/README.md','visualizations/final_a/index.html','visualizations/final_a/README.md']]
    for c in ['string_algorithm','game_theory','heuristic_search']:docs.extend([ROOT/f'docs/{c}/README.md',ROOT/f'docs/{c}/references.md'])
    count=0
    for doc in docs:
        content=re.sub(r'```.*?```','',doc.read_text(),flags=re.S)
        for pair in re.findall(r'\]\(([^)]+)\)|href="([^"]+)"',content):
            target=next(x for x in pair if x).split('#')[0]
            if not target or re.match(r'[a-z]+:',target):continue
            assert (doc.parent/unquote(target)).exists(),(doc,target);count+=1
    print(f'{count} local documentation links passed',flush=True)

if __name__=='__main__':main()
