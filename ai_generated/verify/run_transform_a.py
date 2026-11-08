#!/usr/bin/env python3
"""3.16--3.20 A additions: independent oracles and public entry points."""
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
sys.path.insert(0,str(ROOT/'verify/transform_a'))
from oracles import verify,det,charpoly,xor_span

def run(command,**kwargs):
    r=subprocess.run(list(map(str,command)),text=True,capture_output=True,timeout=180,**kwargs)
    if r.returncode:raise RuntimeError(f'{shlex.join(map(str,command))}\n{r.stdout}{r.stderr}')
    return r.stdout

def judge(binary,name):
    rng=random.Random(320);p=998244353
    flat=lambda a:' '.join(map(str,a))
    for trial in range(24):
        if name in ['karatsuba','bitwise_and','bitwise_or','bitwise_xor','ranked_subset']:
            bits=rng.randrange(7);n=1<<bits;m=n if name!='karatsuba' else 1+rng.randrange(100)
            a=[rng.randrange(p)for _ in range(n)];b=[rng.randrange(p)for _ in range(m)]
            c=[0]*(n+m-1 if name=='karatsuba' else n)
            for i,x in enumerate(a):
                for j,y in enumerate(b):
                    if name=='ranked_subset' and i&j:continue
                    k=i+j if name=='karatsuba' else i^j if name=='bitwise_xor' else i|j if name=='ranked_subset' else i&j
                    c[k]=(c[k]+x*y)%p
            data=(f'{n} {m}\n'if name=='karatsuba'else f'{bits}\n')+flat(a)+'\n'+flat(b)+'\n'
            assert list(map(int,run([binary],input=data).split()))==c,name
        elif name=='gf2':
            n,m=1+rng.randrange(6),1+rng.randrange(8);rows=[rng.randrange(1<<m)for _ in range(n)];b=[rng.randrange(2)for _ in range(n)]
            data=f'{n} {m}\n'+'\n'.join(''.join(str(x>>j&1)for j in range(m))for x in rows)+'\n'+''.join(map(str,b))+'\n'
            tokens=run([binary],input=data).split();want={x for x in range(1<<m)if all((x&r).bit_count()%2==y for r,y in zip(rows,b))}
            if not want:assert tokens==['-1'];continue
            count=int(tokens[0]);particular=int(tokens[1][::-1],2);basis=[int(x[::-1],2)for x in tokens[2:]];assert count==len(basis) and len(xor_span(basis))==2**count and {particular^x for x in xor_span(basis)}==want
        else:
            n=rng.randrange(6);a=[[rng.randrange(p)for _ in range(n)]for _ in range(n)]
            if trial%4==0 and n>1:a[-1]=a[0][:]
            tokens=list(map(int,run([binary],input=f'{n}\n'+'\n'.join(flat(row)for row in a)+'\n').split()))
            if name=='characteristic':assert tokens==charpoly(a,p)
            elif det(a)%p==0:assert tokens==[-1]
            else:
                assert len(tokens)==n*n;inverse=[tokens[i*n:(i+1)*n]for i in range(n)]
                assert all(sum(a[i][k]*inverse[k][j]for k in range(n))%p==(i==j)for i in range(n)for j in range(n))

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--compiler',default=os.environ.get('CXX','g++'));parser.add_argument('--sanitize',action='store_true');args=parser.parse_args()
    compiler=shlex.split(args.compiler);entries=json.loads((ROOT/'verify/transform_a/manifest.json').read_text())['algorithms'];headers=[e['header']for e in entries]
    flags=['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',str(ROOT)]
    runtime=['-O2']if not args.sanitize else ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all','-fno-pie','-no-pie']
    for h in headers:
        for extra in [[],['-DNDEBUG']]:run(compiler+flags+extra+['-fsyntax-only','-x','c++','-'],input=f'#include "{h}"\n#include "{h}"\nint main(){{}}\n')
    print(f'{len(headers)} standalone/repeated headers debug/release passed',flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-transform-a-')as d:
        temp=Path(d)
        def compile(source,name,extra=()):
            binary=temp/name;run(compiler+flags+runtime+list(extra)+[source,'-o',binary]);return binary
        includes=''.join(f'#include "{h}"\n'for h in reversed(headers))+''.join(f'#include "{h.relative_to(ROOT)}"\n'for h in sorted(ROOT.glob('*/all.hpp')))
        (temp/'a.cpp').write_text(includes+'int first(){return cp::characteristic_polynomial_mod(cp::Matrix<int>(0,0))[0]-1;}\n');(temp/'b.cpp').write_text(includes+'int first();int main(){return first();}\n')
        run(compiler+flags+runtime+[temp/'a.cpp',temp/'b.cpp','-o',temp/'multi']);run([temp/'multi'])
        for e in entries:
            example=ROOT/'examples'/e['example'];doc=ROOT/'docs'/e['doc'];content=doc.read_text();assert f"documentation_of: //{e['header']}"in content
            assert re.search(r'```cpp\n(.*?)```',content,re.S).group(1)==example.read_text(),doc
            assert run([compile(example,e['header'].replace('/','_'))])==e['output'],example
        print('mixed category TUs, 10 examples and exact documentation usage passed',flush=True)
        for source in sorted((ROOT/'verify/transform_a').glob('*.test.cpp')):judge(compile(source,source.stem),source.name.split('.')[0])
        print('8 judge wrappers x 24 cases passed (local only)',flush=True)
        for name,extra in [('debug',[]),('release',['-DNDEBUG'])]:
            print(name+' '+run([compile(ROOT/'verify/transform_a/unit_test.cpp','unit'+name,extra)]),end='',flush=True)
            count=verify(run,compile(ROOT/'verify/transform_a/query_driver.cpp','probe'+name,extra));print(f'{name}: {count} Python arbitrary-precision / permutation determinants / enumerated spaces and solutions passed',flush=True)
    docs=[ROOT/'docs'/e['doc']for e in entries]+[ROOT/p for p in ['README.md','docs/README.md','docs/transform_a/README.md','docs/transform_a/references.md','visualizations/index.html','visualizations/README.md','visualizations/transform_a/index.html','visualizations/transform_a/README.md']]
    for c in ['convolution','set_function','linear_algebra']:docs.extend([ROOT/f'docs/{c}/README.md',ROOT/f'docs/{c}/references.md'])
    count=0
    for doc in docs:
        content=re.sub(r'```.*?```','',doc.read_text(),flags=re.S)
        for pair in re.findall(r'\]\(([^)]+)\)|href="([^"]+)"',content):
            target=next(x for x in pair if x).split('#')[0]
            if not target or re.match(r'[a-z]+:',target):continue
            assert (doc.parent/unquote(target)).exists(),(doc,target);count+=1
    print(f'{count} local documentation links passed',flush=True)

if __name__=='__main__':main()
