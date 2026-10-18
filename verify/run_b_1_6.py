#!/usr/bin/env python3
"""3.1--3.6 B: public C++17 entries, examples, independent differential tests."""
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

ROOT=Path(__file__).resolve().parents[1]

def run(command,**kwargs):
    result=subprocess.run(list(map(str,command)),text=True,capture_output=True,timeout=240,**kwargs)
    if result.returncode:
        raise RuntimeError(f'{shlex.join(map(str,command))}\n{result.stdout}{result.stderr}')
    return result.stdout

def judge(binary,name):
    rng=random.Random(3162026)
    mod=998244353
    for trial in range(24):
        if name=='integer_root':
            cases=[(rng.getrandbits(64),rng.randrange(1,65)) for _ in range(60)]
            cases.extend([(0,1),((1<<64)-1,1),((1<<64)-1,2),((1<<64)-1,64)])
            expected=[]
            for n,k in cases:
                l,r=0,n
                while l<r:
                    m=(l+r+1)//2
                    if m**k<=n:l=m
                    else:r=m-1
                expected.append(l)
            data=str(len(cases))+'\n'+'\n'.join(f'{n} {k}' for n,k in cases)+'\n'
        elif name=='rectangle_add':
            rectangles=[]
            for i in range(30):
                l=rng.randrange(10**9);d=rng.randrange(10**9)
                rectangles.append((l,d,l+rng.randrange(1,10000),d+rng.randrange(1,10000),rng.randrange(mod)))
            queries=[]
            for i in range(120):
                base=rectangles[rng.randrange(len(rectangles))]
                queries.append((max(0,base[0]-rng.randrange(10000)),max(0,base[1]-rng.randrange(10000)),base[2]+rng.randrange(10000),base[3]+rng.randrange(10000)))
            expected=[sum(max(0,min(r,c)-max(l,a))*max(0,min(u,e)-max(d,b))*w for a,b,c,e,w in rectangles)%mod for l,d,r,u in queries]
            data=f'{len(rectangles)} {len(queries)}\n'+'\n'.join(' '.join(map(str,row)) for row in rectangles+queries)+'\n'
        elif name=='distinct':
            n=1+rng.randrange(80);a=[rng.randrange(11) for _ in range(n)];queries=[]
            for i in range(120):
                l=rng.randrange(n);r=rng.randrange(l+1,n+1);queries.append((l,r))
            expected=[len(set(a[l:r])) for l,r in queries]
            data=f'{n} {len(queries)}\n'+' '.join(map(str,a))+'\n'+'\n'.join(f'{l} {r}' for l,r in queries)+'\n'
        else:
            n=1+rng.randrange(60);keys=rng.sample(range(10**9),n+120);items=[(keys[i],rng.randrange(1,mod),rng.randrange(mod)) for i in range(n)]
            data=f'{n} 120\n'+'\n'.join(' '.join(map(str,item)) for item in items)+'\n';expected=[];next_key=n
            for i in range(120):
                type=rng.randrange(4)
                if type==0:
                    p=rng.randrange(n);key=keys[next_key];next_key+=1;a=rng.randrange(1,mod);b=rng.randrange(mod);items[p]=(key,a,b);data+=f'0 {p} {key} {a} {b}\n'
                else:
                    l=rng.randrange(n);r=rng.randrange(l+1,n+1)
                    if type==1:
                        x=rng.randrange(mod);data+=f'1 {l} {r} {x}\n'
                        for _,a,b in items[l:r]:x=(a*x+b)%mod
                        expected.append(x)
                    else:
                        data+=f'{type} {l} {r}\n';items[l:r]=sorted(items[l:r],reverse=type==3)
        assert list(map(int,run([binary],input=data).split()))==expected,(name,trial)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler',default=os.environ.get('CXX','g++'))
    parser.add_argument('--sanitize',action='store_true')
    args=parser.parse_args()
    compiler=shlex.split(args.compiler)
    entries=json.loads((ROOT/'verify/b_1_6/manifest.json').read_text())['algorithms']
    headers=[entry['header'] for entry in entries]
    flags=['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',str(ROOT)]
    runtime=['-O2'] if not args.sanitize else ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all','-fno-pie','-no-pie']
    for h in headers:
        for release in [[],['-DNDEBUG']]:
            run(compiler+flags+release+['-fsyntax-only','-x','c++','-'],input=f'#include "{h}"\n#include "{h}"\nint main(){{}}\n')
    print(f'{len(headers)} standalone/repeated entries debug/release passed',flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-b-1-6-') as folder:
        temporary=Path(folder)
        def compile(source,name,extra=()):
            binary=temporary/name
            run(compiler+flags+runtime+list(extra)+[source,'-o',binary])
            return binary
        includes=''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes+=''.join(f'#include "{h.relative_to(ROOT)}"\n' for h in sorted(ROOT.glob('*/all.hpp')))
        (temporary/'a.cpp').write_text(includes+'int first(){return int(cp::integer_kth_root(27,3))-3;}\n')
        (temporary/'b.cpp').write_text(includes+'int first();int main(){return first();}\n')
        run(compiler+flags+runtime+[temporary/'a.cpp',temporary/'b.cpp','-o',temporary/'multi'])
        run([temporary/'multi'])
        for entry in entries:
            source=ROOT/'examples'/entry['example']
            content=(ROOT/'docs'/entry['doc']).read_text()
            assert f"documentation_of: //{entry['header']}" in content
            assert re.search(r'```cpp\n(.*?)```',content,re.S).group(1)==source.read_text()
            assert run([compile(source,entry['header'].replace('/','_'))])==entry['output'],entry['header']
        print('all-category mixed translation units, 43 examples and exact documentation usage passed',flush=True)
        for label,extra in [('debug',[]),('release',['-DNDEBUG'])]:
            for source in sorted((ROOT/'verify/b_1_6').glob('*_test.cpp')):
                print(label+': '+run([compile(source,label+source.stem,extra)]),end='',flush=True)
        for source in sorted((ROOT/'verify/b_1_6').glob('*.test.cpp')):
            judge(compile(source,source.stem),source.name.split('.')[0])
        print('4 judge protocols x 24 Python-oracle batches passed (local only)',flush=True)
    docs=[ROOT/'docs'/entry['doc'] for entry in entries]
    docs.extend(ROOT/path for path in ['README.md','HANDOFF.md','docs/README.md','docs/implementation_plan.md','docs/b_1_6/README.md','docs/b_1_6/references.md','visualizations/index.html','visualizations/README.md','visualizations/b_1_6/index.html','visualizations/b_1_6/README.md'])
    docs.extend(ROOT/f'docs/{category}/README.md' for category in ['basic_utility','union_find','range_query','static_query','ordered_structure','offline_query'])
    count=0
    for doc in docs:
        content=re.sub(r'```.*?```','',doc.read_text(),flags=re.S)
        for pair in re.findall(r'\]\(([^)]+)\)|href="([^"]+)"',content):
            target=next(x for x in pair if x).split('#')[0]
            if not target or re.match(r'[a-z]+:',target):
                continue
            # Python移植先は今後作成する計画なので、現時点のリンクにはしない。
            assert (doc.parent/unquote(target)).exists(),(doc,target)
            count+=1
    print(f'{count} local documentation links passed',flush=True)

if __name__=='__main__':
    main()
