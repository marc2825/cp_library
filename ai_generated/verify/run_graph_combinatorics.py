#!/usr/bin/env python3
"""3.10: graph enumeration, exact optimization/coloring and matrix-tree counts."""
import argparse
import itertools
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[1]
MOD=998244353

def run(command,**kwargs):
    result=subprocess.run(command,text=True,capture_output=True,**kwargs)
    if result.returncode:raise RuntimeError(f"{shlex.join(map(str,command))}\n{result.stdout}{result.stderr}")
    return result.stdout

def subsets(n,edges,independent=False):
    adjacent={frozenset(e) for e in edges}
    return [list(v for v in range(n) if mask>>v&1) for mask in range(1<<n)
            if all((frozenset((u,v)) in adjacent)!=independent for u,v in itertools.combinations([v for v in range(n) if mask>>v&1],2))]

def coloring(n,edges):
    if not n:return 0
    prior=[[] for _ in range(n)]
    for u,v in edges:prior[max(u,v)].append(min(u,v))
    assigned=[-1]*n
    def search(v,k):
        if v==n:return True
        for c in range(k):
            if all(assigned[u]!=c for u in prior[v]):
                assigned[v]=c
                if search(v+1,k):return True
        assigned[v]=-1
        return False
    return next(k for k in range(1,n+1) if search(0,k))

def trees(n,edges):
    if not n:return 0
    total=0
    for selected in itertools.combinations(edges,n-1):
        reached={0}
        for _ in range(n):
            for u,v in selected:
                if u in reached or v in reached:reached.update((u,v))
        total+=len(reached)==n
    return total%MOD

def verify_protocol(name,binary):
    rng=random.Random(310010)
    for trial in range(48):
        n=rng.randrange(1 if name in ('chromatic_number','maximum_clique','counting_spanning_tree_undirected') else 2,9)
        candidates=list(itertools.combinations(range(n),2));rng.shuffle(candidates)
        edges=candidates[:rng.randrange(int(name.startswith('enumerate_')),len(candidates)+1)]
        if name=='counting_spanning_tree_undirected':edges=[(rng.randrange(n),rng.randrange(n)) for _ in range(rng.randrange(13))]
        data=f"{n} {len(edges)}\n"
        x=[rng.randrange(MOD) for _ in range(n)]
        if name.startswith('enumerate_'):data+=' '.join(map(str,x))+'\n'
        offset=int(name=='maximum_clique');data+=''.join(f"{u+offset} {v+offset}\n" for u,v in edges)
        output=run([str(binary)],input=data)
        if name.startswith('enumerate_'):
            choices=subsets(n,edges)
            if name=='enumerate_triangles':choices=[vs for vs in choices if len(vs)==3]
            else:choices=[vs for vs in choices if vs]
            total=0
            for vs in choices:
                product=1
                for v in vs:product=product*x[v]%MOD
                total=(total+product)%MOD
            assert output==f"{total}\n"
        elif name=='maximum_clique':assert output==f"{max(map(len,subsets(n,edges)))}\n"
        elif name=='maximum_independent_set':
            values=list(map(int,output.split()));size,*vertices=values
            assert size==len(vertices)==len(set(vertices))==max(map(len,subsets(n,edges,True)))
            assert all(0<=v<n for v in vertices) and not any(u in vertices and v in vertices for u,v in edges)
        elif name=='chromatic_number':assert output==f"{coloring(n,edges)}\n"
        else:assert output==f"{trees(n,edges)}\n"

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--compiler',default=os.environ.get('CXX','g++'));parser.add_argument('--sanitize',action='store_true');args=parser.parse_args()
    compiler=shlex.split(args.compiler);flags=['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',str(ROOT)]
    runtime=['-O2','-g'] if not args.sanitize else ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all','-fno-pie','-no-pie']
    headers=sorted(str(p.relative_to(ROOT)) for p in (ROOT/'graph_combinatorics').rglob('*.hpp'))
    for h in headers:
        for extra in [[],['-DNDEBUG']]:run(compiler+flags+extra+['-fsyntax-only','-x','c++','-'],input=f'#include "{h}"\n#include "{h}"\nint main(){{}}\n')
    print(f'standalone headers (debug/release): {len(headers)} passed',flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-graph-combinatorics-') as directory:
        temp=Path(directory)
        def compile(source,suffix='',extra=()):
            binary=temp/(source.stem+suffix);run(compiler+flags+list(extra)+runtime+[str(source),'-o',str(binary)]);return binary
        includes=''.join(f'#include "{h}"\n' for h in reversed(headers))+'#include "graph/all.hpp"\n#include "graph_flow/all.hpp"\n#include "tree/all.hpp"\n'
        (temp/'first.cpp').write_text(includes+'int first(){return cp::chromatic_number(cp::Graph<>(0));}\n')
        (temp/'second.cpp').write_text(includes+'int first();int main(){return first();}\n')
        run(compiler+flags+runtime+[str(temp/'first.cpp'),str(temp/'second.cpp'),'-o',str(temp/'multi')]);run([str(temp/'multi')])
        print('reverse includes / existing categories / multiple translation units: passed',flush=True)
        for name,want in [('enumeration','0 1 2\n9\n0 1 2 \n2 3 \n'),('optimization','2 2\n'),('matrix_tree','3\n31\n1\n')]:assert run([str(compile(ROOT/f'examples/graph_combinatorics/{name}.cpp'))])==want,name
        values=list(map(int,run([str(compile(ROOT/'examples/graph_combinatorics/coloring.cpp'))]).split()));assert values[0]==3 and len(values)==6 and all(values[v+1]!=values[(v+1)%5+1] for v in range(5))
        for source in sorted((ROOT/'verify/graph_combinatorics').glob('*.test.cpp')):verify_protocol(source.name.removesuffix('.test.cpp'),compile(source))
        print('4 examples; 6 judge wrappers / 288 independent protocol cases passed',flush=True)
        for suffix,extra in [('_debug',[]),('_release',['-DNDEBUG'])]:print(suffix[1:]+':\n'+run([str(compile(ROOT/'verify/graph_combinatorics/unit_test.cpp',suffix,extra))]),end='',flush=True)
if __name__=='__main__':main()
