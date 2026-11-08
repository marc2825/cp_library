#!/usr/bin/env python3
"""3.8: SCC/condensation, Lowlink, bridges/cuts and bipartite coloring."""
import argparse
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]


def run(command, **kwargs):
    result=subprocess.run(command,text=True,capture_output=True,**kwargs)
    if result.returncode:raise RuntimeError(f"{shlex.join(map(str,command))}\n{result.stdout}{result.stderr}")
    return result.stdout


def reachability(n,edges):
    adj=[[] for _ in range(n)]
    for u,v in edges:adj[u].append(v)
    reach=[]
    for s in range(n):
        seen={s};queue=[s]
        for u in queue:
            for v in adj[u]:
                if v not in seen:seen.add(v);queue.append(v)
        reach.append(seen)
    return reach


def components(n,edges,vertex=-1,edge=-1):
    adj=[[] for _ in range(n)]
    for i,(u,v) in enumerate(edges):
        if i==edge or u==vertex or v==vertex:continue
        adj[u].append(v);adj[v].append(u)
    seen={vertex};groups=0
    for s in range(n):
        if s in seen:continue
        groups+=1;queue=[s];seen.add(s)
        for u in queue:
            for v in adj[u]:
                if v not in seen:seen.add(v);queue.append(v)
    return groups


def bipartite(n,edges):
    return any(all(((mask>>u)&1)!=((mask>>v)&1) for u,v in edges) for mask in range(1<<n))


def verify_protocol(name,binary):
    rng=random.Random(38002)
    for trial in range(48):
        n=rng.randrange(1,9)
        if name.startswith("scc"):
            edges=[(rng.randrange(n),rng.randrange(n)) for _ in range(rng.randrange(20))]
        else:
            pairs=[(u,v) for u in range(n) for v in range(u+1,n)];rng.shuffle(pairs)
            edges=pairs[:rng.randrange(len(pairs)+1)]
        data=f"{n} {len(edges)}\n"+''.join(f"{u+(name=='bipartite')} {v+(name=='bipartite')}\n" for u,v in edges)
        if name=="scc_queries":
            queries=[(rng.randrange(n),rng.randrange(n)) for _ in range(64)]
            data+="64\n"+''.join(f"{u} {v}\n" for u,v in queries)
        output=run([str(binary)],input=data)
        if name=="scc":
            reach=reachability(n,edges);lines=output.splitlines();count=int(lines[0]);assert len(lines)==count+1
            groups=[];component=[-1]*n
            for c,line in enumerate(lines[1:]):
                size,*group=map(int,line.split());assert size==len(group)>0
                for v in group:assert component[v]<0;component[v]=c
                groups.append(group)
            assert all(x>=0 for x in component)
            for u in range(n):
                for v in range(n):assert (component[u]==component[v])==(v in reach[u] and u in reach[v])
            assert all(component[u]<=component[v] for u,v in edges)
        elif name=="scc_queries":
            reach=reachability(n,edges)
            assert output==''.join(f"{int(v in reach[u] and u in reach[v])}\n" for u,v in queries)
        elif name in ("bridges","articulation_points"):
            base=components(n,edges)
            if name=="bridges":
                expected=sorted(edges[i] for i in range(len(edges)) if components(n,edges,edge=i)>base)
                assert output==''.join(f"{u} {v}\n" for u,v in expected)
            else:
                assert output==''.join(f"{v}\n" for v in range(n) if components(n,edges,vertex=v)>base)
        else:
            existing=set(edges)
            expected=sum(bipartite(n,edges+[(u,v)]) for u in range(n) for v in range(u+1,n) if (u,v) not in existing)
            assert output==f"{expected}\n"


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler",default=os.environ.get("CXX","g++"))
    parser.add_argument("--sanitize",action="store_true")
    args=parser.parse_args();compiler=shlex.split(args.compiler)
    flags=["-std=c++17","-Wall","-Wextra","-Wpedantic","-Werror","-I",str(ROOT)]
    runtime=["-O2","-g"] if not args.sanitize else ["-O1","-g","-fsanitize=address,undefined","-fno-omit-frame-pointer","-fno-sanitize-recover=all","-fno-pie","-no-pie"]
    headers=sorted(str(p.relative_to(ROOT)) for p in (ROOT/"graph_connectivity").glob("*.hpp"))
    for header in headers:
        for extra in [[],["-DNDEBUG"]]:
            run(compiler+flags+extra+["-fsyntax-only","-x","c++","-"],input=f'#include "{header}"\n#include "{header}"\nint main() {{}}\n')
    print(f"standalone headers (debug/release): {len(headers)} passed",flush=True)
    with tempfile.TemporaryDirectory(prefix="cp-graph-connectivity-") as directory:
        temp=Path(directory)
        def compile(source,suffix="",extra=()):
            binary=temp/(source.stem+suffix);run(compiler+flags+list(extra)+runtime+[str(source),"-o",str(binary)]);return binary
        includes=''.join(f'#include "{h}"\n' for h in reversed(headers))+'#include "graph/all.hpp"\n#include "graph_basic/all.hpp"\n#include "tree/all.hpp"\n'
        (temp/"first.cpp").write_text(includes+'int first(){return cp::scc(cp::Graph<>(0,cp::GraphKind::Directed)).size();}\n')
        (temp/"second.cpp").write_text(includes+'int first();int main(){return first();}\n')
        run(compiler+flags+runtime+[str(temp/"first.cpp"),str(temp/"second.cpp"),"-o",str(temp/"multi")]);run([str(temp/"multi")])
        print("reverse includes / existing graph+tree / multiple translation units: passed",flush=True)
        for name,want in [("scc","3 1 0\n0 1 \n2 3 \n4 \n"),("condensation","2 1\n0 1 2\n2\n"),("lowlink","3 \n1 3 \n"),("components_and_bipartite","2 1\n0 1 0 0 1 \n0\n")]:
            assert run([str(compile(ROOT/f"examples/graph_connectivity/{name}.cpp"))])==want,name
        for source in sorted((ROOT/"verify/graph_connectivity").glob("*.test.cpp")):
            verify_protocol(source.name.removesuffix(".test.cpp"),compile(source))
        print("4 examples; 5 judge wrappers / 240 independent Python protocol cases passed",flush=True)
        for suffix,extra in [("_debug",[]),("_release",["-DNDEBUG"])]:
            print(suffix[1:]+":\n"+run([str(compile(ROOT/"verify/graph_connectivity/unit_test.cpp",suffix,extra))]),end="",flush=True)


if __name__=="__main__":main()
