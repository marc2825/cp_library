#!/usr/bin/env python3
"""3.7: standalone headers, examples, judge protocols and independent graph oracles."""
import argparse
from itertools import combinations
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
INF = 10**15


def run(command, **kwargs):
    r = subprocess.run(command, text=True, capture_output=True, **kwargs)
    if r.returncode:
        raise RuntimeError(f"{shlex.join(map(str,command))}\n{r.stdout}{r.stderr}")
    return r.stdout


def distances(n, edges):
    d = [[INF] * n for _ in range(n)]
    for i in range(n): d[i][i] = 0
    for u, v, w in edges: d[u][v] = min(d[u][v], w)
    for k in range(n):
        for u in range(n):
            for v in range(n):
                if d[u][k] < INF and d[k][v] < INF: d[u][v] = min(d[u][v], d[u][k]+d[k][v])
    return d


def verify_protocol(name, binary):
    rng = random.Random(37002)
    if name == "zero_one_bfs":
        for k in range(2, 36):
            d = [INF]*k; d[1] = 0
            for _ in range(k):
                nxt = d.copy()
                for i in range(k):
                    nxt[i*10 % k] = min(nxt[i*10 % k], d[i])
                    nxt[(i+1) % k] = min(nxt[(i+1) % k], d[i]+1)
                d = nxt
            assert run([str(binary)], input=f"{k}\n") == f"{d[0]+1}\n"
        return
    for trial in range(24):
        n = rng.randrange(2, 8)
        edges = []
        if name in ("kruskal", "prim"):
            edges = [(i-1, i, rng.randrange(11)) for i in range(1,n)]
            edges += [(rng.randrange(n), rng.randrange(n), rng.randrange(11)) for _ in range(9-len(edges))]
        else:
            pairs = [(u,v) for u in range(n) for v in range(n) if u!=v and (name!="topological_sort" or u<v)]
            rng.shuffle(pairs)
            for u,v in pairs[:rng.randrange(min(len(pairs),12)+1)]:
                edges.append((u,v,rng.randrange(-4,8) if name in ("bellman_ford","floyd_warshall") else rng.randrange(8)))
        s, t = rng.sample(range(n),2)
        if name in ("bfs","dfs"):
            adj = [[v for u,v,_ in edges if u==i] for i in range(n)]
            data = f"{n}\n"+"".join(f"{i+1} {len(a)} "+" ".join(str(v+1) for v in a)+"\n" for i,a in enumerate(adj))
            if name=="bfs":
                expected = [-1]*n;expected[0]=0;queue=[0]
                for u in queue:
                    for v in adj[u]:
                        if expected[v]<0:expected[v]=expected[u]+1;queue.append(v)
                want="".join(f"{i+1} {d}\n" for i,d in enumerate(expected))
            else:
                enter=[0]*n;leave=[0]*n;clock=0
                def dfs(v):
                    nonlocal clock
                    clock+=1;enter[v]=clock
                    for to in adj[v]:
                        if not enter[to]:dfs(to)
                    clock+=1;leave[v]=clock
                for i in range(n):
                    if not enter[i]:dfs(i)
                want="".join(f"{i+1} {enter[i]} {leave[i]}\n" for i in range(n))
            assert run([str(binary)],input=data)==want,name
            continue
        header=f"{n} {len(edges)}"+(f" {s} {t}" if name=="shortest_path" else f" {s}" if name=="bellman_ford" else "")+"\n"
        weighted=name not in ("topological_sort","cycle_detection")
        data=header+"".join(f"{u} {v}"+(f" {w}" if weighted else "")+"\n" for u,v,w in edges)
        output=run([str(binary)],input=data)
        if name in ("kruskal","prim"):
            def connected(ids):
                adj=[[] for _ in range(n)]
                for i in ids:u,v,_=edges[i];adj[u].append(v);adj[v].append(u)
                seen={0};queue=[0]
                for u in queue:
                    for v in adj[u]:
                        if v not in seen:seen.add(v);queue.append(v)
                return len(seen)==n
            best=min(sum(edges[i][2] for i in ids) for ids in combinations(range(len(edges)),n-1) if connected(ids))
            cost,*ids=map(int,output.split())
            assert cost==best and len(ids)==n-1 and len(set(ids))==n-1 and connected(ids)
            assert sum(edges[i][2] for i in ids)==cost
        elif name=="topological_sort":
            order=list(map(int,output.split()));assert sorted(order)==list(range(n))
            pos={v:i for i,v in enumerate(order)};assert all(pos[u]<pos[v] for u,v,_ in edges)
        elif name=="cycle_detection":
            reach=distances(n,[(u,v,1) for u,v,_ in edges])
            cyclic=any(reach[v][u]<INF for u,v,_ in edges)
            ids=list(map(int,output.split()))
            if not cyclic:assert ids==[-1]
            else:
                length,*ids=ids;assert length==len(ids) and len(ids)==len(set(ids)) and length>0
                assert all(edges[ids[i]][1]==edges[ids[(i+1)%length]][0] for i in range(length))
        else:
            d=distances(n,edges)
            negative=any(d[s][k]<INF and d[k][k]<0 for k in range(n)) if name=="bellman_ford" else any(d[k][k]<0 for k in range(n))
            if name=="shortest_path":
                if d[s][t]==INF:assert output=="-1\n"
                else:
                    lines=output.splitlines();cost,length=map(int,lines[0].split());pairs=[tuple(map(int,line.split())) for line in lines[1:]]
                    assert cost==d[s][t] and length==len(pairs) and pairs[0][0]==s and pairs[-1][1]==t
                    assert all(pairs[i][1]==pairs[i+1][0] for i in range(length-1))
                    weights={(u,v):w for u,v,w in edges};assert sum(weights[p] for p in pairs)==cost
                    assert len(set([s]+[v for _,v in pairs]))==length+1
            elif negative:assert output=="NEGATIVE CYCLE\n"
            else:
                rows=[d[s]] if name=="bellman_ford" else d
                expected="\n".join(("\n" if name=="bellman_ford" else " ").join("INF" if x==INF else str(x) for x in row) for row in rows)+"\n"
                assert output==expected,name


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler",default=os.environ.get("CXX","g++"))
    parser.add_argument("--sanitize",action="store_true")
    args=parser.parse_args();compiler=shlex.split(args.compiler)
    flags=["-std=c++17","-Wall","-Wextra","-Wpedantic","-Werror","-I",str(ROOT)]
    runtime=["-O2","-g"] if not args.sanitize else ["-O1","-g","-fsanitize=address,undefined","-fno-omit-frame-pointer","-fno-sanitize-recover=all","-fno-pie","-no-pie"]
    headers=sorted(str(p.relative_to(ROOT)) for p in (ROOT/"graph_basic").glob("*.hpp"))
    for header in headers:
        for extra in [[],["-DNDEBUG"]]:
            run(compiler+flags+extra+["-fsyntax-only","-x","c++","-"],input=f'#include "{header}"\n#include "{header}"\nint main() {{}}\n')
    print(f"standalone headers (debug/release): {len(headers)} passed",flush=True)
    with tempfile.TemporaryDirectory(prefix="cp-graph-basic-") as directory:
        temp=Path(directory)
        def compile(source,suffix="",extra=()):
            binary=temp/(source.stem+suffix)
            run(compiler+flags+list(extra)+runtime+[str(source),"-o",str(binary)])
            return binary
        includes=''.join(f'#include "{h}"\n' for h in reversed(headers))+'#include "graph/all.hpp"\n#include "tree/all.hpp"\n'
        (temp/"first.cpp").write_text(includes+'int first() { return cp::dfs_forest(cp::Graph<>(0)).roots.size(); }\n')
        (temp/"second.cpp").write_text(includes+'int first();int main(){return first();}\n')
        run(compiler+flags+runtime+[str(temp/"first.cpp"),str(temp/"second.cpp"),"-o",str(temp/"multi")]);run([str(temp/"multi")])
        print("reverse includes / existing graph+tree / multiple translation units: passed",flush=True)
        for name,want in [("traversal","1\n0 1 2 3 4 \n2 1 0 4 3 \n"),("shortest_paths","1 1 1 1\n0 2 1 3 \n1 2 3 \n"),("order_and_cycle","0 1 2 \n0\n0 1 2 \n"),("spanning_forest","2 2 2\n3 1 2 \n")]:
            assert run([str(compile(ROOT/f"examples/graph_basic/{name}.cpp"))])==want,name
        for source in sorted((ROOT/"verify/graph_basic").glob("*.test.cpp")):
            verify_protocol(source.name.removesuffix(".test.cpp"),compile(source))
        print("4 examples; 10 judge wrappers / 250 independent Python protocol cases passed",flush=True)
        for suffix,extra in [("_debug",[]),("_release",["-DNDEBUG"])]:
            print(suffix[1:]+":\n"+run([str(compile(ROOT/"verify/graph_basic/unit_test.cpp",suffix,extra))]),end="",flush=True)


if __name__=="__main__":main()
