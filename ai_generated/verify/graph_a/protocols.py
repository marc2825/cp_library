"""Independent protocol checks, including non-unique assignments and trails."""
import heapq
import itertools

def verify(run, binary, name, rng):
    n=rng.randrange(1,9)
    edges=[(rng.randrange(n),rng.randrange(n),rng.randrange(7)) for _ in range(18)]
    def execute(lines): return run([binary],input='\n'.join(lines)+'\n')
    def floyd(arcs):
        d=[[None]*n for _ in range(n)]
        for i in range(n): d[i][i]=0
        for u,v,w in arcs: d[u][v]=w if d[u][v] is None else min(d[u][v],w)
        for k in range(n):
            for u in range(n):
                for v in range(n):
                    if d[u][k] is not None and d[k][v] is not None:
                        candidate=d[u][k]+d[k][v]
                        d[u][v]=candidate if d[u][v] is None else min(d[u][v],candidate)
        return d
    if name=='radix_shortest_path':
        s=rng.randrange(n);d=floyd(edges)[s]
        got=execute([f'{n} {len(edges)} {s}']+[' '.join(map(str,e)) for e in edges])
        assert got==''.join(f'{x if x is not None else "INF"}\n' for x in d)
    elif name=='johnson':
        h=[rng.randrange(-8,9) for _ in range(n)]
        edges=[(u,v,w+h[v]-h[u]) for u,v,w in edges]
        if rng.randrange(3)==0: edges.append((0,0,-1))
        d=floyd(edges);got=execute([f'{n} {len(edges)}']+[' '.join(map(str,e)) for e in edges])
        expected='NEGATIVE CYCLE\n' if any(d[i][i]<0 for i in range(n)) else ''.join(' '.join(str(x) if x is not None else 'INF' for x in row)+'\n' for row in d)
        assert got==expected
    elif name=='k_shortest_walk':
        s,t=rng.randrange(n),rng.randrange(n);k=12;queue=[(0,s)];popped=[0]*n;answers=[]
        while queue and len(answers)<k:
            cost,v=heapq.heappop(queue)
            if popped[v]>=k: continue
            popped[v]+=1
            if v==t: answers.append(cost)
            for u,w,c in edges:
                if u==v and popped[w]<k: heapq.heappush(queue,(cost+c,w))
        got=execute([f'{n} {len(edges)} {s} {t} {k}']+[' '.join(map(str,e)) for e in edges])
        assert list(map(int,got.split()))==answers+[-1]*(k-len(answers))
    elif name=='two_sat':
        clauses=[((1 if rng.randrange(2) else -1)*rng.randrange(1,n+1),(1 if rng.randrange(2) else -1)*rng.randrange(1,n+1)) for _ in range(12)]
        def valid(a): return all(a[abs(x)-1]==(x>0) or a[abs(y)-1]==(y>0) for x,y in clauses)
        exists=any(valid(a) for a in itertools.product([False,True],repeat=n))
        got=execute([f'p cnf {n} {len(clauses)}']+[f'{a} {b} 0' for a,b in clauses]).split()
        if not exists: assert got==['s','UNSATISFIABLE']
        else:
            assert got[:3]==['s','SATISFIABLE','v'] and got[-1]=='0'
            literals=list(map(int,got[3:-1]));assert len(literals)==n and [abs(x) for x in literals]==list(range(1,n+1))
            assert valid([x>0 for x in literals])
    elif name=='eulerian_trail_directed':
        edges=[(rng.randrange(n),rng.randrange(n)) for _ in range(rng.randrange(8))]
        exists=not edges or any(all(edges[p[i]][1]==edges[p[i+1]][0] for i in range(len(p)-1)) for p in itertools.permutations(range(len(edges))))
        got=execute(['1',f'{n} {len(edges)}']+[f'{u} {v}' for u,v in edges]).splitlines()
        if not exists: assert got==['No']
        else:
            assert got[0]=='Yes';vertices=list(map(int,got[1].split()));ids=list(map(int,got[2].split()))
            assert sorted(ids)==list(range(len(edges))) and len(vertices)==len(ids)+1
            assert all(edges[e]==(vertices[i],vertices[i+1]) for i,e in enumerate(ids))
    elif name=='rooted_tree_isomorphism':
        n=80;parents=[rng.randrange(v) for v in range(1,n)];children=[[] for _ in range(n)]
        for v,p in enumerate(parents,1): children[p].append(v)
        def shape(v): return '('+''.join(sorted(shape(u) for u in children[v]))+')'
        shapes=[shape(v) for v in range(n)]
        got=list(map(int,execute([str(n),' '.join(map(str,parents))]).split()));assert got[0]==len(set(shapes)) and len(got)==n+1
        assert set(got[1:])==set(range(got[0]))
        for u in range(n):
            for v in range(n): assert (got[u+1]==got[v+1])==(shapes[u]==shapes[v])
    else: raise ValueError(name)
