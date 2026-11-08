#!/usr/bin/env python3
"""3.23: exhaustive injections, vertex subsets and integral flows; no external packages."""
import argparse
from functools import lru_cache
from itertools import permutations, product
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[1]
EXAMPLES={'hungarian':'5\n1 0 2 \n','assignment_problem':'9\n15\n0\n','general_matching':'2\n0 1\n2 3\n','bipartite_matching':'2\n0 1\n1 0\n','min_cost_flow':'1 2\n2 7\n0:0 2:4 3:9 \n'}
def run(command,**kwargs):
 r=subprocess.run(command,text=True,capture_output=True,**kwargs)
 if r.returncode:raise RuntimeError(f'{shlex.join(map(str,command))}\n{r.stdout}{r.stderr}')
 return r.stdout

def assignment(a,m,maximum=False):
 values=[sum(a[i][j]for i,j in enumerate(p))for p in permutations(range(m),len(a))if all(a[i][j]is not None for i,j in enumerate(p))]
 return (max if maximum else min)(values)if values else None

def matching(n,edges):
 adj=[set()for _ in range(n)]
 for a,b in edges:
  if a!=b:adj[a].add(b);adj[b].add(a)
 @lru_cache(None)
 def solve(mask):
  if not mask:return 0
  low=mask&-mask;u=low.bit_length()-1;rest=mask^low
  return max([solve(rest)]+[1+solve(rest^(1<<v))for v in adj[u]if rest>>v&1])
 return solve((1<<n)-1)

def integral_flows(n,edges):
 best={}
 for amounts in product(*(range(c+1)for u,v,c,w in edges)):
  balance=[0]*n
  for x,(u,v,c,w)in zip(amounts,edges):balance[u]+=x;balance[v]-=x
  f=balance[0]
  if f<0 or balance[-1]!=-f or any(balance[1:-1]):continue
  cost=sum(x*w for x,(_,_,_,w)in zip(amounts,edges))
  best[f]=min(best.get(f,cost),cost)
 return best

def certificate(a,m,maximum,line):
 best=assignment(a,m,maximum)
 if best is None:assert line=='NONE';return
 if not -2**63<=best<2**63:assert line=='OVERFLOW';return
 parts=[list(map(int,p.split()))for p in line.split('|')];assert len(parts)==3,line
 (cost,*cols),u,v=parts;assert cost==best and len(cols)==len(a)and len(set(cols))==len(a)and len(v)==m and len(u)==len(a)
 for i,j in enumerate(cols):
  assert 0<=j<m and a[i][j]is not None and u[i]+v[j]==a[i][j]
  for k,x in enumerate(a[i]):
   if x is not None:assert u[i]+v[k]>=x if maximum else u[i]+v[k]<=x
 assert cost==sum(u)+sum(v)
 for j,x in enumerate(v):assert (x>=0 if maximum else x<=0)and (j in cols or x==0)

def arbitrary_precision(binary):
 rng=random.Random(323023);cases=[]
 def add_assignment(a,m,maximum):cases.append(('A '+f'{len(a)} {m} {int(maximum)} '+' '.join('x'if x is None else str(x)for row in a for x in row),('A',a,m,maximum)))
 for _ in range(280):
  n,m=rng.randrange(6),rng.randrange(7);a=[[None if rng.randrange(4)==0 else rng.randrange(-10**12,10**12)for _ in range(m)]for _ in range(n)]
  for maximum in [False,True]:add_assignment(a,m,maximum)
 for a in [[[2**63-1]], [[-2**63]], [[2**63-1]*2,[-2**63]*2], [[2**63-1]*2]*2, [[-2**63]*2]*2, [[-2**63,2**63-1],[2**63-1,-2**63]], [[2**63-1,-2**63],[-2**63,2**63-1]]]:
  for maximum in [False,True]:add_assignment(a,len(a[0]),maximum)
 for _ in range(320):
  n=rng.randrange(13);edges=[(rng.randrange(n),rng.randrange(n))for _ in range(40)]if n else []
  cases.append(('M '+f'{n} {len(edges)} '+' '.join(f'{a} {b}'for a,b in edges),('M',n,edges)))
 for _ in range(200):
  n=rng.randrange(2,6);p=[rng.randrange(-20,21)for _ in range(n)];edges=[(u,v,rng.randrange(3),rng.randrange(8)+p[v]-p[u])for u,v in [(rng.randrange(n),rng.randrange(n))for _ in range(7)]];limits=[rng.randrange(4),rng.randrange(4),rng.randrange(4)]
  cases.append(('F '+f'{n} {len(edges)} 3 '+' '.join(' '.join(map(str,e))for e in edges)+' '+' '.join(map(str,limits)),('F',n,edges,limits)))
 for cost in [-2**63,2**63-1]:cases.append((f'F 2 1 1 0 1 1 {cost} 1',('F',2,[(0,1,1,cost)],[1])))
 for command,want in [('F 2 1 1 0 1 2 9223372036854775807 2','OVERFLOW'),('F 4 3 1 0 3 1 0 1 2 1 -2 2 1 1 1 1','NEGATIVE_CYCLE'),('F 2 2 1 0 0 1 -1 0 1 1 0 1','NEGATIVE_CYCLE')]:cases.append((command,('E',want)))
 actual=run([str(binary)],input='\n'.join(c[0]for c in cases)+'\n').splitlines();assert len(actual)==len(cases),(len(actual),len(cases))
 for index,(line,(_,spec))in enumerate(zip(actual,cases)):
  kind,*rest=spec
  try:
   if kind=='A':certificate(*rest,line)
   elif kind=='M':
    n,edges=rest;count,*mate=map(int,line.split());assert count==matching(n,edges)and len(mate)==n
    for u,v in enumerate(mate):assert v==-1 or 0<=v<n and v!=u and mate[v]==u and ((u,v)in edges or (v,u)in edges)
    assert sum(v!=-1 for v in mate)==2*count
   elif kind=='F':
    n,edges,limits=rest;truth=integral_flows(n,edges);maximum=max(truth);parts=[list(map(int,p.split()))for p in line.split('|')];calls,amounts,slope=parts;assert len(calls)==2*len(limits)and len(amounts)==len(edges);sent=0;cost=0
    for limit,(f,c)in zip(limits,zip(calls[::2],calls[1::2])):
     take=min(limit,maximum-sent);assert f==take and c==truth[sent+take]-truth[sent];sent+=f;cost+=c
    balance=[0]*n
    for x,(u,v,cap,c)in zip(amounts,edges):assert 0<=x<=cap;balance[u]+=x;balance[v]-=x
    assert balance==[sent]+[0]*(n-2)+[-sent]and sum(x*e[3]for x,e in zip(amounts,edges))==cost
    points=list(zip(slope[::2],slope[1::2]));assert points[0]==(0,0)and points[-1]==(maximum,truth[maximum])
    for (a,b),(c,d)in zip(points,points[1:]):
     assert a<c
     for f in range(a,c+1):assert (truth[f]-b)*(c-a)==(d-b)*(f-a)
    gradients=[(d-b)//(c-a)for (a,b),(c,d)in zip(points,points[1:])];assert all(x<y for x,y in zip(gradients,gradients[1:]))
   else:assert line==rest[0]
  except AssertionError as e:raise AssertionError((index,cases[index][0],line,spec))from e
 return len(cases)

def protocol(name,binary):
 rng=random.Random(323023)
 for _ in range(64):
  if name in ('hungarian','assignment_problem'):
   n=rng.randrange(1,7);a=[[rng.randrange(-30,31)for _ in range(n)]for _ in range(n)];tokens=list(map(int,run([str(binary)],input=f'{n}\n'+'\n'.join(' '.join(map(str,row))for row in a)+'\n').split()));cost,*cols=tokens;assert cost==assignment(a,n)and len(cols)==len(set(cols))==n and all(0<=j<n for j in cols)and sum(a[i][j]for i,j in enumerate(cols))==cost
  elif name in ('general_matching','bipartite_matching'):
   if name=='general_matching':
    n=rng.randrange(1,11);edges=[(i,j)for i in range(n)for j in range(i+1,n)if rng.randrange(2)];data=f'{n} {len(edges)}\n';offset=0
   else:
    l,r=rng.randrange(1,7),rng.randrange(1,7);n=l+r;raw=[(i,j)for i in range(l)for j in range(r)if rng.randrange(2)];edges=[(i,l+j)for i,j in raw];data=f'{l} {r} {len(raw)}\n';offset=l
   data+=''.join(f'{u} {v-offset}\n'for u,v in edges);tokens=list(map(int,run([str(binary)],input=data).split()));count,*flat=tokens;pairs=[(u,v+offset)for u,v in zip(flat[::2],flat[1::2])];assert count==matching(n,edges)and len(pairs)==count and len({v for e in pairs for v in e})==2*count and all(e in edges for e in pairs)
  else:
   n=rng.randrange(2,6);edges=[(u,v,rng.randrange(3),rng.randrange(8))for u,v in [(rng.randrange(n),rng.randrange(n))for _ in range(7)]];edges=[e for e in edges if e[0]!=e[1]];limit=rng.randrange(7);truth=integral_flows(n,edges);data=f'{n} {len(edges)} {limit}\n'+''.join(' '.join(map(str,e))+'\n'for e in edges);assert int(run([str(binary)],input=data))==truth.get(limit,-1)

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--compiler',default=os.environ.get('CXX','g++'));parser.add_argument('--sanitize',action='store_true');args=parser.parse_args();compiler=shlex.split(args.compiler)
 flags=['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',str(ROOT)];runtime=['-O2','-g']if not args.sanitize else ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all','-fno-pie','-no-pie']
 headers=sorted(str(p.relative_to(ROOT))for p in (ROOT/'combinatorial_optimization').rglob('*.hpp'))
 for header in headers:
  for extra in [[],['-DNDEBUG']]:run(compiler+flags+extra+['-fsyntax-only','-x','c++','-'],input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
 print(f'{len(headers)} standalone headers (debug/release) passed',flush=True)
 with tempfile.TemporaryDirectory(prefix='cp-combinatorial-optimization-')as directory:
  temp=Path(directory)
  def compile(source,suffix='',extra=()):
   binary=temp/(source.stem+suffix);run(compiler+flags+list(extra)+runtime+[str(source),'-o',str(binary)]);return binary
  includes=''.join(f'#include "{h}"\n'for h in reversed(headers))+'#include "graph_flow/all.hpp"\n#include "computational_geometry/all.hpp"\n#include "linear_algebra/all.hpp"\n'
  (temp/'one.cpp').write_text(includes+'int one(){return cp::GeneralMatching().size();}\n');(temp/'two.cpp').write_text('#include "graph_flow/all.hpp"\n'+includes+'int one();int main(){return one();}\n');run(compiler+flags+runtime+[str(temp/'one.cpp'),str(temp/'two.cpp'),'-o',str(temp/'multi')]);run([str(temp/'multi')])
  for name,want in EXAMPLES.items():
   got=run([str(compile(ROOT/f'examples/combinatorial_optimization/{name}.cpp'))]);assert got==want,(name,got,want)
  for source in sorted((ROOT/'verify/combinatorial_optimization').glob('*.test.cpp')):protocol(source.name.removesuffix('.test.cpp'),compile(source))
  print('mixed includes, multiple TUs, 5 examples, 5 judge wrappers / 320 cases passed',flush=True)
  for suffix,extra in [('_debug',[]),('_release',['-DNDEBUG'])]:
   count=arbitrary_precision(compile(ROOT/'verify/combinatorial_optimization/query_driver.cpp',suffix,extra));print(f'{suffix[1:]}: {count} Python arbitrary-precision optimization cases passed',flush=True)
   print(run([str(compile(ROOT/'verify/combinatorial_optimization/unit_test.cpp',suffix,extra))]),end='',flush=True)
if __name__=='__main__':main()
