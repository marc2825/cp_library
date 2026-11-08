#!/usr/bin/env python3
"""3.7--3.11 B: standalone headers, examples, independent oracles and judge I/O."""
import argparse
import itertools
import json
import os
from pathlib import Path
import random
import re
import shlex
import subprocess
import tempfile
from urllib.parse import unquote
ROOT=Path(__file__).resolve().parents[1]
def run(cmd,**kwargs):
 r=subprocess.run(list(map(str,cmd)),text=True,capture_output=True,timeout=300,**kwargs)
 if r.returncode:raise RuntimeError(f'{shlex.join(map(str,cmd))}\n{r.stdout}{r.stderr}')
 return r.stdout

def judge(binary,name):
 rng=random.Random(7112026)
 for trial in range(24):
  if name=='directed_mst':
   n=rng.randrange(1,7);root=rng.randrange(n);edges=[(rng.randrange(n),rng.randrange(n),rng.randrange(-9,10)) for _ in range(rng.randrange(11))]
   best=None
   for ids in itertools.combinations(range(len(edges)),n-1):
    degree=[0]*n;adj=[[] for _ in range(n)];cost=0
    for i in ids:u,v,w=edges[i];degree[v]+=1;adj[u].append(v);cost+=w
    if degree!=[int(v!=root) for v in range(n)]:continue
    seen={root};q=[root]
    for v in q:
     for u in adj[v]:
      if u not in seen:seen.add(u);q.append(u)
    if len(seen)==n and (best is None or cost<best):best=cost
   data=f'{n} {len(edges)} {root}\n'+''.join(f'{u} {v} {w}\n' for u,v,w in edges)
   output=list(map(int,run([binary],input=data).split()))
   if best is None:assert output==[-1]
   else:
    assert output[0]==best and len(output)==n+1
    parent=output[1:];assert parent[root]==root
    cost=0
    for v in range(n):
     if v!=root:cost+=min(w for u,t,w in edges if u==parent[v] and t==v)
     seen=set();u=v
     while u!=root:assert u not in seen;seen.add(u);u=parent[u]
    assert cost==best
  elif name=='min_cost_b_flow':
   n=rng.randrange(1,5);b=[rng.randrange(-2,3) for _ in range(n)]
   edges=[]
   for i in range(rng.randrange(8)):
    u,v=rng.randrange(n),rng.randrange(n);l=rng.randrange(-1,2);edges.append((u,v,l,l+rng.randrange(3),rng.randrange(-4,5)))
   best=None
   for flow in itertools.product(*(range(l,r+1) for u,v,l,r,c in edges)):
    net=[0]*n
    for (u,v,l,r,c),f in zip(edges,flow):net[u]+=f;net[v]-=f
    cost=sum(c*f for (u,v,l,r,c),f in zip(edges,flow))
    if net==b and (best is None or cost<best):best=cost
   data=f'{n} {len(edges)}\n'+'\n'.join(map(str,b))+'\n'+''.join(' '.join(map(str,e))+'\n' for e in edges)
   output=run([binary],input=data).split()
   if best is None:assert output==['infeasible']
   else:
    output=list(map(int,output));assert output[0]==best and len(output)==1+n+len(edges);potential=output[1:n+1];flow=output[n+1:];net=[0]*n
    for (u,v,l,r,c),f in zip(edges,flow):
     assert l<=f<=r;net[u]+=f;net[v]-=f
     if f<r:assert c+potential[u]-potential[v]>=0
     if f>l:assert -c+potential[v]-potential[u]>=0
    assert net==b and sum(c*f for (u,v,l,r,c),f in zip(edges,flow))==best
  else:
   n=rng.randrange(1,85);edges=[(v,rng.randrange(v)) for v in range(1,n)];adj=[[] for _ in range(n)]
   for u,v in edges:adj[u].append(v);adj[v].append(u)
   counts=[0]*n
   for source in range(n):
    distance=[-1]*n;distance[source]=0;q=[source]
    for v in q:
     for u in adj[v]:
      if distance[u]<0:distance[u]=distance[v]+1;q.append(u)
    for v in range(source+1,n):counts[distance[v]]+=1
   data=f'{n}\n'+''.join(f'{u} {v}\n' for u,v in edges)
   assert list(map(int,run([binary],input=data).split()))==counts[1:]

def check_links(entries):
 paths=[ROOT/'README.md',ROOT/'docs/README.md',*list((ROOT/'docs/b_7_11').glob('*.md')),ROOT/'visualizations/b_7_11/README.md']+[ROOT/'docs'/e['doc'] for e in entries]+[ROOT/f'docs/{cat}/README.md' for cat in ['graph_basic','graph_connectivity','graph_flow','graph_combinatorics','tree']]
 checked=0
 for path in paths:
  content=re.sub(r'```.*?```','',path.read_text(),flags=re.S)
  for link in re.findall(r'\]\(([^)]+)\)',content):
   if '://' in link or link.startswith('#') or link.startswith('mailto:'):continue
   target=unquote(link.split('#',1)[0]);assert (path.parent/target).exists(),(path,link);checked+=1
 print(f'{checked} local document links passed',flush=True)

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--compiler',default=os.environ.get('CXX','g++'));parser.add_argument('--sanitize',action='store_true');parser.add_argument('--links-only',action='store_true');args=parser.parse_args()
 compiler=shlex.split(args.compiler);entries=json.loads((ROOT/'verify/b_7_11/manifest.json').read_text())['algorithms'];headers=[e['header'] for e in entries]
 if args.links_only:
  check_links(entries);return
 flags=['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',ROOT];runtime=['-O2'] if not args.sanitize else ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all','-fno-pie','-no-pie']
 for h in headers:
  for release in [[],['-DNDEBUG']]:run(compiler+flags+release+['-fsyntax-only','-x','c++','-'],input=f'#include "{h}"\n#include "{h}"\nint main(){{}}\n')
 print(f'{len(headers)} standalone/repeated entries debug/release passed',flush=True)
 with tempfile.TemporaryDirectory(prefix='cp-b-7-11-') as folder:
  temp=Path(folder)
  def compile(source,name,extra=()):
   binary=temp/name;run(compiler+flags+runtime+list(extra)+[source,'-o',binary]);return binary
  includes=''.join(f'#include "{h}"\n' for h in reversed(headers))+''.join(f'#include "{h.relative_to(ROOT)}"\n' for h in sorted(ROOT.glob('*/all.hpp')))
  (temp/'a.cpp').write_text(includes+'int first(){return cp::tree_distance_distribution(cp::Graph<int>(0)).size();}\n');(temp/'b.cpp').write_text(includes+'int first();int main(){return first();}\n')
  run(compiler+flags+runtime+[temp/'a.cpp',temp/'b.cpp','-o',temp/'multi']);run([temp/'multi'])
  for e in entries:
   source=ROOT/'examples'/e['example'];doc=(ROOT/'docs'/e['doc']).read_text();assert re.search(r'```cpp\n(.*?)```',doc,re.S).group(1)==source.read_text(),e['doc'];output=run([compile(source,Path(e['header']).stem+'_example')]);assert output==e['output'],(e['header'],output,e['output'])
  print('mixed all.hpp translation units and 36 documented examples passed',flush=True)
  for release in [[],['-DNDEBUG']]:
   for name in ['graph','flow','combinatorics','tree']:
    binary=compile(ROOT/f'verify/b_7_11/{name}_test.cpp',name,release);print(run([binary]).strip(),flush=True)
   for name in ['directed_mst','min_cost_b_flow','tree_distance']:
    binary=compile(ROOT/f'verify/b_7_11/{name}.test.cpp',name,release);judge(binary,name)
   print('3 judge formats x 24 independent Python cases passed '+('release' if release else 'debug'),flush=True)
 check_links(entries)
if __name__=='__main__':main()
