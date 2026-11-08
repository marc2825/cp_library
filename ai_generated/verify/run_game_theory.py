#!/usr/bin/env python3
"""3.24: complete game trees, independent mex sets and product games; no extra packages."""
import argparse
from functools import lru_cache
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[1]
EXAMPLES={'nim':'2 1\n0 1 2\n','misere_nim':'1\n2 1 4\n1 0\n','grundy':'1\n0 1 2 0 \n','dag_game':'3 1\n1 2 1\n'}
def run(command,**kwargs):
 r=subprocess.run(command,text=True,capture_output=True,**kwargs)
 if r.returncode:raise RuntimeError(f'{shlex.join(map(str,command))}\n{r.stdout}{r.stderr}')
 return r.stdout

def canonical(a):return tuple(sorted(x for x in a if x))
@lru_cache(None)
def tree(state,misere=False):
 if not state:return misere
 return any(not tree(canonical(state[:i]+(x,)+state[i+1:]),misere)for i,pile in enumerate(state)for x in range(pile))
def formula(a,misere=False):
 total=0
 for x in a:total^=x
 return sum(x==1 for x in a)%2==0 if misere and all(x<=1 for x in a)else total!=0

def grundy(moves):
 @lru_cache(None)
 def solve(v):
  options={solve(u)for u in moves[v]};g=0
  while g in options:g+=1
  return g
 return [solve(v)for v in range(len(moves))]
def product_game(moves):
 @lru_cache(None)
 def solve(state):return any(not solve(tuple(sorted(state[:i]+(u,)+state[i+1:])))for i,v in enumerate(state)for u in moves[v])
 return solve

def parse_move(tokens):return None if tokens==['NONE']else tuple(map(int,tokens))
def arbitrary_precision(binary):
 rng=random.Random(324024);cases=[]
 for i in range(520):
  large=i>=370;a=[rng.choice([0,1,2**64-1,1<<63,rng.getrandbits(64)])if large else rng.randrange(7)for _ in range(rng.randrange(7))]
  cases.append(('N '+str(len(a))+' '+' '.join(map(str,a)),('N',a,large)))
 for _ in range(280):
  a=[rng.choice([2**31-1,rng.randrange(90)])for _ in range(rng.randrange(80))];want=0
  while want in a:want+=1
  cases.append(('X '+str(len(a))+' '+' '.join(map(str,a)),('E',str(want))))
 for _ in range(220):
  n=rng.randrange(11);order=list(range(n));rng.shuffle(order);edges=[]
  for i in range(n):
   for j in range(i+1,n):
    if rng.randrange(4)==0:edges.extend([(order[i],order[j])]*(1+rng.randrange(2)))
  moves=[[]for _ in range(n)]
  for u,v in edges:moves[u].append(v)
  queries=[[rng.randrange(n)for _ in range(rng.randrange(4))]if n else []for _ in range(8)]
  cmd=f'D {n} {len(edges)} {len(queries)} '+' '.join(f'{u} {v}'for u,v in edges)+' '+' '.join(str(len(q))+' '+' '.join(map(str,q))for q in queries)
  cases.append((cmd,('D',moves,queries)))
 for command,want in [('S 2 1 -1','INVALID'),('S 1 -9223372036854775808','INVALID'),('X 2 0 -1','INVALID'),('D 1 1 0 0 0','INVALID'),('D 3 2 1 1 2 2 1 1 0','INVALID'),('D 1 1 0 0 1','RANGE'),('D 1 1 0 0 -1','RANGE'),('D 2 1 1 1 0 2 1 -1','RANGE')]:cases.append((command,('E',want)))
 actual=run([str(binary)],input='\n'.join(c[0]for c in cases)+'\n').splitlines();assert len(actual)==len(cases),(len(actual),len(cases))
 for i,(line,(command,spec))in enumerate(zip(actual,cases)):
  try:
   if spec[0]=='E':assert line==spec[1]
   elif spec[0]=='N':
    _,a,large=spec;normal,misere=line.split('|');sum_,win,*move=normal.split();total=0
    for x in a:total^=x
    assert int(sum_)==total
    for kind,tokens in [(False,[win]+move),(True,misere.split())]:
     want=formula(a,kind)if large else tree(canonical(a),kind);assert int(tokens[0])==want
     m=parse_move(tokens[1:]);assert bool(m)==(want and any(a))
     if m:
      pile,left,removed=m;assert 0<=pile<len(a)and 0<=left<a[pile]and removed==a[pile]-left;b=a.copy();b[pile]=left;assert not (formula(b,kind)if large else tree(canonical(b),kind))
   else:
    _,moves,queries=spec;parts=line.split('|');g=list(map(int,parts[0].split()));order=list(map(int,parts[1].split()));assert g==grundy(moves)and sorted(order)==list(range(len(moves)));rank={v:i for i,v in enumerate(order)};assert all(rank[u]<rank[v]for u,row in enumerate(moves)for v in row);oracle=product_game(moves);assert len(parts)==2+len(queries)
    for q,raw in zip(queries,parts[2:]):
     sum_,win,*tokens=raw.split();want=oracle(tuple(sorted(q)));total=0
     for v in q:total^=g[v]
     assert int(sum_)==total and int(win)==want;m=parse_move(tokens);assert bool(m)==want
     if m:
      component,u,v=m;assert 0<=component<len(q)and q[component]==u and v in moves[u];b=q.copy();b[component]=v;assert not oracle(tuple(sorted(b)))
  except AssertionError as e:raise AssertionError((i,command,line,spec))from e
 return len(cases)

def protocol(name,binary):
 rng=random.Random(324024)
 for trial in range(64):
  if name=='dag_game':
   k=rng.randrange(1,70);allowed=sorted(set(rng.randrange(1,k+1)for _ in range(1+rng.randrange(10))));win=[False]*(k+1)
   for v in range(1,k+1):win[v]=any(not win[v-x]for x in allowed if x<=v)
   actual=run([str(binary)],input=f'{len(allowed)} {k}\n'+' '.join(map(str,allowed))+'\n').strip();assert actual==('First'if win[k]else 'Second')
  else:
   data=[];expected=[]
   for _ in range(5):
    a=[rng.randrange(1,7 if name!='grundy'else 25)for _ in range(1+rng.randrange(6))];data.extend([str(len(a)),' '.join(map(str,a))])
    if name=='grundy':
     moves=[list(range(max(0,x-3),x))for x in range(max(a)+1)];g=grundy(moves);total=0
     for x in a:total^=g[x]
     win=total!=0
    else:win=tree(canonical(a),name=='misere_nim')
    expected.append(('First'if win else 'Second')if name=='misere_nim'else ('first'if win else 'second'))
   assert run([str(binary)],input='5\n'+'\n'.join(data)+'\n').split()==expected
 if name=='dag_game':assert run([str(binary)],input='1 100000\n1\n')=='Second\n'

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--compiler',default=os.environ.get('CXX','g++'));parser.add_argument('--sanitize',action='store_true');args=parser.parse_args();compiler=shlex.split(args.compiler)
 flags=['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',str(ROOT)];runtime=['-O2','-g']if not args.sanitize else ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all','-fno-pie','-no-pie']
 headers=sorted(str(p.relative_to(ROOT))for p in (ROOT/'game_theory').rglob('*.hpp'))
 for header in headers:
  for extra in [[],['-DNDEBUG']]:run(compiler+flags+extra+['-fsyntax-only','-x','c++','-'],input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
 for invalid in ['std::vector<bool>{true}','std::vector<double>{1.0}']:
  rejected=subprocess.run(compiler+flags+['-fsyntax-only','-x','c++','-'],input=f'#include "game_theory/nim.hpp"\nint main(){{cp::nim_sum({invalid});}}\n',text=True,capture_output=True);assert rejected.returncode and 'Nim needs integer piles'in rejected.stderr
 print(f'{len(headers)} standalone headers (debug/release), bool/real pile compile-time rejection passed',flush=True)
 with tempfile.TemporaryDirectory(prefix='cp-game-theory-')as directory:
  temp=Path(directory)
  def compile(source,suffix='',extra=()):
   binary=temp/(source.stem+suffix);run(compiler+flags+list(extra)+runtime+[str(source),'-o',str(binary)]);return binary
  includes=''.join(f'#include "{h}"\n'for h in reversed(headers))+'#include "graph_basic/all.hpp"\n#include "combinatorial_optimization/all.hpp"\n'
  (temp/'one.cpp').write_text(includes+'int one(){return cp::DAGGame().size();}\n');(temp/'two.cpp').write_text('#include "graph_basic/all.hpp"\n'+includes+'int one();int main(){return one();}\n');run(compiler+flags+runtime+[str(temp/'one.cpp'),str(temp/'two.cpp'),'-o',str(temp/'multi')]);run([str(temp/'multi')])
  for name,want in EXAMPLES.items():
   got=run([str(compile(ROOT/f'examples/game_theory/{name}.cpp'))]);assert got==want,(name,got,want)
  for source in sorted((ROOT/'verify/game_theory').glob('*.test.cpp')):protocol(source.name.removesuffix('.test.cpp'),compile(source))
  print('mixed includes, multiple TUs, 4 examples, 4 judge wrappers / 256 inputs and 100000-stone protocol passed',flush=True)
  for suffix,extra in [('_debug',[]),('_release',['-DNDEBUG'])]:
   count=arbitrary_precision(compile(ROOT/'verify/game_theory/query_driver.cpp',suffix,extra));print(f'{suffix[1:]}: {count} independent Python game/64-bit/mex/DAG cases passed',flush=True)
   print(run([str(compile(ROOT/'verify/game_theory/unit_test.cpp',suffix,extra))]),end='',flush=True)
if __name__=='__main__':main()
