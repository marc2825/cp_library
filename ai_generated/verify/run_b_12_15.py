#!/usr/bin/env python3
"""3.12--3.15 B: standalone headers, examples, independent oracles and judge I/O."""
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
 rng=random.Random(312152026)
 for trial in range(24):
  if name=='discrete_log':
   cases=[];expected=[]
   for i in range(70):
    m=rng.randrange(1,700);a=rng.randrange(-m,2*m);b=rng.randrange(-m,2*m);cases.append((a,b,m));seen=set();x=1%m;e=0;answer=-1
    while x not in seen:
     if x==b%m:answer=e;break
     seen.add(x);x=x*a%m;e+=1
    expected.append(answer)
   data=str(len(cases))+'\n'+''.join(f'{a} {b} {m}\n' for a,b,m in cases)
  elif name=='mod_tetration':
   cases=[];expected=[]
   for i in range(25):
    a=rng.randrange(5);h=rng.randrange(5);m=rng.randrange(1,1<<63)
    if h==0:value=1%m
    elif h==1:value=a%m
    else:
     exponent=1
     for j in range(h-1):exponent=a**exponent
     value=pow(a,exponent,m)
    cases.append((a,h,m));expected.append(value)
   for i in range(4):
    a=rng.getrandbits(64);m=rng.randrange(1,1<<63);cases.append((a,2,m));expected.append(pow(a,a,m))
   data=str(len(cases))+'\n'+''.join(f'{a} {h} {m}\n' for a,h,m in cases)
  elif name=='prime_count':
   n=rng.randrange(1,100000);prime=bytearray(b'\x01')*(n+1);prime[0:2]=b'\x00\x00'
   for p in range(2,int(n**0.5)+1):
    if prime[p]:prime[p*p::p]=bytes((n-p*p)//p+1)
   expected=[sum(prime)];data=f'{n}\n'
  else:
   n=rng.randrange(1,100);m=rng.randrange(1,100);a=[];value=rng.randrange(-100,100);slope=100
   for i in range(n):slope-=rng.randrange(6);value+=slope;a.append(value)
   b=[rng.randrange(-1000,1000) for _ in range(m)];expected=[min(a[i]+b[k-i] for i in range(max(0,k-m+1),min(n-1,k)+1)) for k in range(n+m-1)]
   data=f'{n} {m}\n'+' '.join(map(str,a))+'\n'+' '.join(map(str,b))+'\n'
  assert list(map(int,run([binary],input=data).split()))==expected,(name,trial)

def check_links(entries):
 paths=[ROOT/'README.md',ROOT/'docs/README.md',*list((ROOT/'docs/b_12_15').glob('*.md')),ROOT/'visualizations/b_12_15/README.md']+[ROOT/'docs'/e['doc'] for e in entries]+[ROOT/f'docs/{cat}/README.md' for cat in ['dp_optimization','number_theory_basic','number_theory_convolution','combinatorics']]
 checked=0
 for path in paths:
  content=re.sub(r'```.*?```','',path.read_text(),flags=re.S)
  for link in re.findall(r'\]\(([^)]+)\)',content):
   if '://' in link or link.startswith('#') or link.startswith('mailto:'):continue
   target=unquote(link.split('#',1)[0]);assert (path.parent/target).exists(),(path,link);checked+=1
 print(f'{checked} local document links passed',flush=True)

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--compiler',default=os.environ.get('CXX','g++'));parser.add_argument('--sanitize',action='store_true');parser.add_argument('--links-only',action='store_true');args=parser.parse_args()
 compiler=shlex.split(args.compiler);entries=json.loads((ROOT/'verify/b_12_15/manifest.json').read_text())['algorithms'];headers=[e['header'] for e in entries]
 if args.links_only:
  check_links(entries);return
 flags=['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',ROOT];runtime=['-O2'] if not args.sanitize else ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all','-fno-pie','-no-pie']
 for h in headers:
  for release in [[],['-DNDEBUG']]:run(compiler+flags+release+['-fsyntax-only','-x','c++','-'],input=f'#include "{h}"\n#include "{h}"\nint main(){{}}\n')
 print(f'{len(headers)} standalone/repeated entries debug/release passed',flush=True)
 with tempfile.TemporaryDirectory(prefix='cp-b-12-15-') as folder:
  temp=Path(folder)
  def compile(source,name,extra=()):
   binary=temp/name;run(compiler+flags+runtime+list(extra)+[source,'-o',binary]);return binary
  includes=''.join(f'#include "{h}"\n' for h in reversed(headers))+''.join(f'#include "{h.relative_to(ROOT)}"\n' for h in sorted(ROOT.glob('*/all.hpp')))
  (temp/'a.cpp').write_text(includes+'int first(){return int(cp::prime_count(10))-4;}\n');(temp/'b.cpp').write_text(includes+'int first();int main(){return first();}\n')
  run(compiler+flags+runtime+[temp/'a.cpp',temp/'b.cpp','-o',temp/'multi']);run([temp/'multi'])
  for e in entries:
   source=ROOT/'examples'/e['example'];doc=(ROOT/'docs'/e['doc']).read_text();assert re.search(r'```cpp\n(.*?)```',doc,re.S).group(1)==source.read_text(),e['doc'];output=run([compile(source,Path(e['header']).stem+'_example')]);assert output==e['output'],(e['header'],output,e['output'])
  print('mixed all.hpp translation units and 30 documented examples passed',flush=True)
  for release in [[],['-DNDEBUG']]:
   for name in ['dp','number','combinatorics']:
    binary=compile(ROOT/f'verify/b_12_15/{name}_test.cpp',name,release);print(run([binary]).strip(),flush=True)
   for name in ['discrete_log','mod_tetration','prime_count','min_plus']:
    binary=compile(ROOT/f'verify/b_12_15/{name}.test.cpp',name,release);judge(binary,name)
   print('4 judge formats x 24 independent Python cases passed '+('release' if release else 'debug'),flush=True)
 check_links(entries)
if __name__=='__main__':main()
