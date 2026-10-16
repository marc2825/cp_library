#!/usr/bin/env python3
"""3.21 string algorithms: direct byte/vector oracles, judge protocols and sanitizers."""
import argparse
from collections import Counter
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[1]
EXAMPLES={
 'z_algorithm':'12 1 0 0 3 1 0 0 2 2 1 0 \n',
 'prefix_function':'0 0 1 2 3 0 1 \n',
 'kmp':'0 2 4 \n0 1 2 \n',
 'rolling_hash':'1\n4\n1\n',
 'trie':'4 2 3\n1 0 1\n',
 'suffix_array':'5 3 1 0 4 2 \n',
 'lcp_array':'1 3 0 0 2 \n',
 'manacher':'1 1 1 1 1 2 1 \n0 0 2 0 2 0 0 \n1 0\n',
}
def run(command,**kwargs):
 r=subprocess.run(command,text=True,capture_output=True,**kwargs)
 if r.returncode:raise RuntimeError(f'{shlex.join(map(str,command))}\n{r.stdout}{r.stderr}')
 return r.stdout

def naive_arrays(s):
 n=len(s);z=[];prefix=[];odd=[];even=[]
 for i in range(n):
  z.append(next((k for k in range(n-i)if s[k]!=s[i+k]),n-i))
  prefix.append(max((k for k in range(1,i+1)if s[:k]==s[i+1-k:i+1]),default=0))
  odd.append(max(k for k in range(1,min(i+1,n-i)+1)if s[i-k+1:i+k]==s[i-k+1:i+k][::-1]))
  even.append(max(k for k in range(min(i,n-i)+1)if s[i-k:i+k]==s[i-k:i+k][::-1]))
 sa=sorted(range(n),key=lambda i:s[i:]);lcp=[]
 for a,b in zip(sa,sa[1:]):lcp.append(next((k for k in range(n-max(a,b))if s[a+k]!=s[b+k]),n-max(a,b)))
 return [z,prefix,sa,lcp,odd,even]
def encoded(s):return s.hex()or '-'
def hash_sum(s,base):return [sum((byte+1)*pow(base,len(s)-1-i,p)for i,byte in enumerate(s))%p for p in (1000000007,1000000009)]

def arbitrary_precision(binary):
 rng=random.Random(321021);commands=[];expected=[]
 def arrays(s,vector=False):
  commands.append(('V '+str(len(s))+' '+' '.join(map(str,s)))if vector else 'A '+encoded(s));a=naive_arrays(s);expected.append(' '.join(str(x)for row in a for x in [len(row)]+row))
 for n in range(9):
  for _ in range(20):arrays(bytes(rng.randrange(3)for _ in range(n)))
 for trial in range(220):arrays(bytes(rng.randrange(256)for _ in range(rng.randrange(60))))
 for trial in range(180):arrays([rng.choice([-2**63,2**63-1,0,rng.randrange(-8,9)])for _ in range(rng.randrange(40))],True)
 for trial in range(240):
  s=bytes(rng.randrange(6)for _ in range(rng.randrange(45)));p=bytes(rng.randrange(6)for _ in range(rng.randrange(12)))
  commands.append('K '+encoded(s)+' '+encoded(p));matches=[i for i in range(len(s)+1)if s[i:i+len(p)]==p and len(p)<=len(s)-i];expected.append(' '.join(map(str,[len(matches)]+matches)))
 for trial in range(420):
  s=bytes(rng.randrange(256)for _ in range(rng.randrange(40)));t=bytes(rng.randrange(256)for _ in range(rng.randrange(40)));base=rng.choice([257,911382323,1000000006,rng.randrange(257,1000000007)])
  l,r=sorted([rng.randrange(len(s)+1),rng.randrange(len(s)+1)]);bl,br=sorted([rng.randrange(len(t)+1),rng.randrange(len(t)+1)]);a,b=s[l:r],t[bl:br];common=next((i for i in range(min(len(a),len(b)))if a[i]!=b[i]),min(len(a),len(b)))
  commands.append(f'H {base} {encoded(s)} {encoded(t)} {l} {r} {bl} {br}');expected.append(' '.join(map(str,hash_sum(a,base)+hash_sum(b,base)+[common,int(a==b)]+hash_sum(s[:r],base))))
 for base,l,r,want in [(0,0,1,'INVALID'),(256,0,1,'INVALID'),(1000000007,0,1,'INVALID'),(257,-1,1,'RANGE'),(257,2,1,'RANGE'),(257,0,4,'RANGE')]:commands.append(f'H {base} 616263 616263 {l} {r} 0 1');expected.append(want)
 trie_operations=0
 for trial in range(24):
  bag=Counter();prefixes={b''};ops=[]
  for step in range(120):
   op=rng.choice('IIECPR'if step%37==0 else 'IIIEECP');s=bytes(rng.randrange(6)for _ in range(rng.randrange(7)));ops.append(op+' '+encoded(s))
   if op=='I':bag[s]+=1;prefixes|={s[:k]for k in range(1,len(s)+1)};answer=bag[s]
   elif op=='E':answer=int(bag[s]>0);bag[s]-=answer
   elif op=='C':answer=bag[s]
   elif op=='P':answer=sum(v for k,v in bag.items()if k.startswith(s))
   else:bag.clear();prefixes={b''};answer=0
   expected.append(f'{answer} {sum(bag.values())} {len(prefixes)}');trie_operations+=1
  commands.append(f'T {len(ops)}\n'+'\n'.join(ops))
 actual=run([str(binary)],input='\n'.join(commands)+'\n').splitlines();assert len(actual)==len(expected),(len(actual),len(expected))
 for i,(line,want)in enumerate(zip(actual,expected)):assert line.split()==want.split(),(i,line,want)
 return len(expected),trie_operations

def protocol(name,binary):
 rng=random.Random(321021)
 for trial in range(64):
  s=''.join(rng.choice('abc')for _ in range(1+rng.randrange(100)))
  if name in ('z_algorithm','suffix_array','lcp_array','prefix_function','manacher'):
   data=s+'\n';z,prefix,sa,lcp,odd,even=naive_arrays(s)
   if name=='z_algorithm':want=z
   elif name=='suffix_array':want=sa
   elif name=='lcp_array':want=[len({s[l:r]for l in range(len(s))for r in range(l+1,len(s)+1)})]
   elif name=='prefix_function':want=[k for k in range(1,len(s))if s[:k]==s[-k:]]
   else:
    want=[]
    for i in range(len(s)):
     if i:want.append(2*even[i])
     want.append(2*odd[i]-1)
   assert list(map(int,run([str(binary)],input=data).split()))==want,(name,trial)
  elif name in ('kmp','rolling_hash'):
   p=''.join(rng.choice('abc')for _ in range(1+rng.randrange(20)));want=[i for i in range(len(s)-len(p)+1)if s[i:i+len(p)]==p];assert list(map(int,run([str(binary)],input=s+'\n'+p+'\n').split()))==want,(name,trial)
  else:
   words=set();ops=[];want=[]
   for _ in range(100):
    word=''.join(rng.choice('ACGT')for _ in range(1+rng.randrange(9)))
    if rng.randrange(2):ops.append('insert '+word);words.add(word)
    else:ops.append('find '+word);want.append('yes'if word in words else 'no')
   assert run([str(binary)],input=f'{len(ops)}\n'+'\n'.join(ops)+'\n').split()==want

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--compiler',default=os.environ.get('CXX','g++'));parser.add_argument('--sanitize',action='store_true');args=parser.parse_args();compiler=shlex.split(args.compiler)
 flags=['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',str(ROOT)];runtime=['-O2','-g']if not args.sanitize else ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all','-fno-pie','-no-pie']
 headers=sorted(str(p.relative_to(ROOT))for p in (ROOT/'string_algorithm').rglob('*.hpp'))
 for header in headers:
  for extra in [[],['-DNDEBUG']]:run(compiler+flags+extra+['-fsyntax-only','-x','c++','-'],input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
 print(f'{len(headers)} standalone headers (debug/release) passed',flush=True)
 with tempfile.TemporaryDirectory(prefix='cp-string-algorithm-')as directory:
  temp=Path(directory)
  def compile(source,suffix='',extra=()):
   binary=temp/(source.stem+suffix);run(compiler+flags+list(extra)+runtime+[str(source),'-o',str(binary)]);return binary
  includes=''.join(f'#include "{h}"\n'for h in reversed(headers))+'#include "linear_algebra/all.hpp"\n#include "set_function/all.hpp"\n#include "dp_optimization/all.hpp"\n'
  (temp/'one.cpp').write_text(includes+'int one(){return cp::z_algorithm("abc")[0]-3;}\n');(temp/'two.cpp').write_text('#include "dp_optimization/all.hpp"\n'+includes+'int one();int main(){cp::RollingHash h("x");return one()+h.size()-1;}\n');run(compiler+flags+runtime+[str(temp/'one.cpp'),str(temp/'two.cpp'),'-o',str(temp/'multi')]);run([str(temp/'multi')])
  for name,want in EXAMPLES.items():
   got=run([str(compile(ROOT/f'examples/string_algorithm/{name}.cpp'))]);assert got==want,(name,got,want)
  for source in sorted((ROOT/'verify/string_algorithm').glob('*.test.cpp')):protocol(source.name.removesuffix('.test.cpp'),compile(source))
  print('mixed includes, multiple TUs, 8 examples, 8 judge wrappers / 512 cases passed',flush=True)
  for suffix,extra in [('_debug',[]),('_release',['-DNDEBUG'])]:
   count,trie=arbitrary_precision(compile(ROOT/'verify/string_algorithm/query_driver.cpp',suffix,extra));print(f'{suffix[1:]}: {count} Python byte/vector/hash protocol cases including {trie} trie operations passed',flush=True)
   print(run([str(compile(ROOT/'verify/string_algorithm/unit_test.cpp',suffix,extra))]),end='',flush=True)
if __name__=='__main__':main()
