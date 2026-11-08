#!/usr/bin/env python3
"""3.22: independent Fraction geometry, judge protocols, standalone headers and sanitizers."""
import argparse
from fractions import Fraction as Q
from functools import cmp_to_key
import math
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[1]
EXAMPLES={'point_vector':'5 5\n1\n','segment_intersection':'1\n0\n','line_intersection':'4/3 4/3\n','projection_distance':'8/5 4/5 9/5\n1\n','convex_hull':'0 0\n2 0\n2 2\n0 2\n','angular_sort':'0 0\n1 0\n0 1\n-1 0\n0 -1\n','polygon_area':'3 3/2\n','point_in_polygon':'2\n1\n0\n','convex_polygon_contains':'2 0\n','convex_polygon_cut':'8/3\n7/6\n','rotating_calipers':'25\n','closest_pair':'0 2 2\n'}
def run(command,**kwargs):
 r=subprocess.run(command,text=True,capture_output=True,**kwargs)
 if r.returncode:raise RuntimeError(f'{shlex.join(map(str,command))}\n{r.stdout}{r.stderr}')
 return r.stdout

def cross(a,b,c=(0,0)):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
def d2(a,b):return sum((x-y)**2 for x,y in zip(a,b))
def on(a,b,p):return cross(a,b,p)==0 and all(min(x,y)<=z<=max(x,y)for x,y,z in zip(a,b,p))
def coefficients(a,b):return (a[1]-b[1],b[0]-a[0],a[1]*b[0]-a[0]*b[1])
def intersection(a,b,c,d):
 A,B,C=coefficients(a,b);D,E,F=coefficients(c,d);det=A*E-B*D
 if not det:return None
 return (Q(C*E-B*F,det),Q(A*F-C*D,det))
def intersects(a,b,c,d):
 p=intersection(a,b,c,d)
 return on(a,b,p)and on(c,d,p)if p else any(on(u,v,p)for u,v,p in [(a,b,c),(a,b,d),(c,d,a),(c,d,b)])
def projection(a,b,p):
 A,B,C=coefficients(a,b);v=Q(A*p[0]+B*p[1]-C,A*A+B*B)
 return (p[0]-A*v,p[1]-B*v)
def segment_distance(a,b,p):
 if a==b:return Q(d2(a,p))
 q=projection(a,b,p)
 return d2(q,p)if on(a,b,q)else min(d2(a,p),d2(b,p))
def distance(a,b,c,d):return Q(0)if intersects(a,b,c,d)else min(segment_distance(a,b,c),segment_distance(a,b,d),segment_distance(c,d,a),segment_distance(c,d,b))
def hull(points):
 p=sorted(set(points))
 if len(p)<2:return p
 out=[];at=0
 while True:
  out.append(p[at]);nxt=(at+1)%len(p)
  for j in range(len(p)):
   z=cross(p[at],p[nxt],p[j])
   if z<0 or z==0 and d2(p[at],p[j])>d2(p[at],p[nxt]):nxt=j
  at=nxt
  if not at:return out

def location(p,q):
 inside=False
 for a,b in zip(p,p[1:]+p[:1]):
  if on(a,b,q):return 1
  if (a[1]>q[1])!=(b[1]>q[1]):
   x=a[0]+Q(b[0]-a[0],b[1]-a[1])*(q[1]-a[1])
   if q[0]<x:inside=not inside
 return 2 if inside else 0

def area(p):return abs(sum(a[0]*b[1]-a[1]*b[0]for a,b in zip(p,p[1:]+p[:1])))/Q(2)
def cut(p,a,b):
 # Enumerate all feasible vertices, then gift-wrap them; independent of edge clipping.
 candidates=[v for v in p if cross(a,b,v)>=0]
 for u,v in zip(p,p[1:]+p[:1]):
  z=intersection(a,b,u,v)
  if z and on(u,v,z):candidates.append(z)
 return hull(candidates)
def angle_compare(a,b):
 if a==(0,0)or b==(0,0):return (a!=(0,0))-(b!=(0,0))
 ha=0 if a[1]>0 or a[1]==0 and a[0]>0 else 1;hb=0 if b[1]>0 or b[1]==0 and b[0]>0 else 1
 z=a[0]*b[1]-a[1]*b[0]
 return (ha>hb)-(ha<hb)if ha!=hb else (z<0)-(z>0)
def flat(p):return ' '.join(str(v)for a in p for v in a)
def encoded(p):return str(len(p))+' '+flat(p)

def arbitrary_precision(binary):
 rng=random.Random(322022);commands=[];expected=[]
 def add(command,values):commands.append(command);expected.append(list(map(str,values)))
 for _ in range(320):
  a,b,c,d=[rng.randrange(-10**12,10**12)for _ in range(4)];b=b or 1;c=c or 1;d=d or 1;x,y=Q(a,b),Q(c,d);add(f'R {a} {b} {c} {d}',[x+y,x-y,x*y,x/y,int(x<y)])
 for _ in range(260):
  p=[(rng.randrange(-10**9,10**9),rng.randrange(-10**9,10**9))for _ in range(3)];a,b,c=p
  add('P '+flat(p),[sum(x*y for x,y in zip(a,b)),a[0]*b[1]-a[1]*b[0],cross(a,b,c),d2(a,b)])
 for _ in range(280):
  a,b,c,d=[(rng.randrange(-100,101),rng.randrange(-100,101))for _ in range(4)]
  if a==b or c==d:continue
  p=intersection(a,b,c,d)
  add('L '+flat([a,b,c,d]),[0,*p]if p else [2 if cross(a,b,c)==0 else 1])
  add('D '+flat([a,b,c,d]),[*projection(a,b,c),d2(projection(a,b,c),c),segment_distance(a,b,c),distance(a,b,c,d)])
 for _ in range(220):
  p=[(rng.randrange(-30,31),rng.randrange(-30,31))for _ in range(rng.randrange(25))];h=hull(p)
  add('H '+encoded(p),[len(h),*(v for a in h for v in a)])
  add('A '+encoded(p),[v for a in sorted(p,key=cmp_to_key(angle_compare))for v in a])
  q=(rng.randrange(-40,41),rng.randrange(-40,41));add('I '+encoded(h)+' '+flat([q]),[location(h,q),location(h,q)])
  distances=[d2(a,b)for i,a in enumerate(p)for b in p[i+1:]];add('F '+encoded(p),[min(distances)if distances else 'NONE',max(distances)if distances else 0 if p else 'NONE'])
  a=(rng.randrange(-30,31),rng.randrange(-30,31));b=(a[0]+rng.randrange(1,10),a[1]+rng.randrange(-10,11));z=cut(h,a,b)
  commands.append('C '+encoded(h)+' '+flat([a,b]));expected.append(('cut',area(z),set(z)))
 for cmd,want in [('L 0 0 1 0 0 1 1 1',[1]),('L 0 0 1 0 2 0 3 0',[2]),('L 0 0 0 0 0 1 1 1',['INVALID']),('P -9223372036854775808 -9223372036854775808 -9223372036854775808 -9223372036854775808 0 0',['OVERFLOW'])]:add(cmd,want)
 actual=run([str(binary)],input='\n'.join(commands)+'\n').splitlines();assert len(actual)==len(expected),(len(actual),len(expected))
 for i,(line,want)in enumerate(zip(actual,expected)):
  tokens=line.split()
  if isinstance(want,tuple):
   assert Q(tokens[0])==want[1]and int(tokens[1])*2==len(tokens)-2,(i,line,want)
   assert {(Q(x),Q(y))for x,y in zip(tokens[2::2],tokens[3::2])}==want[2],(i,line,want)
  else:assert tokens==want,(i,commands[i],line,want)
 return len(expected)

def protocol(name,binary):
 rng=random.Random(322022)
 for trial in range(64):
  p=[(rng.randrange(-20,21),rng.randrange(-20,21))for _ in range(2+rng.randrange(25))];h=hull(p);a,b=p[:2]
  if a==b:b=(a[0]+1,a[1])
  queries=[(rng.randrange(-30,31),rng.randrange(-30,31))for _ in range(10)]
  if name in ('closest_pair','rotating_calipers'):
   data='1\n'+encoded(p)+'\n';i,j=map(int,run([str(binary)],input=data).split());want=(min if name=='closest_pair'else max)(d2(u,v)for k,u in enumerate(p)for v in p[k+1:]);assert 0<=i<len(p)and 0<=j<len(p)and i!=j and d2(p[i],p[j])==want
  elif name=='angular_sort':
   out=list(map(int,run([str(binary)],input=encoded(p)+'\n').split()));q=list(zip(out[::2],out[1::2]));want=sorted(p,key=lambda v:(0 if v[1]<0 else 1,math.atan2(v[1],v[0])if v!=(0,0)else 0));assert sorted(q)==sorted(p);assert [math.atan2(v[1],v[0])if v!=(0,0)else 0 for v in q]==[math.atan2(v[1],v[0])if v!=(0,0)else 0 for v in want]
  elif name=='convex_hull':
   out=list(map(int,run([str(binary)],input=encoded(p)+'\n').split()));q=list(zip(out[1::2],out[2::2]));boundary={v for v in p if any(on(u,w,v)for u,w in zip(h,h[1:]+h[:1]))};assert out[0]==len(q)==len(set(q))and set(q)==boundary;assert q[0]==min(q,key=lambda v:(v[1],v[0]));assert area(q)==area(h)
  elif name=='polygon_area':assert math.isclose(float(run([str(binary)],input=encoded(h)+'\n')),float(area(h)),abs_tol=1e-8)
  elif name=='point_in_polygon':assert list(map(int,run([str(binary)],input=encoded(h)+'\n10\n'+flat(queries)+'\n').split()))==[location(h,q)for q in queries]
  elif name in ('ccw','projection'):
   got=run([str(binary)],input=flat([a,b])+'\n10\n'+flat(queries)+'\n').split()
   if name=='projection':want=[float(v)for q in queries for v in projection(a,b,q)];assert all(math.isclose(float(x),y,abs_tol=1e-8)for x,y in zip(got,want))and len(got)==len(want)
   else:
    def ccw(q):
     z=cross(a,b,q);dp=sum((x-u)*(y-u)for x,y,u in zip(b,q,a))
     return 'COUNTER_CLOCKWISE'if z>0 else 'CLOCKWISE'if z<0 else 'ONLINE_BACK'if dp<0 else 'ONLINE_FRONT'if d2(a,q)>d2(a,b)else 'ON_SEGMENT'
    assert got==[ccw(q)for q in queries]
  elif name=='convex_polygon_cut':
   lines=[(a,b),((0,0),(1,0)),((0,0),(0,1))];got=run([str(binary)],input=encoded(h)+'\n3\n'+flat([v for l in lines for v in l])+'\n').split();assert len(got)==3 and all(math.isclose(float(x),float(area(cut(h,*l))),abs_tol=1e-8)for x,l in zip(got,lines))
  else:
   segments=[]
   while len(segments)<10:
    c,d,e,f=[(rng.randrange(-30,31),rng.randrange(-30,31))for _ in range(4)]
    if name=='line_intersection'and (c==d or e==f or intersection(c,d,e,f)is None):continue
    segments.append((c,d,e,f))
   got=run([str(binary)],input='10\n'+flat([v for line in segments for v in line])+'\n').split()
   if name=='segment_intersection':assert list(map(int,got))==[int(intersects(*v))for v in segments]
   else:
    want=[float(v)for line in segments for v in intersection(*line)]if name=='line_intersection'else [math.sqrt(float(distance(*v)))for v in segments]
    assert len(got)==len(want)and all(math.isclose(float(x),y,abs_tol=1e-8)for x,y in zip(got,want)),(name,got,want)

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--compiler',default=os.environ.get('CXX','g++'));parser.add_argument('--sanitize',action='store_true');args=parser.parse_args();compiler=shlex.split(args.compiler)
 flags=['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',str(ROOT)];runtime=['-O2','-g']if not args.sanitize else ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all','-fno-pie','-no-pie']
 headers=sorted(str(p.relative_to(ROOT))for p in (ROOT/'computational_geometry').rglob('*.hpp'))
 for header in headers:
  for extra in [[],['-DNDEBUG']]:run(compiler+flags+extra+['-fsyntax-only','-x','c++','-'],input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
 print(f'{len(headers)} standalone headers (debug/release) passed',flush=True)
 with tempfile.TemporaryDirectory(prefix='cp-computational-geometry-')as directory:
  temp=Path(directory)
  def compile(source,suffix='',extra=()):
   binary=temp/(source.stem+suffix);run(compiler+flags+list(extra)+runtime+[str(source),'-o',str(binary)]);return binary
  includes=''.join(f'#include "{h}"\n'for h in reversed(headers))+'#include "string_algorithm/all.hpp"\n#include "linear_algebra/all.hpp"\n#include "set_function/all.hpp"\n'
  (temp/'one.cpp').write_text(includes+'int one(){return cp::geometry::exact::orientation({0,0},{1,0},{0,1})-1;}\n');(temp/'two.cpp').write_text('#include "string_algorithm/all.hpp"\n'+includes+'int one();int main(){return one();}\n');run(compiler+flags+runtime+[str(temp/'one.cpp'),str(temp/'two.cpp'),'-o',str(temp/'multi')]);run([str(temp/'multi')])
  for name,want in EXAMPLES.items():
   got=run([str(compile(ROOT/f'examples/computational_geometry/{name}.cpp'))]);assert got==want,(name,got,want)
  for source in sorted((ROOT/'verify/computational_geometry').glob('*.test.cpp')):protocol(source.name.removesuffix('.test.cpp'),compile(source))
  print('mixed includes, multiple TUs, 12 examples, 12 judge wrappers / 768 cases passed',flush=True)
  for suffix,extra in [('_debug',[]),('_release',['-DNDEBUG'])]:
   count=arbitrary_precision(compile(ROOT/'verify/computational_geometry/query_driver.cpp',suffix,extra));print(f'{suffix[1:]}: {count} Python Fraction geometry cases passed',flush=True)
   print(run([str(compile(ROOT/'verify/computational_geometry/unit_test.cpp',suffix,extra))]),end='',flush=True)
if __name__=='__main__':main()
