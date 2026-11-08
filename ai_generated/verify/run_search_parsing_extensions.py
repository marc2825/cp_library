#!/usr/bin/env python3
"""E/X: reversible and parallel search, distributions, expression ASTs and automata."""
import argparse
import json
import os
from pathlib import Path
import re
import random
import shlex
import subprocess
import tempfile
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]


def run(command, **kwargs):
    result = subprocess.run(list(map(str, command)), capture_output=True, text=True, timeout=300, **kwargs)
    if result.returncode:
        raise RuntimeError(f'{shlex.join(map(str, command))}\n{result.stdout}{result.stderr}')
    return result.stdout


def links(entries):
    names = ['README.md', 'docs/README.md', 'docs/search_parsing_extensions/README.md', 'docs/search_parsing_extensions/references.md',
             'visualizations/README.md', 'visualizations/search_parsing_extensions/README.md']
    names += ['docs/' + e['doc'] for e in entries]
    count = 0
    for name in names:
        path = ROOT / name
        source = re.sub(r'```.*?```', '', path.read_text(), flags=re.S)
        source = re.sub(r'`[^`]*`', '', source)
        for target in re.findall(r'\]\(([^)]+)\)', source):
            if '://' in target or target.startswith(('#', 'mailto:')):
                continue
            assert (path.parent / unquote(target.split('#', 1)[0])).exists(), (name, target)
            count += 1
    print(f'{count} local document links passed', flush=True)


def independent(compile, extra, run):
    import random, math, heapq
    rng=random.Random(771);cases=[];wants=[];floats=[]
    def add(text,expected,floating=False):cases.append(text);wants.append(str(expected) if isinstance(expected,(str,int)) else ' '.join(map(str,expected)));floats.append(floating)
    for _ in range(150):
        a,b,c=[rng.randrange(-100,101) for _ in range(3)]
        expression=f'({a}+{b})*({c}) - 2^3^2'
        add('P '+json.dumps(expression),(a+b)*c-512)
    for _ in range(140):
        n=rng.randrange(2,10);tour=list(range(n));rng.shuffle(tour);w=[[rng.randrange(-5,50) for _ in range(n)] for _ in range(n)];cost=lambda t:sum(w[t[i]][t[(i+1)%n]] for i in range(n))
        kind=rng.randrange(2);changed=tour[:]
        if kind==0:
            l=rng.randrange(n+1);r=rng.randrange(l,n+1);changed[l:r]=reversed(changed[l:r]);move=[l,r]
        else:
            l=rng.randrange(n);k=rng.randrange(1,min(3,n-l,n-1)+1);after=rng.choice([i for i in range(n) if not l<=i<l+k]);move=[l,k,after]
            if after!=(l+n-1)%n:
                block=tour[l:l+k];remainder=tour[:l]+tour[l+k:];at=remainder.index(tour[after])+1;changed=remainder[:at]+block+remainder[at:]
        add('T '+' '.join(map(str,[n,kind,*tour,*sum(w,[]),*move])),[cost(changed)-cost(tour),cost(changed),1])
    for _ in range(100):
        initial,target=rng.randrange(-5,6),rng.randrange(-5,6);depth,width=rng.randrange(7),rng.randrange(1,8);moves=[rng.randrange(-3,4) for _ in range(rng.randrange(1,4))];beam=[initial]
        for _ in range(depth):beam=sorted([x+m for x in beam for m in moves],key=lambda x:(x-target)**2)[:width]
        add('B '+' '.join(map(str,[initial,target,depth,width,len(moves),*moves])),[beam[0],(beam[0]-target)**2,initial])
    for _ in range(30):
        initial,target=rng.randrange(-5,6),rng.randrange(-5,6);depth,width=rng.randrange(6),rng.randrange(1,4);moves=[-1,2];queues=[[] for _ in range(depth)];serial=0;best=[None]*(depth+1);best[0]=initial
        if depth:heapq.heappush(queues[0],((initial-target)**2,serial,initial))
        while any(queues):
            for d in range(depth):
                for _ in range(width):
                    if not queues[d]:break
                    _,_,x=heapq.heappop(queues[d])
                    for m in moves:
                        y=x+m
                        if best[d+1] is None or (y-target)**2<(best[d+1]-target)**2:best[d+1]=y
                        if d+1<depth:serial+=1;heapq.heappush(queues[d+1],((y-target)**2,serial,y))
        add('C '+' '.join(map(str,[initial,target,depth,width,2,*moves])),[best[-1],(best[-1]-target)**2,depth])
    for _ in range(50):
        n=rng.randrange(1,20);ops=[(rng.randrange(2),rng.randrange(n)) for _ in range(40)];s=set()
        for op,x in ops:
            if op:s.add(x)
            else:s.discard(x)
        add('I '+' '.join(map(str,[n,len(ops),*sum((list(v) for v in ops),[])])),[len(s),*sorted(s)])
    for _ in range(50):
        n=rng.randrange(1,9);w=[rng.randrange(8) for _ in range(n)]
        if not any(w):w[0]=1
        add('A '+' '.join(map(str,[n,*w])),[n*x for x in w])
    for _ in range(120):
        n=rng.randrange(1,7);edges=[(rng.randrange(n),rng.randrange(-1,2),rng.randrange(n)) for _ in range(rng.randrange(15))];accept={i for i in range(n) if rng.randrange(2)};word=[rng.randrange(2) for _ in range(rng.randrange(8))]
        seen={(0,0)};queue=[(0,0)]
        for q,p in queue:
            for u,c,v in edges:
                if u!=q or c!=-1 and (p==len(word) or c!=word[p]):continue
                pair=(v,p+(c!=-1))
                if pair not in seen:seen.add(pair);queue.append(pair)
        answer=int(any((q,len(word)) in seen for q in accept))
        add('N '+' '.join(map(str,[n,2,len(edges),len(accept),*sorted(accept),*sum((list(e) for e in edges),[]),len(word),*word])),[answer]*3)
    import re
    for _ in range(100):
        pattern=rng.choice(['(ab|c)*d?','a*b+','[a-c]*','([^a]b?)+','(a|b)*a(a|b)(a|b)','(a?b+)*','a|',''])
        word=''.join(rng.choice('abcd') for _ in range(rng.randrange(9)))
        add('R '+json.dumps(pattern)+' '+json.dumps(word),int(re.fullmatch(pattern,word) is not None))
    for _ in range(50):
        bits=rng.randrange(1,12);loss=rng.random()*8;temperature=.1+rng.random()*4;n=1<<bits
        p=math.exp(-loss/temperature);count=sum((i+.5)/n<p for i in range(n))
        add(f'F {bits} {loss:.17g} {temperature:.17g}',count)
    mask=(1<<64)-1
    class SplitMix:
        def __init__(self,seed):self.state=seed
        def raw(self):
            self.state=(self.state+0x9e3779b97f4a7c15)&mask;z=self.state;z=((z^(z>>30))*0xbf58476d1ce4e5b9)&mask;z=((z^(z>>27))*0x94d049bb133111eb)&mask;return z^(z>>31)
        def uniform(self,n):
            threshold=((1<<64)-n)%n
            while True:
                x=self.raw()
                if x>=threshold:return x%n
        def opened(self):return ((self.raw()>>12)+.5)*2**-52
    for _ in range(50):
        seed=rng.randrange(1<<32);r=SplitMix(seed);radius=math.sqrt(-2*math.log(r.opened()));angle=2*math.pi*r.opened();normal=[radius*math.cos(angle),radius*math.sin(angle)];exponential=-math.log(r.opened())/2;truncated=1-math.log1p(r.opened()*math.expm1(-4))/2
        add(f'D {seed}',normal+[exponential,truncated],True)
    for _ in range(30):
        starts,workers,seed=rng.randrange(1,9),rng.randrange(1,5),rng.randrange(1<<32);seeds=SplitMix(seed);runs=[]
        for _ in range(starts):
            r=SplitMix(seeds.raw());values=[r.uniform(100) for _ in range(3)];x=min(values,key=lambda x:abs(x-37));runs.append((x,abs(x-37)))
        best=min(range(starts),key=lambda i:runs[i][1]);add(f'M {starts} {workers} {seed}',[best,*runs[best]])
    binary=compile(ROOT/'verify/search_parsing_extensions/query_driver.cpp','query',extra)
    actual=run([binary],input='\n'.join(cases)+'\n').splitlines()
    assert len(actual)==len(wants)
    for case,want,got,floating in zip(cases,wants,actual,floats):
        if floating:assert all(math.isclose(float(a),float(b),rel_tol=1e-12,abs_tol=1e-12) for a,b in zip(want.split(),got.split())),(case,want,got)
        else:assert want==got,(case,want,got)
    print(f'{"release" if extra else "debug"}: {len(cases)} independent Python expression/list/automata/regex/random comparisons passed',flush=True)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    parser.add_argument('--links-only', action='store_true')
    args = parser.parse_args()
    entries = json.loads((ROOT / 'verify/search_parsing_extensions/manifest.json').read_text())['algorithms']
    if args.links_only:
        links(entries)
        return
    flags = shlex.split(args.compiler) + ['-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-pthread', '-I', ROOT]
    runtime = ['-O1', '-g', '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer', '-fno-pie', '-no-pie'] if args.sanitize else ['-O2']
    headers = sorted({e['header'] for e in entries} | {'heuristic_search/detail/reversible_path.hpp', 'parsing/parse_error.hpp', 'parsing/parsing_template.hpp', 'parsing/parse_tree.hpp', 'parsing/all.hpp', 'heuristic_search/all.hpp'})
    for header in headers:
        for extra in [[], ['-DNDEBUG']]:
            run(flags + extra + ['-x', 'c++', '-fsyntax-only', '-'], input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    print(f'{len(headers)} standalone/repeated headers in debug/release passed', flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-search-parsing-extensions-') as folder:
        temp = Path(folder)
        def compile(source, name, extra=()):
            binary = temp / name
            run(flags + runtime + list(extra) + [source, '-o', binary])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes += ''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in sorted(ROOT.glob('*/all.hpp')))
        (temp / 'a.cpp').write_text(includes + 'int first(){return cp::evaluate_integer_expression(cp::parse_expression("1+1"))==2;}\n')
        (temp / 'b.cpp').write_text(includes + 'int first();int main(){return first()!=1;}\n')
        run(flags + runtime + [temp / 'a.cpp', temp / 'b.cpp', '-o', temp / 'multi'])
        run([temp / 'multi'])
        observed = {}
        for e in entries:
            source = ROOT / 'examples' / e['example']
            doc = (ROOT / 'docs' / e['doc']).read_text()
            assert 'documentation_of: //' + e['header'] in doc
            assert re.search(r'```cpp\n(.*?)```', doc, re.S).group(1) == source.read_text(), e['doc']
            output = run([compile(source, e['mode'])])
            assert output == e['output'], (e['mode'], output)
            observed[e['mode']] = output.split()
        assert observed == json.loads((ROOT / 'verify/search_parsing_extensions/visualization_expected.json').read_text())
        print('all-category translation units and 11 documented/visualization examples passed', flush=True)
        for extra in [[], ['-DNDEBUG']]:
            for name in ['unit_test']:
                print(run([compile(ROOT / f'verify/search_parsing_extensions/{name}.cpp', name, extra)]).strip(), flush=True)
            independent(compile, extra, run)
    links(entries)


if __name__ == '__main__':
    main()
