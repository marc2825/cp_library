#!/usr/bin/env python3
"""O/B: exact optimization, oracle algorithms, ordered omega field and integration."""
import argparse
import json
import os
from pathlib import Path
import re
import random
from fractions import Fraction
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
    names = ['README.md', 'docs/README.md', 'docs/optimization_extensions/README.md', 'docs/optimization_extensions/references.md',
             'visualizations/README.md', 'visualizations/optimization_extensions/README.md']
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
    import itertools
    rng=random.Random(20261029)
    cases=[];checks=[]
    def add(case,check):cases.append(case);checks.append(check)
    def words(values):return ' '.join(map(str,values))
    def solve(matrix,rhs):
        n=len(rhs);a=[[Fraction(x) for x in row]+[Fraction(b)] for row,b in zip(matrix,rhs)]
        for j in range(n):
            pivot=next((i for i in range(j,n) if a[i][j]),None)
            if pivot is None:return None
            a[j],a[pivot]=a[pivot],a[j];d=a[j][j];a[j]=[x/d for x in a[j]]
            for i in range(n):
                if i!=j:
                    d=a[i][j];a[i]=[x-d*y for x,y in zip(a[i],a[j])]
        return [row[-1] for row in a]
    def dot(a,b):return sum(x*y for x,y in zip(a,b))
    def vertex_max(a,b,c):
        n=len(c);best=None
        for ids in itertools.combinations(range(len(a)),n):
            x=solve([a[i] for i in ids],[b[i] for i in ids])
            if x is not None and all(dot(row,x)<=v for row,v in zip(a,b)):
                value=dot(c,x);best=value if best is None else max(best,value)
        return best
    for mode,count in [('L',100),('D',50),('I',80)]:
        for _ in range(count):
            n=2 if mode=='D' else rng.randrange(1,4);m=rng.randrange(0,5)
            a=[[rng.randrange(-3,4) for _ in range(n)] for _ in range(m)];b=[rng.randrange(-4,5) for _ in range(m)];c=[rng.randrange(-3,4) for _ in range(n)]
            original_a=[row[:] for row in a];original_b=b[:]
            if mode!='I':
                for i in range(n):
                    row=[0]*n;row[i]=1;a.append(row);b.append(3)
                    if mode=='D':row=[0]*n;row[i]=-1;a.append(row);b.append(3)
                vertices_a=[row[:] for row in a];vertices_b=b[:]
                if mode=='L':
                    for i in range(n):row=[0]*n;row[i]=-1;vertices_a.append(row);vertices_b.append(0)
                expected=vertex_max(vertices_a,vertices_b,c)
            else:
                values=[dot(c,x) for x in itertools.product(range(-2,3),repeat=n) if all(dot(row,x)<=v for row,v in zip(a,b))]
                expected=max(values) if values else None
            case=f'{mode} {n} {len(a)} '+words([x for row in a for x in row]+b+c)
            if mode=='I':case+=' '+words([-2]*n+[2]*n)
            def check(t,want=expected,mode=mode,a=a,b=b,c=c):
                assert t[0]==('infeasible' if want is None else 'optimal'),t
                if want is not None:
                    assert Fraction(t[1])==want,(t,want)
                    if mode!='D':
                        n=int(t[2]);x=list(map(Fraction,t[3:3+n]));assert len(x)==len(c) and dot(c,x)==want
                        assert all(dot(row,x)<=v for row,v in zip(a,b))
                        if mode=='I':assert all(v.denominator==1 and -2<=v<=2 for v in x)
            add(case,check)
    def independent_set(s,groups,cap):return all(sum(groups[e]==g for e in s)<=cap[g] for g in range(3))
    for _ in range(80):
        n=rng.randrange(0,9);g1=[rng.randrange(3) for _ in range(n)];g2=[rng.randrange(3) for _ in range(n)];c1=[rng.randrange(3) for _ in range(3)];c2=[rng.randrange(3) for _ in range(3)];w=[rng.randrange(-10,11) for _ in range(n)];best={}
        for mask in range(1<<n):
            s=[i for i in range(n) if mask>>i&1]
            if independent_set(s,g1,c1) and independent_set(s,g2,c2):best[len(s)]=max(best.get(len(s),-10**9),sum(w[e] for e in s))
        def check(t,best=best):assert list(map(int,t))==[max(best.values()),len(best)]+[best[k] for k in range(len(best))],(t,best)
        add(f'W {n} '+words(g1+g2+c1+c2+w),check)
    for _ in range(50):
        n=rng.randrange(0,7);k=rng.randrange(1,4);groups=[[rng.randrange(3) for _ in range(n)] for _ in range(k)];caps=[[rng.randrange(3) for _ in range(3)] for _ in range(k)];best=0
        for assignment in itertools.product(range(-1,k),repeat=n):
            if all(independent_set([i for i in range(n) if assignment[i]==j],groups[j],caps[j]) for j in range(k)):best=max(best,sum(j>=0 for j in assignment))
        def check(t,best=best,groups=groups,caps=caps,n=n,k=k):
            assert int(t[0])==best and int(t[1])==n,t
            assignment=list(map(int,t[2:]));assert len(assignment)==n and sum(j>=0 for j in assignment)==best
            assert all(-1<=j<k for j in assignment)
            assert all(independent_set([i for i in range(n) if assignment[i]==j],groups[j],caps[j]) for j in range(k))
        add(f'U {n} {k} '+words([x for g,c in zip(groups,caps) for x in g+c]),check)
    for mode,count in [('S',60),('B',40)]:
        for _ in range(count):
            n=rng.randrange(0,7);modular=[rng.randrange(-5,6) for _ in range(n)];edges=[[rng.randrange(4) for _ in range(n)] for _ in range(n)];weights=[rng.randrange(-3,4) for _ in range(n)]
            values=[2+sum(modular[i] for i in range(n) if mask>>i&1)+sum(edges[i][j] for i in range(n) for j in range(n) if mask>>i&1 and not mask>>j&1) for mask in range(1<<n)]
            case=f'{mode} {n} '+words(modular+[x for row in edges for x in row])
            if mode=='S':
                def check(t,values=values):
                    assert Fraction(t[0])==Fraction(t[1])==min(values),t
                    chosen=list(map(int,t[3:]));assert len(chosen)==int(t[2]) and values[sum(1<<i for i in chosen)]==min(values)
            else:
                case+=' '+words(weights);best=None
                for order in itertools.permutations(range(n)):
                    mask=0;score=0
                    for e in order:score+=weights[e]*(values[mask|1<<e]-values[mask]);mask|=1<<e
                    best=score if best is None else max(best,score)
                def check(t,values=values,weights=weights,n=n,best=best):
                    base=list(map(Fraction,t[2:]));assert int(t[1])==n and len(base)==n
                    assert Fraction(t[0])==dot(weights,base)==best
                    assert sum(base)==values[-1]-values[0]
                    assert all(sum(base[i] for i in range(n) if mask>>i&1)<=values[mask]-values[0] for mask in range(1<<n))
            add(case,check)
    def trim(p):
        while p and not p[-1]:p.pop()
        return p
    def plus(a,b):return trim([(a[i] if i<len(a) else 0)+(b[i] if i<len(b) else 0) for i in range(max(len(a),len(b)))])
    def times(a,b):
        if not a or not b:return []
        p=[Fraction(0)]*(len(a)+len(b)-1)
        for i,x in enumerate(a):
            for j,y in enumerate(b):p[i+j]+=x*y
        return trim(p)
    def divide(a,b):
        a=a[:];q=[Fraction(0)]*max(0,len(a)-len(b)+1)
        while a and len(a)>=len(b):
            k=len(a)-len(b);v=a[-1]/b[-1];q[k]=v
            for j,c in enumerate(b):a[k+j]-=v*c
            trim(a)
        return trim(q),a
    def canonical(p,q):
        a=p[:];b=q[:]
        while b:a,b=b,divide(a,b)[1]
        p=divide(p,a)[0];q=divide(q,a)[0];scale=q[-1]
        return [v/scale for v in p],[v/scale for v in q]
    for _ in range(80):
        polys=[]
        for j in range(4):
            p=[Fraction(rng.randrange(-4,5)) for _ in range(rng.randrange(1,4))];p[-1]=rng.choice([Fraction(-2),Fraction(1),Fraction(3)]);polys.append(p)
        p,q,r,s=polys;op=rng.choice('+-*/')
        if op in '+-':num=plus(times(p,s),[v*(1 if op=='+' else -1) for v in times(r,q)]);den=times(q,s)
        elif op=='*':num=times(p,r);den=times(q,s)
        else:num=times(p,s);den=times(q,r)
        num,den=canonical(num,den)
        def check(t,num=num,den=den):
            sign=int(t[0]);count=int(t[1]);a=list(map(Fraction,t[2:2+count]));rest=t[2+count:];b=list(map(Fraction,rest[1:]));assert int(rest[0])==len(b)
            assert a==num and b==den and sign==(0 if not num else 1 if num[-1]>0 else -1),(t,num,den)
        add('O '+op+' '+' '.join(str(len(p))+' '+words(p) for p in polys),check)
    binary=compile(ROOT/'verify/optimization_extensions/query_driver.cpp','query-driver',extra)
    lines=run([binary],input=str(len(cases))+'\n'+'\n'.join(cases)+'\n').splitlines()
    assert len(lines)==len(checks)
    for case,line,check in zip(cases,lines,checks):
        try:check(line.split())
        except AssertionError as e:raise AssertionError((case,line,e)) from e
    print(('release' if extra else 'debug')+f': {len(cases)} independent Python Fraction/enumeration cases passed',flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    parser.add_argument('--links-only', action='store_true')
    args = parser.parse_args()
    entries = json.loads((ROOT / 'verify/optimization_extensions/manifest.json').read_text())['algorithms']
    if args.links_only:
        links(entries)
        return
    flags = shlex.split(args.compiler) + ['-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-I', ROOT]
    runtime = ['-O1', '-g', '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer', '-fno-pie', '-no-pie'] if args.sanitize else ['-O2']
    headers = sorted({e['header'] for e in entries} | {'basic_utility/detail/rational_polynomial.hpp'})
    for header in headers:
        for extra in [[], ['-DNDEBUG']]:
            run(flags + extra + ['-x', 'c++', '-fsyntax-only', '-'], input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    print(f'{len(headers)} standalone/repeated headers in debug/release passed', flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-optimization-extensions-') as folder:
        temp = Path(folder)
        def compile(source, name, extra=()):
            binary = temp / name
            run(flags + runtime + list(extra) + [source, '-o', binary])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes += ''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in sorted(ROOT.glob('*/all.hpp')))
        (temp / 'a.cpp').write_text(includes + 'int first(){return cp::OmegaRational::omega()>cp::OmegaRational(1);}\n')
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
        assert observed == json.loads((ROOT / 'verify/optimization_extensions/visualization_expected.json').read_text())
        print('all-category translation units and 9 documented/visualization examples passed', flush=True)
        for extra in [[], ['-DNDEBUG']]:
            for name in ['unit_test']:
                print(run([compile(ROOT / f'verify/optimization_extensions/{name}.cpp', name, extra)]).strip(), flush=True)
            independent(compile, extra, run)
    links(entries)


if __name__ == '__main__':
    main()
