#!/usr/bin/env python3
"""H: 22 geometry APIs: exact/Fraction oracles, examples, and header integration."""
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
    names = ['README.md', 'docs/README.md', 'docs/computational_geometry/README.md',
             'docs/geometry_extensions/README.md', 'docs/geometry_extensions/references.md',
             'visualizations/README.md', 'visualizations/geometry_extensions/README.md']
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
    rng = random.Random(20261028)
    driver = compile(ROOT / 'verify/geometry_extensions/query_driver.cpp', 'query-driver', extra)
    judge = compile(ROOT / 'verify/geometry_extensions/area_of_union_of_rectangles.test.cpp', 'judge', extra)
    def det(a,b,c): return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    def circle(p):
        a,b,c = [[q[i]-p[3][i] for i in range(2)] for q in p[:3]]
        cross = lambda a,b:a[0]*b[1]-a[1]*b[0]
        return sum(x*x for x in a)*cross(b,c)-sum(x*x for x in b)*cross(a,c)+sum(x*x for x in c)*cross(a,b)
    sign = lambda x: (x>0)-(x<0)
    cases, expected = [], []
    for trial in range(160):
        if trial<80:
            p=[(rng.randrange(-2**63,2**63),rng.randrange(-2**63,2**63)) for _ in range(4)]
            cases.append('P '+' '.join(str(x) for q in p for x in q))
        else:
            exponent=rng.choice([-1000,-500,-50,0,50,500,1000])
            f=[(rng.randrange(-16,17)*2.0**exponent,rng.randrange(-16,17)*2.0**exponent) for _ in range(4)]
            cases.append('F '+' '.join(x.hex() for q in f for x in q));p=[tuple(map(Fraction,q)) for q in f]
        expected.append([str(sign(det(*p[:3]))),str(sign(circle(p)))])
    def measure(rectangles):
        xs=sorted({x for r in rectangles for x in (r[0],r[2])});ys=sorted({y for r in rectangles for y in (r[1],r[3])})
        cells={(i,j) for i in range(len(xs)-1) for j in range(len(ys)-1) if any(r[0]<=xs[i] and xs[i+1]<=r[2] and r[1]<=ys[j] and ys[j+1]<=r[3] for r in rectangles)}
        area=sum((xs[i+1]-xs[i])*(ys[j+1]-ys[j]) for i,j in cells)
        perimeter=sum((ys[j+1]-ys[j]) if di else (xs[i+1]-xs[i]) for i,j in cells for di,dj in [(-1,0),(1,0),(0,-1),(0,1)] if (i+di,j+dj) not in cells)
        return area,perimeter
    for trial in range(48):
        rectangles=[]
        for _ in range(1+rng.randrange(8)):
            x1,x2=sorted(rng.sample(range(-100,101),2));y1,y2=sorted(rng.sample(range(-100,101),2));rectangles.append((x1,y1,x2,y2))
        area,perimeter=measure(rectangles);cases.append('R '+str(len(rectangles))+' '+' '.join(str(x) for r in rectangles for x in r));expected.append(list(map(str,[area,perimeter])))
        assert int(run([judge],input=str(len(rectangles))+'\n'+'\n'.join(' '.join(map(str,r)) for r in rectangles)+'\n'))==area
    def hull(p):
        p=sorted(set(p))
        if len(p)<3:return p
        def chain(p):
            h=[]
            for x in p:
                while len(h)>1 and det(h[-2],h[-1],x)<=0:h.pop()
                h.append(x)
            return h
        return chain(p)[:-1]+chain(p[::-1])[:-1]
    def on(a,b,p):return det(a,b,p)==0 and min(a[0],b[0])<=p[0]<=max(a[0],b[0]) and min(a[1],b[1])<=p[1]<=max(a[1],b[1])
    def inside(p,q):
        if not p:return False
        if len(p)<3:return on(p[0],p[-1],q)
        return all(det(p[i],p[(i+1)%len(p)],q)>=0 for i in range(len(p)))
    frac=lambda x:f'{x.numerator}/{x.denominator}'
    for trial in range(80):
        p=hull([(Fraction(rng.randrange(-20,21),rng.randrange(1,6)),Fraction(rng.randrange(-20,21),rng.randrange(1,6))) for _ in range(6)])
        cases.append('L '+str(len(p))+' '+' '.join(frac(x) for q in p for x in q));expected.append([str(sum(inside(p,(x,y)) for x in range(-20,21) for y in range(-20,21)))])
        cases.append('C '+str(len(p))+' '+' '.join(frac(x) for q in p for x in q))
        twice=sum(p[i][0]*p[(i+1)%len(p)][1]-p[i][1]*p[(i+1)%len(p)][0] for i in range(len(p)))
        expected.append([frac(sum((p[i][j]+p[(i+1)%len(p)][j])*(p[i][0]*p[(i+1)%len(p)][1]-p[i][1]*p[(i+1)%len(p)][0]) for i in range(len(p)))/(3*twice)) for j in range(2)] if twice else ['none'])
    def intersect(a,b,c,d):
        u=tuple(b[i]-a[i] for i in range(2));v=tuple(d[i]-c[i] for i in range(2));den=u[0]*v[1]-u[1]*v[0]
        if den:
            t=((c[0]-a[0])*v[1]-(c[1]-a[1])*v[0])/den;s=((c[0]-a[0])*u[1]-(c[1]-a[1])*u[0])/den
            return [(a[0]+u[0]*t,a[1]+u[1]*t)] if 0<=t<=1 and 0<=s<=1 else []
        return sorted({p for p in (a,b,c,d) if on(a,b,p) and on(c,d,p)})
    for trial in range(48):
        segments=[[(Fraction(rng.randrange(-5,6)),Fraction(rng.randrange(-5,6))) for _ in range(2)] for _ in range(6)]
        cuts=[set(s) for s in segments]
        for i in range(len(segments)):
            for j in range(i):
                ps=intersect(*segments[i],*segments[j]);cuts[i].update(ps);cuts[j].update(ps)
        vertices=sorted(set.union(*cuts));edges=set()
        for cut in cuts:
            cut=sorted(cut);edges.update(zip(cut,cut[1:]))
        cases.append('A '+str(len(segments))+' '+' '.join(frac(x) for s in segments for p in s for x in p));expected.append([str(len(vertices)),str(len(edges)),*[frac(x) for p in vertices for x in p]])
    got=run([driver],input=str(len(cases))+'\n'+'\n'.join(cases)+'\n')
    assert [line.split() for line in got.splitlines()]==expected
    print(('release' if extra else 'debug')+f': Python Fraction/int {len(cases)} independent cases and 48 rectangle judge inputs passed',flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    parser.add_argument('--links-only', action='store_true')
    args = parser.parse_args()
    entries = json.loads((ROOT / 'verify/geometry_extensions/manifest.json').read_text())['algorithms']
    if args.links_only:
        links(entries)
        return
    flags = shlex.split(args.compiler) + ['-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-I', ROOT]
    runtime = ['-O1', '-g', '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer', '-fno-pie', '-no-pie'] if args.sanitize else ['-O2']
    headers = sorted({e['header'] for e in entries})
    for header in headers:
        for extra in [[], ['-DNDEBUG']]:
            run(flags + extra + ['-x', 'c++', '-fsyntax-only', '-'], input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    print(f'{len(headers)} standalone/repeated headers in debug/release passed', flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-geometry-extensions-') as folder:
        temp = Path(folder)
        def compile(source, name, extra=()):
            binary = temp / name
            run(flags + runtime + list(extra) + [source, '-o', binary])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes += ''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in sorted(ROOT.glob('*/all.hpp')))
        (temp / 'a.cpp').write_text(includes + 'int first(){return cp::geometry::exact::rectangle_union_area({{0,0,2,1}}).to_integral<int>(); }\n')
        (temp / 'b.cpp').write_text(includes + 'int first();int main(){return first()!=2;}\n')
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
        assert observed == json.loads((ROOT / 'verify/geometry_extensions/visualization_expected.json').read_text())
        print('all-category translation units and 22 documented/visualization examples passed', flush=True)
        for extra in [[], ['-DNDEBUG']]:
            for name in ['measure_test', 'topology_test', 'index_test', 'spatial_test']:
                print(run([compile(ROOT / f'verify/geometry_extensions/{name}.cpp', name, extra)]).strip(), flush=True)
            independent(compile, extra, run)
    links(entries)


if __name__ == '__main__':
    main()
