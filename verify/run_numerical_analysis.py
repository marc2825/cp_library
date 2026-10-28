#!/usr/bin/env python3
"""Y: numerical APIs, exact and high-precision oracles, examples, and header integration."""
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
    names = ['README.md', 'HANDOFF.md', 'docs/README.md', 'docs/remaining_work.md',
             'docs/implementation_plan.md', 'docs/numerical_analysis/README.md', 'docs/numerical_analysis/references.md',
             'visualizations/README.md', 'visualizations/numerical_analysis/README.md']
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
    from decimal import Decimal, localcontext
    rng = random.Random(20261028)
    cases, checks = [], []
    def number(actual, expected, tolerance=Decimal('2e-9')):
        actual=Decimal(actual);expected=Decimal(expected)
        assert abs(actual-expected)<=tolerance*(1+abs(expected)), (actual,expected)
    def output_check(expected,status=True):
        def check(tokens):
            if status:
                assert tokens[0] in ('converged','stagnation'),tokens
                tokens=tokens[1:]
            assert len(tokens)==len(expected),(tokens,expected)
            for actual,want in zip(tokens,expected):number(actual,want)
        return check
    with localcontext() as ctx:
        ctx.prec=90
        for _ in range(90):
            a,b,c=[rng.randrange(-50,51) for _ in range(3)];disc=b*b-4*a*c;all_real=False
            if not a:
                roots=[] if not b else [-Decimal(c)/b];all_real=not b and not c
            elif disc<0:roots=[]
            elif not disc:roots=[-Decimal(b)/(2*a)]
            else:roots=sorted([(-Decimal(b)-Decimal(disc).sqrt())/(2*a),(-Decimal(b)+Decimal(disc).sqrt())/(2*a)])
            cases.append(f'Q {a} {b} {c}')
            def check(t,roots=roots,all_real=all_real):
                assert t[:3]==['converged',str(int(all_real)),str(len(roots))],t
                for actual,want in zip(t[3:],roots):number(actual,want)
            checks.append(check)
        for _ in range(40):
            a=Decimal(rng.randrange(1,10000))/100
            for method in range(3):cases.append(f'R {method} {a}');checks.append(output_check([a.sqrt()]))
        for method in range(2):
            for _ in range(40):
                # Adaptive estimates can cancel for arbitrary signed polynomials;
                # that limitation and manual splitting are covered by unit_test.
                p=[rng.randrange(-5,6) for _ in range(4)] if not method else [rng.randrange(1,6) for _ in range(7)]
                a,b=sorted(rng.sample(range(-2,3),2)) if not method else (0,rng.randrange(1,4))
                integral=sum(Fraction(c*(b**(i+1)-a**(i+1)),i+1) for i,c in enumerate(p))
                cases.append(f'I {method} {len(p)} {a} {b} '+' '.join(map(str,p)));checks.append(output_check([Decimal(integral.numerator)/integral.denominator]))
        for _ in range(90):
            n=rng.randrange(1,10);l=[rng.randrange(-4,5) for _ in range(n-1)];d=[rng.randrange(-4,5) for _ in range(n)];u=[rng.randrange(-4,5) for _ in range(n-1)];b=[rng.randrange(-4,5) for _ in range(n)]
            matrix=[[Fraction(d[i] if i==j else l[j] if i==j+1 else u[i] if j==i+1 else 0) for j in range(n)]+[Fraction(b[i])] for i in range(n)]
            singular=False
            for j in range(n):
                pivot=next((i for i in range(j,n) if matrix[i][j]),None)
                if pivot is None:singular=True;break
                matrix[j],matrix[pivot]=matrix[pivot],matrix[j];divisor=matrix[j][j];matrix[j]=[x/divisor for x in matrix[j]]
                for i in range(n):
                    if i!=j:
                        factor=matrix[i][j];matrix[i]=[x-factor*y for x,y in zip(matrix[i],matrix[j])]
            cases.append(f'T {n} '+' '.join(map(str,l+d+u+b)))
            if singular:checks.append(lambda t: t==['singular'] or (_ for _ in ()).throw(AssertionError(t)))
            else:checks.append(output_check([Decimal(row[-1].numerator)/row[-1].denominator for row in matrix]))
        for _ in range(40):
            roots=sorted(rng.sample(range(-6,7),rng.randrange(1,5)));p=[1]
            for root in roots:
                for repeat in range(1+rng.randrange(2)):
                    q=[0]*(len(p)+1)
                    for i,c in enumerate(p):q[i]-=root*c;q[i+1]+=c
                    p=q
            if rng.randrange(2):
                q=[0]*(len(p)+2)
                for i,c in enumerate(p):q[i]+=c;q[i+2]+=c
                p=q
            cases.append(f'P {len(p)} '+' '.join(map(str,p)))
            def check(t,roots=roots):
                assert t[:2]==['converged',str(len(roots))],t
                assert len(t)==2+4*len(roots)
                for i,root in enumerate(roots):
                    lo,hi,count,value=t[2+4*i:6+4*i];lo=Fraction(lo);hi=Fraction(hi)
                    assert lo<=root<=hi and count=='1',(lo,root,hi,count)
                    number(value,root)
            checks.append(check)
        for _ in range(40):
            large=2**rng.randrange(80,200);small=rng.randrange(-100,101)
            cases.append(f'S 3 {large} {small} {-large}');checks.append(output_check([small],False))
            values=[rng.randrange(-1000,1001) for _ in range(8)];m=max(values)
            expected=Decimal(m)+sum((Decimal(x-m).exp() for x in values),Decimal(0)).ln()
            cases.append('L 8 '+' '.join(map(str,values)));checks.append(output_check([expected],False))
    binary=compile(ROOT/'verify/numerical_analysis/query_driver.cpp','query-driver',extra)
    lines=run([binary],input=str(len(cases))+'\n'+'\n'.join(cases)+'\n').splitlines()
    assert len(lines)==len(checks)
    for case,line,check in zip(cases,lines,checks):
        try:check(line.split())
        except AssertionError as e:raise AssertionError((case,line,e)) from e
    print(('release' if extra else 'debug')+f': {len(cases)} independent Python Decimal/Fraction cases passed',flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    parser.add_argument('--links-only', action='store_true')
    args = parser.parse_args()
    entries = json.loads((ROOT / 'verify/numerical_analysis/manifest.json').read_text())['algorithms']
    if args.links_only:
        links(entries)
        return
    flags = shlex.split(args.compiler) + ['-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror', '-I', ROOT]
    runtime = ['-O1', '-g', '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer', '-fno-pie', '-no-pie'] if args.sanitize else ['-O2']
    headers = sorted({e['header'] for e in entries} | {'basic_utility/binary_rational.hpp', 'numerical_analysis/all.hpp'})
    for header in headers:
        for extra in [[], ['-DNDEBUG']]:
            run(flags + extra + ['-x', 'c++', '-fsyntax-only', '-'], input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    print(f'{len(headers)} standalone/repeated headers in debug/release passed', flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-numerical-analysis-') as folder:
        temp = Path(folder)
        def compile(source, name, extra=()):
            binary = temp / name
            run(flags + runtime + list(extra) + [source, '-o', binary])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes += ''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in sorted(ROOT.glob('*/all.hpp')))
        (temp / 'a.cpp').write_text(includes + 'int first(){return cp::numerical::quadratic_roots(1,-5,6).roots.size();}\n')
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
        assert observed == json.loads((ROOT / 'verify/numerical_analysis/visualization_expected.json').read_text())
        print('all-category translation units and 11 documented/visualization examples passed', flush=True)
        for extra in [[], ['-DNDEBUG']]:
            for name in ['unit_test']:
                print(run([compile(ROOT / f'verify/numerical_analysis/{name}.cpp', name, extra)]).strip(), flush=True)
            independent(compile, extra, run)
    links(entries)


if __name__ == '__main__':
    main()
