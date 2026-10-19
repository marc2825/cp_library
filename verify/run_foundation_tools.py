#!/usr/bin/env python3
"""F-12..F-17: rational, arena, bits, grid, rollback and radix sort."""
import argparse
import json
from fractions import Fraction
import os
from pathlib import Path
import random
import re
import shlex
import subprocess
import tempfile
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]


def run(command, **kwargs):
    result = subprocess.run(list(map(str,command)),capture_output=True,text=True,timeout=300,**kwargs)
    if result.returncode:
        raise RuntimeError(f'{shlex.join(map(str,command))}\n{result.stdout}{result.stderr}')
    return result.stdout


def oracle(binary):
    rng=random.Random(12171026)
    inputs,expected=[],[]
    def fraction(x):return f'{x.numerator}/{x.denominator}'
    for i in range(1200):
        bits=2048 if i<12 else rng.randrange(1,513)
        a,b,c,d=[rng.getrandbits(bits)*rng.choice([-1,1]) for _ in range(4)]
        b=b or 1;d=d or 1
        x,y=Fraction(a,b),Fraction(c,d)
        inputs.append(f'R {a}/{b} {c}/{d}')
        expected.append(' '.join([fraction(x+y),fraction(x-y),fraction(x*y),fraction(x/y) if y else 'none',str(int(x<y)),str(int(x==y))]))
    for value in [0,(1<<64)-1]+[1<<i for i in range(64)]+[rng.getrandbits(64) for _ in range(500)]:
        bits=[i for i in range(64) if value>>i&1]
        next_value=None
        for pivot in range(1,64):
            lower=sum(i<pivot for i in bits)
            if not (value>>pivot&1) and lower:
                next_value=((value>>(pivot+1))<<(pivot+1))+(1<<pivot)+(1<<(lower-1))-1
                break
        inputs.append(f'B {value}')
        expected.append(f'{max(bits) if bits else -1} {min(bits) if bits else -1} {len(bits)} {next_value if next_value is not None else "none"}')
    for _ in range(150):
        values=[rng.randrange(-(1<<127),1<<127) if rng.randrange(3) else rng.randrange(-3,4) for _ in range(rng.randrange(100))]
        inputs.append(' '.join(map(str,['S',len(values)]+values)))
        expected.append(' '.join(map(str,sorted(range(len(values)),key=lambda i:values[i]))))
    actual=run([binary],input='\n'.join(inputs)).splitlines()
    assert actual==expected,next(((i,a[:100],b[:100]) for i,(a,b) in enumerate(zip(actual,expected)) if a!=b),(len(actual),len(expected)))
    print(f'{len(expected)} Python Fraction/bit/radix comparisons passed',flush=True)


def links(entries):
    paths=[ROOT/p for p in ['README.md','HANDOFF.md','docs/README.md','docs/remaining_work.md','docs/implementation_plan.md',
        'docs/basic_utility/README.md','docs/graph_tree.md',
        'docs/foundation_tools/README.md','docs/foundation_tools/references.md',
        'visualizations/README.md','visualizations/foundation_tools/README.md']]
    paths += [ROOT/'docs'/e['doc'] for e in entries]
    count=0
    for path in paths:
        text=re.sub(r'```.*?```','',path.read_text(),flags=re.S)
        text=re.sub(r'`[^`]*`','',text)
        for target in re.findall(r'\]\(([^)]+)\)',text):
            if '://' in target or target.startswith(('#','mailto:')):
                continue
            assert (path.parent/unquote(target.split('#',1)[0])).exists(),(path,target)
            count+=1
    print(f'{count} local document links passed',flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler',default=os.environ.get('CXX','g++'))
    parser.add_argument('--sanitize',action='store_true')
    parser.add_argument('--links-only',action='store_true')
    args=parser.parse_args()
    manifest=json.loads((ROOT/'verify/foundation_tools/manifest.json').read_text())
    entries=manifest['algorithms']
    if args.links_only:
        links(entries);return
    flags=shlex.split(args.compiler)+['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',ROOT]
    runtime=['-O1','-g','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-fno-pie','-no-pie'] if args.sanitize else ['-O2']
    headers=[e['header'] for e in entries]+manifest['extra_headers']
    for header in headers:
        for extra in [[],['-DNDEBUG']]:
            run(flags+extra+['-x','c++','-fsyntax-only','-'],input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    print('7 standalone/repeated headers in debug/release passed',flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-foundation-tools-') as folder:
        temp=Path(folder)
        def compile(source,name,extra=()):
            binary=temp/name;run(flags+runtime+list(extra)+[source,'-o',binary]);return binary
        includes=''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes+=''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in sorted(ROOT.glob('*/all.hpp')))
        (temp/'a.cpp').write_text(includes+'cp::BigRational first(){return cp::BigRational(1,2);}\n')
        (temp/'b.cpp').write_text(includes+'cp::BigRational first();int main(){return first()!=cp::BigRational(1,2);}\n')
        run(flags+runtime+[temp/'a.cpp',temp/'b.cpp','-o',temp/'multi']);run([temp/'multi'])
        for e in entries:
            source=ROOT/'examples'/e['example'];doc=(ROOT/'docs'/e['doc']).read_text()
            assert 'documentation_of: //'+e['header'] in doc
            assert re.search(r'```cpp\n(.*?)```',doc,re.S).group(1)==source.read_text(),e['doc']
            assert run([compile(source,e['mode'])])==e['output'],e['mode']
        print('all-category translation units and 6 documented examples passed',flush=True)
        observed={}
        for line in run([compile(ROOT/'verify/foundation_tools/visualization_bridge.cpp','bridge')]).splitlines():
            name,value=line.split(' ',1);observed[name]=json.loads(value)
        assert observed==json.loads((ROOT/'verify/foundation_tools/visualization_expected.json').read_text())
        print('6 C++/visualization reference cases passed',flush=True)
        for extra in [[],['-DNDEBUG']]:
            print(run([compile(ROOT/'verify/foundation_tools/unit_test.cpp','unit',extra)]).strip(),flush=True)
            oracle(compile(ROOT/'verify/foundation_tools/query_driver.cpp','driver',extra))
            print(('release' if extra else 'debug')+' passed',flush=True)
    links(entries)


if __name__=='__main__':
    main()
