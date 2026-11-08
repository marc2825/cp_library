#!/usr/bin/env python3
"""F-11 BigInt: Boost wrapper contracts and Python independent comparisons."""
import argparse
import json
import math
import os
from pathlib import Path
import random
import re
import shlex
import subprocess
import sys
import tempfile
from urllib.parse import unquote

ROOT=Path(__file__).resolve().parents[1]
if hasattr(sys,'set_int_max_str_digits'):
    sys.set_int_max_str_digits(0)

def run(command,**kwargs):
    result=subprocess.run(list(map(str,command)),capture_output=True,text=True,timeout=300,**kwargs)
    if result.returncode:
        raise RuntimeError(f'{shlex.join(map(str,command))}\n{result.stdout}{result.stderr}')
    return result.stdout

def oracle(binary):
    rng=random.Random(111026)
    inputs,expected=[],[]
    def operation(op,a,b):
        inputs.append(f'{op} {a} {b}')
        if op=='Q':
            q=abs(a)//abs(b)
            if (a<0)!=(b<0):q=-q
            value=f'{q} {a-q*b}'
        elif op=='C':value=f'{int(a<b)} {int(a==b)} {int(a>b)}'
        elif op=='A':value=str(a+b)
        elif op=='S':value=str(a-b)
        elif op=='M':value=str(a*b)
        elif op=='G':value=str(math.gcd(a,b))
        else:value=str(abs(a//math.gcd(a,b)*b) if a and b else 0)
        expected.append(value)
    def pair(a,b):
        for op in 'ASMGLC':operation(op,a,b)
        if b:operation('Q',a,b)
        for x in [a,b]:
            h=('-' if x<0 else '')+format(abs(x),'x')
            inputs.append(f'D {x}');expected.append(f'{x} {h}')
            token=('-' if x<0 else '+')+'0X'+format(abs(x),'X')
            inputs.append(f'H {token}');expected.append(f'{x} {h}')
    for _ in range(400):
        a=rng.getrandbits(rng.randrange(0,4097))*(-1 if rng.randrange(2) else 1)
        b=rng.getrandbits(rng.randrange(0,4097))*(-1 if rng.randrange(2) else 1)
        pair(a,b)
    for power in [1,9,18,63,64,65,127,128,129,256,2048]:
        for sign in [-1,1]:
            for delta in [-1,0,1]:pair(sign*((1<<power)+delta),sign*((1<<power)-1))
    pair(0,0)
    for digits in [1000,10000]:
        a=int('9'*digits);b=int('1234567890'*(digits//10))
        for sign in [-1,1]:pair(sign*a,b)
        operation('Q',a*b+123,b)
        # Random large numbers exercise unequal limbs and gcd beyond patterned inputs.
        a=int('1'+''.join(str(rng.randrange(10)) for _ in range(digits-1)))
        b=int('3'+''.join(str(rng.randrange(10)) for _ in range(digits-1)))
        pair(a,b)
    for digits in [100,1000,10000]:
        inputs.append('D +'+'0'*digits+'9');expected.append('9 9')
    actual=run([binary],input='\n'.join(inputs)).splitlines()
    assert actual==expected,next(((i,a[:100],b[:100]) for i,(a,b) in enumerate(zip(actual,expected)) if a!=b),(len(actual),len(expected)))
    print(f'{len(expected)} Python comparisons, up to 10000-digit inputs / 20000-digit products passed',flush=True)

def links():
    files=['README.md','docs/README.md','docs/basic_utility/README.md','docs/basic_utility/big_integer.md','docs/basic_utility/references.md',
        'visualizations/README.md','visualizations/big_integer/README.md']
    count=0
    for name in files:
        file=ROOT/name;text=re.sub(r'```.*?```','',file.read_text(),flags=re.S);text=re.sub(r'`[^`]*`','',text)
        for target in re.findall(r'\]\(([^)]+)\)',text):
            if '://' in target or target.startswith(('#','mailto:')):continue
            assert (file.parent/unquote(target.split('#',1)[0])).exists(),(file,target)
            count+=1
    print(f'{count} document links passed',flush=True)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler',default=os.environ.get('CXX','g++'))
    parser.add_argument('--sanitize',action='store_true')
    parser.add_argument('--links-only',action='store_true')
    parser.add_argument('--benchmark',action='store_true',help='also run a non-gating local timing sample')
    args=parser.parse_args()
    if args.links_only:links();return
    flags=shlex.split(args.compiler)+['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',ROOT]
    runtime=['-O1','-g','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-fno-pie','-no-pie'] if args.sanitize else ['-O2']
    with tempfile.TemporaryDirectory(prefix='cp-big-integer-') as folder:
        temp=Path(folder)
        def compile(source,name,extra=()):
            binary=temp/name;run(flags+runtime+list(extra)+[source,'-o',binary]);return binary
        header='#include "basic_utility/big_integer.hpp"\n'*2
        for extra in [[],['-DNDEBUG']]:run(flags+extra+['-x','c++','-fsyntax-only','-'],input=header+'int main(){}\n')
        includes=header+''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in sorted(ROOT.glob('*/all.hpp')))
        (temp/'a.cpp').write_text(includes+'cp::BigInt first(){return cp::BigInt("12345678901234567890");}\n')
        (temp/'b.cpp').write_text(includes+'cp::BigInt first();int main(){return first()!=cp::BigInt("12345678901234567890");}\n')
        run(flags+runtime+[temp/'a.cpp',temp/'b.cpp','-o',temp/'multi']);run([temp/'multi'])
        print('standalone/repeated header, all-category includes and two translation units passed',flush=True)
        doc=(ROOT/'docs/basic_utility/big_integer.md').read_text()
        assert 'documentation_of: //basic_utility/big_integer.hpp' in doc
        example=ROOT/'examples/basic_utility/big_integer.cpp'
        assert re.search(r'```cpp\n(.*?)```',doc,re.S).group(1)==example.read_text()
        assert run([compile(example,'example')])=='1272750402189130710322005854 52\n90\n-ff\n-3 -2\n'
        actual={}
        for line in run([compile(ROOT/'verify/big_integer/visualization_bridge.cpp','bridge')]).splitlines():
            mode,result=line.split(' ',1);actual[mode]=json.loads(result)
        assert actual==json.loads((ROOT/'verify/big_integer/visualization_expected.json').read_text())
        print('documented example and 4 C++ visualization references passed',flush=True)
        for extra in [[],['-DNDEBUG']]:
            print(run([compile(ROOT/'verify/big_integer/unit_test.cpp','unit',extra)]).strip(),flush=True)
            oracle(compile(ROOT/'verify/big_integer/query_driver.cpp','driver',extra))
            print(('release' if extra else 'debug')+' passed',flush=True)
        if args.benchmark:
            print(run([compile(ROOT/'verify/big_integer/benchmark.cpp','benchmark')]).strip(),flush=True)
    links()

if __name__=='__main__':main()
