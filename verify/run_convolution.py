#!/usr/bin/env python3
"""3.16 NTT, modular/exact integer convolution, real/complex FFT."""
import argparse
import math
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = {
    'ntt': '10 6 15 7 \n1 2 3 4 \n',
    'convolution': '4 998244350 2 15 \n4 14 2 15 \n',
    'convolution_ll': '4000000000 4999999992 2 15 \n',
    'fft': '1.000 2.000 3.000 4.000 \n3.000,0.000 -1.000,2.000 \n',
    'convolution_fft': '1.000 -0.500 -3.000 \n-1.000,1.000 0.000,2.000 \n',
}

def run(command, **kwargs):
    result = subprocess.run(command, text=True, capture_output=True, **kwargs)
    if result.returncode:
        raise RuntimeError(f'{shlex.join(map(str, command))}\n{result.stdout}{result.stderr}')
    return result.stdout


def product(a,b):
    if not a or not b: return []
    c = [0]*(len(a)+len(b)-1)
    for i,x in enumerate(a):
        for j,y in enumerate(b): c[i+j] += x*y
    return c


def protocol(name,binary):
    rng = random.Random(316016)
    for trial in range(72):
        n,m = rng.randrange(1,55),rng.randrange(1,55)
        if name in ('ntt','convolution'):
            a = [rng.randrange(998244353) for _ in range(n)]
            b = [rng.randrange(998244353) for _ in range(m)]
            data = f'{n} {m}\n'+' '.join(map(str,a))+'\n'+' '.join(map(str,b))+'\n'
            expected = [x%998244353 for x in product(a,b)]
        else:
            a = [0]+[rng.randrange(1,1001) for _ in range(n)]
            b = [0]+[rng.randrange(1,1001) for _ in range(n)]
            data = f'{n}\n'+''.join(f'{x} {y}\n'for x,y in zip(a[1:],b[1:]))
            expected = product(a,b)[1:]
        actual = list(map(int,run([str(binary)],input=data).split()))
        assert actual == expected,(name,data,actual,expected)


def arbitrary_precision(binary):
    rng = random.Random(316016); commands,expected=[],[]
    lo,hi = -2**63,2**63-1
    extreme = [lo,lo+1,-hi,-2**32,-1224736769,-469762049,-167772161,-1,0,1,2,167772161,469762049,1224736769,2**32,hi-1,hi]
    cases = [(a,b) for a in [[],[0],[1],[-1],[lo],[hi],[lo,hi]] for b in [[],[0],[1],[-1],[2],[lo],[hi]]]
    cases += [([x],[y]) for x in extreme for y in extreme]
    for _ in range(520):
        scale = rng.choice([10,10**6,10**12,2**63])
        cases.append(([rng.randrange(-scale,min(scale,2**63-1)+1)for _ in range(rng.randrange(45))],
                      [rng.randrange(-scale,min(scale,2**63-1)+1)for _ in range(rng.randrange(45))]))
    for a,b in cases:
        c = product(a,b)
        for kind in 'MSLF':
            # Large arbitrary integers belong to exact NTT/CRT checks, not a floating accuracy claim.
            if kind=='F' and any(abs(x)>10**6 for x in a+b): continue
            commands.append(f'{kind} {len(a)} {len(b)}\n'+' '.join(map(str,a))+'\n'+' '.join(map(str,b))+'\n')
            if kind=='S' and c and len(c)>16: value='LENGTH'
            elif kind=='L' and a and b and min(sum(map(abs,a))*max(map(abs,b)),sum(map(abs,b))*max(map(abs,a)))>2**63: value='OVERFLOW'
            elif kind=='L' and any(x<lo or x>hi for x in c): value='OVERFLOW'
            else:
                value = [x%(998244353 if kind=='M' else 17)for x in c] if kind in 'MS' else c
            expected.append((kind,value))
    actual = run([str(binary)],input=''.join(commands)).splitlines()
    assert len(actual)==len(expected),(len(actual),len(expected))
    for index,(line,(kind,want)) in enumerate(zip(actual,expected)):
        if isinstance(want,str): assert line==want,(index,line,want)
        else:
            tokens=line.split();assert int(tokens[0])==len(want),(index,line,want)
            if kind=='F':
                got=list(map(float,tokens[1:]));assert len(got)==len(want)
                assert all(abs(x-y)<=0.002+abs(y)*1e-12 for x,y in zip(got,want)),(index,got,want)
            else: assert list(map(int,tokens[1:]))==want,(index,line,want)
    return len(cases),len(expected)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler',default=os.environ.get('CXX','g++'));parser.add_argument('--sanitize',action='store_true');args=parser.parse_args()
    compiler=shlex.split(args.compiler);flags=['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',str(ROOT)]
    runtime=['-O2','-g'] if not args.sanitize else ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all','-fno-pie','-no-pie']
    headers=sorted(str(p.relative_to(ROOT))for p in (ROOT/'convolution').rglob('*.hpp'))
    for header in headers:
        for extra in [[],['-DNDEBUG']]:run(compiler+flags+extra+['-fsyntax-only','-x','c++','-'],input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    print(f'standalone headers (debug/release): {len(headers)} passed',flush=True)
    for declaration in ['cp::ntt<15,2>(a);','cp::ntt<17,4>(a);','cp::ntt<1,1>(a);','cp::ntt<2000000011,2>(a);']:
        result=subprocess.run(compiler+flags+['-fsyntax-only','-x','c++','-'],input='#include "convolution/ntt.hpp"\nint main(){std::vector<int>a;'+declaration+'}\n',text=True,capture_output=True)
        assert result.returncode and 'static assertion failed' in result.stderr,(declaration,result.stderr)
    print('4 invalid modulus/root configurations rejected at compile time',flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-convolution-')as directory:
        temp=Path(directory)
        def compile(source,suffix='',extra=()):
            binary=temp/(source.stem+suffix);run(compiler+flags+list(extra)+runtime+[str(source),'-o',str(binary)]);return binary
        includes=''.join(f'#include "{h}"\n'for h in reversed(headers))+'#include "combinatorics/all.hpp"\n#include "number_theory_convolution/all.hpp"\n#include "dp_optimization/all.hpp"\n#include "tree/all.hpp"\n'
        (temp/'first.cpp').write_text(includes+'int first(){std::vector<int>a{1};cp::ntt(a);return a[0]-1;}\n')
        (temp/'second.cpp').write_text(includes+'int first();int main(){return first();}\n')
        run(compiler+flags+runtime+[str(temp/'first.cpp'),str(temp/'second.cpp'),'-o',str(temp/'multi')]);run([str(temp/'multi')])
        print('reverse/mixed includes and multiple translation units passed',flush=True)
        for name,want in EXAMPLES.items():
            actual=run([str(compile(ROOT/f'examples/convolution/{name}.cpp'))]);assert actual==want,(name,actual,want)
        for source in sorted((ROOT/'verify/convolution').glob('*.test.cpp')):protocol(source.name.removesuffix('.test.cpp'),compile(source))
        print('5 examples; 5 judge wrappers / 360 independent protocol cases passed',flush=True)
        for suffix,extra in [('_debug',[]),('_release',['-DNDEBUG'])]:
            cases,outputs=arbitrary_precision(compile(ROOT/'verify/convolution/query_driver.cpp',suffix,extra))
            print(f'{suffix[1:]}: {cases} integer pairs / {outputs} arbitrary-precision protocol outputs passed',flush=True)
            print(run([str(compile(ROOT/'verify/convolution/unit_test.cpp',suffix,extra))]),end='',flush=True)
if __name__=='__main__': main()
