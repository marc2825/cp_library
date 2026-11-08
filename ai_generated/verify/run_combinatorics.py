#!/usr/bin/env python3
"""3.15: factorials, binomial/ordered/multiset selections, Catalan, inclusion-exclusion."""
import argparse
import itertools
import math
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = {'factorials':'120\n808333339\n0\n', 'binomial':'10\n8\n',
            'permutation_combination':'60\n10\n', 'catalan':'1 1 2 5 14 42 \n2\n',
            'multiset_combination':'15\n1 0\n', 'inclusion_exclusion':'4\n1\n'}


def run(command, **kwargs):
    result = subprocess.run(command, text=True, capture_output=True, **kwargs)
    if result.returncode: raise RuntimeError(f"{shlex.join(map(str, command))}\n{result.stdout}{result.stderr}")
    return result.stdout


def bracket_count(length):
    if length % 2: return 0
    result = 0
    for positions in itertools.combinations(range(length), length//2):
        chosen = set(positions)
        if all(2*sum(i <= end for i in chosen) >= end+1 for end in range(length)): result += 1
    return result


def compositions(types, total):
    if types == 0: return int(total == 0)
    return sum(compositions(types-1,total-take) for take in range(total+1))


def protocol(name,binary):
    rng = random.Random(315016)
    for trial in range(64):
        if name == 'factorials':
            word = ''.join(rng.choice('abc') for _ in range(rng.randrange(1,9)))
            data = word+'\n'; expected = [len(set(itertools.permutations(word)))]
        elif name == 'binomial':
            queries = [(rng.randrange(101),rng.randrange(101)) for _ in range(10)]
            queries = [(max(a,b),min(a,b)) for a,b in queries]
            data = f'{len(queries)}\n'+''.join(f'{n} {k}\n' for n,k in queries)
            expected = [math.comb(n,k)%1000000007 for n,k in queries]
        elif name == 'permutation_combination':
            m = rng.randrange(1,6); n = rng.randrange(1,min(m,3)+1)
            seqs = list(itertools.permutations(range(m),n))
            data = f'{n} {m}\n'; expected = [sum(all(x!=y for x,y in zip(a,b)) for a in seqs for b in seqs)]
        elif name == 'catalan':
            length = rng.randrange(1,15); data = f'{length}\n'; expected = [bracket_count(length)]
        elif name == 'multiset_combination':
            n,k = rng.randrange(1,6),rng.randrange(1,8); data = f'{n} {k}\n'; expected = [compositions(n,k)]
        elif name == 'inclusion_exclusion':
            upper = rng.randrange(20,301); primes = rng.sample([2,3,5,7,11,13],rng.randrange(1,7))
            data = f'{upper} {len(primes)}\n'+' '.join(map(str,primes))+'\n'
            expected = [sum(any(x%p==0 for p in primes) for x in range(1,upper+1))]
        else: raise AssertionError(name)
        actual = list(map(int,run([str(binary)],input=data).split()))
        assert actual == expected, (name,data,actual,expected)


def arbitrary_precision(binary):
    rng = random.Random(315016); commands,expected = [],[]
    moduli = [1,2,4,6,12,17,101,1022117,998244353,1000000007,4294967291,2**63-2,2**63-1]
    blocks = [('P',180,m) for m in moduli] + [('F',min(80,m-1) if m>1 else 80,m) for m in moduli]
    blocks += [(rng.choice('FP'),rng.randrange(81),rng.choice(moduli)) for _ in range(160)]
    for kind,bound,modulus in blocks:
        queries = []
        for _ in range(48):
            operation = rng.choice('CHA' if kind=='P' else 'CHAIFP')
            n = rng.choice([rng.randrange(-2,bound+3),0,1,bound,bound+1,2**63-1,-2**63])
            k = rng.choice([rng.randrange(-1,bound+2),0,1,n,n+1 if n<2**63-1 else n,2**63-1,-2**63])
            queries.append((operation,n,k))
        commands.append(f'{kind} {bound} {modulus} {len(queries)}\n'+''.join(f'{op} {n} {k}\n' for op,n,k in queries))
        if kind=='F' and math.gcd(math.factorial(bound),modulus)!=1:
            expected.append('DOMAIN');continue
        for operation,n,k in queries:
            if operation in 'FI':
                if n<0 or n>bound: value='RANGE'
                else:
                    value=math.factorial(n)%modulus
                    if operation=='I': value=pow(value,-1,modulus)
            elif operation=='A':
                if n<0:value=0
                elif 2*n>bound:value='RANGE'
                else:value=(math.comb(2*n,n)//(n+1))%modulus
            elif operation=='H':
                if n<0 or k<0:value=0
                elif k==0:value=1%modulus
                elif n==0:value=0
                elif n+k-1>bound:value='RANGE'
                else:value=math.comb(n+k-1,k)%modulus
            else:
                if n<0 or k<0 or k>n:value=0
                elif n>bound:value='RANGE'
                else:value=(math.comb(n,k) if operation=='C' else math.perm(n,k))%modulus
            expected.append(str(value))
    actual = run([str(binary)],input=''.join(commands)).splitlines()
    assert actual==expected,next(((i,a,b) for i,(a,b) in enumerate(zip(actual,expected)) if a!=b), (len(actual),len(expected)))
    return len(blocks),len(expected)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler',default=os.environ.get('CXX','g++'));parser.add_argument('--sanitize',action='store_true');args=parser.parse_args()
    compiler=shlex.split(args.compiler);flags=['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',str(ROOT)]
    runtime=['-O2','-g'] if not args.sanitize else ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all','-fno-pie','-no-pie']
    headers=sorted(str(p.relative_to(ROOT)) for p in (ROOT/'combinatorics').rglob('*.hpp'))
    for header in headers:
        for extra in [[],['-DNDEBUG']]:run(compiler+flags+extra+['-fsyntax-only','-x','c++','-'],input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    print(f'standalone headers (debug/release): {len(headers)} passed',flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-combinatorics-')as directory:
        temp=Path(directory)
        def compile(source,suffix='',extra=()):
            binary=temp/(source.stem+suffix);run(compiler+flags+list(extra)+runtime+[str(source),'-o',str(binary)]);return binary
        includes=''.join(f'#include "{h}"\n'for h in reversed(headers))+'#include "number_theory_convolution/all.hpp"\n#include "dp_optimization/all.hpp"\n#include "tree/all.hpp"\n#include "graph_combinatorics/all.hpp"\n'
        (temp/'first.cpp').write_text(includes+'int first(){return static_cast<int>(cp::factorials(0)[0])-1;}\n');(temp/'second.cpp').write_text(includes+'int first();int main(){return first();}\n')
        run(compiler+flags+runtime+[str(temp/'first.cpp'),str(temp/'second.cpp'),'-o',str(temp/'multi')]);run([str(temp/'multi')])
        print('reverse includes / existing categories / multiple translation units: passed',flush=True)
        for name,expected in EXAMPLES.items():
            actual=run([str(compile(ROOT/f'examples/combinatorics/{name}.cpp'))]);assert actual==expected,(name,actual,expected)
        for source in sorted((ROOT/'verify/combinatorics').glob('*.test.cpp')):protocol(source.name.removesuffix('.test.cpp'),compile(source))
        print('6 examples; 6 judge wrappers / 384 independent protocol cases passed',flush=True)
        for suffix,extra in [('_debug',[]),('_release',['-DNDEBUG'])]:
            blocks,cases=arbitrary_precision(compile(ROOT/'verify/combinatorics/query_driver.cpp',suffix,extra))
            print(f'{suffix[1:]}: {blocks} modulus/table blocks / {cases} arbitrary-precision outputs passed',flush=True)
            print(run([str(compile(ROOT/'verify/combinatorics/unit_test.cpp',suffix,extra))]),end='',flush=True)
if __name__=='__main__':main()
