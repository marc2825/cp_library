#!/usr/bin/env python3
"""3.25: deterministic RNG, exact integer ranges and independent search simulations."""
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
    'random': '16294208416658607535\n7960286522194355700\n-7\n1\n',
    'timer': '1.9\n1 1\n', 'simulated_annealing': '0 0\n',
    'beam_search': '7 5\n', 'local_search': '3 0\n',
}

def run(command, **kwargs):
    result = subprocess.run(command, text=True, capture_output=True, **kwargs)
    if result.returncode:
        raise RuntimeError(f'{shlex.join(map(str, command))}\n{result.stdout}{result.stderr}')
    return result.stdout

class SplitMix:
    def __init__(self, seed): self.state = seed
    def raw(self):
        mask = 2**64-1
        self.state = (self.state + 0x9e3779b97f4a7c15) & mask
        z = self.state
        z = ((z ^ (z >> 30)) * 0xbf58476d1ce4e5b9) & mask
        z = ((z ^ (z >> 27)) * 0x94d049bb133111eb) & mask
        return z ^ (z >> 31)
    def uniform(self, bound):
        threshold = 2**64 % bound
        while True:
            value = self.raw()
            if value >= threshold: return value % bound
    def integer(self, lo, hi):
        span = hi-lo+1
        return lo + (self.raw() if span == 2**64 else self.uniform(span))
    def real(self): return (self.raw() >> 11) / 2**53

def independent(binary):
    rng = random.Random(325025)
    cases = []
    for i in range(450):
        seed = rng.getrandbits(64)
        kind = 'RUIX'[i % 4]
        oracle = SplitMix(seed)
        if kind == 'R':
            command = f'R {seed} 24'; want = [oracle.raw() for _ in range(24)]
        elif kind == 'U':
            bound = rng.choice([1, 3, 7, 2**63, 2**63+1, 2**64-1, rng.randrange(1,2**64)])
            command = f'U {seed} {bound} 24'; want = [oracle.uniform(bound) for _ in range(24)]
        else:
            low, high = (-2**63, 2**63-1) if kind == 'I' else (0,2**64-1)
            if i % 7:
                lo, hi = sorted([rng.randint(low, high), rng.randint(low, high)])
            else: lo, hi = low, high
            command = f'{kind} {seed} {lo} {hi} 24'; want = [oracle.integer(lo,hi) for _ in range(24)]
        cases.append((command, want))
    for i in range(700):
        n = rng.randrange(1,16); start = rng.randrange(n); goal = rng.randrange(2)
        scores = [rng.choice([-2**63, 2**63-1, 2**63-2, rng.randint(-100,100)]) for _ in range(n)]
        graph = [[rng.randrange(n) for _ in range(rng.randrange(7))] for _ in range(n)]
        cap = rng.randrange(12); width_or_mode = rng.randrange(1,9) if i % 2 else rng.randrange(2)
        kind = 'B' if i % 2 else 'L'
        command = f'{kind} {n} {start} {goal} {width_or_mode} {cap} ' + ' '.join(map(str,scores))
        command += ' ' + ' '.join(str(len(row))+' '+' '.join(map(str,row)) for row in graph)
        better = (lambda a,b: a>b) if goal else (lambda a,b: a<b)
        if kind == 'L':
            current = start; scans = accepted = 0; evaluations = 1; termination = 0
            for _ in range(cap):
                chosen = current
                for child in graph[current]:
                    evaluations += 1
                    if better(scores[child],scores[chosen]):
                        chosen = child
                        if width_or_mode == 0: break
                scans += 1
                if chosen == current: termination = 2; break
                current = chosen; accepted += 1
            want = [current, scans, evaluations, accepted, termination]
        else:
            frontier = [start]; depth = 0; evaluations = 1; termination = 1
            for _ in range(cap):
                children = [child for parent in frontier for child in graph[parent]]
                evaluations += len(children)
                if not children: termination = 3; break
                frontier = sorted(children,key=lambda x:scores[x],reverse=bool(goal))[:width_or_mode]
                depth += 1
            want = [depth, evaluations, termination, len(frontier), *frontier]
        cases.append((command,want))
    for i in range(400):
        n = rng.randrange(1,16); start = rng.randrange(n); goal = rng.randrange(2); cap = rng.randrange(60)
        cooling = rng.randrange(2); seed = rng.getrandbits(64); oracle = SplitMix(seed)
        scores = [rng.randint(-100,100) for _ in range(n)]
        proposals = [rng.randrange(n) for _ in range(cap)]
        command = f'S {n} {start} {goal} {cap} 12 0.125 {cooling} {seed} ' + ' '.join(map(str,scores+proposals))
        current = best = start; accepted = 0
        for step, candidate in enumerate(proposals):
            progress = step/(cap-1) if cap > 1 else 0
            temperature = 12*(1-progress)+0.125*progress if cooling else 12*(0.125/12)**progress
            gain = (scores[candidate]-scores[current])*(1 if goal else -1)
            if gain >= 0 or oracle.real() < math.exp(gain/temperature):
                current = candidate; accepted += 1
                if (scores[current]-scores[best])*(1 if goal else -1) > 0: best = current
        cases.append((command, [best,current,cap,cap+1,accepted,0]))
    lines = run([str(binary)], input='\n'.join(command for command,_ in cases)+'\n').splitlines()
    assert len(lines) == len(cases)
    for line,(command,want) in zip(lines,cases):
        actual = list(map(int,line.split()))
        assert actual == want, (command,actual,want)
    return len(cases)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=os.environ.get('CXX','g++'))
    parser.add_argument('--sanitize', action='store_true')
    args = parser.parse_args(); compiler = shlex.split(args.compiler)
    flags = ['-std=c++17','-Wall','-Wextra','-Wpedantic','-Werror','-I',str(ROOT)]
    runtime = ['-O2','-g'] if not args.sanitize else ['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all','-fno-pie','-no-pie']
    headers = sorted(str(p.relative_to(ROOT)) for p in (ROOT/'heuristic_search').glob('*.hpp'))
    for header in headers:
        for extra in [[],['-DNDEBUG']]:
            run(compiler+flags+extra+['-fsyntax-only','-x','c++','-'],input=f'#include "{header}"\n#include "{header}"\nint main(){{}}\n')
    for source, message in [
        ('cp::Random r; r.integer(true,false);','Random::integer needs'),
        ('cp::Random r; r.integer(0.0,1.0);','Random::integer needs'),
        ('cp::local_search(0,[](int){return true;},[](int){return std::vector<int>{};});','Search needs a numeric score'),
    ]:
        rejected = subprocess.run(compiler+flags+['-fsyntax-only','-x','c++','-'],input='#include "heuristic_search/all.hpp"\nint main(){'+source+'}\n',text=True,capture_output=True)
        assert rejected.returncode and message in rejected.stderr
    print(f'{len(headers)} standalone/repeated headers (debug/release), invalid types rejected',flush=True)
    with tempfile.TemporaryDirectory(prefix='cp-heuristic-search-') as directory:
        temp = Path(directory)
        def compile(source, suffix='', extra=()):
            binary = temp/(source.stem+suffix)
            run(compiler+flags+list(extra)+runtime+[str(source),'-o',str(binary)])
            return binary
        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        includes += '#include "game_theory/all.hpp"\n#include "combinatorial_optimization/all.hpp"\n#include "string_algorithm/all.hpp"\n'
        (temp/'one.cpp').write_text(includes+'int one(){return cp::Random::min();}\n')
        (temp/'two.cpp').write_text('#include "game_theory/all.hpp"\n'+includes+'int one();int main(){return one();}\n')
        run(compiler+flags+runtime+[str(temp/'one.cpp'),str(temp/'two.cpp'),'-o',str(temp/'multi')]);run([str(temp/'multi')])
        for name,want in EXAMPLES.items():
            actual = run([str(compile(ROOT/f'examples/heuristic_search/{name}.cpp'))])
            assert actual == want, (name,actual,want)
        print('mixed/reversed includes, multiple TUs and 5 examples passed',flush=True)
        for suffix,extra in [('_debug',[]),('_release',['-DNDEBUG'])]:
            count = independent(compile(ROOT/'verify/heuristic_search/query_driver.cpp',suffix,extra))
            print(f'{suffix[1:]}: {count} independent Python RNG/full-64-bit/search cases passed',flush=True)
            print(run([str(compile(ROOT/'verify/heuristic_search/unit_test.cpp',suffix,extra))]),end='',flush=True)

if __name__ == '__main__': main()
