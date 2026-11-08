#!/usr/bin/env python3
"""3.9: iterative Dinic, minimum cut and Hopcroft-Karp matching."""
import argparse
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]


def run(command, **kwargs):
    result=subprocess.run(command,text=True,capture_output=True,**kwargs)
    if result.returncode:raise RuntimeError(f"{shlex.join(map(str,command))}\n{result.stdout}{result.stderr}")
    return result.stdout


def cut_value(n, edges, s, t):
    return min(sum(c for u,v,c in edges if (mask>>u)&1 and not (mask>>v)&1)
               for mask in range(1<<n) if (mask>>s)&1 and not (mask>>t)&1)


def matching_value(left, right, edges):
    states={0}
    for u in range(left):
        states|={mask|(1<<v) for mask in states for a,v in edges if a==u and not (mask>>v)&1}
    return max(mask.bit_count() for mask in states)


def verify_protocol(name, binary):
    rng=random.Random(39003)
    for trial in range(64):
        if name=="max_flow":
            n=rng.randrange(2,9)
            edges=[(rng.randrange(n),rng.randrange(n),rng.randrange(10)) for _ in range(rng.randrange(1,30))]
            data=f"{n} {len(edges)}\n"+''.join(f"{u} {v} {c}\n" for u,v,c in edges)
            output=run([str(binary)],input=data)
            assert output==f"{cut_value(n,edges,0,n-1)}\n"
        else:
            left,right=rng.randrange(1,8),rng.randrange(1,8)
            candidates=[(u,v) for u in range(left) for v in range(right)]
            rng.shuffle(candidates)
            edges=candidates[:rng.randrange(int(name=="bipartite_matching"),len(candidates)+1)]
            data=f"{left} {right} {len(edges)}\n"+''.join(f"{u} {v}\n" for u,v in edges)
            lines=run([str(binary)],input=data).splitlines()
            expected=matching_value(left,right,edges)
            assert int(lines[0])==expected
            if name=="matching_size":assert len(lines)==1
            else:
                pairs=[tuple(map(int,line.split())) for line in lines[1:]]
                assert len(pairs)==expected
                assert len({u for u,v in pairs})==len({v for u,v in pairs})==expected
                assert all(pair in edges for pair in pairs)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler",default=os.environ.get("CXX","g++"))
    parser.add_argument("--sanitize",action="store_true")
    args=parser.parse_args();compiler=shlex.split(args.compiler)
    flags=["-std=c++17","-Wall","-Wextra","-Wpedantic","-Werror","-I",str(ROOT)]
    runtime=["-O2","-g"] if not args.sanitize else ["-O1","-g","-fsanitize=address,undefined","-fno-omit-frame-pointer","-fno-sanitize-recover=all","-fno-pie","-no-pie"]
    headers=sorted(str(p.relative_to(ROOT)) for p in (ROOT/"graph_flow").glob("*.hpp"))
    for header in headers:
        for extra in [[],["-DNDEBUG"]]:
            run(compiler+flags+extra+["-fsyntax-only","-x","c++","-"],input=f'#include "{header}"\n#include "{header}"\nint main() {{}}\n')
    print(f"standalone headers (debug/release): {len(headers)} passed",flush=True)
    with tempfile.TemporaryDirectory(prefix="cp-graph-flow-") as directory:
        temp=Path(directory)
        def compile(source,suffix="",extra=()):
            binary=temp/(source.stem+suffix);run(compiler+flags+list(extra)+runtime+[str(source),"-o",str(binary)]);return binary
        includes=''.join(f'#include "{h}"\n' for h in reversed(headers))+'#include "graph/all.hpp"\n#include "graph_basic/all.hpp"\n#include "tree/all.hpp"\n'
        (temp/"first.cpp").write_text(includes+'int first(){return cp::Dinic<>(0).size();}\n')
        (temp/"second.cpp").write_text(includes+'int first();int main(){return first();}\n')
        run(compiler+flags+runtime+[str(temp/"first.cpp"),str(temp/"second.cpp"),"-o",str(temp/"multi")]);run([str(temp/"multi")])
        print("reverse includes / existing graph+tree / multiple translation units: passed",flush=True)
        for name,want in [("dinic","2\n3\n0\n3/3 2/2 1/1 2/2 3/3 \n"),("min_cut","5\n0 \n0 1 \n"),("bipartite_matching","3\n0 1\n1 0\n2 2\n")]:
            assert run([str(compile(ROOT/f"examples/graph_flow/{name}.cpp"))])==want,name
        for source in sorted((ROOT/"verify/graph_flow").glob("*.test.cpp")):
            verify_protocol(source.name.removesuffix(".test.cpp"),compile(source))
        print("3 examples; 3 judge wrappers / 192 independent Python protocol cases passed",flush=True)
        for suffix,extra in [("_debug",[]),("_release",["-DNDEBUG"])]:
            print(suffix[1:]+":\n"+run([str(compile(ROOT/"verify/graph_flow/unit_test.cpp",suffix,extra))]),end="",flush=True)


if __name__=="__main__":main()
