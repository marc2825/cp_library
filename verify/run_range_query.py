#!/usr/bin/env python3
"""3.3 range queries: standalone headers, naive differential tests and judge protocols."""
import argparse
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
MOD = 998244353


def run(command, **kwargs):
    result = subprocess.run(command, text=True, capture_output=True, **kwargs)
    if result.returncode:
        raise RuntimeError(f"command failed: {shlex.join(map(str, command))}\n{result.stdout}{result.stderr}")
    return result.stdout


def protocol(kind):
    rng = random.Random(33001)
    n, q = 37, 1200
    a = [(rng.randrange(MOD), rng.randrange(MOD)) for _ in range(n)] if kind == "composite" else [rng.randrange(100) for _ in range(n)]
    if kind == "bit":
        a = [0] * n
        initial = ""
    else:
        initial = " ".join(f"{x[0]} {x[1]}" if isinstance(x, tuple) else str(x) for x in a) + "\n"
    queries, answers = [], []
    for _ in range(q):
        update = rng.randrange(2) == 0
        l = rng.randrange(n)
        r = rng.randrange(l + 1, n + 1)
        if kind == "point":
            if update:
                x = rng.randrange(10000)
                queries.append(f"0 {l} {x}"); a[l] += x
            else:
                queries.append(f"1 {l} {r}"); answers.append(str(sum(a[l:r])))
        elif kind == "composite":
            if update:
                f = (rng.randrange(MOD), rng.randrange(MOD))
                queries.append(f"0 {l} {f[0]} {f[1]}"); a[l] = f
            else:
                x = rng.randrange(MOD); queries.append(f"1 {l} {r} {x}")
                for mul, add in a[l:r]:
                    x = (mul * x + add) % MOD
                answers.append(str(x))
        elif kind == "bit":
            if update:
                x = rng.randrange(100)
                queries.append(f"0 {l+1} {r} {x}")
                for i in range(l, r): a[i] += x
            else:
                queries.append(f"1 {l+1} {r}"); answers.append(str(sum(a[l:r])))
        elif update:
            mul, add = rng.randrange(MOD), rng.randrange(MOD)
            queries.append(f"0 {l} {r} {mul} {add}")
            for i in range(l, r): a[i] = (mul * a[i] + add) % MOD
        elif kind == "lazy":
            queries.append(f"1 {l} {r}"); answers.append(str(sum(a[l:r]) % MOD))
        else:
            queries.append(f"1 {l}"); answers.append(str(a[l]))
    return f"{n} {q}\n" + initial + "\n".join(queries) + "\n", "\n".join(answers) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=os.environ.get("CXX", "g++"))
    parser.add_argument("--sanitize", action="store_true")
    args = parser.parse_args()
    compiler = shlex.split(args.compiler)
    flags = ["-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-Werror", "-I", str(ROOT)]
    runtime = ["-O2", "-g"] if not args.sanitize else ["-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-sanitize-recover=all", "-fno-pie", "-no-pie"]
    headers = sorted(str(p.relative_to(ROOT)) for p in (ROOT / "range_query").glob("*.hpp"))
    headers += ["data_structure/segment_tree.hpp", "data_structure/binary_indexed_tree.hpp"]
    for header in headers:
        for extra in [[], ["-DNDEBUG"]]:
            run(compiler + flags + extra + ["-fsyntax-only", "-x", "c++", "-"], input=f'#include "{header}"\n#include "{header}"\nint main() {{}}\n')
    print(f"standalone headers (debug/release): {len(headers)} passed", flush=True)
    with tempfile.TemporaryDirectory(prefix="cp-range-query-") as directory:
        temp = Path(directory)

        def compile_source(source, suffix="", extra=()):
            binary = temp / (source.stem + suffix)
            run(compiler + flags + list(extra) + runtime + [str(source), "-o", str(binary)])
            return binary

        includes = ''.join(f'#include "{header}"\n' for header in reversed(headers))
        (temp / "first.cpp").write_text(includes + 'int first() { return cp::FenwickTree<int>(3).size(); }\n')
        (temp / "second.cpp").write_text(includes + 'int first(); int main() { return first() != 3; }\n')
        run(compiler + flags + runtime + [str(temp / "first.cpp"), str(temp / "second.cpp"), "-o", str(temp / "multi")])
        run([str(temp / "multi")])
        print("reverse include order / multiple translation units: passed", flush=True)
        for name, expected in [("fenwick", "6 2\n25 8\n"), ("segtree", "3 5\n3\n"), ("range_actions", "19 2\n")]:
            binary = compile_source(ROOT / f"examples/range_query/{name}.cpp")
            assert run([str(binary)]) == expected, name
        cases = [("point_add_range_sum", "point"), ("point_set_range_composite", "composite"), ("range_affine_range_sum", "lazy"), ("range_affine_point_get", "dual"), ("range_add_range_sum_aoj", "bit")]
        for name, kind in cases:
            binary = compile_source(ROOT / f"verify/range_query/{name}.test.cpp")
            sample, expected = protocol(kind)
            assert run([str(binary)], input=sample) == expected, name
        print("examples: 3 passed; judge protocols: 5 passed (1200 naive-oracle queries each)", flush=True)
        source = ROOT / "verify/range_query/unit_test.cpp"
        for suffix, extra in [("_debug", []), ("_release", ["-DNDEBUG"])]:
            binary = compile_source(source, suffix, extra)
            print(suffix[1:] + ": " + run([str(binary)]), end="", flush=True)


if __name__ == "__main__":
    main()
