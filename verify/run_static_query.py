#!/usr/bin/env python3
"""3.4: standalone headers, exhaustive/naive oracles, FIFO order and judge protocols."""
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


def protocols(kind):
    rng = random.Random(34001)
    if kind == "rmq":
        n, q = 137, 900
        a = [rng.randrange(10**9) for _ in range(n)]
        queries, expected = [], []
        for _ in range(q):
            l = rng.randrange(n); r = rng.randrange(l + 1, n + 1)
            queries.append(f"{l} {r}"); expected.append(str(min(a[l:r])))
        yield f"{n} {q}\n" + " ".join(map(str, a)) + "\n" + "\n".join(queries) + "\n", "\n".join(expected) + "\n"
    elif kind == "window":
        for n, k in [(1, 1), (10, 1), (31, 31), (2000, 87)]:
            a = [rng.randrange(30) for _ in range(n)]
            yield f"{n} {k}\n" + " ".join(map(str, a)) + "\n", " ".join(str(min(a[i:i+k])) for i in range(n-k+1)) + "\n"
    elif kind == "histogram":
        for n in [1, 2, 10, 31, 64]:
            a = [rng.randrange(20) for _ in range(n)]
            answer = 0
            for l in range(n):
                for r in range(l + 1, n + 1):
                    answer = max(answer, min(a[l:r]) * (r-l))
            yield f"{n}\n" + " ".join(map(str, a)) + "\n", f"{answer}\n"
    else:
        queries, expected, queue = ["2 17"], ["17"], []
        for _ in range(1499):
            kind = rng.randrange(3)
            if kind == 0 or (kind == 1 and not queue):
                a, b = rng.randrange(1, MOD), rng.randrange(MOD)
                queue.append((a, b)); queries.append(f"0 {a} {b}")
            elif kind == 1:
                queue.pop(0); queries.append("1")
            else:
                x = rng.randrange(MOD); queries.append(f"2 {x}")
                for a, b in queue: x = (a*x+b) % MOD
                expected.append(str(x))
        yield f"{len(queries)}\n" + "\n".join(queries) + "\n", "\n".join(expected) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=os.environ.get("CXX", "g++"))
    parser.add_argument("--sanitize", action="store_true")
    args = parser.parse_args()
    compiler = shlex.split(args.compiler)
    flags = ["-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-Werror", "-I", str(ROOT)]
    runtime = ["-O2", "-g"] if not args.sanitize else ["-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-sanitize-recover=all", "-fno-pie", "-no-pie"]
    headers = sorted(str(p.relative_to(ROOT)) for p in (ROOT / "static_query").glob("*.hpp"))
    for header in headers:
        for extra in [[], ["-DNDEBUG"]]:
            run(compiler + flags + extra + ["-fsyntax-only", "-x", "c++", "-"], input=f'#include "{header}"\n#include "{header}"\nint main() {{}}\n')
    print(f"standalone headers (debug/release): {len(headers)} passed", flush=True)
    with tempfile.TemporaryDirectory(prefix="cp-static-query-") as directory:
        temp = Path(directory)

        def compile_source(source, suffix="", extra=()):
            binary = temp / (source.stem + suffix)
            run(compiler + flags + list(extra) + runtime + [str(source), "-o", str(binary)])
            return binary

        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        (temp / "first.cpp").write_text(includes + 'int first() { return cp::MonotonicQueue<int>().size(); }\n')
        (temp / "second.cpp").write_text(includes + 'int first(); int main() { return first(); }\n')
        run(compiler + flags + runtime + [str(temp / "first.cpp"), str(temp / "second.cpp"), "-o", str(temp / "multi")])
        run([str(temp / "multi")])
        print("reverse includes / multiple translation units: passed", flush=True)
        examples = [("sparse_and_nearest", "1 2\n-1 -1 -1 2 \n1 2 -1 -1 \n"), ("window_minimum", "2 2 1 \n1 1 4 \n2 2 1\n"), ("swag", "37\n19\n7\n5\n")]
        for name, expected in examples:
            binary = compile_source(ROOT / f"examples/static_query/{name}.cpp")
            assert run([str(binary)]) == expected, name
        cases = [("static_rmq", "rmq"), ("sliding_window_minimum_aoj", "window"), ("monotonic_queue_aoj", "window"), ("largest_rectangle_aoj", "histogram"), ("queue_operate_all_composite", "swag")]
        for name, kind in cases:
            binary = compile_source(ROOT / f"verify/static_query/{name}.test.cpp")
            for sample, expected in protocols(kind):
                assert run([str(binary)], input=sample) == expected, name
        print("examples: 3 passed; judge wrappers: 5 passed (independent Python oracles)", flush=True)
        for suffix, extra in [("_debug", []), ("_release", ["-DNDEBUG"])]:
            binary = compile_source(ROOT / "verify/static_query/unit_test.cpp", suffix, extra)
            print(suffix[1:] + ": " + run([str(binary)]), end="", flush=True)


if __name__ == "__main__":
    main()
