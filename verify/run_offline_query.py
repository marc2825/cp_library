#!/usr/bin/env python3
"""3.6: Mo, parallel binary search, sweep line and CDQ, with independent oracles."""
import argparse
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def run(command, **kwargs):
    result = subprocess.run(command, text=True, capture_output=True, **kwargs)
    if result.returncode:
        raise RuntimeError(f"command failed: {shlex.join(map(str, command))}\n{result.stdout}{result.stderr}")
    return result.stdout


def protocols(kind):
    rng = random.Random(36002)
    n, q = 55, 600
    lines, expected = [], []
    if kind == "mo":
        a = [rng.randrange(30) for _ in range(n)]
        for _ in range(q):
            l = rng.randrange(n); r = rng.randrange(l+1, n+1)
            lines.append(f"{l} {r}")
            expected.append(sum(a[i] > a[j] for i in range(l, r) for j in range(i+1, r)))
        data = f"{n} {q}\n" + " ".join(map(str, a)) + "\n" + "\n".join(lines) + "\n"
    elif kind == "pbs":
        edges = [(i-1, i) for i in range(1, n)] + [tuple(sorted(rng.sample(range(n), 2))) for _ in range(n)]
        rng.shuffle(edges)
        queries = [(*sorted(rng.sample(range(n), 2)), rng.randrange(3, n+1)) for _ in range(q)]
        expected = [None] * q
        adjacency = [[] for _ in range(n)]
        for t, (u, v) in enumerate(edges, 1):
            adjacency[u].append(v); adjacency[v].append(u)
            for i, (x, y, z) in enumerate(queries):
                if expected[i] is not None: continue
                seen, queue = {x, y}, list({x, y})
                for node in queue:
                    for to in adjacency[node]:
                        if to not in seen: seen.add(to); queue.append(to)
                if len(seen) >= z: expected[i] = t
        assert None not in expected
        data = f"{n} {len(edges)}\n" + "\n".join(f"{u+1} {v+1}" for u, v in edges) + f"\n{q}\n"
        data += "\n".join(f"{x+1} {y+1} {z}" for x, y, z in queries) + "\n"
    else:
        points = [(rng.randrange(20), rng.randrange(20), rng.randrange(100)) for _ in range(n)]
        data = f"{n} {q}\n" + "\n".join(" ".join(map(str, p)) for p in points) + "\n"
        for _ in range(q):
            if kind == "cdq" and rng.randrange(2):
                p = (rng.randrange(20), rng.randrange(20), rng.randrange(100))
                points.append(p); lines.append("0 " + " ".join(map(str, p)))
            else:
                l, r = sorted(rng.sample(range(22), 2)); d, u = sorted(rng.sample(range(22), 2))
                lines.append(("1 " if kind == "cdq" else "") + f"{l} {d} {r} {u}")
                expected.append(sum(w for x, y, w in points if l <= x < r and d <= y < u))
        data += "\n".join(lines) + "\n"
    return data, "".join(f"{x}\n" for x in expected)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=os.environ.get("CXX", "g++"))
    parser.add_argument("--sanitize", action="store_true")
    args = parser.parse_args()
    compiler = shlex.split(args.compiler)
    flags = ["-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-Werror", "-I", str(ROOT)]
    runtime = ["-O2", "-g"] if not args.sanitize else ["-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-sanitize-recover=all", "-fno-pie", "-no-pie"]
    headers = sorted(str(p.relative_to(ROOT)) for p in (ROOT / "offline_query").glob("*.hpp"))
    for header in headers:
        for extra in [[], ["-DNDEBUG"]]:
            run(compiler + flags + extra + ["-fsyntax-only", "-x", "c++", "-"], input=f'#include "{header}"\n#include "{header}"\nint main() {{}}\n')
    print(f"standalone headers (debug/release): {len(headers)} passed", flush=True)
    with tempfile.TemporaryDirectory(prefix="cp-offline-query-") as directory:
        temp = Path(directory)

        def compile_source(source, suffix="", extra=()):
            binary = temp / (source.stem + suffix)
            run(compiler + flags + list(extra) + runtime + [str(source), "-o", str(binary)])
            return binary

        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        (temp / "first.cpp").write_text(includes + 'int first() { return cp::Mo(0).query_count(); }\n')
        (temp / "second.cpp").write_text(includes + 'int first(); int main() { return first(); }\n')
        run(compiler + flags + runtime + [str(temp / "first.cpp"), str(temp / "second.cpp"), "-o", str(temp / "multi")])
        run([str(temp / "multi")])
        print("reverse includes / multiple translation units: passed", flush=True)
        for name, expected in [("mo_distinct", "2\n3\n0\n"), ("first_connected", "0\n3\n4\n"), ("static_rectangle", "3\n10\n0\n"), ("dynamic_rectangle", "0\n4\n3\n")]:
            assert run([str(compile_source(ROOT / f"examples/offline_query/{name}.cpp"))]) == expected, name
        for name, kind in [("range_inversions", "mo"), ("stamp_rally", "pbs"), ("rectangle_sum", "sweep"), ("point_add_rectangle_sum", "cdq")]:
            sample, expected = protocols(kind)
            assert run([str(compile_source(ROOT / f"verify/offline_query/{name}.test.cpp"))], input=sample) == expected, name
        print("examples: 4 passed; judge wrappers: 4 passed (600 queries each, Python oracles)", flush=True)
        for suffix, extra in [("_debug", []), ("_release", ["-DNDEBUG"])]:
            print(suffix[1:] + ":\n" + run([str(compile_source(ROOT / "verify/offline_query/unit_test.cpp", suffix, extra))]), end="", flush=True)


if __name__ == "__main__":
    main()
