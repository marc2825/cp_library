#!/usr/bin/env python3
"""3.4: standalone headers, naive oracles, FIFO/deque order and judge protocols."""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import random
import re
import shlex
import subprocess
import tempfile
from urllib.parse import unquote

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
    elif kind == "deque":
        for seed in range(8):
            rng = random.Random(340101 + seed)
            items, queries, expected = deque(), ["4 17"], ["17"]
            for _ in range(2000):
                t = rng.randrange(5)
                if not items and t in [2, 3]:
                    t -= 2
                if t < 2:
                    a = rng.choice([1, MOD - 1, rng.randrange(1, MOD)])
                    b = rng.choice([0, MOD - 1, rng.randrange(MOD)])
                    (items.appendleft if t == 0 else items.append)((a, b))
                    queries.append(f"{t} {a} {b}")
                elif t == 2:
                    items.popleft(); queries.append("2")
                elif t == 3:
                    items.pop(); queries.append("3")
                else:
                    x = rng.randrange(MOD); queries.append(f"4 {x}")
                    for a, b in items: x = (a * x + b) % MOD
                    expected.append(str(x))
            yield f"{len(queries)}\n" + "\n".join(queries) + "\n", "\n".join(expected) + "\n"
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


def check_deque_docs():
    paths = ["README.md", "docs/README.md", "docs/static_query/README.md", "docs/static_query/swag.md",
             "docs/static_query/swag_deque.md", "docs/static_query/references.md", "visualizations/README.md"]
    count = 0
    for name in paths:
        path = ROOT / name
        source = re.sub(r"```.*?```", "", path.read_text(), flags=re.S)
        source = re.sub(r"`[^`]*`", "", source)
        for target in re.findall(r"\]\(([^)]+)\)", source):
            if "://" in target or target.startswith(("#", "mailto:")): continue
            assert (path.parent / unquote(target.split("#", 1)[0])).exists(), (name, target)
            count += 1
    print(f"local document links: {count} passed", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=os.environ.get("CXX", "g++"))
    parser.add_argument("--sanitize", action="store_true")
    parser.add_argument("--links-only", action="store_true")
    args = parser.parse_args()
    if args.links_only:
        check_deque_docs()
        return
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
        includes += ''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in sorted(ROOT.glob('*/all.hpp')))
        (temp / "first.cpp").write_text(includes + 'int first() { return cp::MonotonicQueue<int>().size(); }\n')
        (temp / "second.cpp").write_text(includes + 'int first(); int main() { return first(); }\n')
        run(compiler + flags + runtime + [str(temp / "first.cpp"), str(temp / "second.cpp"), "-o", str(temp / "multi")])
        run([str(temp / "multi")])
        print("reverse includes / multiple translation units: passed", flush=True)
        examples = [("sparse_and_nearest", "1 2\n-1 -1 -1 2 \n1 2 -1 -1 \n"), ("window_minimum", "2 2 1 \n1 1 4 \n2 2 1\n"), ("swag", "37\n19\n7\n5\n"), ("swag_deque", "37\n11\n15\n5\n")]
        doc = (ROOT / "docs/static_query/swag_deque.md").read_text()
        assert "documentation_of: //static_query/swag_deque.hpp" in doc
        assert re.search(r"```cpp\n(.*?)```", doc, re.S).group(1) == (ROOT / "examples/static_query/swag_deque.cpp").read_text()
        for name, expected in examples:
            binary = compile_source(ROOT / f"examples/static_query/{name}.cpp")
            assert run([str(binary)]) == expected, name
        cases = [("static_rmq", "rmq"), ("sliding_window_minimum_aoj", "window"), ("monotonic_queue_aoj", "window"), ("largest_rectangle_aoj", "histogram"), ("queue_operate_all_composite", "swag"), ("deque_operate_all_composite", "deque")]
        for name, kind in cases:
            binary = compile_source(ROOT / f"verify/static_query/{name}.test.cpp")
            for sample, expected in protocols(kind):
                assert run([str(binary)], input=sample) == expected, name
        print("examples: 4 passed; judge wrappers: 6 passed (deque: 8 independent Python cases)", flush=True)
        bridge = compile_source(ROOT / "verify/static_query/swag_deque_visualization.cpp")
        observed = [json.loads(line) for line in run([str(bridge)]).splitlines()]
        expected = json.loads((ROOT / "verify/static_query/swag_deque_visualization_expected.json").read_text())["states"]
        assert observed == expected
        print("SWAG Deque: 13 C++/visualization states passed", flush=True)
        for suffix, extra in [("_debug", []), ("_release", ["-DNDEBUG"])]:
            for name in ["unit_test", "swag_deque_test"]:
                binary = compile_source(ROOT / f"verify/static_query/{name}.cpp", suffix, extra)
                print(suffix[1:] + ": " + run([str(binary)]), end="", flush=True)
    check_deque_docs()


if __name__ == "__main__":
    main()
