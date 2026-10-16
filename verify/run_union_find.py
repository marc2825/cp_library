#!/usr/bin/env python3
"""3.2 Union-Find: 独立性、差分テスト、ジャッジ形式の入出力をローカルで検証。"""
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


def protocol_cases(kind):
    rng = random.Random(3205)
    n, q = 23, 700
    modulus = 998244353 if kind == "mod" else None
    edges = [[] for _ in range(n)]
    hidden = [rng.randrange(10000) for _ in range(n)]
    queries, answers = [], []

    def difference(start, target):
        values = {start: 0}
        todo = [start]
        for v in todo:
            for to, weight in edges[v]:
                if to in values:
                    continue
                values[to] = values[v] + weight
                if modulus:
                    values[to] %= modulus
                todo.append(to)
        return values.get(target)

    for _ in range(q):
        u, v = rng.randrange(n), rng.randrange(n)
        if kind == "aoj":
            while u == v:
                v = rng.randrange(n)
            if hidden[u] > hidden[v]:
                u, v = v, u
        if rng.randrange(2) == 0:
            if kind == "plain":
                queries.append(f"0 {u} {v}")
                if difference(u, v) is None:
                    edges[u].append((v, 0)); edges[v].append((u, 0))
            else:
                weight = rng.randrange(modulus) if modulus else hidden[v] - hidden[u]
                queries.append(f"0 {u} {v} {weight}")
                # Library Checker は a[u]-a[v]、AOJ は a[v]-a[u]。
                forward = (-weight) % modulus if modulus else weight
                previous = difference(u, v)
                valid = previous is None or previous == forward
                if modulus:
                    answers.append(str(int(valid)))
                if valid and previous is None:
                    edges[u].append((v, forward))
                    edges[v].append((u, (-forward) % modulus if modulus else -forward))
        else:
            queries.append(f"1 {u} {v}")
            value = difference(u, v)
            if kind == "plain":
                answers.append(str(int(value is not None)))
            elif modulus:
                answers.append(str((-value) % modulus if value is not None else -1))
            else:
                answers.append(str(value) if value is not None else "?")
    return f"{n} {q}\n" + "\n".join(queries) + "\n", "\n".join(answers) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=os.environ.get("CXX", "g++"))
    parser.add_argument("--sanitize", action="store_true")
    args = parser.parse_args()
    compiler = shlex.split(args.compiler)
    flags = ["-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-Werror", "-I", str(ROOT)]
    runtime = ["-O2", "-g"]
    if args.sanitize:
        runtime = ["-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                   "-fno-sanitize-recover=all", "-fno-pie", "-no-pie"]
    headers = ["union_find/disjoint_set_union.hpp", "union_find/potentialized_disjoint_set_union.hpp",
               "union_find/all.hpp", "data_structure/disjoint_set_union.hpp", "data_structure/UnionFind.hpp"]
    for header in headers:
        source = f'#include "{header}"\n#include "{header}"\nint main() {{}}\n'
        for extra in [[], ["-DNDEBUG"]]:
            run(compiler + flags + extra + ["-fsyntax-only", "-x", "c++", "-"], input=source)
    print("standalone headers (debug/release): 5 passed", flush=True)
    with tempfile.TemporaryDirectory(prefix="cp-union-find-") as directory:
        temp = Path(directory)

        def compile_and_check(source, sample, expected):
            binary = temp / source.stem
            run(compiler + flags + runtime + [str(source), "-o", str(binary)])
            actual = run([str(binary)], input=sample)
            if actual != expected:
                raise AssertionError(f"{source.name}: {actual!r} != {expected!r}")
            return binary

        includes = ''.join(f'#include "{header}"\n' for header in headers)
        (temp / "first.cpp").write_text(includes + "int first() { return cp::DisjointSetUnion(3).component_count(); }\n")
        (temp / "second.cpp").write_text(includes + "int first(); int main() { return first() != 3; }\n")
        binary = temp / "multi_tu"
        run(compiler + flags + runtime + [str(temp / "first.cpp"), str(temp / "second.cpp"), "-o", str(binary)])
        run([str(binary)])
        print("legacy includes and multiple translation units: passed", flush=True)
        examples = [("connectivity", "1 3 3\n0 1 2\n3\n4\n"),
                    ("potential_constraints", "3 3 2\n1 1\n?\n")]
        for name, expected in examples:
            compile_and_check(ROOT / f"examples/union_find/{name}.cpp", "", expected)
        cases = [
            ("unionfind.test.cpp", "plain", "5 8\n1 0 1\n0 0 1\n1 0 1\n0 2 3\n1 0 3\n0 1 3\n1 0 2\n1 4 4\n", "0\n1\n0\n1\n1\n"),
            ("weighted_unionfind_aoj.test.cpp", "aoj", "5 6\n0 0 2 5\n0 1 2 3\n1 0 1\n1 1 3\n0 1 4 8\n1 0 4\n", "2\n?\n10\n"),
            ("unionfind_with_potential.test.cpp", "mod", "4 12\n1 0 3\n0 0 1 5\n1 0 1\n1 1 0\n0 1 2 7\n1 0 2\n0 0 2 12\n0 0 2 13\n1 0 2\n0 3 3 1\n1 3 3\n0 2 0 998244341\n", "-1\n1\n5\n998244348\n1\n12\n1\n0\n12\n0\n0\n1\n"),
        ]
        for name, kind, sample, expected in cases:
            binary = compile_and_check(ROOT / f"verify/union_find/{name}", sample, expected)
            sample, expected = protocol_cases(kind)
            assert run([str(binary)], input=sample) == expected
        print("examples: 2 passed; judge protocols: 3 passed (700 random queries each)", flush=True)
        binary = temp / "unit_test"
        run(compiler + flags + runtime + [str(ROOT / "verify/union_find/unit_test.cpp"), "-o", str(binary)])
        print(run([str(binary)]), end="", flush=True)


if __name__ == "__main__":
    main()
