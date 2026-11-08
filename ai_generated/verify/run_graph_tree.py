#!/usr/bin/env python3
"""C++17 でヘッダ独立性・ODR・使用例・ランダム差分テストを検証する。"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def run(command, **kwargs):
    result = subprocess.run(command, text=True, capture_output=True, **kwargs)
    if result.returncode:
        raise RuntimeError(f"command failed: {shlex.join(map(str, command))}\n"
                           f"{result.stdout}{result.stderr}")
    return result.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=os.environ.get("CXX", "g++"))
    parser.add_argument("--sanitize", action="store_true")
    parser.add_argument("--stress", action="store_true", help="20万頂点のパスで深さ依存の不具合を検証")
    args = parser.parse_args()
    compiler = shlex.split(args.compiler)
    flags = ["-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-Werror", "-I", str(ROOT)]
    runtime_flags = ["-O2", "-g"]
    if args.sanitize:
        runtime_flags = ["-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                         "-fno-sanitize-recover=all", "-fno-pie", "-no-pie"]
    headers = sorted({*ROOT.glob("graph/**/*.hpp"), *ROOT.glob("tree/**/*.hpp"),
                      *ROOT.glob("graph_theory/detail/*.hpp"),
                      *(ROOT / p for p in ["graph_theory/graph_template.hpp", "graph_theory/Tree/tree_template.hpp",
                                           "graph_theory/bfs.hpp", "graph_theory/dijkstra.hpp", "graph_theory/grid_bfs.hpp",
                                           "data_structure/UnionFind.hpp", "data_structure/disjoint_set_union.hpp"])})

    def check_header(path):
        header = path.relative_to(ROOT).as_posix()
        source = f'#include "{header}"\n#include "{header}"\nint main() {{}}\n'
        run(compiler + flags + ["-fsyntax-only", "-x", "c++", "-"], input=source)

    with ThreadPoolExecutor(max_workers=4) as pool:
        list(pool.map(check_header, headers))
    print(f"standalone headers: {len(headers)} passed", flush=True)

    includes = ''.join(f'#include "{p.relative_to(ROOT).as_posix()}"\n' for p in headers)
    reverse = ''.join(f'#include "{p.relative_to(ROOT).as_posix()}"\n' for p in reversed(headers))
    run(compiler + flags + ["-fsyntax-only", "-x", "c++", "-"], input=reverse + "int main() {}\n")
    with tempfile.TemporaryDirectory(prefix="cp-graph-tree-") as temp:
        temp = Path(temp)
        (temp / "first.cpp").write_text(includes + "int first() { return cp::grid_bfs({\".\"},0,0)[0][0]; }\n")
        (temp / "second.cpp").write_text(reverse + "int first(); int main() { return first(); }\n")
        binary = temp / "multi_tu"
        run(compiler + flags + runtime_flags + [str(temp / "first.cpp"), str(temp / "second.cpp"), "-o", str(binary)])
        run([str(binary)])
        print("include order and multiple translation units: passed", flush=True)
        examples = [
            ("graph_shortest_path", "4 4 0 3\n0 1 2\n1 3 4\n0 2 1\n2 3 2\n", "3\n0 2 3\n"),
            ("tree_queries", "5 2\n0 1 5\n1 2 0\n1 3 2\n3 4 3\n2 4\n0 4\n", "1 5\n0 10\n"),
            ("tree_hld", "", "CBDE\n"),
        ]
        for name, sample, expected in examples:
            binary = temp / name
            run(compiler + flags + runtime_flags + [str(ROOT / "examples" / (name + ".cpp")), "-o", str(binary)])
            actual = run([str(binary)], input=sample)
            if actual != expected:
                raise AssertionError(f"{name}: {actual!r} != {expected!r}")
        print("examples: 3 passed", flush=True)
        binary = temp / "graph_tree_test"
        run(compiler + flags + runtime_flags + [str(ROOT / "verify/graph_tree_test.cpp"), "-o", str(binary)])
        print(run([str(binary)] + (["--stress"] if args.stress else [])), end="", flush=True)


if __name__ == "__main__":
    main()
