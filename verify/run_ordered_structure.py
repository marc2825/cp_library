#!/usr/bin/env python3
"""3.5: standard associative operations, compression and Fenwick order statistics."""
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
    rng = random.Random(35001)
    queries, answers = [], []
    if kind == "predecessor":
        yield "4 4\n0000\n2 0\n3 0\n4 3\n1 2\n", "0\n-1\n-1\n"
        n, q = 67, 1200
        values = {i for i in range(n) if rng.randrange(4) == 0}
        initial = ''.join('1' if i in values else '0' for i in range(n))
        for _ in range(q):
            kind, x = rng.randrange(5), rng.randrange(n)
            queries.append(f"{kind} {x}")
            if kind == 0: values.add(x)
            elif kind == 1: values.discard(x)
            elif kind == 2: answers.append(str(int(x in values)))
            elif kind == 3: answers.append(str(min((v for v in values if v >= x), default=-1)))
            else: answers.append(str(max((v for v in values if v <= x), default=-1)))
        yield f"{n} {q}\n{initial}\n" + "\n".join(queries) + "\n", "\n".join(answers) + "\n"
    elif kind == "ordered":
        initial = [3, 17, 21, 35]
        values = set(initial)
        q = 1500
        for _ in range(q):
            type_ = rng.randrange(6)
            x = rng.randrange(1, len(values) + 4) if type_ == 2 else rng.randrange(50)
            queries.append(f"{type_} {x}")
            if type_ == 0: values.add(x)
            elif type_ == 1: values.discard(x)
            elif type_ == 2: answers.append(str(sorted(values)[x-1] if x <= len(values) else -1))
            elif type_ == 3: answers.append(str(sum(v <= x for v in values)))
            elif type_ == 4: answers.append(str(max((v for v in values if v <= x), default=-1)))
            else: answers.append(str(min((v for v in values if v >= x), default=-1)))
        yield f"{len(initial)} {q}\n" + " ".join(map(str, initial)) + "\n" + "\n".join(queries) + "\n", "\n".join(answers) + "\n"
    else:
        yield "5\n1 11\n1 29\n1 89\n2 2\n2 2\n", "29\n89\n"
        values = set()
        q = 1200
        for _ in range(q):
            if not values or rng.randrange(2) == 0:
                x = rng.randrange(1, 200001)
                while x in values: x = rng.randrange(1, 200001)
                values.add(x); queries.append(f"1 {x}")
            else:
                k = rng.randrange(1, len(values) + 1)
                x = sorted(values)[k-1]; values.remove(x)
                queries.append(f"2 {k}"); answers.append(str(x))
        yield f"{q}\n" + "\n".join(queries) + "\n", "\n".join(answers) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=os.environ.get("CXX", "g++"))
    parser.add_argument("--sanitize", action="store_true")
    args = parser.parse_args()
    compiler = shlex.split(args.compiler)
    flags = ["-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-Werror", "-I", str(ROOT)]
    runtime = ["-O2", "-g"] if not args.sanitize else ["-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-sanitize-recover=all", "-fno-pie", "-no-pie"]
    headers = sorted(str(p.relative_to(ROOT)) for p in (ROOT / "ordered_structure").glob("*.hpp"))
    for header in headers:
        for extra in [[], ["-DNDEBUG"]]:
            run(compiler + flags + extra + ["-fsyntax-only", "-x", "c++", "-"], input=f'#include "{header}"\n#include "{header}"\nint main() {{}}\n')
    print(f"standalone headers (debug/release): {len(headers)} passed", flush=True)
    with tempfile.TemporaryDirectory(prefix="cp-ordered-structure-") as directory:
        temp = Path(directory)

        def compile_source(source, suffix="", extra=()):
            binary = temp / (source.stem + suffix)
            run(compiler + flags + list(extra) + runtime + [str(source), "-o", str(binary)])
            return binary

        includes = ''.join(f'#include "{h}"\n' for h in reversed(headers))
        (temp / "first.cpp").write_text(includes + 'int first() { return cp::CoordinateCompression<int>().size(); }\n')
        (temp / "second.cpp").write_text(includes + 'int first(); int main() { return first(); }\n')
        run(compiler + flags + runtime + [str(temp / "first.cpp"), str(temp / "second.cpp"), "-o", str(temp / "multi")])
        run([str(temp / "multi")])
        print("reverse includes / multiple translation units: passed", flush=True)
        examples = [("standard_containers", "1 4\n1 1\n3 first\n7\n"), ("compression", "2 0 2 1 \n1 2\n0 20\n"), ("order_statistics", "20 1\n3 2\n-5 20\n0 0\n3 100\n")]
        for name, expected in examples:
            binary = compile_source(ROOT / f"examples/ordered_structure/{name}.cpp")
            assert run([str(binary)]) == expected, name
        for name, kind in [("predecessor_std", "predecessor"), ("predecessor_compressed", "predecessor"), ("ordered_set", "ordered"), ("kth_extract_arc033", "extract")]:
            binary = compile_source(ROOT / f"verify/ordered_structure/{name}.test.cpp")
            for sample, expected in protocols(kind):
                assert run([str(binary)], input=sample) == expected, name
        print("examples: 3 passed; judge wrappers: 4 passed (independent Python oracles)", flush=True)
        for suffix, extra in [("_debug", []), ("_release", ["-DNDEBUG"])]:
            binary = compile_source(ROOT / "verify/ordered_structure/unit_test.cpp", suffix, extra)
            print(suffix[1:] + ": " + run([str(binary)]), end="", flush=True)


if __name__ == "__main__":
    main()
