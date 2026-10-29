"""Exercise the public contest CLI in a separate, space-containing workspace."""
import argparse
import json
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.cp_tools.contest import ALIASES, header_catalog, public_headers, resolve_header


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default="g++")
    parser.add_argument("--sanitize", action="store_true")
    args = parser.parse_args()
    compiler = shlex.split(args.compiler)
    flags = ["-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror"]
    if args.sanitize:
        flags += ["-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                  "-fno-sanitize-recover=all", "-fno-pie", "-no-pie"]
    checks = 0
    with tempfile.TemporaryDirectory(prefix="cp contest workspace ") as directory:
        work = Path(directory)
        def run(command, *, expected=0, data=None):
            nonlocal checks
            result = subprocess.run(list(map(str, command)), cwd=work, input=data, capture_output=True,
                                    text=True, timeout=90)
            assert result.returncode == expected, (command, result.stdout, result.stderr)
            checks += 1
            return result
        def cli(*arguments, expected=0):
            return run([sys.executable, ROOT / "tools/cplib.py", *arguments], expected=expected)
        def compile(source, binary):
            run([*compiler, *flags, source, "-o", binary])
        assert cli("root").stdout.strip() == str(ROOT)
        assert "bundle" in cli("--help").stdout
        assert "dependencies" in cli("bundle", "--help").stdout
        assert "disjoint_set_union.hpp" in cli("find", "dsu").stdout
        assert "dynamic_modint.hpp" in cli("find", "modint").stdout
        assert "suffix_array.hpp" in cli("find", "suffix", "array").stdout
        assert "undo_manager.hpp" in cli("find", "Undo").stdout
        cli("find", "no-such-algorithm-937", expected=1)
        cli("snippet", "all", expected=2)
        cli("snippet", "no-such-header", expected=2)
        cli("snippet", "../outside.hpp", expected=2)
        cli("snippet", "tools/cplib.py", expected=2)
        for alias, header in ALIASES.items():
            assert resolve_header(alias) == ROOT / header
        assert resolve_header("range_query/fenwick_tree") == ROOT / "range_query/fenwick_tree.hpp"
        records = header_catalog()
        assert set(e["header"] for e in records) == set(public_headers())
        assert all(not e["doc"] or (ROOT / e["doc"]).exists() for e in records)
        assert all("detail" not in Path(e["header"]).parts for e in records)
        cli("init", "solution file.cpp", "--use", "dsu", "--use", "modint", "--use", "dsu")
        solution = work / "solution file.cpp"
        template = solution.read_text()
        assert template.count('#include "union_find/disjoint_set_union.hpp"') == 1
        cli("init", solution, expected=2)
        assert solution.read_text() == template
        run([*compiler, *flags, "-I", ROOT, solution, "-o", work / "empty"])
        run([work / "empty"])
        cli("init", "unknown.cpp", "--use", "no-such-header", expected=2)
        assert not (work / "unknown.cpp").exists()
        # Generated includes can be used directly, then bundled without a local -I.
        solution.write_text(template.replace("    // Write your solution here. Library APIs use the cp:: namespace.",
            '    cp::DisjointSetUnion dsu(4);\n    dsu.merge(0, 1);\n'
            '    cp::StaticModInt<17> value = 20;\n'
            '    std::cout << dsu.same(0, 1) << " " << value.val() << "\\n";'))
        before = solution.read_text()
        flat = work / "submission file.cpp"
        cli("bundle", solution, "-o", flat, "--dependencies", "deps.json", "--check")
        dependencies = json.loads((work / "deps.json").read_text())
        assert str(solution) in dependencies
        assert str(ROOT / "basic_utility/static_modint.hpp") in dependencies
        assert solution.read_text() == before
        # The guide's complete example is executable, and its displayed source stays in sync.
        example = ROOT / "examples/development/contest_connectivity.cpp"
        guide = (ROOT / "docs/usage.md").read_text()
        assert re.search(r"```cpp\n(.*?)```", guide, re.S)[1] == example.read_text()
        example_flat = work / "guide.cpp"
        cli("bundle", example, "-o", example_flat, "--check")
        compile(example_flat, work / "guide")
        assert run([work / "guide"], data="4 5\n1 0 1\n0 0 1\n1 0 1\n0 2 3\n1 1 3\n").stdout == "0\n1\n0\n"
        # Compiler flags and user include directories survive the delegated CLI.
        (work / "extra headers").mkdir()
        (work / "extra headers/value.hpp").write_text("#pragma once\nconstexpr int value = 17;\n")
        configured = work / "configured.cpp"
        configured.write_text('#include "value.hpp"\n#ifndef CONTEST_FLAG\n#error flag missing\n#endif\n'
                              'int main(){return value != 17;}\n')
        cli("bundle", configured, "-I", "extra headers", "-o", "configured_flat.cpp", "--check",
            "--flag=-DCONTEST_FLAG")
        for release in [False, True]:
            extra = ["-DNDEBUG"] if release else []
            run([*compiler, *flags, *extra, "-I", ROOT, solution, "-o", work / "original"])
            run([*compiler, *flags, *extra, flat, "-o", work / "flat"])
            assert run([work / "original"]).stdout == run([work / "flat"]).stdout == "1 3\n"
        cli("bundle", solution, "-o", solution, expected=2)
        assert solution.read_text() == before
        bad = work / "bad.cpp"
        bad.write_text("this is not valid C++;\n")
        old_output = flat.read_text()
        cli("bundle", bad, "-o", flat, "--check", expected=2)
        assert flat.read_text() == old_output
        # Extraction shares the bundler across multiple roots: one definition per dependency.
        snippet = cli("snippet", "fenwick", "range_query/range_add_range_sum_fenwick_tree.hpp",
                      "fenwick", "--check").stdout
        assert snippet.count("class FenwickTree") == 1
        (work / "snippet.hpp").write_text(snippet)
        cli("snippet", "fenwick", "-o", "snippet.hpp", expected=2)
        cli("snippet", "fenwick", "-o", "snippet.hpp", "--force", "--check")
        (work / "using_snippet.cpp").write_text('#include "snippet.hpp"\n#include <iostream>\n'
            'int main(){cp::FenwickTree<long long> f(4);f.add(0,3);f.add(2,7);'
            'std::cout<<f.sum(0,4)<<"\\n";}\n')
        compile(work / "using_snippet.cpp", work / "using_snippet")
        assert run([work / "using_snippet"]).stdout == "10\n"
        header = ROOT / "union_find/disjoint_set_union.hpp"
        original_header = header.read_bytes()
        cli("snippet", "dsu", "-o", header, "--force", expected=2)
        assert header.read_bytes() == original_header
        before = (work / "snippet.hpp").read_bytes()
        cli("snippet", "dsu", "-o", "snippet.hpp", "--force", "--check",
            "--compiler", "missing-compiler-937", expected=2)
        assert (work / "snippet.hpp").read_bytes() == before
        # Existing user snippets and settings must survive generation.
        (work / ".vscode").mkdir()
        user = work / ".vscode/user.code-snippets"
        user.write_text('{"my snippet": {"prefix":"mine","body":["mine"]}}')
        cli("snippets")
        generated = work / ".vscode/cp-library.code-snippets"
        data = json.loads(generated.read_text())
        assert set(data) == set(public_headers())
        prefixes = [p for entry in data.values() for p in entry["prefix"]]
        assert len(prefixes) == len(set(prefixes)), "ambiguous completion prefix"
        assert "cp/dsu" in data["union_find/disjoint_set_union.hpp"]["prefix"]
        assert all(e["scope"] == "cpp" for e in data.values())
        cli("snippets", expected=2)
        cli("snippets", "--force")
        assert json.loads(generated.read_text()) == data
        assert json.loads(user.read_text())["my snippet"]["prefix"] == "mine"
        assert not (work / ".vscode/settings.json").exists()
        assert not (work / ".vscode/tasks.json").exists()
    print(f"contest CLI: {checks} commands, {len(records)} discoverable headers, external workspace, "
          "Debug/Release include/bundle equivalence, snippet dependency sharing, failure preservation "
          "and unique VS Code completions passed", flush=True)


if __name__ == "__main__":
    main()
