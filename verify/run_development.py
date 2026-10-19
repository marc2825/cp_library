#!/usr/bin/env python3
"""Offline tool tests and C++ integration; --with-verifier checks the pinned CLI."""
import argparse
import json
import os
import shlex
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.cp_tools.bundle import Bundler, bundle


def run(command, **kwargs):
    result = subprocess.run(list(map(str, command)), cwd=kwargs.pop("cwd", ROOT), text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, **kwargs)
    if result.returncode:
        raise RuntimeError(f"command failed: {command}\n{result.stdout}")
    return result.stdout


def links():
    paths = [ROOT / name for name in ["README.md", "HANDOFF.md", "docs/README.md", "docs/remaining_work.md",
        "docs/implementation_plan.md", "visualizations/README.md", "visualizations/development/README.md"]]
    paths += list((ROOT / "docs/development").glob("*.md"))
    count = 0
    for path in paths:
        for target in re.findall(r'\]\(([^\s)]+)\)', path.read_text()):
            if "://" in target or target.startswith("#"):
                continue
            target = target.split("#")[0]
            if target:
                assert (path.parent / target).exists(), (path, target)
                count += 1
    print(f"documentation: {count} local links passed", flush=True)


def verifier_integration(temporary):
    # The upstream resolver intentionally reads git-tracked files only. Use an
    # isolated snapshot to test uncommitted additions without touching the index.
    snapshot = temporary / "snapshot"
    snapshot.mkdir()
    paths = set()
    manifest = json.loads((ROOT / ".competitive-verifier/targets.json").read_text())
    for target in manifest["online"] + manifest["local"]:
        bundler = Bundler([ROOT])
        bundler.expand(ROOT / target)
        paths.update(bundler.dependencies)
    for directory in ["tools", ".competitive-verifier/docs", "docs/basic_utility", "docs/union_find",
                      "docs/range_query", "docs/string_algorithm"]:
        paths.update(p for p in (ROOT / directory).rglob("*") if p.is_file() and "__pycache__" not in p.parts)
    paths.update(ROOT / name for name in [".competitive-verifier/config.toml", ".competitive-verifier/targets.json"])
    for path in paths:
        dest = snapshot / path.relative_to(ROOT)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, dest)
    run(["git", "init", "-q"], cwd=snapshot)
    run(["git", "add", "."], cwd=snapshot)
    run(["git", "-c", "user.name=Local verifier test", "-c", "user.email=test@example.invalid",
         "-c", "commit.gpgsign=false", "commit", "-qm", "temporary verification snapshot"], cwd=snapshot)
    run([sys.executable, "tools/verify_ci.py", "--offline"], cwd=snapshot)
    output = snapshot / "build/verifier"
    result = json.loads((output / "result.json").read_text())
    assert result["files"], result
    assert (output / "docs/_config.yml").is_file()
    assert list((output / "docs").rglob("verifier_smoke.cpp.md")), "missing generated smoke documentation"
    run([sys.executable, "tools/verify_ci.py", "--resolve-only"], cwd=snapshot)
    resolved = json.loads((output / "resolved.json").read_text())["files"]
    for target in manifest["online"]:
        assert any(v["type"] == "problem" for v in resolved[target]["verification"]), target
    print("competitive-verifier: standalone execution, docs generation, 3 online problem resolutions passed", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=os.environ.get("CXX", "g++"))
    parser.add_argument("--sanitize", action="store_true")
    parser.add_argument("--with-verifier", action="store_true")
    parser.add_argument("--links-only", action="store_true")
    args = parser.parse_args()
    if args.links_only:
        links()
        return
    fixtures = json.loads((ROOT / "verify/development/visualization_expected.json").read_text())
    reference = json.loads(run([sys.executable, "verify/development/visualization_reference.py"],
        input=json.dumps([case["input"] for case in fixtures])))
    assert reference == [case["result"] for case in fixtures], "stale visualization reference"
    print("visualization: 200 Python reference cases passed", flush=True)
    example = (ROOT / "examples/development/random_data.py").read_text()
    documented = (ROOT / "docs/development/random_data.md").read_text().split("```python\n", 1)[1].split("```", 1)[0]
    assert example.strip() == documented.strip(), "random data usage example differs from documentation"
    run([sys.executable, "-m", "examples.development.random_data"])
    print(run([sys.executable, "verify/development/test_tools.py", "-v"]), end="", flush=True)
    with tempfile.TemporaryDirectory(prefix="cp-development-") as td:
        temp = Path(td)
        flags = ["-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror"]
        if args.sanitize:
            flags += ["-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                      "-fno-sanitize-recover=all", "-fno-pie", "-no-pie"]
        cases = [
            ("verify/union_find/unionfind.test.cpp", "4 5\n1 0 1\n0 0 1\n1 0 1\n0 2 3\n1 1 3\n", "0\n1\n0\n"),
            ("verify/range_query/point_add_range_sum.test.cpp", "3 3\n2 4 8\n1 0 3\n0 1 3\n1 1 3\n", "14\n15\n"),
            ("verify/string_algorithm/suffix_array.test.cpp", "banana\n", "5 3 1 0 4 2\n"),
            ("examples/basic_utility/big_rational.cpp", "", None),
            ("examples/development/fast.cpp", "4 2\n0 1\n2 3\n", "2\n"),
        ]
        for i, (source, data, expected) in enumerate(cases):
            flat = temp / f"flat{i}.cpp"
            flat.write_text(bundle(ROOT / source, [ROOT]))
            for release in (False, True):
                extra = ["-DNDEBUG"] if release else []
                original, flattened = temp / "original", temp / "flattened"
                run([*shlex.split(args.compiler), *flags, *extra, "-I", ROOT, ROOT / source, "-o", original])
                run([*shlex.split(args.compiler), *flags, *extra, flat, "-o", flattened])
                before, after = run([original], input=data), run([flattened], input=data)
                assert before == after and (expected is None or after == expected), (source, before, after)
        print("C++: 5 original/flattened pairs in Debug/Release passed (no local -I for flattened files)", flush=True)
        fast = temp / "fast"
        run([*shlex.split(args.compiler), *flags, "-I", ROOT, ROOT / "examples/development/fast.cpp", "-o", fast])
        config = json.loads((ROOT / "examples/development/stress.json").read_text())
        config.update(cwd=str(ROOT), fast=[str(fast)])
        config_path = temp / "stress.json"
        config_path.write_text(json.dumps(config))
        print(run([sys.executable, "-m", "tools.cp_tools.stress", "--config", config_path, "--cases", "80",
                   "--seed", "100", "--failures", temp / "failures"]), end="", flush=True)
        if args.with_verifier:
            verifier_integration(temp)
    links()


if __name__ == "__main__":
    main()
