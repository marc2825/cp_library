#!/usr/bin/env python3
"""V-05..V-07: debug formatting, interactive protocol, parallel external scoring."""
import argparse
import json
import os
from pathlib import Path
import random
import shlex
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.cp_tools import benchmark, interactive
from tools.cp_tools.bundle import bundle


def run(command, **kwargs):
    result = subprocess.run(list(map(str, command)), cwd=ROOT, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, timeout=300, **kwargs)
    if result.returncode:
        raise RuntimeError(f"{command}\n{result.stdout}\n{result.stderr}")
    return result.stdout


def check_links():
    from verify.run_development import links
    links()
    import re
    paths = list((ROOT / "docs/development_runtime").glob("*.md")) + [ROOT / "docs/basic_utility/debug.md",
             ROOT / "visualizations/development_runtime/README.md"]
    count = 0
    for path in paths:
        text = re.sub(r'```.*?```|`[^`]*`', '', path.read_text(), flags=re.S)
        for target in re.findall(r'\]\(([^\s)]+)\)', text):
            if "://" not in target and not target.startswith("#"):
                assert (path.parent / target.split("#")[0]).exists(), (path, target)
                count += 1
    print(f"runtime documentation: {count} local links passed", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=os.environ.get("CXX", "g++"))
    parser.add_argument("--sanitize", action="store_true")
    parser.add_argument("--links-only", action="store_true")
    args = parser.parse_args()
    if args.links_only:
        check_links()
        return
    print(run([sys.executable, "verify/development_runtime/test_runtime.py", "-v"]), end="", flush=True)
    print(run([sys.executable, "-m", "unittest", "verify.development.test_tools.Stress", "-v"]), end="", flush=True)
    print("15 runtime cases and 5 existing stress tests passed", flush=True)
    with tempfile.TemporaryDirectory(prefix="cp-development-runtime-") as td:
        tmp = Path(td)
        flags = ["-std=c++17", "-O2", "-g", "-Wall", "-Wextra", "-Wpedantic", "-Werror", "-I", ROOT]
        if args.sanitize:
            flags += ["-O1", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-sanitize-recover=all", "-fno-pie", "-no-pie"]
        def compile(source, name, extra=()):
            executable = tmp / name
            run([*shlex.split(args.compiler), *flags, *extra, source, "-o", executable])
            return executable
        for release in ([], ["-DNDEBUG"]):
            for mode in ([], ["-DCP_ENABLE_DEBUG"], ["-DCP_ENABLE_DEBUG", "-DONLINE_JUDGE"], ["-DCP_ENABLE_DEBUG", "-DCP_DISABLE_DEBUG"]):
                run([compile(ROOT / "verify/development_runtime/debug_test.cpp", "debug", [*release, *mode])])
        all_headers = sorted(ROOT.glob("*/all.hpp"))
        includes = ''.join(f'#include "{p.relative_to(ROOT)}"\n' for p in all_headers)
        (tmp / "first.cpp").write_text(includes + '#include "basic_utility/debug.hpp"\nstd::string other();\nint main(){return other()!=cp::debug_string(42);}\n')
        (tmp / "second.cpp").write_text(includes + '#include "basic_utility/debug.hpp"\nstd::string other(){return cp::debug_string(42);}\n')
        mixed = compile(tmp / "first.cpp", "mixed", [tmp / "second.cpp"])
        run([mixed])
        # Isolated header check does not inherit any other library include.
        (tmp / "standalone.cpp").write_text('#include "basic_utility/debug.hpp"\n#include "basic_utility/debug.hpp"\nint main(){CP_DEBUG(42);return cp::debug_string(42)!="42";}\n')
        run([compile(tmp / "standalone.cpp", "standalone", ["-DCP_ENABLE_DEBUG"])])
        example = ROOT / "examples/basic_utility/debug.cpp"
        documented = (ROOT / "docs/basic_utility/debug.md").read_text().split("```cpp\n", 1)[1].split("```", 1)[0]
        assert example.read_text().strip() == documented.strip()
        assert run([compile(example, "example")]) == '("state", [3, 1, 4], true)\n[3, 1, ...]\n'
        rng = random.Random(5071026)
        matrices = [[[rng.randint(-(1 << 63), (1 << 63) - 1) for _ in range(rng.randrange(10))]
                     for _ in range(rng.randrange(8))] for _ in range(500)]
        inputs = [' '.join(map(str, [len(m), *(x for row in m for x in [len(row), *row])])) for m in matrices]
        for extra in ([], ["-DNDEBUG"]):
            driver = compile(ROOT / "verify/development_runtime/debug_driver.cpp", "driver", extra)
            actual = run([driver], input='\n'.join(inputs)).splitlines()
            assert actual == [str(m) for m in matrices]
        fixtures = json.loads((ROOT / "verify/development_runtime/visualization_expected.json").read_text())
        reference = compile(ROOT / "verify/development_runtime/visualization_reference.cpp", "visual-reference")
        assert run([reference]).splitlines() == [case["output"] for case in fixtures["debug"]]
        for case in fixtures["benchmark"]:
            data = case["input"]
            rows = [dict(name=c["name"], seed=s, status="ok" if score is not None else "failed", score=score)
                    for c in data["configurations"] for s, score in zip(data["seeds"], c["scores"])]
            assert benchmark.summarize(rows, [c["name"] for c in data["configurations"]], data["seeds"], data["goal"]) == case["result"]
        print("visualization references: 6 C++ outputs and 40 Python score summaries passed", flush=True)
        print("debug: 4 flag combinations x Debug/Release, standalone/repeated/mixed/two-TU, usage and 500 Python comparisons passed", flush=True)
        solver = compile(ROOT / "examples/development_runtime/guess_solver.cpp", "guess", ["-DCP_ENABLE_DEBUG"])
        config = interactive.load_config(ROOT / "examples/development_runtime/interactive.json")
        config["solver"] = [str(solver)]
        for seed in [0, 1, 41, 49, 50, 98, 99, -1]:
            result = interactive.run_session(config, tmp / f"guess-{seed}", seed)
            assert result["status"] == "ok", result
        config_path = tmp / "interactive.json"
        config_path.write_text(json.dumps(config))
        run([sys.executable, "-m", "tools.cp_tools.interactive", "--config", config_path, "--seed", "42", "--output", tmp / "guess-cli"])
        selected = compile(ROOT / "examples/development_runtime/select_solver.cpp", "select", ["-DCP_ENABLE_DEBUG"])
        config = benchmark.load_config(ROOT / "examples/development_runtime/benchmark.json")
        for c in config["configurations"]:
            c["solver"][0] = str(selected)
        summary = benchmark.run_batch(config, list(range(12)), 4, tmp / "batch")
        assert summary["best"] == "best" and all(r["success"] == 12 for r in summary["configurations"]), summary
        # Independent expected scores read the saved generator inputs.
        for row in summary["configurations"]:
            scores = []
            for seed in range(12):
                values = list(map(int, (tmp / f"batch/cases/{row['name']}/{seed}/input.txt").read_text().split()))[1:]
                scores.append(values[0] if row["name"] == "first" else max(values))
            assert row["mean"] == sum(scores) / 12
        run([sys.executable, "-m", "tools.cp_tools.benchmark", "--replay", tmp / "batch/cases/best/4", "--output", tmp / "replay"])
        flat = tmp / "flat.cpp"
        flat.write_text(bundle(example, [ROOT]))
        assert run([compile(flat, "flat")]) == run([tmp / "example"])
        print("integration: 9 C++ interactive sessions, 2 configurations x 12 seeds / 4 workers, scorer and replay, debug header bundle passed", flush=True)
    check_links()


if __name__ == "__main__":
    main()
