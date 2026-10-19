#!/usr/bin/env python3
"""V-08..V-10: provenance integrity, evidence semantics and standard debug mode."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.cp_tools.catalog import collect
from tools.cp_tools.provenance import inventory
from tools.cp_tools.repository import runners


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=os.environ.get("CXX", "g++"))
    parser.add_argument("--sanitize", action="store_true", help="Python tool tests; C++ sanitizer coverage is provided by the selected C++ runners")
    args = parser.parse_args()
    # The negative debug-mode test owns its macros and intentionally aborts.
    import shlex
    compiler = [a for a in shlex.split(args.compiler) if not a.startswith("-D_GLIBCXX_")]
    env = {**os.environ, "CP_TEST_CXX": shlex.join(compiler)}
    subprocess.run([sys.executable, "-m", "unittest", "verify.development_records.test_records", "-v"], cwd=ROOT, env=env, check=True)
    inventory(ROOT)
    data = collect(ROOT)
    assert len(data["features"]) == len({row["path"] for row in data["features"]})
    # All existing runners expose the same compiler and sanitizer options.
    for name, script in runners(ROOT).items():
        help_text = subprocess.run([sys.executable, str(ROOT / script), "--help"], cwd=ROOT, capture_output=True, text=True, check=True).stdout
        assert "--compiler" in help_text and "--sanitize" in help_text, name
    print(f"repository: {len(data['features'])} headers, {len(data['tests'])} C++ test sources, {len(data['runners'])} compatible runners passed", flush=True)


if __name__ == "__main__":
    main()
