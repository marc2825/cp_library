"""Run existing verification runners with shared libstdc++ debug/sanitizer modes."""
import argparse
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import shlex
import sys
import tempfile
import time

from .catalog import snapshot_id
from .process import execute
from .repository import ROOT, runners, write_json

DEBUG_FLAGS = ["-D_GLIBCXX_DEBUG", "-D_GLIBCXX_DEBUG_PEDANTIC", "-D_GLIBCXX_ASSERTIONS"]


def compiler_command(compiler, stdlib_debug):
    command = shlex.split(compiler)
    if not command:
        raise ValueError("empty compiler command")
    # User flags may select a compiler or target; the profile owns these macros.
    if any("_GLIBCXX_DEBUG" in flag or "_GLIBCXX_ASSERTIONS" in flag for flag in command[1:]):
        raise ValueError("select libstdc++ macros with --stdlib-debug, not --compiler")
    return command + (DEBUG_FLAGS if stdlib_debug else [])


def preflight(compiler, stdlib_debug, root=ROOT):
    if not stdlib_debug:
        return
    source = '''#include <vector>
#ifndef __GLIBCXX__
#error libstdc++ is required by the stdlib-debug profile
#endif
#if !defined(_GLIBCXX_DEBUG) || !defined(_GLIBCXX_DEBUG_PEDANTIC) || !defined(_GLIBCXX_ASSERTIONS)
#error missing debug mode macros
#endif
int main(){std::vector<int> a{1,2,3};return a.at(1)!=2;}
'''
    with tempfile.TemporaryDirectory(prefix="cp-stdlib-preflight-") as folder:
        path = Path(folder)
        (path / "check.cpp").write_text(source)
        compiled = execute([*compiler, "-std=c++17", str(path / "check.cpp"), "-o", str(path / "check")], b"", root, 120)
        if not compiled.ok:
            raise ValueError("stdlib-debug preflight failed: " + compiled.stderr.decode(errors="replace"))
        checked = execute([str(path / "check")], b"", root, 10)
        if not checked.ok:
            raise ValueError("stdlib-debug preflight executable failed")


def run_one(name, script, compiler, *, stdlib_debug=False, sanitize=False,
            output=Path("build/suites"), evidence=None, timeout=1800, root=ROOT):
    profile = ("stdlib-debug" if stdlib_debug else "standard")
    if sanitize:
        profile = "stdlib-debug-sanitize" if stdlib_debug else "sanitize"
    path = output / name / profile
    path.mkdir(parents=True, exist_ok=True)
    before = snapshot_id(root)
    command = [sys.executable, str(root / script), "--compiler", shlex.join(compiler)]
    if sanitize:
        command.append("--sanitize")
    started = datetime.now(timezone.utc).isoformat()
    record = {"schema_version": 1, "runner": name, "profile": profile, "command": command,
              "started_at": started, "snapshot": before, "status": "interrupted"}
    evidence = evidence if evidence is not None else root / "verification/evidence"
    receipt_path = evidence / name / (profile + ".json")
    # Replace a previous pass before starting, including when interrupted later.
    write_json(receipt_path, record)
    then = time.monotonic()
    try:
        result = execute(command, b"", root, timeout)
        (path / "stdout.txt").write_bytes(result.stdout)
        (path / "stderr.txt").write_bytes(result.stderr)
        record.update(returncode=result.returncode, timeout=result.timeout, elapsed=result.elapsed)
        stable = before == snapshot_id(root)
        record["sources_unchanged"] = stable
        record["status"] = "passed" if result.ok and stable else "failed"
    finally:
        record["elapsed"] = time.monotonic() - then
        write_json(receipt_path, record)
    print(f"{name} [{profile}]: {record['status']} ({record['elapsed']:.1f}s), logs: {path}", flush=True)
    return record["status"] == "passed"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runners", nargs="+", default=["union_find", "range_query"], help="runner names or all")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--compiler", default=os.environ.get("CXX", "g++"))
    parser.add_argument("--stdlib-debug", action="store_true")
    parser.add_argument("--sanitize", action="store_true")
    parser.add_argument("--output", type=Path, default=Path("build/suites"))
    parser.add_argument("--evidence", type=Path)
    parser.add_argument("--timeout", type=float, default=1800)
    args = parser.parse_args(argv)
    available = runners(ROOT)
    if args.list:
        print('\n'.join(available))
        return 0
    names = sorted(available) if args.runners == ["all"] else list(dict.fromkeys(args.runners))
    if not set(names) <= available.keys():
        parser.error("unknown runners: " + ', '.join(sorted(set(names) - available.keys())))
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be positive and finite")
    compiler = compiler_command(args.compiler, args.stdlib_debug)
    preflight(compiler, args.stdlib_debug)
    success = True
    for name in names:
        if not run_one(name, available[name], compiler, stdlib_debug=args.stdlib_debug,
                       sanitize=args.sanitize, output=args.output, evidence=args.evidence, timeout=args.timeout):
            success = False
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
