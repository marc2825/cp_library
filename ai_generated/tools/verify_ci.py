#!/usr/bin/env python3
"""Run the checked-in competitive-verifier scope and generate Jekyll documents."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from cp_tools.bundle import Bundler

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--offline", action="store_true", help="only the standalone smoke test; no downloaded cases")
    parser.add_argument("--resolve-only", action="store_true", help="validate dependency/config resolution without judging")
    parser.add_argument("--output", type=Path, default=Path("build/verifier"))
    args = parser.parse_args()
    output = (ROOT / args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((ROOT / ".competitive-verifier/targets.json").read_text())
    targets = manifest["local"] + ([] if args.offline else manifest["online"])
    sources = set()
    for target in targets:
        bundler = Bundler([ROOT])
        bundler.expand(ROOT / target)
        sources.update(str(p.relative_to(ROOT)) for p in bundler.dependencies)
    command = [sys.executable, "-c", "from competitive_verifier.app import main; raise SystemExit(main())"]
    resolved = subprocess.run(command + ["oj-resolve", "--config", ".competitive-verifier/config.toml",
        "--no-bundle", "--include", *sorted(sources)], cwd=ROOT, text=True, stdout=subprocess.PIPE, check=True)
    files = json.loads(resolved.stdout)["files"]
    missing = sources - files.keys()
    if missing or any(not files[t].get("verification") for t in targets):
        raise RuntimeError(f"unresolved/untracked source or empty verification: {sorted(missing)}; commit files before running CI")
    resolved_path = output / "resolved.json"
    resolved_path.write_text(resolved.stdout)
    if args.resolve_only:
        return 0
    result = output / "result.json"
    # Do not consume a stale successful result if this run fails before writing it.
    result.unlink(missing_ok=True)
    verified = subprocess.run(command + ["verify", "--verify-json", str(resolved_path),
        "--check-error", "--timeout", "600", "--tle", "10", "--output", str(result)], cwd=ROOT)
    if result.exists():
        subprocess.run(command + ["docs", "--verify-json", str(resolved_path), "--check-error",
            "--destination", str(output / "docs"), str(result), "--include", *sorted(sources),
            "docs/basic_utility", "docs/union_find", "docs/range_query", "docs/string_algorithm"],
            cwd=ROOT, check=True)
    return verified.returncode


if __name__ == "__main__":
    raise SystemExit(main())
