#!/usr/bin/env python3
"""Generator invariants, JSON/CLI and feature inventory; no external packages."""
import argparse
import hashlib
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.cp_tools.feature_catalog import collect_features
from tools.cp_tools.repository import write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default="g++", help="accepted for shared suite compatibility; Python tests")
    parser.add_argument("--sanitize", action="store_true", help="Python invariants; no compiled code in this suite")
    parser.add_argument("--record-evidence", action="store_true", help="record hashes after all tests pass, then regenerate catalog")
    args = parser.parse_args()
    receipt = ROOT / "catalog/test_evidence.json"
    if args.record_evidence:
        receipt.unlink(missing_ok=True)
    paths = sorted([*ROOT.glob("tools/**/*.py"), *ROOT.glob("verify/generation/*.py"),
                    ROOT / "verify/run_generation.py", *ROOT.glob("examples/development/generation/*.py"),
                    *ROOT.glob("examples/development/generation/*.json")])
    def snapshot():
        return {p.relative_to(ROOT.parent).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    before = snapshot()
    subprocess.run([sys.executable, "-m", "unittest", "verify.generation.test_generation", "-v"], cwd=ROOT, check=True)
    if before != snapshot():
        raise RuntimeError("sources changed during tests; no evidence recorded")
    if args.record_evidence:
        modules = {"generators.py", "structures.py", "generation.py", "formatting.py", "feature_catalog.py"}
        ids = [f["id"] for f in collect_features()["features"] if Path(f["source"]).name in modules]
        ids.append("random.Random")
        write_json(receipt, {"schema_version":1, "status":"passed", "command":"python3 verify/run_generation.py --record-evidence",
                             "sources":before, "feature_ids":sorted(ids), "scope":"Independent invariants and CLI integration; partial coverage"})
        from tools.cp_tools.feature_catalog import main as catalog_main
        catalog_main(["catalog", "--update"])


if __name__ == "__main__":
    main()
