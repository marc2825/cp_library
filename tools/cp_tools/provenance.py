"""Validate imported-code provenance and report unresolved historical records."""
import argparse
from pathlib import Path
import re

from .repository import ROOT, digest, files, local_path, read_json, write_json


def inventory(root=ROOT):
    ledger = read_json(root / "verification/provenance.json")
    if ledger.get("schema_version") != 1:
        raise ValueError("unsupported provenance schema")
    known, identifiers = {}, set()
    for entry in ledger["entries"]:
        if entry["id"] in identifiers:
            raise ValueError(f"duplicate provenance id: {entry['id']}")
        identifiers.add(entry["id"])
        if entry["kind"] not in {"adapted", "dependency", "historical"}:
            raise ValueError(f"invalid provenance kind: {entry['id']}")
        if not entry["sources"] or not entry["changes"]:
            raise ValueError(f"empty source/change record: {entry['id']}")
        for url in entry["sources"]:
            if not url.startswith("https://"):
                raise ValueError(f"source must be HTTPS: {url}")
        revision = entry["imported_revision"]
        if revision is not None and not re.fullmatch(r"[0-9a-f]{40}", revision):
            raise ValueError(f"expected full imported commit: {entry['id']}")
        if revision is None and not entry.get("revision_note"):
            raise ValueError(f"missing explanation for unknown revision: {entry['id']}")
        license = entry["license"]
        if not license["id"] or not license["evidence"]:
            raise ValueError(f"missing license status/evidence: {entry['id']}")
        if license.get("file"):
            if digest(local_path(root, license["file"])) != license.get("sha256"):
                raise ValueError(f"license text changed: {entry['id']}")
        if not entry["local_sha256"]:
            raise ValueError(f"empty tracked file list: {entry['id']}")
        if "reviewed_upstream" in entry:
            review = entry["reviewed_upstream"]
            if not re.fullmatch(r"[0-9a-f]{40}", review["commit"]) or not re.fullmatch(r"[0-9a-f]{64}", review["source_sha256"] or ""):
                raise ValueError(f"incomplete reviewed snapshot: {entry['id']}")
        for name, expected in entry["local_sha256"].items():
            if name in known:
                raise ValueError(f"duplicate provenance path: {name}")
            if digest(local_path(root, name)) != expected:
                raise ValueError(f"provenance hash changed; review changes and update ledger: {name}")
            known[name] = entry
    rows = []
    for name in files(root, {".hpp", ".h"}):
        source = (root / name).read_text(encoding="utf-8")
        # Enforce explicit records for existing adaptation/attribution markers.
        if re.search(r"Adapted from|CC0|Unlicense|移植|Nyaan-san arigatou|yosupo-san arigatou|namespace radix_heap", source) and name not in known:
            raise ValueError(f"unregistered upstream attribution: {name}")
        entry = known.get(name)
        rows.append({"path": name, "record": entry["id"] if entry else None,
                     "kind": entry["kind"] if entry else "unreviewed",
                     "license": entry["license"]["id"] if entry else "NOASSERTION",
                     "imported_revision": entry["imported_revision"] if entry else None,
                     "source_hints": sorted(set(re.findall(r'https?://[^\s<>"\)]+', source)))})
    return {"schema_version": 1, "entries": ledger["entries"], "files": rows,
            "repository_license": "NOASSERTION"}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="write expanded JSON inventory")
    args = parser.parse_args(argv)
    result = inventory()
    if args.output:
        write_json(args.output, result)
    reviewed = sum(r["record"] is not None for r in result["files"])
    print(f"provenance: {len(result['entries'])} records, {reviewed} explicitly tracked files; "
          f"{len(result['files']) - reviewed} files awaiting individual review")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
