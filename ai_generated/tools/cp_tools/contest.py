"""Find, include, extract and bundle C++ contest-library headers."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

from .bundle import Bundler, main as bundle_main
from .repository import ROOT, local_path


ALIASES = {
    "dsu": "union_find/disjoint_set_union.hpp",
    "fenwick": "range_query/fenwick_tree.hpp",
    "segtree": "range_query/segtree.hpp",
    "modint": "basic_utility/static_modint.hpp",
}


def public_headers(root=ROOT):
    """Discover category headers; exclude implementation-detail directories."""
    return sorted({p.relative_to(root).as_posix()
                   for aggregate in root.glob("*/all.hpp")
                   for p in aggregate.parent.rglob("*.hpp")
                   if "detail" not in p.relative_to(root).parts})


def normalize(text):
    return "".join(c for c in text.casefold() if c.isalnum())


def header_catalog(root=ROOT):
    records = {h: {"header": h, "title": Path(h).stem, "doc": None}
               for h in public_headers(root)}
    for doc in sorted((root / "docs").rglob("*.md")):
        text = doc.read_text(encoding="utf-8")
        target = re.search(r"^documentation_of:\s*//([^\r\n]+)", text, re.M)
        if not target or target[1].strip() not in records:
            continue
        entry = records[target[1].strip()]
        if entry["doc"] is not None:
            continue
        title = re.search(r"^title:\s*([^\r\n]+)", text, re.M)
        entry["title"] = title[1].strip().strip("\"'") if title else entry["title"]
        entry["doc"] = doc.relative_to(root).as_posix()
    return list(records.values())


def resolve_header(name, root=ROOT):
    """Exact repository paths, four aliases, then unique category-relative stems."""
    if name in ALIASES:
        return local_path(root, ALIASES[name])
    path = root / name
    if path.is_file():
        path = local_path(root, name)
        if path.suffix not in {".hpp", ".h"}:
            raise ValueError(f"not a C++ header: {name}")
        return path
    names = public_headers(root)
    matches = [h for h in names if name in {h, str(Path(h).with_suffix("")), Path(h).name, Path(h).stem}]
    if len(matches) == 1:
        return root / matches[0]
    if matches:
        raise ValueError("ambiguous header; use its full path:\n  " + "\n  ".join(matches))
    raise ValueError(f"unknown header: {name}; try 'cplib find {name}'")


def include_lines(headers, root=ROOT):
    return [f'#include "{p.relative_to(root).as_posix()}"' for p in dict.fromkeys(headers)]


def write_output(path, text, *, force=False, protected=()):
    """Refuse accidental replacement; --force replaces atomically after validation."""
    if path is None:
        sys.stdout.write(text)
        return
    path = Path(path)
    if path.resolve() in {Path(p).resolve() for p in protected}:
        raise ValueError("output must not overwrite an input dependency")
    path.parent.mkdir(parents=True, exist_ok=True)
    if not force:
        with path.open("x", encoding="utf-8") as stream:
            stream.write(text)
        return
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=".cplib-", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(text)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def make_snippets(root=ROOT):
    entries = header_catalog(root)
    stems = Counter(Path(e["header"]).stem for e in entries)
    snippets = {}
    for entry in entries:
        header = entry["header"]
        prefixes = ["cp/" + str(Path(header).with_suffix(""))]
        stem = Path(header).stem
        if stems[stem] == 1 and (stem not in ALIASES or ALIASES[stem] == header):
            prefixes.append("cp/" + stem)
        prefixes.extend("cp/" + alias for alias, target in ALIASES.items() if target == header)
        # Escape snippet metacharacters even though current repository paths are simple.
        body = f'#include "{header}"'.replace("\\", "\\\\").replace("$", "\\$").replace("}", "\\}")
        snippets[header] = {"scope": "cpp", "prefix": list(dict.fromkeys(prefixes)),
                            "body": [body], "description": entry["title"]}
    return snippets


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    # Preserve the original bundler's CLI/semantics, adding this checkout as an include root.
    if argv and argv[0] == "bundle":
        return bundle_main(["-I", str(ROOT), *argv[1:]])
    parser = argparse.ArgumentParser(prog="cplib", description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("root", help="print the absolute library directory")
    search = commands.add_parser("find", help="search category headers and Japanese documentation titles")
    search.add_argument("query", nargs="*")
    create = commands.add_parser("init", help="create a C++17 main.cpp; never overwrite an existing file")
    create.add_argument("source", nargs="?", type=Path, default=Path("main.cpp"))
    create.add_argument("-u", "--use", action="append", default=[], metavar="HEADER")
    extract = commands.add_parser("snippet", help="extract headers with their local dependencies")
    extract.add_argument("headers", nargs="+")
    extract.add_argument("-I", "--include", action="append", type=Path, default=[])
    extract.add_argument("-o", "--output", type=Path)
    extract.add_argument("--force", action="store_true", help="replace an existing output file")
    extract.add_argument("--check", action="store_true", help="compile as C++17 without requiring main()")
    extract.add_argument("--compiler", default="g++")
    extract.add_argument("--flag", action="append", default=[])
    commands.add_parser("bundle", help="expand a solution for submission; see 'cplib bundle --help'")
    snippets = commands.add_parser("snippets", help="generate VS Code include completions for this workspace")
    snippets.add_argument("-o", "--output", type=Path, default=Path(".vscode/cp-library.code-snippets"))
    snippets.add_argument("--force", action="store_true", help="replace an existing generated snippet file")
    args = parser.parse_args(argv)
    try:
        if args.command == "root":
            print(ROOT)
        elif args.command == "find":
            found = []
            for entry in header_catalog():
                aliases = " ".join(k for k, h in ALIASES.items() if h == entry["header"])
                text = entry["header"] + " " + entry["title"] + " " + aliases
                def matches(query):
                    if query.casefold() == "dsu":
                        return re.search(r"(?<![A-Za-z0-9])dsu(?![A-Za-z0-9])", text, re.I) is not None
                    return normalize(query) in normalize(text)
                if all(matches(q) for q in args.query):
                    found.append(entry)
                    print(f'{entry["header"]}\t{entry["title"]}')
                    if entry["doc"]:
                        print(f'  {ROOT / entry["doc"]}')
            if not found:
                print("No matching headers.", file=sys.stderr)
                return 1
        elif args.command == "init":
            headers = [resolve_header(h) for h in args.use]
            includes = "\n".join(["#include <iostream>", *include_lines(headers)])
            text = includes + "\n\nint main() {\n    std::ios::sync_with_stdio(false);\n    std::cin.tie(nullptr);\n\n    // Write your solution here. Library APIs use the cp:: namespace.\n    return 0;\n}\n"
            write_output(args.source, text, protected=headers)
            print(f"Created {args.source}", file=sys.stderr)
        elif args.command == "snippet":
            headers = [resolve_header(h) for h in args.headers]
            bundler = Bundler([ROOT, *args.include])
            text = "".join(bundler.expand(p) for p in headers)
            if args.check:
                with tempfile.TemporaryDirectory(prefix="cplib-check-") as temporary:
                    source = Path(temporary) / "snippet.cpp"
                    source.write_text(text, encoding="utf-8")
                    subprocess.run([args.compiler, "-std=c++17", *args.flag, "-fsyntax-only", str(source)],
                                   check=True)
            write_output(args.output, text, force=args.force, protected=bundler.dependencies)
        elif args.command == "snippets":
            data = make_snippets()
            write_output(args.output, json.dumps(data, ensure_ascii=False, indent=2) + "\n", force=args.force,
                         protected=[Path(__file__), ROOT / "tools/cplib.py"])
            print(f"Created {args.output}: {len(data)} C++ include snippets", file=sys.stderr)
        return 0
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        parser.exit(2, f"cplib: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
