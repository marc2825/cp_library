"""Conservative repository inventory shared by provenance and verification tools."""
import hashlib
import json
import os
from pathlib import Path
import re

from .bundle import directives

ROOT = Path(__file__).resolve().parents[2]
IGNORED = {"build", ".git", ".venv", "__pycache__", "node_modules"}


def files(root, suffixes):
    result = []
    for directory, children, names in os.walk(root):
        children[:] = [n for n in children if n not in IGNORED and not n.startswith(".")]
        for name in names:
            path = Path(directory) / name
            if not name.startswith(".") and path.suffix in suffixes:
                path = local_path(root, path)
                result.append(path.relative_to(root.resolve()).as_posix())
    return sorted(set(result))


def local_path(root, name):
    path = (root / name).resolve()
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError(f"missing or outside repository: {name}")
    return path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def include_graph(root):
    """Literal includes, both branches retained; no macro/preprocessor evaluation."""
    graph = {}
    for name in files(root, {".hpp", ".h", ".cpp"}):
        dependencies = set()
        for match in directives((root / name).read_text(encoding="utf-8")):
            if match[1] != "include":
                continue
            include = re.match(r'\s*([<"])([^>"\n]+)[>"]', match[2])
            if not include:
                continue
            candidates = ([root / name] if include[1] == '"' else [])
            candidates = [p.parent / include[2] for p in candidates] + [root / include[2]]
            for path in candidates:
                if path.is_file():
                    path = local_path(root, path)
                    dependencies.add(path.relative_to(root.resolve()).as_posix())
                    break
        graph[name] = sorted(dependencies)
    return graph


def closure(graph, name):
    seen, pending = set(), [name]
    while pending:
        node = pending.pop()
        if node not in seen:
            seen.add(node)
            pending.extend(graph.get(node, []))
    return sorted(seen)


def source_snapshot(root):
    # Deliberately conservative: any source/config change expires all local receipts.
    paths = files(root, {".hpp", ".h", ".cpp", ".py", ".json", ".toml"})
    paths = [p for p in paths if not p.startswith("verification/evidence/")]
    paths += [p.relative_to(root).as_posix() for p in sorted((root / ".competitive-verifier").iterdir())
              if p.is_file() and p.suffix in {".toml", ".json", ".txt"}] if (root / ".competitive-verifier").is_dir() else []
    return {p: digest(root / p) for p in sorted(set(paths))}


def runners(root):
    return {p.stem.removeprefix("run_"): p.relative_to(root).as_posix()
            for p in sorted((root / "verify").glob("run_*.py"))}
