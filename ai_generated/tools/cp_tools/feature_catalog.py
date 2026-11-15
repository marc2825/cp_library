"""Deterministic repository-wide feature index; independent of verification ledger.

C++ registration unit is a public header, not every syntactically found function.
Python units are explicit __all__ exports or public top-level definitions.
"""
import argparse
import ast
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import sys
from urllib.parse import quote, unquote

from .contest import header_catalog, normalize
from .repository import ROOT, files, read_json, write_json

__all__ = ["collect_features", "render_markdown", "search_features", "check_catalog", "main"]

_CATEGORIES = {
    "basic_utility": "基本ユーティリティ", "union_find": "素集合データ構造",
    "range_query": "区間クエリ・セグメント木・Fenwick木", "static_query": "静的クエリ",
    "ordered_structure": "平衡木・永続データ構造", "offline_query": "オフラインクエリ",
    "graph_basic": "グラフ探索・最短路", "graph_connectivity": "グラフ連結性",
    "graph_flow": "フロー・マッチング", "graph_combinatorics": "グラフ組合せ",
    "tree": "木アルゴリズム", "dp_optimization": "動的計画法・DP最適化",
    "number_theory_basic": "基本数論", "number_theory_convolution": "高度な数論",
    "combinatorics": "組合せ論", "convolution": "畳み込み", "fps": "形式的冪級数・多項式",
    "linear_recurrence": "線形漸化式", "set_function": "集合関数",
    "linear_algebra": "線形代数", "string_algorithm": "文字列アルゴリズム",
    "computational_geometry": "計算幾何", "combinatorial_optimization": "組合せ最適化",
    "game_theory": "ゲーム理論", "heuristic_search": "探索・ヒューリスティック",
    "numerical_analysis": "数値解析", "parsing": "構文解析", "data_structure": "旧データ構造",
    "graph_theory": "旧グラフ・木", "test_generation": "テストケース生成・乱数生成",
    "development": "開発補助・差分テスト・検証", "legacy": "旧ライブラリ",
}
_SYNONYMS = [
    (r"dsu|union.?find|disjoint.set", ["Union-Find", "DSU", "素集合データ構造", "連結性"]),
    (r"fenwick|binary.indexed", ["Fenwick Tree", "BIT", "Binary Indexed Tree", "フェニック木"]),
    (r"segment.?tree|seg.?tree", ["Segment Tree", "segtree", "セグ木", "セグメント木"]),
    (r"dijkstra|shortest|bellman|warshall", ["Shortest Path", "shortest-path", "最短路", "最短経路"]),
    (r"random|generator|generation|^tree$|^dag$|^forest$", ["Random Generator", "乱数生成", "テストケース生成"]),
    (r"stress", ["differential testing", "差分テスト", "ランダムテスト"]),
    (r"string|suffix|palindrome|anagram|bracket", ["string", "文字列"]),
    (r"graph|dag|bipartite", ["graph", "グラフ"]),
]
_JA = {"random_int": "整数生成", "random_int_tuple": "複数整数・タプル生成",
       "random_ordered_tuple": "順序制約付きタプル生成", "random_interval": "区間生成",
       "random_array": "整数配列生成", "random_array_with_sum": "総和制約付き配列生成",
       "random_string": "文字列生成", "random_anagram_pair": "アナグラム対生成",
       "random_brackets": "正しい括弧列生成", "random_matrix": "行列生成",
       "random_grid": "到達可能性付きグリッド生成", "random_tree": "木生成",
       "random_graph": "疎グラフ生成", "random_dag": "DAG生成", "random_sizes": "ケース間総予算割当",
       "random_queries": "状態付きクエリ列生成", "format_case": "標準入力レイアウト",
       "generate_cases": "JSON制約から複数ケース生成", "Random": "再現可能なSplitMix64乱数エンジン"}
_PY_DOCS = {"random_data": "docs/development/random_data.md", "stress": "docs/development/stress.md",
            "bundle": "docs/development/bundle.md", "contest": "docs/usage.md",
            "generators": "docs/development/generation.md", "structures": "docs/development/generation.md",
            "generation": "docs/development/generation_json.md", "formatting": "docs/development/generation.md",
            "feature_catalog": "catalog/README.md"}
_GROUP = {"generators": "generator", "structures": "generator", "random_data": "random",
          "formatting": "formatting", "generation": "generation"}


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _evidence(repo):
    """Only a passed receipt matching all recorded source/test hashes is evidence."""
    path = repo / "ai_generated/catalog/test_evidence.json"
    if not path.exists():
        return set(), None
    receipt = read_json(path)
    if receipt.get("schema_version") != 1 or receipt.get("status") != "passed" or not receipt.get("sources"):
        raise ValueError("invalid feature test evidence")
    for name, digest in receipt["sources"].items():
        source = (repo / name).resolve()
        if not source.is_relative_to(repo.resolve()):
            raise ValueError("evidence path outside repository")
        if not source.is_file() or _sha(source) != digest:
            return set(), "stale"
    return set(receipt.get("feature_ids", [])), "current"


def _keywords(text, category):
    terms = [category, _CATEGORIES.get(category, category)]
    for pattern, aliases in _SYNONYMS:
        if re.search(pattern, text, re.I):
            terms.extend(aliases)
    return sorted(set(terms))


def _description(text, fallback):
    text = re.sub(r"\A---\n.*?\n---\n", "", text, flags=re.S)
    for paragraph in text.split("\n\n"):
        paragraph = paragraph.strip()
        if paragraph and not paragraph.startswith(("#", "```", "|", "- ", "[")):
            paragraph = re.sub(r"\[([^]]+)\]\([^)]*\)", r"\1", paragraph)
            return " ".join(paragraph.replace("`", "").split())[:180]
    return fallback


def collect_features(repo=ROOT.parent):
    repo = repo.resolve()
    root = repo / "ai_generated"
    docs = {e["header"]: e for e in header_catalog(root)}
    manifests = {}
    for path in sorted((root / "verify").glob("*/manifest.json")):
        for entry in read_json(path).get("algorithms", []):
            manifests.setdefault(entry["header"], entry)
    passed, evidence_state = _evidence(repo)
    features = []
    for area in ("ai_generated", "completed", "depricated"):
        base = repo / area
        for relative in files(base, {".hpp", ".h", ".cpp"}):
            parts = Path(relative).parts
            if area == "ai_generated" and (parts[0] in {"tools", "verify", "examples", "visualizations"} or "detail" in parts):
                continue
            source = f"{area}/{relative}"
            if area == "ai_generated" and Path(relative).suffix == ".cpp":
                # Standalone implementation .cpp files are included if present.
                pass
            category = parts[0] if area == "ai_generated" and len(parts) > 1 else "legacy"
            docrow, metadata = docs.get(relative, {}), manifests.get(relative, {})
            doc = docrow.get("doc")
            if metadata.get("doc") and (root / "docs" / metadata["doc"]).is_file():
                doc = "docs/" + metadata["doc"]
            if doc is None and area == "ai_generated" and (root / "docs" / category / "README.md").is_file():
                doc = f"docs/{category}/README.md"
            title = metadata.get("title") or docrow.get("title") or Path(relative).stem
            text = (root / doc).read_text(encoding="utf-8") if doc else ""
            is_legacy = area == "ai_generated" and category in {"data_structure", "graph_theory"}
            row = {"id": area + "." + str(Path(relative).with_suffix("")).replace("/", "."),
                   "name": Path(relative).stem.replace("_", " "),
                   "name_ja": title if re.search(r"[^\x00-\x7f]", title) else f"{_CATEGORIES.get(category, 'ライブラリ')}：{title}",
                   "category": category, "language": "C++", "area": area,
                   "status": "deprecated" if area == "depricated" else "experimental" if is_legacy else "implemented",
                   "verification": "unknown", "source": source,
                   "documentation": f"ai_generated/{doc}" if doc else None,
                   "description": _description(text, f"{title} ({area})"),
                   "keywords": _keywords(relative + " " + title, category),
                   "public_api": [], "api_summary": metadata.get("api"),
                   "role": "aggregate" if Path(relative).name == "all.hpp" else "template" if Path(relative).suffix == ".cpp" else "public_header",
                   "verification_note": "テストの存在・説明文は実行証跡ではありません。検証台帳で現在の記録を確認してください。"}
            for field, key in (("complexity", "complexity"), ("constraints", "condition")):
                if metadata.get(key):
                    row[field] = metadata[key]
            features.append(row)
    for relative in files(root / "tools", {".py"}):
        path = root / "tools" / relative
        module = path.stem
        if module == "__init__":
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"))
        exports = None
        for node in tree.body:
            if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "__all__" for t in node.targets):
                exports = set(ast.literal_eval(node.value))
        for node in tree.body:
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) or node.name.startswith("_"):
                continue
            if exports is not None and node.name not in exports:
                continue
            category = "test_generation" if module in {"random_data", "generators", "structures", "generation", "formatting"} else "development"
            identifier = f"{_GROUP.get(module, 'tool.' + module)}.{node.name}"
            api = [node.name]
            if isinstance(node, ast.ClassDef):
                api += [node.name + "." + m.name for m in node.body if isinstance(m, ast.FunctionDef) and not m.name.startswith("_")]
            doc = _PY_DOCS.get(module, "docs/development/README.md")
            if not (root / doc).exists():
                doc = None
            features.append({"id": identifier, "name": node.name,
                             "name_ja": _JA.get(node.name, f"{_CATEGORIES[category]}：{node.name}"),
                             "category": category, "language": "Python", "area": "ai_generated",
                             "status": "implemented", "verification": "partially_tested" if identifier in passed else "unknown",
                             "source": "ai_generated/tools/" + relative,
                             "documentation": "ai_generated/" + doc if doc else None,
                             "description": (ast.get_docstring(node) or f"{module}.{node.name}").split("\n\n")[0].replace("\n", " "),
                             "keywords": _keywords(module + " " + node.name, category), "public_api": api,
                             "line": node.lineno, "role": "public_api",
                             "verification_note": "現在のソースと一致するローカルテスト証跡あり。全入力の正しさを保証するものではありません。" if identifier in passed else "実行証跡未登録"})
    features.sort(key=lambda row: row["id"])
    if len({r["id"] for r in features}) != len(features):
        raise ValueError("duplicate feature IDs")
    return {"schema_version": 1, "path_base": "repository_root",
            "registration_unit": "C++ public headers (internal detail excluded); Python public top-level definitions and class methods",
            "summary": dict(sorted(Counter(row["area"] for row in features).items())),
            "evidence": evidence_state, "features": features}


def render_markdown(data):
    def escape(value):
        return str(value).replace("|", "\\|").replace("\n", " ").replace("<", "&lt;").replace(">", "&gt;")
    def link(label, path):
        relative = Path(path).relative_to("ai_generated").as_posix() if path.startswith("ai_generated/") else "../" + path
        return f"[{label}]({quote(relative, safe='/._-')})"
    rows = ["# 実装済み機能一覧・検索", "",
            f"{len(data['features'])} 件。ブラウザのページ内検索（Ctrl/Cmd+F）で日本語・英語・API名を検索できます。",
            "", "[利用方法・更新規約](catalog/README.md) / [生成ライブラリ](docs/development/generation.md) / [検証台帳](docs/development_records/README.md)",
            "", "C++は公開ヘッダ単位、Pythonは公開関数・クラス単位です。クラスのメソッドは同じレコードの公開APIに含めます。",
            "`implemented` は実装の存在、`experimental` は旧構成、`deprecated` は旧版です。検証状態 `unknown` は実行証跡未登録、`partially_tested` はハッシュが一致するローカルテスト証跡ありを表します。AI生成という所属と検証状態は別です。",
            "`completed/` は現在実装ファイルなし。内部 detail ヘッダ、テスト、例、図解の実装は公開機能数に含めません。", ""]
    categories = sorted({f["category"] for f in data["features"]})
    rows.extend(f"- [{_CATEGORIES.get(c, c)} / {c}](#{c.replace('_', '-')})" for c in categories)
    for category in categories:
        rows.extend(["", f"## {category.replace('_', '-')}", "", _CATEGORIES.get(category, category), "",
                     "| 機能 / API / 検索語 | 説明 | 所属 / 実装状態 / 検証状態 | 実装 | ドキュメント |", "|---|---|---|---|---|"])
        for f in data["features"]:
            if f["category"] != category:
                continue
            terms = " / ".join(dict.fromkeys([f["name"], f["name_ja"], *f["public_api"], *f["keywords"]]))
            rows.append(f"| {escape(terms)} | {escape(f['description'])} | {f['area']} / {f['status']} / {f['verification']} | {link('Source', f['source'])} | " +
                        (link("Docs", f["documentation"]) if f["documentation"] else "—") + " |")
    return "\n".join(rows) + "\n"


def search_features(data, query=(), *, category=None, status=None, verification=None):
    if isinstance(query, str):
        query = query.split()
    terms = [normalize(q) for q in query]
    return [row for row in data["features"] if
            (category is None or row["category"] == category) and
            (status is None or row["status"] == status) and
            (verification is None or row["verification"] == verification) and
            all(term in normalize(" ".join(str(row[k]) for k in ("id", "name", "name_ja", "description", "keywords", "public_api", "source"))) for term in terms)]


def check_catalog(repo=ROOT.parent):
    repo = repo.resolve()
    expected = collect_features(repo)
    root = repo / "ai_generated"
    json_path, md_path = root / "catalog/features.json", root / "FEATURES.md"
    if read_json(json_path) != expected or md_path.read_text(encoding="utf-8") != render_markdown(expected):
        raise ValueError("catalog is stale; run cplib catalog --update")
    ids = [f["id"] for f in expected["features"]]
    if len(set(ids)) != len(ids):
        raise ValueError("duplicate feature IDs")
    for feature in expected["features"]:
        for key in ("source", "documentation"):
            if feature[key] is not None:
                path = (repo / feature[key]).resolve()
                if not path.is_relative_to(repo) or not path.is_file():
                    raise ValueError(f"broken {key}: {feature['id']}")
        if feature["status"] not in {"implemented", "experimental", "deprecated", "planned"} or feature["verification"] not in {"tested", "partially_tested", "untested", "unknown"}:
            raise ValueError("invalid feature status")
    # Check links in generated index and navigation pages (ignore fenced examples).
    for path in [md_path, repo / "README.md", root / "README.md", root / "docs/README.md", root / "catalog/README.md"]:
        content = re.sub(r"```.*?```", "", path.read_text(encoding="utf-8"), flags=re.S)
        for target in re.findall(r"\]\(([^\s)]+)\)", content):
            if "://" not in target and not target.startswith("#") and not (path.parent / unquote(target.split("#")[0])).exists():
                raise ValueError(f"broken Markdown link: {path}: {target}")
    # Frontmatter documents must point to an existing, registered public header.
    sources = {f["source"] for f in expected["features"]}
    for doc in (root / "docs").rglob("*.md"):
        match = re.search(r"^documentation_of:\s*//([^\r\n]+)", doc.read_text(encoding="utf-8"), re.M)
        if match and "ai_generated/" + match[1].strip() not in sources:
            raise ValueError(f"orphan public API documentation: {doc}")
    return expected


def main(argv=None):
    parser = argparse.ArgumentParser(prog="cplib", description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    search = commands.add_parser("search")
    search.add_argument("query", nargs="*")
    search.add_argument("--category")
    search.add_argument("--status", choices=["implemented", "experimental", "deprecated", "planned"])
    search.add_argument("--verification", choices=["tested", "partially_tested", "untested", "unknown"])
    search.add_argument("--json", action="store_true")
    catalog = commands.add_parser("catalog")
    actions = catalog.add_mutually_exclusive_group(required=True)
    actions.add_argument("--update", action="store_true")
    actions.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.command == "search":
            data = read_json(ROOT / "catalog/features.json")
            rows = search_features(data, args.query, category=args.category, status=args.status, verification=args.verification)
            if args.json:
                print(json.dumps(rows, ensure_ascii=False, indent=2))
            else:
                for row in rows:
                    print(f"{row['id']}\t{row['name_ja']}\t{row['status']} / {row['verification']}")
                    print(f"  source: {ROOT.parent / row['source']}")
                    if row["documentation"]:
                        print(f"  docs:   {ROOT.parent / row['documentation']}")
            return 0 if rows else 1
        if args.update:
            data = collect_features()
            write_json(ROOT / "catalog/features.json", data)
            (ROOT / "FEATURES.md").write_text(render_markdown(data), encoding="utf-8")
        else:
            data = check_catalog()
        print(f"catalog: {len(data['features'])} features; {data['summary']}")
        return 0
    except (OSError, ValueError) as exc:
        parser.exit(2, f"cplib: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
