#!/usr/bin/env python3
"""図解一覧 visualizations/index.html を visualizations/entries.json から生成する。

entries.json には図解ごとに page（visualizations/ 以下のフォルダ）・mode（#以降）・題名・要約を
書きます。分野・解説・実装へのリンクと検索用の同義語は、verify/*/manifest.json と
catalog/features.json から自動で決めます。決まらない項目だけ category / doc / source を
entries.json に書いてください。解説リンクは tools/build_docs_html.py が作る html/ を指します。

  python3 tools/build_visualization_index.py          # 生成
  python3 tools/build_visualization_index.py --check  # index.html が最新でなければ終了コード 1
"""
from __future__ import annotations

__all__ = ["main"]

import argparse
import glob
import html
import json
import posixpath
import re
import sys
import unicodedata
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIS = "visualizations"
ENTRIES = ROOT / VIS / "entries.json"
INDEX = ROOT / VIS / "index.html"

# 大分類 → 実装カテゴリ。カテゴリ名は tools/cp_tools/feature_catalog.py と揃える。
GROUPS = [
    ("data-structures", "データ構造", "集合・区間・順序・時間軸を管理する構造",
     ["union_find", "range_query", "static_query", "ordered_structure", "offline_query"]),
    ("graphs", "グラフ・木", "探索・最短路・連結性・フロー・木のクエリ",
     ["graph_basic", "graph_connectivity", "graph_flow", "graph_combinatorics", "tree"]),
    ("math", "数学", "数論・組合せ・線形代数・数値計算",
     ["number_theory_basic", "number_theory_convolution", "combinatorics", "linear_algebra", "numerical_analysis"]),
    ("transforms", "畳み込み・多項式", "FFT/NTT・形式的冪級数・漸化式・集合関数",
     ["convolution", "fps", "linear_recurrence", "set_function"]),
    ("strings", "文字列・構文解析", "照合・接尾辞構造・回文・オートマトン",
     ["string_algorithm", "parsing"]),
    ("geometry", "計算幾何", "点・線分・多角形・円・3次元",
     ["computational_geometry"]),
    ("optimization", "DP・最適化・探索", "DP高速化・組合せ最適化・ゲーム・ヒューリスティック",
     ["dp_optimization", "combinatorial_optimization", "game_theory", "heuristic_search"]),
    ("foundation", "基盤・開発ツール", "入出力・多倍長・テスト生成・検証",
     ["basic_utility", "development"]),
]
CATEGORY_NAMES = {
    "basic_utility": "基本ユーティリティ", "union_find": "素集合データ構造（Union-Find）",
    "range_query": "区間クエリ・セグメント木・Fenwick木", "static_query": "静的クエリ・集合管理",
    "ordered_structure": "平衡木・永続データ構造", "offline_query": "オフラインクエリ・二次元",
    "graph_basic": "グラフ探索・最短路",
    "graph_connectivity": "グラフ連結性・分解", "graph_flow": "フロー・マッチング",
    "graph_combinatorics": "グラフ組合せ・数え上げ", "tree": "木アルゴリズム",
    "dp_optimization": "動的計画法・DP最適化", "number_theory_basic": "基本数論",
    "number_theory_convolution": "高度な数論", "combinatorics": "組合せ論", "convolution": "畳み込み",
    "fps": "形式的冪級数・多項式", "linear_recurrence": "線形漸化式", "set_function": "集合関数",
    "linear_algebra": "線形代数", "string_algorithm": "文字列アルゴリズム",
    "computational_geometry": "計算幾何", "combinatorial_optimization": "組合せ最適化",
    "game_theory": "ゲーム理論", "heuristic_search": "探索・ヒューリスティック",
    "numerical_analysis": "数値解析", "parsing": "構文解析", "development": "開発補助・テスト・検証",
}
# 実装の置き場所（先頭のフォルダ）が分野名と異なるもの。
CATEGORY_ALIASES = {"graph": "graph_basic", "graph_theory": "graph_basic", "tools": "development",
                    "test_generation": "development", "data_structure": "static_query"}
INDEX_STOPWORDS = {"mod", "and", "or", "of", "the", "to", "in", "on", "by", "all", "with", "for", "id", "ok"}
LATIN_TERM = re.compile(r"[A-Za-z0-9][A-Za-z0-9]*(?:[ \-–'’.+_][A-Za-z0-9]+)*(?:\([A-Za-z0-9^,]+\))?")


def esc(value: str) -> str:
    return html.escape(value, quote=True)


def normalize(text: str) -> str:
    text = unicodedata.normalize("NFKC", text).lower()
    return re.sub(r"[\s\-–—_・/.,'’()\[\]{}「」『』【】:;|]+", "", text)


def load_manifests() -> dict[str, dict[str, dict]]:
    result: dict[str, dict[str, dict]] = defaultdict(dict)
    for path in sorted(glob.glob(str(ROOT / "verify" / "*" / "manifest.json"))):
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        items = data if isinstance(data, list) else next(
            (value for value in data.values() if isinstance(value, list) and value and isinstance(value[0], dict)), [])
        page = Path(path).parent.name
        for item in items:
            if isinstance(item, dict) and item.get("mode"):
                result[page].setdefault(item["mode"], item)
    return result


def load_features():
    data = json.loads((ROOT / "catalog" / "features.json").read_text(encoding="utf-8"))
    by_source, by_stem = {}, defaultdict(list)
    for feature in data["features"]:
        if feature.get("area") != "ai_generated" or feature.get("role") == "aggregate":
            continue
        source = feature["source"].removeprefix("ai_generated/")
        by_source[source] = feature
        by_stem[posixpath.splitext(posixpath.basename(source))[0]].append(feature)
    return by_source, by_stem


def page_doc(page: str) -> str | None:
    """図解ページ自身が既定で指している解説（docs-link / doc-link）。"""
    text = (ROOT / VIS / page / "index.html").read_text(encoding="utf-8")
    match = re.search(r'id="docs?-link"[^>]*href="([^"#]+\.md)', text) or re.search(r'href="(\.\./\.\./docs/[^"#]+\.md)', text)
    if not match:
        return None
    return posixpath.normpath(posixpath.join(VIS, page, match.group(1)))


class Catalog:
    def __init__(self):
        self.entries = json.loads(ENTRIES.read_text(encoding="utf-8"))["entries"]
        self.manifests = load_manifests()
        self.by_source, self.by_stem = load_features()
        self.errors: list[str] = []
        self.page_sources: dict[str, str] = {}
        self.items = [self.resolve(entry) for entry in self.entries]

    def page_mentions(self, page: str, mode: str) -> bool:
        """モード名がページの HTML・JS に文字列やキーとして現れるか（リンク切れの簡易検査）。"""
        if page not in self.page_sources:
            folder = ROOT / VIS / page
            self.page_sources[page] = "".join(path.read_text(encoding="utf-8", errors="replace")
                                              for path in sorted(folder.iterdir()) if path.suffix in (".html", ".js"))
        pattern = r"""["'`#]""" + re.escape(mode) + r"""["'`]|\b""" + re.escape(mode) + r"""\s*:"""
        return re.search(pattern, self.page_sources[page]) is not None

    def pick_feature(self, mode: str, page: str):
        candidates = self.by_stem.get(mode, [])
        if not candidates:
            return None
        rank = lambda f: (f["category"] != page, f["status"] != "implemented", f["category"] in ("data_structure", "graph_theory"))
        return sorted(candidates, key=rank)[0]

    def resolve(self, entry: dict) -> dict:
        page, mode = entry["page"], entry.get("mode", "")
        label = f"{page}#{mode}" if mode else page
        if not (ROOT / VIS / page / "index.html").is_file():
            self.errors.append(f"{label}: visualizations/{page}/index.html がありません")
        item = self.manifests.get(page, {}).get(mode, {}) if mode else {}
        header = entry.get("source") or item.get("header")
        if isinstance(header, list):
            header = header[0]
        feature = self.by_source.get(header) if header else None
        if not header and mode:
            feature = self.pick_feature(mode, page)
            # カテゴリ別の旧ページでは、別カテゴリの同名ファイルを誤って拾わない
            if feature and page in CATEGORY_NAMES and feature["category"] != page:
                feature = None
            header = feature["source"].removeprefix("ai_generated/") if feature else None
        category = entry.get("category")
        if not category and header:
            category = CATEGORY_ALIASES.get(header.split("/")[0], header.split("/")[0])
        if not category and page in CATEGORY_NAMES:
            category = page
        if category not in CATEGORY_NAMES:
            self.errors.append(f"{label}: 分野を決められません（entries.json に category を指定してください）")
        doc = entry.get("doc")
        if not doc and item.get("doc"):
            doc = posixpath.join("docs", item["doc"])
        if not doc and feature and feature.get("documentation"):
            doc = feature["documentation"].removeprefix("ai_generated/")
        if not doc and page in CATEGORY_NAMES and (ROOT / "docs" / page / "README.md").is_file():
            doc = f"docs/{page}/README.md"  # モード別の解説が決まらないときはカテゴリの目次へ
        if not doc:
            doc = page_doc(page)
        for path, kind in ((doc, "doc"), (header, "source")):
            if path and not (ROOT / path).is_file():
                self.errors.append(f"{label}: {kind} {path} がありません")
        if mode and not self.page_mentions(page, mode):
            self.errors.append(f"{label}: モード {mode} が visualizations/{page}/ の HTML・JS に見つかりません")
        terms = [entry["title"], entry["summary"], entry.get("code", ""), mode.replace("_", " "),
                 CATEGORY_NAMES.get(category, ""), category or ""]
        if feature:
            terms += [feature["name"], feature.get("name_ja", ""), *feature.get("keywords", [])]
        if header:
            terms.append(posixpath.splitext(posixpath.basename(header))[0].replace("_", " "))
        return {**entry, "mode": mode, "category": category, "doc": doc, "source": header,
                "feature": feature, "terms": " ".join(dict.fromkeys(t for t in terms if t))}

    # ---- 索引 ---------------------------------------------------------------
    def index_terms(self) -> dict[str, list[tuple[str, list[dict]]]]:
        found: dict[str, dict] = {}
        for item in self.items:
            names = [match.group(0).strip() for match in LATIN_TERM.finditer(item["title"])]
            if item["feature"]:
                name = item["feature"]["name"]
                names.append(name[:1].upper() + name[1:])
            for name in names:
                key = normalize(name)
                if not useful_term(name, key):
                    continue
                slot = found.setdefault(key, {"name": name, "items": []})
                if item not in slot["items"]:
                    slot["items"].append(item)
        letters: dict[str, list] = defaultdict(list)
        for key, slot in sorted(found.items(), key=lambda pair: (pair[0], pair[1]["name"])):
            letter = key[0].upper() if key[0].isalpha() else "0–9"
            letters[letter].append((slot["name"], slot["items"]))
        return dict(sorted(letters.items(), key=lambda pair: (pair[0] == "0–9", pair[0])))


def useful_term(name: str, key: str) -> bool:
    """索引に載せる語か。小文字だけの1語（apply・split など操作名）や数字だけの断片は除く。"""
    if len(key) < 2 or not re.search(r"[a-z]", key) or name.lower() in INDEX_STOPWORDS:
        return False
    if re.match(r"\d+ ", name) or re.fullmatch(r"mod \w{1,2}(\^\w+)?", name, re.IGNORECASE):
        return False
    return " " in name or not name.islower()


def href(item: dict) -> str:
    return f'{item["page"]}/index.html' + (f'#{item["mode"]}' if item["mode"] else "")


def doc_href(path: str) -> str:
    """docs/x.md → ../html/docs/x.html（visualizations/index.html からの相対）。"""
    return "../html/" + path[:-3] + ".html" if path.endswith(".md") else "../" + path


def render_entry(item: dict) -> str:
    code = f'<span class="entry-code">{esc(item["code"])}</span>' if item.get("code") else ""
    links = []
    if item["doc"]:
        links.append(f'<a class="entry-link" href="{esc(doc_href(item["doc"]))}">解説</a>')
    if item["source"]:
        links.append(f'<a class="entry-link" href="../{esc(item["source"])}">実装</a>')
    return (f'<li class="entry" data-terms="{esc(item["terms"])}">'
            f'<h4 class="entry-title"><a class="entry-open" href="{esc(href(item))}">{esc(item["title"])}</a></h4>'
            f'<p class="entry-summary">{esc(item["summary"])}</p>'
            f'<p class="entry-meta">{code}{"".join(links)}</p></li>')


def render(catalog: Catalog) -> str:
    by_category: dict[str, list[dict]] = defaultdict(list)
    for item in catalog.items:
        by_category[item["category"]].append(item)
    # 分野ごとに、その分野の基本ページを先頭に置き、残りは entries.json で後ろにあるページ
    # （先に作られた基本的なページ）から順に並べる。同じページ内は entries.json の順。
    page_rank = {page: -index for index, page in enumerate(dict.fromkeys(i["page"] for i in catalog.items))}
    for category, items in by_category.items():
        items.sort(key=lambda i: (i["page"] != category, page_rank[i["page"]]))
    pages = len({item["page"] for item in catalog.items})
    categories = [c for _, _, _, cs in GROUPS for c in cs if by_category.get(c)]

    sidebar, sections = [], []
    for key, name, blurb, members in GROUPS:
        present = [c for c in members if by_category.get(c)]
        if not present:
            continue
        total = sum(len(by_category[c]) for c in present)
        sidebar.append(
            f'<li class="nav-group"><a class="nav-group-link" href="#group-{key}">{esc(name)}'
            f'<span class="count" data-total="{total}">{total}</span></a><ul>'
            + "".join(f'<li><a class="nav-link" href="#cat-{c}" data-category="{c}">{esc(CATEGORY_NAMES[c])}'
                      f'<span class="count" data-total="{len(by_category[c])}">{len(by_category[c])}</span></a></li>'
                      for c in present)
            + "</ul></li>")
        blocks = []
        for c in present:
            readme = f"docs/{c}/README.md"
            more = (f'<a class="category-doc" href="{esc(doc_href(readme))}">カテゴリの解説</a>'
                    if (ROOT / readme).is_file() else "")
            blocks.append(
                f'<section class="category" id="cat-{c}" aria-labelledby="cat-{c}-title">'
                f'<div class="category-head"><h3 id="cat-{c}-title">{esc(CATEGORY_NAMES[c])}'
                f'<span class="count" data-total="{len(by_category[c])}">{len(by_category[c])}</span></h3>{more}</div>'
                f'<ul class="entries">{"".join(render_entry(i) for i in by_category[c])}</ul></section>')
        sections.append(
            f'<section class="group" id="group-{key}" aria-labelledby="group-{key}-title">'
            f'<header class="group-head"><h2 id="group-{key}-title">{esc(name)}</h2><p>{esc(blurb)}</p></header>'
            + "".join(blocks) + "</section>")

    letters = catalog.index_terms()
    jump = "".join(f'<a href="#az-{esc(letter)}">{esc(letter)}</a>' for letter in letters)
    az_blocks = []
    for letter, terms in letters.items():
        rows = []
        for name, items in terms:
            refs = ", ".join(f'<a href="{esc(href(i))}">{esc(i["title"])}</a>' for i in items)
            rows.append(f'<li><span class="az-term">{esc(name)}</span><span class="az-refs">{refs}</span></li>')
        az_blocks.append(f'<section class="az-letter" id="az-{esc(letter)}" aria-labelledby="az-{esc(letter)}-title">'
                         f'<h3 id="az-{esc(letter)}-title">{esc(letter)}</h3><ul>{"".join(rows)}</ul></section>')
    term_count = sum(len(t) for t in letters.values())

    return f"""<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="light dark">
<meta name="description" content="cp_library の操作できるアルゴリズム図解 {len(catalog.items)} 件の一覧と索引">
<title>図解一覧 | cp_library</title>
<link rel="stylesheet" href="assets/index.css">
<script defer src="assets/index.js"></script>
</head>
<!-- tools/build_visualization_index.py が entries.json から生成。直接編集しないこと。 -->
<body>
<a class="skip-link" href="#catalog">一覧へ移動</a>
<header class="masthead" id="top">
<div class="masthead-inner">
<nav class="topnav" aria-label="サイト内リンク">
<a class="brand" href="../html/README.html">cp_library</a>
<ul><li><a href="../html/docs/README.html">ドキュメント</a></li><li><a href="../html/FEATURES.html">機能一覧</a></li><li><a href="#az-index">A–Z 索引</a></li></ul>
</nav>
<p class="eyebrow">ALGORITHM NOTEBOOK</p>
<h1>アルゴリズムを、動かして理解する。</h1>
<p class="lede">入力や操作を変えて、内部状態を1ステップずつ観察できる図解の一覧です。分野から辿るか、名前・英名・略称で検索してください。</p>
<dl class="figures"><div><dt>図解</dt><dd>{len(catalog.items)}</dd></div><div><dt>分野</dt><dd>{len(categories)}</dd></div><div><dt>索引語</dt><dd>{term_count}</dd></div></dl>
<ol class="steps">
<li><strong>探す</strong>分野の目次・検索・A–Z 索引から図解を選ぶ</li>
<li><strong>動かす</strong>入力を変え、「次へ」「再生」で状態の変化を追う</li>
<li><strong>読む</strong>「解説」で API・計算量・使用例、「実装」でコードを見る</li>
</ol>
</div>
</header>
<div class="toolbar" role="search" hidden>
<div class="toolbar-inner">
<div class="search">
<label class="visually-hidden" for="search">図解を検索</label>
<svg class="search-icon" viewBox="0 0 20 20" aria-hidden="true"><circle cx="8.5" cy="8.5" r="5.5"/><path d="m13 13 4.5 4.5"/></svg>
<input id="search" type="search" placeholder="検索：セグ木、DSU、Dijkstra、回文…" autocomplete="off" spellcheck="false" aria-describedby="search-hint result-count">
<kbd id="search-hint" title="/ キーで検索欄へ移動">/</kbd>
</div>
<p id="result-count" class="result-count" aria-live="polite"></p>
<div class="view-toggle" role="group" aria-label="表示形式">
<button type="button" data-view="cards" aria-pressed="true">カード</button><button type="button" data-view="list" aria-pressed="false">リスト</button>
</div>
</div>
</div>
<div class="layout">
<details class="sidebar" open>
<summary>分野から探す</summary>
<nav aria-label="分野の目次">
<ul class="nav">{"".join(sidebar)}<li class="nav-group"><a class="nav-group-link" href="#az-index">A–Z 索引<span class="count">{term_count}</span></a></li></ul>
</nav>
</details>
<main id="catalog" class="catalog" data-view="cards" tabindex="-1">
{"".join(sections)}
<div id="empty" class="empty" hidden>
<p class="empty-title">一致する図解がありません。</p>
<p>英名・略称（例：<code>segtree</code>、<code>BIT</code>）や、日本語の一部（例：<code>最短路</code>）でも検索できます。<a href="../html/FEATURES.html">機能一覧</a>には図解のない機能も載っています。</p>
</div>
<section class="az" id="az-index" aria-labelledby="az-title">
<header class="group-head"><h2 id="az-title">A–Z 索引</h2><p>題名と機能名に含まれる英字の語から図解を引けます。日本語名は検索を使ってください。</p></header>
<nav class="az-jump" aria-label="頭文字へ移動">{jump}</nav>
{"".join(az_blocks)}
</section>
</main>
</div>
<footer class="site-footer">
<p>このページはブラウザで直接開けます（サーバー・外部通信は不要）。一覧は <code>visualizations/entries.json</code> から <code>make site</code> で生成しています。</p>
<p><a href="#top">ページの先頭へ ↑</a></p>
</footer>
</body>
</html>
"""


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true", help="index.html が最新かを検査するだけで書き込まない")
    args = parser.parse_args(argv)
    catalog = Catalog()
    if catalog.errors:
        for error in catalog.errors:
            print(f"error: {error}", file=sys.stderr)
        return 2
    output = render(catalog)
    current = INDEX.read_text(encoding="utf-8") if INDEX.is_file() else ""
    if args.check:
        if current != output:
            print("visualizations/index.html が entries.json と一致しません。make site で再生成してください。", file=sys.stderr)
            return 1
        print(f"visualizations/index.html is up to date ({len(catalog.items)} entries)")
        return 0
    if current != output:
        INDEX.write_text(output, encoding="utf-8")
    print(f"visualizations/index.html: {len(catalog.items)} entries")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
