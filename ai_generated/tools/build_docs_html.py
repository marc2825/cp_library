#!/usr/bin/env python3
"""Markdown をブラウザでそのまま読める HTML に変換し、html/ 以下へ鏡像として置く。

ai_generated/ 以下の *.md を html/<同じ相対パス>.html に変換します。file:// で開いたまま
行き来できるよう、.md へのリンクは対応する .html へ、実装・図解・examples などそれ以外の
相対リンクは元のファイルを指すよう書き換えます。ディレクトリへのリンクは、その中に
README.md があれば README.html へ向けます。

  python3 tools/build_docs_html.py          # 生成し、元の .md がなくなった HTML を削除
  python3 tools/build_docs_html.py --check  # 生成結果と html/ が一致しなければ終了コード 1

必要なもの：Python 3.10 以上と markdown-it-py（pip install markdown-it-py）。
"""
from __future__ import annotations

__all__ = ["main"]

import argparse
import html
import os
import posixpath
import re
import sys
from pathlib import Path
from urllib.parse import quote, unquote

try:
    from markdown_it import MarkdownIt
    from markdown_it.token import Token
except ImportError:  # pragma: no cover - 実行環境の案内
    sys.exit("markdown-it-py が必要です: python3 -m pip install markdown-it-py")

ROOT = Path(__file__).resolve().parents[1]
OUT = "html"
ASSET_SOURCES = Path(__file__).resolve().parent / "docs_html"
ASSETS = ("docs.css", "docs.js")
SKIP_DIRS = {".git", ".venv", "__pycache__", "node_modules", "build", OUT, ".stress-failures"}
SKIP_PATHS = {".competitive-verifier", "verification/evidence"}
SCHEME = re.compile(r"^[a-zA-Z][a-zA-Z0-9+.-]*:")
HTML_URL = re.compile(r"""\b(href|src)=(["'])(.*?)\2""")


def markdown_files() -> list[str]:
    result = []
    for directory, children, names in os.walk(ROOT):
        relative_directory = Path(directory).relative_to(ROOT).as_posix()
        children[:] = sorted(
            name for name in children
            if name not in SKIP_DIRS
            and posixpath.normpath(posixpath.join(relative_directory, name)) not in SKIP_PATHS
        )
        for name in names:
            if name.endswith(".md"):
                result.append(posixpath.normpath(posixpath.join(relative_directory, name)))
    return sorted(result)


def split_front_matter(text: str) -> tuple[dict[str, str], str]:
    if not text.startswith("---\n"):
        return {}, text
    end = text.find("\n---", 4)
    if end < 0:
        return {}, text
    meta = {}
    for line in text[4:end].splitlines():
        key, separator, value = line.partition(":")
        if separator and key.strip():
            meta[key.strip()] = value.strip().strip("\"'")
    body = text[end + 4:]
    return meta, body[1:] if body.startswith("\n") else body


def math_block(state, start_line, end_line, silent):
    """$$ だけの行で囲まれた数式を、TeX のまま表示するブロックにする。"""
    if state.sCount[start_line] - state.blkIndent >= 4:
        return False
    first = state.src[state.bMarks[start_line] + state.tShift[start_line]:state.eMarks[start_line]].strip()
    if not first.startswith("$$"):
        return False
    if len(first) > 4 and first.endswith("$$"):
        lines, next_line = [first[2:-2]], start_line + 1
    else:
        lines = [first[2:]] if first[2:] else []
        next_line = start_line + 1
        while next_line < end_line:
            line = state.src[state.bMarks[next_line] + state.tShift[next_line]:state.eMarks[next_line]].strip()
            if line.endswith("$$"):
                if line[:-2]:
                    lines.append(line[:-2])
                break
            lines.append(line)
            next_line += 1
        else:
            return False
        next_line += 1
    if silent:
        return True
    state.line = next_line
    token = state.push("math_block", "div", 0)
    token.block = True
    token.content = "\n".join(lines).strip()
    token.map = [start_line, next_line]
    return True


def render_math(self, tokens, index, options, env):
    return f'<div class="math" role="math">{html.escape(tokens[index].content)}</div>\n'


def make_parser() -> MarkdownIt:
    parser = MarkdownIt("commonmark", {"html": True}).enable(["table", "strikethrough"])
    parser.block.ruler.before("fence", "math_block", math_block,
                              {"alt": ["paragraph", "reference", "blockquote", "list"]})
    parser.add_render_rule("math_block", render_math)
    return parser


def github_slug(text: str) -> str:
    return re.sub(r"[^\w\- ]", "", text.strip().lower()).replace(" ", "-")


def plain_text(inline) -> str:
    return "".join(child.content for child in inline.children or [] if child.type in ("text", "code_inline"))


class Converter:
    def __init__(self):
        self.parser = make_parser()
        self.sources = markdown_files()
        self.source_set = set(self.sources)

    # ---- パス -------------------------------------------------------------
    def output_path(self, source: str) -> str:
        return posixpath.join(OUT, source[:-3] + ".html")

    def relative(self, target: str, page: str) -> str:
        return posixpath.relpath(target, posixpath.dirname(page) or ".")

    def rewrite_url(self, url: str, source: str) -> str:
        if not url or url.startswith(("#", "/")) or SCHEME.match(url):
            return url
        path, hash_mark, fragment = url.partition("#")
        path, question_mark, query = path.partition("?")
        if not path:
            return url
        target = posixpath.normpath(posixpath.join(posixpath.dirname(source), unquote(path)))
        directory_link = path.endswith("/")
        readme = "README.md" if target == "." else posixpath.join(target, "README.md")
        if target in self.source_set:
            mapped, directory_link = self.output_path(target), False
        elif (ROOT / target).is_dir() and readme in self.source_set:
            mapped, directory_link = self.output_path(readme), False
        else:
            mapped = target
        result = self.relative(mapped, self.output_path(source))
        if directory_link and not result.endswith("/"):
            result += "/"
        result = quote(result, safe="/:@!$&'()*+,;=-._~")
        return result + (f"?{query}" if question_mark else "") + (f"#{fragment}" if hash_mark else "")

    def rewrite_html(self, fragment: str, source: str) -> str:
        return HTML_URL.sub(lambda m: f"{m[1]}={m[2]}{self.rewrite_url(m[3], source)}{m[2]}", fragment)

    # ---- 変換 -------------------------------------------------------------
    def convert(self, source: str) -> str:
        meta, body = split_front_matter((ROOT / source).read_text(encoding="utf-8"))
        tokens = self.parser.parse(body)
        seen: dict[str, int] = {}
        headings, first_h1 = [], None
        for index, token in enumerate(tokens):
            if token.type == "heading_open":
                inline = tokens[index + 1]
                text = plain_text(inline)
                slug = github_slug(text) or "section"
                if slug in seen:
                    seen[slug] += 1
                    slug = f"{slug}-{seen[slug]}"
                else:
                    seen[slug] = 0
                token.attrSet("id", slug)
                if token.tag == "h1" and first_h1 is None:
                    first_h1 = text
                if token.tag in ("h2", "h3"):
                    headings.append((int(token.tag[1]), text, slug))
                    anchor = Token("html_inline", "", 0)
                    anchor.content = f' <a class="heading-anchor" href="#{html.escape(slug)}" aria-label="この見出しへのリンク">#</a>'
                    inline.children.append(anchor)
            self.rewrite_token(token, source)
            for child in token.children or []:
                self.rewrite_token(child, source)
        content = self.parser.renderer.render(tokens, self.parser.options, {})
        content = content.replace("<table>", '<div class="table-wrap"><table>').replace("</table>", "</table></div>")
        title = meta.get("title") or first_h1 or posixpath.basename(source)[:-3]
        return self.page(source, title, meta, headings, content)

    def rewrite_token(self, token, source: str) -> None:
        if token.type == "link_open":
            token.attrSet("href", self.rewrite_url(token.attrGet("href") or "", source))
        elif token.type == "image":
            token.attrSet("src", self.rewrite_url(token.attrGet("src") or "", source))
        elif token.type in ("html_inline", "html_block"):
            token.content = self.rewrite_html(token.content, source)

    def page(self, source, title, meta, headings, content) -> str:
        page = self.output_path(source)
        to = lambda target: html.escape(self.relative(target, page))
        crumbs = [f'<a href="{to(self.output_path("README.md"))}">cp_library</a>']
        parts = source.split("/")
        for depth in range(1, len(parts)):
            directory = "/".join(parts[:depth])
            readme = f"{directory}/README.md"
            if readme in self.source_set and readme != source:
                crumbs.append(f'<a href="{to(self.output_path(readme))}">{html.escape(parts[depth - 1])}</a>')
            else:
                crumbs.append(f"<span>{html.escape(parts[depth - 1])}</span>")
        crumbs.append(f'<span aria-current="page">{html.escape(parts[-1])}</span>')
        target_line = ""
        documented = meta.get("documentation_of", "").lstrip("/")
        if documented and (ROOT / documented).is_file():
            target_line = (f'<p class="doc-target">対象ファイル：<a href="{to(documented)}"><code>'
                           f"{html.escape(documented)}</code></a></p>")
        toc = ""
        if len(headings) >= 3:
            items = "".join(
                f'<li class="toc-h{level}"><a href="#{html.escape(slug)}">{html.escape(text)}</a></li>'
                for level, text, slug in headings
            )
            toc = (f'<nav class="toc" aria-label="このページの目次"><p class="toc-title">目次</p>'
                   f"<ol>{items}</ol></nav>")
        assets = self.relative(posixpath.join(OUT, "assets"), page)
        return f"""<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="light dark">
<title>{html.escape(title)} | cp_library</title>
<link rel="stylesheet" href="{assets}/docs.css">
<script defer src="{assets}/docs.js"></script>
</head>
<body>
<a class="skip-link" href="#content">本文へ移動</a>
<header class="site-header">
<div class="site-header-inner">
<nav class="breadcrumb" aria-label="現在の場所">{'<span class="sep" aria-hidden="true">/</span>'.join(crumbs)}</nav>
<nav class="site-links" aria-label="サイト内リンク"><a href="{to(self.output_path("docs/README.md"))}">ドキュメント</a><a href="{to("visualizations/index.html")}">図解一覧</a><a href="{to(self.output_path("FEATURES.md"))}">機能一覧</a></nav>
</div>
</header>
<div class="page{' has-toc' if toc else ''}">
{toc}<main id="content" class="markdown-body">
{target_line}{content}</main>
</div>
<footer class="site-footer"><p><a href="{to(source)}">{html.escape(source)}</a> から生成しました。内容の修正は Markdown 側で行い、<code>make docs-html</code> で再生成してください。</p></footer>
</body>
</html>
"""

    def outputs(self) -> dict[str, bytes]:
        result = {self.output_path(source): self.convert(source).encode("utf-8") for source in self.sources}
        for name in ASSETS:
            result[posixpath.join(OUT, "assets", name)] = (ASSET_SOURCES / name).read_bytes()
        return result


def existing_outputs() -> set[str]:
    base = ROOT / OUT
    if not base.is_dir():
        return set()
    return {path.relative_to(ROOT).as_posix() for path in base.rglob("*") if path.is_file()}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true", help="html/ が最新かを検査するだけで書き込まない")
    args = parser.parse_args(argv)
    expected = Converter().outputs()
    present = existing_outputs()
    stale = sorted(path for path, data in expected.items()
                   if not (ROOT / path).is_file() or (ROOT / path).read_bytes() != data)
    orphaned = sorted(present - expected.keys())
    if args.check:
        for path in stale:
            print(f"outdated: {path}")
        for path in orphaned:
            print(f"orphaned: {path}")
        if stale or orphaned:
            print("html/ が Markdown と一致しません。make docs-html で再生成してください。", file=sys.stderr)
            return 1
        print(f"html/ is up to date ({len(expected) - len(ASSETS)} pages)")
        return 0
    for path in stale:
        target = ROOT / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(expected[path])
    for path in orphaned:
        (ROOT / path).unlink()
    for directory in sorted((ROOT / OUT).rglob("*"), reverse=True):
        if directory.is_dir() and not any(directory.iterdir()):
            directory.rmdir()
    print(f"html/: {len(expected) - len(ASSETS)} pages, {len(stale)} written, {len(orphaned)} removed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
