"""Aggregate header/test/problem mappings and current verification evidence."""
import argparse
from collections import Counter
import hashlib
import html
import json
from pathlib import Path
import re

from .bundle import directives
from .provenance import inventory
from .repository import (ROOT, closure, digest, files, include_graph, local_path,
                         read_json, runners, source_snapshot, write_json)


def snapshot_id(root):
    sources = source_snapshot(root)
    data = json.dumps(sources, sort_keys=True, separators=(",", ":")).encode()
    return {"sha256": hashlib.sha256(data).hexdigest(), "files": len(sources)}


def problem_url(root, name):
    text = (root / name).read_text(encoding="utf-8")
    for match in directives(text):
        if match[1] == "define":
            value = re.fullmatch(r'\s*PROBLEM\s+"(https?://[^"\s]+)"\s*', match[2])
            if value:
                return value[1]
    return None


def collect(root=ROOT, evidence=None):
    root = root.resolve()
    evidence = evidence if evidence is not None else root / "verification/evidence"
    graph = include_graph(root)
    runner_paths = runners(root)
    provenance = inventory(root)
    provenance_by_path = {r["path"]: r for r in provenance["files"]}
    headers = files(root, {".hpp", ".h"})
    mapping = {h: {"tests": set(), "runners": set(), "titles": set(), "manifests": set()} for h in headers}
    tests = {}
    for name in sorted(graph):
        if name.startswith("verify/") and name.endswith(".cpp"):
            parts = Path(name).parts
            runner = parts[1] if len(parts) > 2 and parts[1] in runner_paths else None
            if name == "verify/graph_tree_test.cpp":
                runner = "graph_tree"
            dependencies = [p for p in closure(graph, name) if p in mapping]
            tests[name] = {"problem": problem_url(root, name), "headers": dependencies, "runner": runner}
            for header in dependencies:
                mapping[header]["tests"].add(name)
                if runner:
                    mapping[header]["runners"].add(runner)
    for manifest in sorted((root / "verify").glob("*/manifest.json")):
        runner = manifest.parent.name
        for entry in read_json(manifest).get("algorithms", []):
            header = entry["header"]
            local_path(root, header)
            if header not in mapping:
                raise ValueError(f"manifest target is not an inventoried header: {header}")
            mapping[header]["titles"].add(entry.get("title", Path(header).stem))
            mapping[header]["manifests"].add(manifest.relative_to(root).as_posix())
            if runner in runner_paths:
                mapping[header]["runners"].add(runner)
    snapshot = snapshot_id(root)
    attempts, passed = [], set()
    for path in sorted(evidence.glob("*/*.json")):
        record = read_json(path)
        if record.get("schema_version") != 1 or record.get("runner") not in runner_paths:
            raise ValueError(f"invalid local receipt: {path}")
        if record.get("profile") not in {"standard", "stdlib-debug", "sanitize", "stdlib-debug-sanitize"}:
            raise ValueError(f"invalid receipt profile: {path}")
        if record.get("status") not in {"passed", "failed", "interrupted"}:
            raise ValueError(f"invalid receipt status: {path}")
        current = record.get("snapshot") == snapshot
        attempts.append({**record, "current": current})
        if current and record["status"] == "passed":
            passed.add(record["runner"])
    # Actual accepted submissions require an explicit reviewed receipt. A PROBLEM
    # annotation, previous author's submission, or local judge run never suffices.
    accepted, online = [], set()
    online_file = root / "verification/accepted.json"
    if online_file.exists():
        data = read_json(online_file)
        if data.get("schema_version") != 1:
            raise ValueError("invalid accepted-submission schema")
        for record in data["submissions"]:
            test = record["test"]
            if test not in tests or not tests[test]["problem"] or record["problem"] != tests[test]["problem"]:
                raise ValueError(f"accepted submission does not match PROBLEM: {test}")
            if record.get("verdict") != "AC" or not record.get("reviewed_at") or not re.fullmatch(r"https://[^\s]+", record.get("submission_url", "")):
                raise ValueError(f"incomplete accepted submission evidence: {test}")
            required = closure(graph, test)
            expected = {p: digest(root / p) for p in required}
            current = record.get("sources") == expected
            accepted.append({**record, "current": current})
            if current:
                online.update(tests[test]["headers"])
    rows = []
    for header in headers:
        item = mapping[header]
        role = "aggregate" if Path(header).name == "all.hpp" else "internal" if "detail" in Path(header).parts else "header"
        status = "online-ac" if header in online else "local-only" if item["runners"] & passed else "unverified"
        doc = "docs/" + str(Path(header).with_suffix(".md"))
        rows.append({"path": header, "role": role, "status": status,
                     "titles": sorted(item["titles"]), "doc": doc if (root / doc).exists() else None,
                     "tests": sorted(item["tests"]), "runners": sorted(item["runners"]),
                     "manifests": sorted(item["manifests"]),
                     "problems": sorted({tests[t]["problem"] for t in item["tests"] if tests[t]["problem"]}),
                     "provenance": provenance_by_path[header]})
    return {"schema_version": 1, "snapshot": snapshot, "summary": dict(sorted(Counter(r["status"] for r in rows).items())),
            "features": rows, "tests": tests, "runners": runner_paths, "attempts": attempts,
            "accepted_submissions": accepted, "provenance_entries": provenance["entries"]}


def render_html(report):
    escape = html.escape
    labels = {"unverified": "未verify", "local-only": "ローカル検証済み", "online-ac": "提出AC"}
    rows = []
    for row in report["features"]:
        test_links = ' '.join(f'<a href="{escape(url, quote=True)}">問題{i+1}</a>' for i, url in enumerate(row["problems"]))
        detail = escape(', '.join(row["runners"]) or "対応runnerなし")
        provenance = row["provenance"]
        rows.append(f'<tr data-status="{row["status"]}"><td>{escape(row["path"])}<small>{escape(" / ".join(row["titles"]))}</small></td>'
                    f'<td>{labels[row["status"]]}<small>{escape(row["role"])}</small></td>'
                    f'<td>{len(row["tests"])} tests<small>{detail}</small>{test_links}</td>'
                    f'<td>{escape(provenance["record"] or "個別確認待ち")}<small>{escape(provenance["license"])}</small></td></tr>')
    summary = ' / '.join(f'{labels[k]} {report["summary"].get(k, 0)}' for k in labels)
    return '''<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>出典・検証台帳</title><style>body{font:16px/1.7 system-ui;margin:2rem auto;padding:0 1rem;max-width:1200px;color:#183044;background:#f5f8fb}h1{font-size:1.7rem}input,select{font:inherit;padding:.5rem;max-width:100%;box-sizing:border-box}table{border-collapse:collapse;width:100%;background:white}th,td{padding:.7rem;text-align:left;border-bottom:1px solid #ccd6df;overflow-wrap:anywhere}th:nth-child(2){min-width:8rem}small{display:block;color:#526779}a{margin-right:.4rem}.scroll{overflow-x:auto}label{display:inline-block;margin:0 .7rem .8rem 0}@media(max-width:600px){body{margin:1rem auto}thead{display:none}table,tbody,tr,td{display:block}tr[hidden]{display:none}tr{margin-bottom:1rem;border:1px solid #ccd6df}td{padding:.6rem}td:before{display:block;font-weight:600;color:#526779}td:nth-child(1):before{content:'機能 / ヘッダ'}td:nth-child(2):before{content:'検証状態'}td:nth-child(3):before{content:'検証との対応'}td:nth-child(4):before{content:'出典 / ライセンス'}}</style>
<h1>出典・検証台帳</h1><p>''' + summary + '''</p><p>現在のソースに一致する実行記録だけを集計しています。include到達関係とmanifestによる対応付けであり、全APIのテスト網羅率ではありません。問題リンクだけでは提出ACになりません。</p>
<label>検索 <input id="query" type="search" placeholder="ヘッダ・runner・出典"></label><label>検証状態 <select id="status"><option value="">すべて</option><option value="unverified">未verify</option><option value="local-only">ローカル検証済み</option><option value="online-ac">提出AC</option></select></label><p id="count" aria-live="polite"></p>
<div class="scroll"><table><thead><tr><th>機能 / ヘッダ</th><th>状態</th><th>検証との対応</th><th>出典 / ライセンス</th></tr></thead><tbody>''' + ''.join(rows) + '''</tbody></table></div>
<script>const q=document.querySelector('#query'),s=document.querySelector('#status'),rows=[...document.querySelectorAll('tbody tr')];function filter(){let n=0;for(const r of rows){r.hidden=!(r.textContent.toLowerCase().includes(q.value.toLowerCase())&&(!s.value||r.dataset.status===s.value));if(!r.hidden)n++;}document.querySelector('#count').textContent=n+' 件表示';}q.addEventListener('input',filter);s.addEventListener('change',filter);filter();</script></html>'''


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("build/catalog"))
    parser.add_argument("--evidence", type=Path)
    args = parser.parse_args(argv)
    report = collect(evidence=args.evidence)
    write_json(args.output / "catalog.json", report)
    (args.output / "index.html").write_text(render_html(report), encoding="utf-8")
    print(f"catalog: {len(report['features'])} headers, {len(report['tests'])} tests, {len(report['runners'])} runners; {report['summary']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
