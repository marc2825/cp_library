---
title: competitive-verifier構成とCI
documentation_of: //tools/verify_ci.py
---

# competitive-verifier構成とCI（V-01）

[一覧](README.md) / [操作できる図解](../../visualizations/development/index.html#ci) / [参考資料](references.md)

既存の `PROBLEM` ラッパを使い、依存解決・検証・Jekyll向け文書生成をつなぎます。competitive-verifier 4.1.2と推移依存の版を[requirements.txt](../../.competitive-verifier/requirements.txt)に固定しています。CIはUbuntu 24.04、Python 3.11、g++のC++17、Boostヘッダを使います。g++/BoostはUbuntuの配布版なのでpatch版までの固定ではありません。

## 使用例

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r .competitive-verifier/requirements.txt
.venv/bin/python tools/verify_ci.py --offline
.venv/bin/python tools/verify_ci.py --resolve-only
.venv/bin/python tools/verify_ci.py
```

最後のコマンドはネットワークから問題のテストケースを取得してローカル実行します。ジャッジへの提出は行いません。`--offline` はstandaloneだけ、`--resolve-only` は問題設定・依存解決だけです。出力は既定で `build/verifier/`。`--output PATH` で変えられます。上流resolverはGit追跡ファイルだけを読むため、通常実行前に対象をコミットしてください。対象や依存が抜けた場合は空の成功結果を作らずエラーにします。未コミット時の検証は一覧にある `--with-verifier` を使えます。

## CIの範囲

[workflow](../../.github/workflows/verify.yml)はpush・pull request・手動実行で動きます。

- local job: 開発ツールのテスト、図解モデル、既存graph/treeの回帰検証。
- online job: [targets.json](../../.competitive-verifier/targets.json)のUnion-Find、Point Add Range Sum、Suffix Arrayと、FastIO/BigInt/RollbackArrayのstandalone smoke。
- `build/verifier/resolved.json`、`result.json`、`docs/`をartifactとして保存。検証失敗も終了コードへ反映し、古い成功結果は再利用しません。

これは全ラッパの網羅登録ではありません。対象追加は `targets.json` と対応するローカル比較を更新します。既存のAtCoder/CSES等への問題リンクを、そのまま自動取得可能なケースと扱いません。全体の機能・問題対応台帳は別項目V-09です。

## 文書

`.competitive-verifier/docs/`の設定と既存の `documentation_of` を利用します。生成先の `docs/` はJekyllソースで、HTML配信先ではありません。artifactを取得し、Ruby/Bundler環境で生成先のGemfileに従って `bundle install`、`bundle exec jekyll serve` を実行できます。GitHub Pagesの有効化やデプロイはこのworkflowに含めていません。

依存解決は対象ソースごとのコンパイラ処理、検証時間は各プログラムとテストケース数に依存します。全体timeoutは600秒、各問題の既定TLEは10秒です。standaloneコマンドは上流ツールの直接実行なので、最終的な停止上限はjobの20分です。

## 検証・問題例

一時Git snapshotで実際の固定版CLIを使い、standalone成功、文書ファイル生成、3問題のPROBLEM認識と依存解決を確認します。オンライン全ケース実行やGitHub Actions上での成功は未確認です。

問題: [Union-Find](https://judge.yosupo.jp/problem/unionfind)、[Point Add Range Sum](https://judge.yosupo.jp/problem/point_add_range_sum)、[Suffix Array](https://judge.yosupo.jp/problem/suffixarray)。
