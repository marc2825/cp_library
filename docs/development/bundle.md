---
title: C++提出用バンドラ
documentation_of: //tools/cp_tools/bundle.py
---

# C++提出用バンドラ（V-04）

[一覧](README.md) / [図解](../../visualizations/development/index.html#bundle) / [参考資料](references.md)

[競プロ向けの手順とcplib](../usage.md)も参照してください。`python3 /path/to/cp_library/tools/cplib.py bundle main.cpp -o submission.cpp --check` なら、どの作業フォルダからでもリポジトリのincludeパスを自動追加して同じバンドラを呼べます。

## 使用例

```sh
python3 -m tools.cp_tools.bundle examples/development/fast.cpp -o /tmp/submission.cpp --check
g++ -std=c++17 -O2 /tmp/submission.cpp -o /tmp/submission
```

`#include "..."` の依存を元の位置に展開し、ローカルincludeパスなしで提出できるC++ソースを作ります。システムの `<iostream>` や `<boost/...>` は残します。BigInt等を使う提出ではジャッジにもBoostヘッダが必要です。

## CLI・API

`-I DIR` は追加の検索パス（複数可）、`--compiler` はコンパイラ、`--flag=-DMACRO=1` 等は検査時の追加フラグ、`--dependencies FILE` は展開した絶対パス一覧JSONです。quoted includeは現在のヘッダの親、cwd、追加-I順に探索します。angle includeはcwdと-Iにファイルがあれば展開し、それ以外を残します。

`--check` は一時ディレクトリの展開済みソースをC++17としてリンクまで行い、成功してから出力します。元入力・依存ヘッダへの上書きは禁止します。コンパイル失敗時は既存の出力を保持します。Pythonからは `bundle(Path(source), include_dirs)` で文字列を返します。エラーは `BundleError`、CLIでは終了コード2です。

## 対応構文と重複

- 通常の行頭のリテラルinclude、相対パス、行継続、コメントや文字列内の偽include、raw stringを扱います。
- `#pragma once` はファイル単位の生成guardに置き換えます。先頭の `#ifndef` / `#define` による通常の全体guardも扱います。
- 無条件に展開済みのguard付きヘッダは再展開しません。条件付きincludeの初回が `#if 0` でも後の必要な定義が消えないよう、条件付き経路はguard付き本文を再配置します。その場合は生成テキストに本文が複数回現れますが、コンパイル時の定義は一度です。
- guard付きの循環依存は停止し、guardなしの循環はエラー。guardなしのX-macro用ヘッダはincludeごとに保持します。

完全なプリプロセッサではありません。マクロによるinclude名、include_nextはエラーです。include引数内の特殊なコメント配置、条件付きpragma once、guardを後からundefする用法は対象外です。条件は評価せず全分岐のローカルヘッダを解決するため、使わない分岐でもquoted includeの実ファイルが必要です。`__FILE__` / `__LINE__` は展開後ファイルの位置となります。必ず提出で使うマクロ条件でも `--check` してください。

## 計算量・検証・問題例

時間・メモリは読んだ依存ソースと展開結果のサイズに比例します。条件付き依存やguardなしの反復が多い場合は、入力の総サイズより出力が大きくなります。コンパイル検査はコンパイラの時間・メモリが別途必要です。

diamond依存、guard付き循環、無効な先行分岐、raw string、桁区切り、行継続、X-macro、依存欠落、出力上書き拒否を検査します。Union-Find・Point Add Range Sum・Suffix Array・BigRational・成分数の5プログラムで、元/展開後をDebug/Releaseで実行比較し、サニタイザにも対応します。
