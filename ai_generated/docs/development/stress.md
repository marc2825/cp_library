---
title: 汎用stress tester
documentation_of: //tools/cp_tools/stress.py
---

# 汎用stress tester（V-02）

[一覧](README.md) / [図解](../../visualizations/development/index.html#stress) / [参考資料](references.md)

生成器・高速解・愚直解を外部プロセスとして実行します。最初の失敗で停止し、再現に必要な入力・出力・seed・コマンドを保存します。[動く例](../../examples/development/stress.json)はグラフの成分数をC++ DSUとPython探索で比較します。

## 使用例

```sh
mkdir -p build/development
g++ -std=c++17 -O2 -I . examples/development/fast.cpp -o build/development/fast
python3 -m tools.cp_tools.stress --config examples/development/stress.json --seed 100 --cases 80
```

失敗時の表示パスを使い `python3 -m tools.cp_tools.stress --replay .stress-failures/seed-123-実際のsuffix` で再実行します。`--failures DIR` は保存先を変更します。成功は終了コード0、差分・プロセス失敗は1、設定エラーは2です。

## 設定・API

JSONの `generator` / `fast` / `naive` は文字列のargv配列です。shellは介しません。生成器の引数中の `{seed}` だけを整数に置換します。`cwd` は設定ファイルからの相対パス、`timeout` は各プロセスの正の秒数（既定2）、`compare` は `tokens`（既定）または `exact`。tokensはASCII空白で分割したbyte列の完全一致で、数値誤差比較は行いません。

Python APIは `load_config(path)`、`execute(argv, data, cwd, timeout)`、`run_case(config, seed, saved_input=None)`、`compare(a,b,mode)`、`save_failure(...)`。`Result`はargv、終了コード（起動失敗時None）、timeout、経過時間、bytesのstdout/stderrを保持します。

保存ディレクトリはseedが同じでも一意です。`input.txt`、各プロセスの `.stdout` / `.stderr`、`case.json`（format version 1）を保存します。replayは保存入力で両解を再実行し、生成器が失敗したケースだけは生成器からやり直します。replayは元の保存内容を書き換えず、修正後に一致すれば0を返します。環境変数・実行ファイル本体は保存しないため、実行環境とcwd内のプログラムは利用者が保持してください。

## 条件・計算量・検証

POSIXでは各プログラムを独立したprocess groupで起動し、timeout時と親プロセス終了後に同じgroupの子も終了させます。自らsessionを離脱するプログラムは対象外です。Windowsでは直接の子だけを終了させます。stdin/stdout/stderrは一時ファイルを使い、終了後に出力をメモリへ読みます。出力量の上限は設けないため、出力サイズに応じてディスクとメモリを使います。

時間はケースごとの3プログラムの時間と入出力量の和、保存量は失敗ケースの入力・全出力に比例します。テストは一致/不一致、空白差、起動失敗、各プロセスの異常終了、部分出力とtimeout、子プロセス終了、shell文字の引数保持、保存・修正後replayを含みます。問題例はUnion-Findや任意の小さい全探索解を作れる問題です。
