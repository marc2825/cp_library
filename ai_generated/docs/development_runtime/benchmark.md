---
title: AHC複数seed並列実行と設定比較
documentation_of: //tools/cp_tools/benchmark.py
---

# AHC複数seed並列実行と設定比較（V-07）

[一覧](README.md) / [図解](../../visualizations/development_runtime/index.html#benchmark) / [参考資料](references.md)

外部生成器・solver・scorerを使い、全設定に同じseedと入力を渡して比較します。既存のC++ [tune_parameters](../../heuristic_search/parameter_tuning.hpp)は同一プロセスのcallback向けで、このツールは外部実行・並列化・保存を担当します。

## 使用例

```sh
mkdir -p build/development_runtime
g++ -std=c++17 -O2 -I . examples/development_runtime/select_solver.cpp -o build/development_runtime/select_solver
python3 -m tools.cp_tools.benchmark --config examples/development_runtime/benchmark.json --seeds 0 1 2 3 4 --jobs 4
```

[設定例](../../examples/development_runtime/benchmark.json)は先頭を選ぶ設定と最大値を選ぶ設定を比べます。[生成器](../../examples/development_runtime/generate.py)、[solver](../../examples/development_runtime/select_solver.cpp)、[scorer](../../examples/development_runtime/score.py)をそのまま動かせます。実コンテストでは公式tester/visualizer等の入出力形式へ合わせてargvかadapterを用意してください。

## 設定

- `generator`: `{seed}`を受け取り、stdoutへ入力を出すargv配列。seedごとに1回実行します。
- `configurations`: 一意なname（英数字/_/-）とsolver argvの配列。solverのstdinへ保存入力を渡し、stdoutを解答として保存します。
- `scorer`: `{input}`と`{output}`の絶対ファイルパスを受け取るargv。stdinは空です。solverの正常終了後だけ実行します。
- solver/scorerのargvには `{seed}`、`{input}`、`{output}`、`{workdir}` を使えます。workdirはそのケースの保存ディレクトリです。コマンドのcwdは全て設定のcwd（設定ファイル基準）なので、一時ファイルが必要ならworkdir配下を使ってください。
- `score_stream`: stdoutまたはstderr、`score_regex`: capture groupが1つの正規表現。出力全体で正確に1一致が必要です。欠落・重複・非有限値は失敗。scoreはPython floatです。
- `goal`: maximize（既定）/minimize、`timeout`: 各プロセスのwall-clock秒数（既定2）。

shellは使いません。環境変数は親から継承します。`--jobs` が生成器・solver/scorerの同時実行数上限です。生成フェーズの完了後に各設定×seedを実行し、結果は設定順・指定seed順に保持します。workerはthreadですが計算本体は外部プロセスです。

## 保存・再現・集計

未指定なら `build/benchmark/run-...`、`--output DIR` ならその新規ディレクトリへ保存します。run.jsonは設定とseed列、summary.jsonは各設定の成功/失敗数・平均・中央値・最小/最大・比較対象可否・最良名です。caseごとにinput.txt、各段階のstdout/stderr、case.json（実行argv・終了コード・timeout・時間・score）を保存します。

平均・中央値等は成功ケースだけから計算しますが、**最良設定の候補は指定した全seedが成功した設定だけ**です。失敗を0点に置換しません。同点は設定記載順です。途中のfloat合計のoverflowを避けるため、平均と中央値の計算には正確な有理数を使って最後にfloatへ戻します。公式の相対スコア・順位計算とは別の、各ケースの生スコアの集計です。

`python3 -m tools.cp_tools.benchmark --replay 保存先/cases/設定名/seed --output 新しい保存先` で保存入力からsolver/scorerを再実行します。元の生成器が失敗していたケースだけは生成器も再実行します。元の保存先は変更しません。実行ファイル・依存ファイル・環境変数のsnapshotは取らないため、その環境を保持してください。

CLIは全ケース成功0、失敗ケースあり1、設定等のエラー2。Ctrl+Cや内部例外時は待機jobを取消し、実行中のプロセスにもキャンセルを伝えます。完了済みcaseの保存は残りますが、その場合のsummary完成は保証しません。POSIXはprocess group、Windowsは直接の子だけを終了させます。

## 計算量・検証

seed数S、設定数Cで生成S回・solver最大CS回・scorer最大CS回。時間はそれらの実行時間をjobsで分担したもの、集計はO(CS log S)。入力S個と実行中jobの出力をメモリに持ち、保存は全ケースの入力と出力に比例します。出力量制限は持たず、一時ファイルから終了後に出力を読みます。

直列/並列一致、2プロセスのbarrierによる並行動作、入力一致、正常/異常終了、timeout、score構文と巨大/極小float、失敗除外、同点、replay、キャンセルを検証します。実C++の2設定×12 seedを4並列で走らせ、生成入力から得た独立スコアとも一致させます。
