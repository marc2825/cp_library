---
title: 対話問題用ローカルジャッジ
documentation_of: //tools/cp_tools/interactive.py
---

# 対話問題用ローカルジャッジ（V-06）

[一覧](README.md) / [図解](../../visualizations/development_runtime/index.html#interactive) / [参考資料](references.md)

solver.stdoutをjudge.stdinへ、judge.stdoutをsolver.stdinへ転送します。stderrは通信に混ぜず保存します。改行を待たずbyteのまま流すため、binaryや部分書き込みにも対応します。

## 使用例

```sh
mkdir -p build/development_runtime
g++ -std=c++17 -O2 -I . -DCP_ENABLE_DEBUG examples/development_runtime/guess_solver.cpp -o build/development_runtime/guess_solver
python3 -m tools.cp_tools.interactive --config examples/development_runtime/interactive.json --seed 42
```

[solver](../../examples/development_runtime/guess_solver.cpp)が二分探索し、[judge](../../examples/development_runtime/guess_judge.py)がseedから決まる正解を判定します。`std::endl`などで実際にflushしてください。`CP_DEBUG`はstderrに出るため通信を壊しません。

## 設定・API

[JSON例](../../examples/development_runtime/interactive.json)のsolver/judgeはargv配列、cwdは設定ファイルからの相対パスです。両argvの `{seed}` を置換します。shellは使いません。timeoutは全体の秒数、idle_timeoutは通信またはstderrのbyte読み書きが無かった秒数、max_bytesは両プロセスのstdout/stderrの合計byte上限です。既定は10秒、2秒、1MiB。stderrだけが続く場合もwall timeoutで止まります。

`--output DIR` は新規保存先、未指定なら `build/interactive/run-...` を作ります。既存ディレクトリは上書きしません。Pythonでは `load_config(Path)` → `run_session(config, Path(directory), seed=0)` と呼びます。

保存物は正規化済みconfig.json、solver/judgeのstdout/stderr、result.json、transcript.jsonlです。transcriptは観測時刻の経過秒・送信元・stream種別・base64データを1読み取りごとに記録します。行やプロトコルのメッセージ単位ではありません。再実行は同じ設定とseed、新しい出力先で行います。

## 終了規約

正常終了は両プロセスが0で終了し、pipeを読み切った状態です。wall/idle timeout、output_limit、solver_exit、judge_exit、broken_pipe、起動/I/Oエラーを別のstatusとして保存します。終了コード0だけでは正解を意味しないので、judgeは不正解・不正プロトコル時に非0を返してください。CLIは成功0、セッション失敗1、設定エラー2です。

EOFは未送信分を転送してから相手のstdinを閉じて伝えます。受信側が先にstdinを閉じて転送できない場合はbroken_pipeです。失敗検出までの通信を残し、強制停止後に未読のpipe内容まで取得する保証はありません。出力量上限では上限までのprefixを保存します。

POSIXのnonblocking pipeとselectorsを使います。全体終了時に両process groupを終了し、直接の子をwaitします。子がsessionを離脱するケースは対象外です。WindowsのpipeはこのAPIの対象外です。

## 計算量・検証・用途

入力/出力総byte数Bとselector event数Eに対し、I/OはO(B+E)、保存量はO(B)（base64等の定数倍）、未送信bufferは最大O(max_bytes)です。CPU時間制限ではなくwall-clock制限です。

数字当て、対話的探索、外部ローカルjudgeの動作確認に使います。双方向40万byte、同時送信、部分write、EOF待ち、stderrの分離、起動失敗、終了コード、flushなし、timeout、上限を検証します。
