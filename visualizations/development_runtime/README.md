# 実行・観察ツールの3モード

[図解を開く](index.html) / [API・使用例](../../docs/development_runtime/README.md) / [共通図解一覧](../README.md)

入力JSONを編集し、次へ・戻る・先頭・最後・再生で手順を追えます。

- debug: 整数・bool・ASCII文字列・tuple・listの整形、深さ/個数の打ち切り、提出時の引数評価抑制。formatterや任意byte文字列などC++の全対応型を模倣するものではありません。
- interactive: 指定時刻の通信と終了を追います。実プロセスは起動せず、nonblocking pipeの部分writeやOSの実行順はPythonの実テストで検証します。eventsが途中で終わるとwaitingです。
- benchmark: 各seedの成功/失敗と設定間比較。入力scoreは±10億で、Python側の巨大floatに対する有理数集計は実テストで確認します。

検証は `node verify/visualizations/development_runtime_model_test.js` と `node verify/visualizations/development_runtime_browser_test.mjs --browser /path/to/chromium`。C++整形参照とPython集計参照、各モードの境界・エラー・入力編集、再生、390px表示を確認します。
