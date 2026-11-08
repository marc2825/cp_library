# 開発・検証の4モード

[図解を開く](index.html) / [API・使用例](../../docs/development/README.md) / [共通図解一覧](../README.md)

JSON入力を変え、先頭・次へ・戻る・最後・再生で内部状態を追えます。

- CI: 入力した各テストの状態から結果と保存する成果物を表示します。実際のCI・ネットワーク通信は行いません。
- stress: 小さい出力をexact/tokensで比較し、最初の差分やtimeoutで保存・停止する流れを表示します。外部プロセスは起動しません。
- random: tree/forest/dag/polygonはPython共有生成器と同じSplitMix64・抽選順で生成します。nは24以下。多重グラフ等の全APIはPython側にあります。
- bundle: 全ヘッダがguard付きで、無条件にincludeされる依存グラフのモデルです。C++の構文解析・条件付きincludeはPython側の実テストで検証します。

検証: `node verify/visualizations/development_model_test.js`、`node verify/visualizations/development_browser_test.mjs --browser /path/to/chromium`。Pythonとの200生成比較、4モードの結果・入力編集・エラー・再生・390px表示を確認します。
