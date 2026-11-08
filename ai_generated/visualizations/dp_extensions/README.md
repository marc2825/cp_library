# DP・探索の追加14モード

[図解を開く](index.html) / [API・検証](../../docs/dp_extensions/README.md) / [図解一覧](../README.md)

JSON入力を編集し、実行・前後移動・再生で状態を追えます。全てローカルで動作し、外部通信はありません。

LCS/MyersはJavaScript BigIntで差分bit列を表示します。C++の64bit block間伝播の代わりに一つの多倍長bit列を使い、同じ距離を計算します。Frontierは正規化した境界状態、A*/IDA*/双方向BFSは展開状態、Exact Cover/SATは小規模の探索・伝播を示します。Exact Coverのノードリンク、SATの監視位置自体は再現しません。

Knapsackは段階別表と復元、最適解数は添字で区別した数、Monge積と少種類重量版は候補全走査でargmin/argmaxの意味を示します。C++のSMAWK/分割統治による評価順序とは異なります。重み付きSlope Trickは圧縮したbreakpoint、動的DPは遷移行列と状態別費用を表示します。

図解の入力上限は小規模整数・短いバイト列用で、C++の制限とは異なります。各モードのサンプル出力を14個のC++使用例と照合し、独立したモデルテストと実ブラウザ操作を検証します。

```sh
node verify/visualizations/dp_extensions_model_test.js
node verify/visualizations/dp_extensions_browser_test.mjs --browser /path/to/chromium
```
