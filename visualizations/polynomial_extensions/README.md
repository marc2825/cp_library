# 多項式・変換・母関数の図解（Q全31モード）

[開く](index.html) / [API・使用例・検証](../../docs/polynomial_extensions/README.md) / [図解一覧](../README.md)

入力JSONを変更して実行し、前後移動・先頭/最後・再生で係数表を追います。外部通信・CDNなしでローカルHTMLから動きます。整数modはBigIntを経由し、uint64の入力は10進文字列です。通常は係数32個、集合表64個以下に制限しています。

これは小規模の説明モデルです。FFT/NTTは直接DFT、mod2^64/GF畳み込みは定義の積、因数分解は小素数での試し割り、根は全走査、合成はHorner、Power Projectionは逐次冪、行列式は順列和、prefixとHolonomicは逐次評価、Reeds–Sloaneは最小長の全探索、集合演算は部分集合列挙を使います。C++のSIMD命令・乱択分割・高速block処理の実行トレースではありません。各ステップでその差を説明します。Holonomic図解は既知の式の生成を示し、推定APIはC++の比較テストで検証します。

31サンプルの出力を実C++使用例と共有し、独立のランダム比較も置いています。FFT表示は実部・虚部を10^6倍して丸めます。図の長い数字は省略し、下の表で全桁を表示します。

```sh
node verify/visualizations/polynomial_extensions_model_test.js
node verify/visualizations/polynomial_extensions_browser_test.mjs --browser /path/to/chromium
```
