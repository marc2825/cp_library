# 計算幾何 Hの図解（22モード）

[開く](index.html) / [API・検証](../../docs/geometry_extensions/README.md) / [全図解](../README.md)

JSONの点・円・矩形・辺・操作を変更し、境界や交点、凸包・分割を前後移動・再生で確認します。外部通信・CDNは不要です。座標は絶対値32以下、通常の点は16個以下、Delaunay/Voronoiは10個、3Dは12個以下です。分数文字列も受け付けますが計算はJavaScript Numberです。

図解は小規模な説明モデルです。矩形は圧縮格子、格子点は直接列挙、動的凸包と三角形計数は素朴比較、Delaunayは空円候補の列挙、和集合は縦切片を積分します。C++のsweep・floor-sum・ブロック・KD-tree・Lawson flip・面列挙と実行手順が異なります。C++の任意精度有理値や適応精度predicateのbit単位の再現ではありません。

円の面積3モードは使用例に合わせて面積/πを出力します。凸多角形intersectionの図解ではBに面積を要求しますが、C++は点・線分も扱います。和集合の図解は穴なしringの配列です。C++のPolygonRegionは穴も扱います。3Dは斜投影で、外向き面と体積を表示します。

22使用例の実C++出力を参照値とし、独立モデル比較・Chromium全モードの入力編集・前後移動・再生・エラー復帰・390px表示を検証します。

```sh
node verify/visualizations/geometry_extensions_model_test.js
node verify/visualizations/geometry_extensions_browser_test.mjs --browser /path/to/chromium
```

今回のモデル検証は2,327入力・16,283状態・5,014比較が成功。22モードと22変更入力のブラウザ検証、390px表示が成功し、Delaunay・円弧・3Dとモバイルの画像を確認しました。
