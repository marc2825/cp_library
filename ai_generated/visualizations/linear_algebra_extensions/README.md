# 線形代数Lの図解（全11項目・12モード）

[開く](index.html) / [APIと検証](../../docs/linear_algebra_extensions/README.md) / [図解一覧](../README.md)

JSONを編集し、前後移動・再生で行列や係数表の変化を追います。整数はBigInt、有限体もBigIntを通して正確に計算します。長い数字は図中で省略し、表と結果欄には全桁を表示します。外部通信・CDNなしでローカルHTMLから動きます。

通常の行列は6×6以下、標準形と特性多項式は5×5以下、マッチングは8×8以下です。拡大体図解は素数17以下・次数4以下、GF(2)は256bit未満。半環は5次元・指数32以下。C++の上限とは別です。

合成数法のEuclid消去・整数のBareiss消去・Smith/Hermiteの基本操作・半環の二分累乗を表示します。Permanentは集合順、Pfaffian/Hafnianは完全マッチングの部分集合再帰、特性多項式は順列展開による説明モデルです。C++のGray code・Pfaffian消去・Krylov/Strassen高速経路の実行トレースではありません。Frobenius図解は多項式Smith形とcompanion blockを表示し、C++が返す相似基底の生成はC++比較テストで検証します。

全12モードのサンプル出力を実C++使用例と共有しています。独立オラクルによるランダムモデル検証と、ブラウザの入力編集・ナビゲーション・再生・エラー復帰・390px表示を検証します。

```sh
node verify/visualizations/linear_algebra_extensions_model_test.js
node verify/visualizations/linear_algebra_extensions_browser_test.mjs --browser /path/to/chromium
```
