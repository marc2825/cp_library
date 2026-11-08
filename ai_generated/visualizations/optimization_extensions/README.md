# O/Bの操作できる図解

[開く](index.html) / [API・検証](../../docs/optimization_extensions/README.md) / [図解一覧](../index.html)

O全8項目・B-20を9モードで追います。JSONを編集し、予算や制約、重みを変えて再実行できます。前後移動・先頭／最後・自動再生で、pivot表、分配の交換路、要素数別の重み、貪欲基、cutting-planeの上下界、整数分枝、Q(ω)の約分を表示します。

図解のLP・低次元LP・分枝限定はJavaScript Numberと小さいepsilonを用いる説明モデルです。C++のBigRationalによる証明を再現するものではありません。通常の整数係数・有理入力を小規模で比較する教材として使います。分数計画もNumberです。Q(ω)の多項式だけは既存の[BigInt有理数モデル](../computational_geometry/model.js)を共有します。

LPは変数6・制約16、低次元LPは2Dのみ、マトロイド・集合関数は8要素まで。LP/整数計画/集合関数の一般係数は絶対値1000以下、反復・候補・ノード予算は0〜256です。図の多角形は表示窓で切ったもので、そこから非有界性を判定しません。C++の低次元LPは1〜3Dを扱います。

マトロイドの図解oracleはグループ容量、基多面体は被覆関数、劣モジュラ最小化は無向カット＋モジュラ項に限定しています。C++は独立性・集合関数の一般oracleを受けます。整数計画は2変数の図表示を中心とし、有限上下限のLP緩和と分枝を追います。Q(ω)は整数係数・次数6以下を入力し、約分結果とω+1との比較、1/ωと0の比較、1+1/ωの標準部分を表示します。

[サンプル参照値](../../verify/optimization_extensions/visualization_expected.json)をC++の実行例と共用し、モデルとChromiumで照合します。全モードの入力変更・移動・再生・エラー復帰・リンク・390px表示も検証します。

```sh
node verify/visualizations/optimization_extensions_model_test.js
node verify/visualizations/optimization_extensions_browser_test.mjs --browser /path/to/chromium --screenshot /tmp/optimization.png
```
