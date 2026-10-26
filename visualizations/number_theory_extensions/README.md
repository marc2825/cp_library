# 数論の19モード

[図解を開く](index.html) / [API・検証](../../docs/number_theory_extensions/README.md) / [全図解](../README.md)

JSONを編集し、実行・先頭/前/次/最後・再生を操作できます。区間の残り因子、剰余の点、累積積、BSGSの共通表、商prefix、Gaussianの剰余、連分数の収束値を表示します。計算結果は19個のC++使用例と照合します。390px幅で操作できるレイアウトです。

図解は小入力の説明モデルです。Floor Sumとmod最小値/総和は点ごとに表示し、C++はEuclidを使います。巨大階乗はブロック積を直接計算して表示し、C++は多点評価を使います。mod根、二平方数和、有理数の分母制限近似は独立した全探索を含みます。素数和とMin_25の表更新、固定BSGSの再利用、GaussianとPellの状態遷移は対応する手順を表示します。Min_25の素数初期表は図解では小さい素数を直接列挙します。

入力上限は各モードのエラーに表示します。剰余の法は正、根・階乗は素数、PellのDは非平方数です。画面のplot座標はNumberによる表示、出力の大きな整数はBigIntの文字列です。

```sh
node verify/visualizations/number_theory_extensions_model_test.js
node verify/visualizations/number_theory_extensions_browser_test.mjs --browser /path/to/chromium
```

モデルは5,386件の独立比較、ブラウザは19モード・8編集入力・再生・不正入力・リンク・390px表示を検査します。実行結果は[検証記録](../../docs/number_theory_extensions/README.md#検証)に記載します。
