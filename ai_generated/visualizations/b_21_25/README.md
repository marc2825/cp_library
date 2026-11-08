# 3.21〜3.25 B項目の図解

[28モードを開く](index.html) / [API・使用例・検証](../../docs/b_21_25/README.md) / [出典と設計判断](../../docs/b_21_25/references.md) / [全図解](../index.html)。ローカルHTMLを直接開けます。

JSON入力の変更、最初・戻る・次へ・最後・再生、サンプル復元に対応します。係数やDPのセル図に加えて、幾何の点・多角形・円、交換グラフやゲームの遷移を操作できます。

## 範囲と簡略化

- 文字列は24byte以下（Suffix Treeは12、編集後の動的文字列は32）。JSONの各文字は0..255のbyteに対応させ、UTF-8の文字単位には変換しません。Runsは周期を直接検査、Suffix Treeは全suffixとLCPから圧縮構造を表示。Wildcardは不一致式の寄与、動的Hashは直接編集とBigIntの二重Hash、最小回転は全回転、複数比較は直接比較、回文分割は二乗DPです。C++のLCE/series links/Treapの高速性を図の実行時間から推測しないでください。
- 幾何は最大24点、包含円は8点、半平面は12本。Numberの小座標で計算し、C++の有理数・128bit・long doubleとその超過検査は再現しません。最小円は全1/2/3点候補、Minkowskiは全点対の凸包、最遠対は全点対、MSTは密Prim、Pickは座標-10..10の全格子点。半平面は入力矩形からの逐次切断です。図の直線は表示された有限区間で描きますが、計算では直線として扱います。
- Matroid Intersectionは交換グラフとBFS、Graphicは閉路、Partitionは容量を表示。要素は10〜12以下。MWISはマトロイド貪欲または一般グラフの全subset。線形パリティは8対・8次元以下、101以下の素数法で厳密な全subset rankを使い、C++代数版の確率的失敗は再現しません。
- NimberはBigIntによる64bitの二次拡大と逆元の二分累乗。グラフゲームは10局面まで、同期固定点更新でwin/lose/drawを表示。Partizanは12入力ノード、生成3000ノード・birthday20以下で和・負号・部分順序を追い、C++arenaの既定予算とは異なります。
- 探索は固定seedのSplitMix64、整数状態x、評価x²の教材。Multi-Startは5run・20反復以下、managed SAは30反復以下。温度はNumberで計算するのでlong doubleの採用判定を全入力で再現する保証はありません。較正は小さい悪化幅の平均採用率。Tuningはsetting²+seed%7、Banditは入力したarmごとの有限周期報酬を使用します。最適性やスコア改善を保証する図ではありません。

## 検証

```sh
node verify/visualizations/b_21_25_model_test.js
node verify/visualizations/b_21_25_browser_test.mjs --browser /path/to/chromium
```

全28モードの標準入力は[C++ bridge](../../verify/b_21_25/visualization_bridge.cpp)と相互比較します。小入力の独立オラクル、途中状態と拒否条件、ブラウザで全フレームのSVG描画範囲・入力編集・履歴・再生・390px表示を検査します。
