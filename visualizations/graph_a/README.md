# 3.7〜3.11 A項目の操作できる解説

[index.html](index.html) を直接開き、JSON入力を変更して「入力を反映」します。14モードの履歴を前後移動・再生できます。[実装と検証](../../docs/graph_a/README.md) / [一次資料](../../docs/graph_a/references.md)。

頂点は原則10個、辺24本、K-shortestは10個まで。番号は0始まりです。重みと費用は小さな整数を用い、C++の全整数型や最大規模は再現しません。グラフの矢印は向き、辺ラベルは重み/容量、色は選択した経路や処理対象です。

- Radixは現在キーとのXORによるバケット、Dialは循環バケット、DAGはトポロジカル順、Johnsonはpotentialとreweightを表示します。
- K-th PathはYenの禁止頂点/辺と分岐候補。K-th WalkはEppsteinのsidetrack heapと候補置換/追加を表示します。図解は短いwalkをその場で復元し、C++は要求したものだけ遅延復元します。
- Functional Graphは閉路/深さ/入口を表示。jumpは図解では閉路長で剰余を取り、C++の64段BinaryLiftingそのものは描きません。stepsは十進文字列でuint64上限まで入力できます。
- Block-Cutは辺スタックとlow、2-SATは含意とSCC、Eulerは使用辺とHierholzerのスタックを表示します。教材の小さなDFSは再帰、C++は反復です。
- 下限制約/Project Selectionは構築した網と増加路を表示。教材はBFS増加路で流し、C++のDinicの層/反復DFSはこの図解では再現しません。人工辺の容量は図のsoft/upper総和から設定し、C++の整数上限の扱いとは異なります。
- 木ハッシュはC++と同じSplitMix64の固定seedと二重modをBigIntで計算。部分木の高さ・salt・積を表示します。衝突があり得る点はC++と同じです。

```sh
node verify/visualizations/graph_a_model_test.js
node verify/visualizations/graph_a_browser_test.mjs --browser /path/to/chromium
```
