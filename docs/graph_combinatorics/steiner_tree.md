---
title: Steiner Tree DP
documentation_of: //graph_combinatorics/steiner_tree.hpp
---

# Steiner Tree DP

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#steiner_tree) / [実装](../../graph_combinatorics/steiner_tree.hpp) / [一次資料と設計](../b_7_11/references.md)

terminal集合を部分集合ごとに併合し、multi-source Dijkstraで根の位置を広げるDreyfus–Wagner法です。使用した辺も復元します。

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/steiner_tree.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,1); g.add_edge(1,2,2); g.add_edge(0,2,4);
    std::cout << cp::steiner_tree(g,{0,2})->cost << '\n';
}
```

出力:

```text
3
```

## API・使用条件

`steiner_tree(g,vector<int> terminals)`→optional<SteinerTreeResult<T>>（cost,vertices,edge_ids）。非負無向、重複terminalは除去。異なるterminal数K<=20。接続不能はnullopt、terminal空は費用0で空の木。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

復元stateを重複処理せず、得られた辺集合から閉路と非terminal葉を除きます。ゼロ重みも扱います。距離・併合費用の中間値がTの有限範囲に収まる必要があります。Kの上限とは別に状態数のメモリ・時間を見積もってください。

## 計算量

時間O(3^K V+2^K(V+E) log(V+E+1))、領域O(2^K V+V+E)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: terminal集合を部分集合ごとに併合し、multi-source Dijkstraで根の位置を広げるDreyfus–Wagner法です。使用した辺も復元します。 [実行できる小問題](../../examples/graph_combinatorics/steiner_tree_b.cpp)。

Steiner Tree DPを必要とする3.10の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
