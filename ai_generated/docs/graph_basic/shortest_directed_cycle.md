---
title: 最短有向閉路
documentation_of: //graph_basic/shortest_directed_cycle.hpp
---

# 最短有向閉路

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#shortest_directed_cycle) / [実装](../../graph_basic/shortest_directed_cycle.hpp) / [一次資料と設計](../b_7_11/references.md)

非負重みの最短閉路を求め、閉じた頂点列と辺ID列を復元します。

## usage

```cpp
#include <iostream>
#include "graph_basic/shortest_directed_cycle.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1,2); g.add_edge(1,2,3); g.add_edge(2,0,4);
    std::cout << cp::shortest_directed_cycle(g)->cost << '\n';
}
```

出力:

```text
9
```

## API・使用条件

`shortest_directed_cycle(g)` → `optional<WeightedPath<T>>`。有向グラフ（無向グラフは`invalid_argument`）。負辺はinvalid_argument、閉路なしはnullopt。返値verticesの先頭と末尾は同じ、edge_idsの長さはverticesの長さ−1。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

各始点でDijkstraを行い、始点へ入る辺で閉じます。

距離和の全中間値はTの有限範囲に収めてください。整数型の最大値は到達不能の番兵に使い、その値と等しくなる距離もoverflow_errorで拒否します。

## 計算量

時間O(V(V+E) log(V+E+1))、領域O(V+E)。 Vは頂点数、Eは論理辺数。重みの比較・加減算はO(1)として数えています。

## 使用する問題・検証

想定問題: 非負重みの有向グラフで、重み最小の有向閉路とその頂点列・辺ID列。 [実行できる小問題](../../examples/graph_basic/shortest_directed_cycle_b.cpp)。

検証では、n≤7・m≤11 の有向350グラフを単純閉路の全列挙と比較し、費用、辺IDの重複なし、各辺の向き、閉路が閉じていることを検査します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/graph_test.cpp)。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
