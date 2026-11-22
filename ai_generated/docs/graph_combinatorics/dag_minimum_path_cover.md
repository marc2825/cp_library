---
title: DAGの最小Path Cover
documentation_of: //graph_combinatorics/dag_minimum_path_cover.hpp
---

# DAGの最小Path Cover

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#dag_minimum_path_cover) / [実装](../../graph_combinatorics/dag_minimum_path_cover.hpp) / [一次資料と設計](../b_7_11/references.md)

元の辺を左→右に写した二部グラフの最大マッチングを求め、頂点を一度ずつ覆うパスへ戻します。

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/dag_minimum_path_cover.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1); g.add_edge(1,2);
    std::cout << cp::dag_minimum_path_cover(g).size() << '\n';
}
```

出力:

```text
1
```

## API・使用条件

`dag_minimum_path_cover(g)`→DAGPathCoverResult（paths,edge_ids,size()）。有向DAG、重み無視。閉路はinvalid_argument。孤立頂点も長さ0のパスに含む。

edge_ids は g.edges() の添字。無向グラフも`invalid_argument`です。

## 仕組みと境界

最小パス数はV−最大マッチング数。推移閉包の辺は追加しないため、返す各パスは元グラフの実際の辺だけから成ります。

## 計算量

時間O(E sqrt(V)+V+E)、領域O(V+E)。 Vは頂点数、Eは論理辺数。

## 使用する問題・検証

想定問題: DAG の全頂点を、元の辺に沿う頂点素なパスの最小本数で覆う（到達関係で飛び越してよい鎖分割は dag_maximum_antichain）。 [実行できる小問題](../../examples/graph_combinatorics/dag_minimum_path_cover_b.cpp)。

検証では、辺を小さい番号から大きい番号へ向けた8頂点未満のランダム DAG 350個で、パス数が V−（全探索の最大マッチング）に等しく、各頂点がちょうど1回現れ、edge_ids が連続する頂点を結ぶ辺であることを確かめます。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/combinatorics_test.cpp)。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
