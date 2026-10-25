---
title: 区間辺グラフ
documentation_of: //graph_basic/segment_tree_graph.hpp
---

# 区間辺グラフ

点→点・点→半開区間・区間→点・区間→区間を追加し、明示グラフへの展開を省きます。既存SegmentTreeCoverの標準ノード分解を共有します。

[操作できる図解](../../visualizations/graph_extensions/index.html#segment_tree_graph) / [15項目の検証](../graph_extensions/README.md) / [実装](../../graph_basic/segment_tree_graph.hpp)

## usage

```cpp
#include <iostream>
#include "graph_basic/segment_tree_graph.hpp"
#include "graph_basic/dijkstra.hpp"
int main() {
    cp::SegmentTreeGraph<> builder(4);
    builder.add_point_to_range(0,1,3,2);
    builder.add_range_to_point(1,3,3,5);
    builder.add_range_to_range(0,2,2,4,4);
    auto result=cp::dijkstra(builder.graph(),0);
    for (int v=0;v<4;++v) std::cout << result.dist[v] << (v==3?'\n':' ');
}
```

出力:

```text
0 2 2 4
```

## API・使用条件

`SegmentTreeGraph<T>(n)`、`add_edge(u,v,w)`、`add_point_to_range(u,l,r,w)`、`add_range_to_point(l,r,v,w)`、`add_range_to_range(l,r,a,b,w)`。追加は操作IDを返します。`graph()`は独立した有向Graphのコピー。`original_size()`、`vertex_count()`、`query_count()`も公開。

元頂点は常に[0,n)。`vertex_info()`はOriginal/IntoRange/OutOfRange/Connectorと半開区間、connectorの操作ID。`edge_query_ids()[edge_id]`は対応する操作ID、構造辺は−1。区間→区間の入口は構造辺、出口だけに費用と操作IDが付くため、経路上で費用を二重計上しません。空区間も操作IDを消費します。重みTに0が必要で、負辺の可否は後段の最短路に従います。範囲外はout_of_range。

## 計算量

初期化O(n)。1操作O(log(n+1))時間・追加辺、区間→区間の補助頂点は1個。q操作後O(n+q)頂点、O(n+q log(n+1))辺。snapshotのgraph()もこの総量に比例。

## 使用する問題・検証

区間へ一括移動できる最短路。区間→区間を完全二部辺に展開すると大きすぎる問題。

[動く使用例](../../examples/graph_basic/segment_tree_graph.cpp)と[独立比較テスト](../../verify/graph_extensions/)を`make verify-graph-extensions`で実行します。小規模全探索、復元した頂点・辺ID、最適値、境界・不正入力を確認します。オンラインACは未確認です。

[一次資料・設計判断](../graph_extensions/references.md) / [カテゴリ索引](README.md)。
