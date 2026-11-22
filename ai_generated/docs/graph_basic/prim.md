---
title: Prim
documentation_of: //graph_basic/prim.hpp
---

# Prim

木と外側を結ぶ最小の辺を選び、最小全域森を求めます。 [操作できる図解](../../visualizations/graph_basic/index.html#prim)で入力と内部状態を変更して追えます。

## API・使用条件

```text
cp::prim(g) -> MinimumSpanningForestResult<T>
```

無向グラフ用。負辺・多重辺・自己ループ・非連結・空グラフに対応。 戻り値の [最小全域森の結果型](spanning_forest.md) を参照してください。


## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_basic/prim.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0,1,4); g.add_edge(0,2,1); g.add_edge(1,2,2);
    auto forest=cp::prim(g);
    assert(forest.cost==3 && forest.component_count==2);
    assert(forest.edges.size()==2 && !forest.connected());
}
```

## 原理と注意点

現在の木から未訪問頂点へ出る辺を優先度付きキューへ入れ、最小の辺で木を拡張します。両端とも木の中に入った古い候補は捨て、別成分は新しい根から再開します。

Dijkstra の優先値が始点からの距離であるのに対し、Prim は辺単体の重みを優先します。採用した Edge の from/to は入力時の向きを保持し、探索時の親子の向きとは限りません。

## 計算量

時間 O(V+E log(E+1))、追加メモリ O(V+E)。 E は論理辺の数で、無向辺の隣接リストには両方向を保持します。入力グラフ自体の O(V+E) メモリは別途必要です。

## 問題・検証・参考

[Library Checker: Minimum Spanning Tree](https://judge.yosupo.jp/problem/minimum_spanning_tree) / [ジャッジ形式](../../verify/graph_basic/prim.test.cpp)。ローカルの Python オラクルと比較しており、オンライン AC を実施したという意味ではありません。

無向300グラフで、全辺部分集合から求めた最小全域森の費用・成分数と比較し、採用辺が元のグラフと同じ成分を張ることを検査します。ジャッジ形式24ケースと20万頂点の鎖も確認します（[共通検証](README.md#検証)）。

[CP-Algorithms の解説](https://cp-algorithms.com/graph/mst_prim.html) を参照しました。[参考資料・設計判断](references.md)。
