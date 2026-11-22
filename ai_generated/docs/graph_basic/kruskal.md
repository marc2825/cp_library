---
title: Kruskal
documentation_of: //graph_basic/kruskal.hpp
---

# Kruskal

軽い辺から別成分をつなぎ、最小全域森を求めます。 [操作できる図解](../../visualizations/graph_basic/index.html#kruskal)で入力と内部状態を変更して追えます。

## API・使用条件

```text
cp::kruskal(g) -> MinimumSpanningForestResult<T>
```

無向グラフ用。負辺・多重辺・自己ループ・非連結・空グラフに対応。 戻り値の [最小全域森の結果型](spanning_forest.md) を参照してください。


## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_basic/kruskal.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0,1,4); g.add_edge(0,2,1); g.add_edge(1,2,2);
    auto forest=cp::kruskal(g);
    assert(forest.cost==3 && forest.component_count==2 && !forest.connected());
    assert(forest.edges[0].idx==1 && forest.edges[1].idx==2);
}
```

## 原理と注意点

重み順（同重みなら辺 ID 順）に調べ、Union-Find で端点が別成分なら採用します。閉路を作る辺と自己ループは除外します。入力の辺列は並べ替えません。

非連結でも失敗にせず、各成分の最小全域木を合わせた森を返します。MST が必要なら connected() を確認してください。総コストの加算は型の範囲内である必要があります。

## 計算量

時間 O(V+E log(E+1))、追加メモリ O(V+E)。3.2 の Union-Find を利用します。 E は論理辺の数で、無向辺の隣接リストには両方向を保持します。入力グラフ自体の O(V+E) メモリは別途必要です。

## 問題・検証・参考

[Library Checker: Minimum Spanning Tree](https://judge.yosupo.jp/problem/minimum_spanning_tree) / [ジャッジ形式](../../verify/graph_basic/kruskal.test.cpp)。ローカルの Python オラクルと比較しており、オンライン AC を実施したという意味ではありません。

無向300グラフで、全辺部分集合から求めた最小全域森の費用・成分数と比較し、採用辺が元のグラフと同じ成分を張ることを検査します。ジャッジ形式24ケースと20万頂点の鎖も確認します（[共通検証](README.md#検証)）。

[CP-Algorithms の解説](https://cp-algorithms.com/graph/mst_kruskal_with_dsu.html) を参照しました。[参考資料・設計判断](references.md)。
