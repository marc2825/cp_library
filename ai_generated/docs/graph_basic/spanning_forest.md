---
title: 最小全域森の共通結果型
documentation_of: //graph_basic/spanning_forest.hpp
---

# MinimumSpanningForestResult

`cp::MinimumSpanningForestResult<T>` は Kruskal と Prim の共通結果です。

| フィールド・関数 | 意味 |
|---|---|
| `cost` | 採用辺の重みの総和 |
| `edges` | 採用した `Edge<T>` の vector。入力時の from/to/cost/idx を保持 |
| `component_count` | 入力グラフの連結成分数 |
| `connected()` | component_count≤1 か。空グラフでも true |

非連結なら成分ごとの最小全域木を合わせた森。採用辺数は V−component_count、孤立点も一成分です。空グラフは cost=0、辺列は空、成分数=0。辺列の順序は採用順で、DFS の親子順や頂点番号順ではありません。

```cpp
#include <cassert>
#include "graph_basic/kruskal.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0,1,-2); g.add_edge(2,3,5);
    auto result=cp::kruskal(g);
    assert(result.cost==3 && result.component_count==2);
    assert(!result.connected() && result.edges.size()==2);
}
```

結果のメモリは採用辺数 F に対して O(F)、フィールド参照・connected は O(1)。算法全体の計算量・使用条件・図解は [Kruskal](kruskal.md) / [Prim](prim.md) を参照してください。総コストの加算にも [数値範囲の契約](README.md#共通の約束) が適用されます。

[Library Checker: Minimum Spanning Tree](https://judge.yosupo.jp/problem/minimum_spanning_tree) のジャッジ形式と、非連結を含む辺部分集合全探索、20 万頂点の検証があります。[検証](README.md#検証) / [参考資料](references.md)。
