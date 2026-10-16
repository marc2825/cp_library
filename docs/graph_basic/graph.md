---
title: 共通のグラフ表現
documentation_of: //graph_basic/graph.hpp
---

# Graph と Edge

`cp::Graph<T=long long>` はグラフ表現、`cp::Edge<T=long long>` は `{from,to,cost,idx}` を保持します。共有の本体は [graph/graph.hpp](../../graph/graph.hpp)、3.7 の入口は [graph_basic/graph.hpp](../../graph_basic/graph.hpp) です。

| API | 意味・計算量 |
|---|---|
| `Graph(n=0,kind=GraphKind::Undirected)` | n≥0、既定は無向。O(n) |
| `add_edge(u,v,cost=T(1))` | 有効な頂点同士の辺を追加し、追加順の ID を返す。償却 O(1) |
| `size()`, `edge_count()`, `kind()`, `directed()` | 頂点数、論理辺数、向き。O(1) |
| `g[v]` | v から出る辺の読み取り専用 vector 参照。O(1) |
| `edges()` | 入力した元の辺を一度ずつ保持した vector の const 参照。O(1) |

無向辺は隣接リストに両方向を入れ、同じ idx を持ちます。無向の自己ループも両方向分を保持します。多重辺の ID は別です。メモリ O(V+E)、頂点・辺数は int で表現できる範囲を前提とします。

```cpp
#include <cassert>
#include "graph_basic/graph.hpp"
int main() {
    cp::Graph<> g(2); // 無向
    int id=g.add_edge(0,1,5);
    assert(id==0 && g.edges().size()==1);
    assert(g[1][0].to==0 && g[1][0].idx==id);
    cp::Graph<> directed(2,cp::GraphKind::Directed);
    directed.add_edge(0,1);
    assert(directed[1].empty());
}
```

全アルゴリズムは const Graph を受け取り、入力を変更しません。無向グラフと有向グラフの入力を混同しないでください。計算後に辺を追加しても、既存の結果は更新されません。

使用する問題・図解は [カテゴリ一覧](README.md) に掲載しています。構造・ID・自己ループ・多重辺は [共通検証](README.md#検証) と旧 API 検証で確認します。[参考資料](references.md)。
