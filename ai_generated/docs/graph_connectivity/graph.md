---
title: 連結性を調べるグラフ表現
documentation_of: //graph_connectivity/graph.hpp
---

# 共通 Graph の入口

`graph_connectivity/graph.hpp` は共有の [graph/graph.hpp](../../graph/graph.hpp) を読み込みます。新しいグラフ型を重複して定義しません。基本 API は [3.7 の Graph ガイド](../graph_basic/graph.md) と同じです。

```cpp
#include <cassert>
#include "graph_connectivity/graph.hpp"
int main() {
    cp::Graph<> directed(2,cp::GraphKind::Directed);
    directed.add_edge(0,1);
    cp::Graph<> undirected(2); // 既定は無向
    int id=undirected.add_edge(0,1);
    assert(directed[1].empty());
    assert(undirected[1][0].idx==id && undirected.edge_count()==1);
}
```

Graph は n≥0 で構築し、`add_edge(u,v,cost)` は有効な頂点同士に辺を追加して論理辺 ID を返します。無向辺の両方向は同じ ID。`g[v]` は隣接辺の const 参照、`edges()` は元の辺を一度ずつ保持します。構築 O(V)、追加償却 O(1)、メモリ O(V+E)。頂点数・辺数は int に収まる範囲を前提とします。

今回の関数は重みを使わず、コピーできる任意のコスト型で動作します。全関数が const Graph を受け取り、入力を変更しません。入力補助は [3.7 の read_graph](../graph_basic/input.md) を使えます。

[各関数の説明・問題例・図解](README.md) / [検証](README.md#検証) / [参考資料](references.md)。
