---
title: BFS（幅優先探索）
documentation_of: //graph_basic/bfs.hpp
---

# BFS（幅優先探索）

重みを無視し、始点からの最小辺数を求めます。 [操作できる図解](../../visualizations/graph_basic/index.html#bfs)で入力と内部状態を変更して追えます。

## API・使用条件

```text
cp::bfs(g,start=0) -> ShortestPathResult<int>
```

有向・無向のどちらも可。頂点数は 1 以上で、有効な始点が必要です。自己ループ・多重辺も扱います。 戻り値の [距離・経路の共通結果型](shortest_path.md) を参照してください。


## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_basic/bfs.hpp"
int main() {
    cp::Graph<> g(3, cp::GraphKind::Directed);
    g.add_edge(0,1,100); g.add_edge(1,2,1); g.add_edge(0,2,500);
    auto r=cp::bfs(g,0);
    assert(r.dist[2]==1); // cost を無視して辺数を数える
    assert(r.vertex_path(2)==(std::vector<int>{0,2}));
}
```

## 原理と注意点

FIFO キューから発見順に頂点を取り出します。最初に発見したときに距離・親を保存すると、辺数の少ない層から処理できます。

重み付きグラフでも cost は無視します。最小重みの経路が必要なら 0-1 BFS / Dijkstra 等を選んでください。

## 計算量

時間 O(V+E)、追加メモリ O(V)。到達不能も含む長さ V の結果を返します。 E は論理辺の数で、無向辺の隣接リストには両方向を保持します。入力グラフ自体の O(V+E) メモリは別途必要です。

## 問題・検証・参考

[AOJ ALDS1_11_C: Breadth First Search](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_11_C) / [ジャッジ形式](../../verify/graph_basic/bfs.test.cpp)。ローカルの Python オラクルと比較しており、オンライン AC を実施したという意味ではありません。

600 グラフの差分検証、経路の始終点・辺・重みの検査、DFS の再帰版比較、DAG の順列全探索、最小全域森の辺部分集合全探索、20 万頂点を含む [共通検証](README.md#検証) を用意しています。

[CP-Algorithms の解説](https://cp-algorithms.com/graph/breadth-first-search.html) を参照しました。[参考資料・設計判断](references.md)。
