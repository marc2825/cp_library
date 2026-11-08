---
title: Dijkstra
documentation_of: //graph_basic/dijkstra.hpp
---

# Dijkstra

非負重みのグラフで単一始点の最短路を求めます。 [操作できる図解](../../visualizations/graph_basic/index.html#dijkstra)で入力と内部状態を変更して追えます。

## API・使用条件

```text
cp::dijkstra(g,start=0) -> ShortestPathResult<T>
```

全辺が非負。有向・無向、自己ループ・多重辺に対応。頂点数は 1 以上で有効な始点が必要です。 戻り値の [距離・経路の共通結果型](shortest_path.md) を参照してください。


## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_basic/dijkstra.hpp"
int main() {
    cp::Graph<> g(4, cp::GraphKind::Directed);
    g.add_edge(0,1,7); g.add_edge(0,2,2); g.add_edge(2,1,1); g.add_edge(1,3,2);
    auto r=cp::dijkstra(g,0);
    assert(r.has_finite_distance(3) && r.dist[3]==5);
    assert(r.vertex_path(3)==(std::vector<int>{0,2,1,3}));
    assert(r.edge_path(3)==(std::vector<int>{1,2,3}));
}
```

## 原理と注意点

優先度付きキューに (仮距離,頂点) を追加します。最小候補から緩和し、古い候補は dist との比較で破棄します。

負辺には使えません。同じ最短距離で親を変更せず、経路を一つ返します。辞書順最小などのタイブレークは保証しません。

## 計算量

時間 O(V+E log(E+1))、追加メモリ O(V+E)。decrease-key の代わりに更新後の候補を追加する実装です。 E は論理辺の数で、無向辺の隣接リストには両方向を保持します。入力グラフ自体の O(V+E) メモリは別途必要です。

## 問題・検証・参考

[Library Checker: Shortest Path](https://judge.yosupo.jp/problem/shortest_path) / [ジャッジ形式](../../verify/graph_basic/shortest_path.test.cpp)。ローカルの Python オラクルと比較しており、オンライン AC を実施したという意味ではありません。

600 グラフの差分検証、経路の始終点・辺・重みの検査、DFS の再帰版比較、DAG の順列全探索、最小全域森の辺部分集合全探索、20 万頂点を含む [共通検証](README.md#検証) を用意しています。

[CP-Algorithms の解説](https://cp-algorithms.com/graph/dijkstra_sparse.html) を参照しました。[参考資料・設計判断](references.md)。
