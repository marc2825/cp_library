---
title: Bellman–Ford
documentation_of: //graph_basic/bellman_ford.hpp
---

# Bellman–Ford

負辺を含む最短路と、到達可能な負閉路の影響範囲を求めます。 [操作できる図解](../../visualizations/graph_basic/index.html#bellman)で入力と内部状態を変更して追えます。

## API・使用条件

```text
cp::bellman_ford(g,start=0) -> ShortestPathResult<T>
```

有向・無向に対応。頂点数は 1 以上で有効な始点が必要です。負辺には符号付き型を使い、検出途中の値を含む全加算が型に収まる必要があります。 戻り値の [距離・経路の共通結果型](shortest_path.md) を参照してください。


## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_basic/bellman_ford.hpp"
int main() {
    cp::Graph<> g(4, cp::GraphKind::Directed);
    g.add_edge(0,1,2); g.add_edge(1,1,-1); g.add_edge(1,2,3);
    auto r=cp::bellman_ford(g,0);
    assert(r.has_finite_distance(0));
    assert(r.reachable(2) && r.negative_cycle[2]);
    assert(r.vertex_path(2).empty() && !r.reachable(3));
}
```

## 原理と注意点

元の辺列を最大 V−1 周緩和します。無向辺は両向きを調べます。さらに改善できる辺の行先から到達する頂点へ、負閉路の影響を伝播します。

negative_cycle[v] は「v が負閉路上にある」ではなく「始点から届く負閉路の影響を受ける」です。届かない負閉路には反応しません。影響下の dist は最短距離として使わないでください。

## 計算量

時間 O(V+VE)、追加メモリ O(V)。辺列を走査するため、孤立点が多い場合にも各周の全頂点走査をしません。更新のない周では早期終了します。 E は論理辺の数で、無向辺の隣接リストには両方向を保持します。入力グラフ自体の O(V+E) メモリは別途必要です。

## 問題・検証・参考

[AOJ GRL_1_B: Single Source Shortest Path (Negative Edges)](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_1_B) / [ジャッジ形式](../../verify/graph_basic/bellman_ford.test.cpp)。ローカルの Python オラクルと比較しており、オンライン AC を実施したという意味ではありません。

600 グラフの差分検証、経路の始終点・辺・重みの検査、DFS の再帰版比較、DAG の順列全探索、最小全域森の辺部分集合全探索、20 万頂点を含む [共通検証](README.md#検証) を用意しています。

[CP-Algorithms の解説](https://cp-algorithms.com/graph/bellman_ford.html) を参照しました。[参考資料・設計判断](references.md)。
