---
title: Warshall–Floyd（Floyd–Warshall）
documentation_of: //graph_basic/floyd_warshall.hpp
---

# Warshall–Floyd（Floyd–Warshall）

全始点・終点間の最短距離と、負閉路が影響する組を求めます。 [操作できる図解](../../visualizations/graph_basic/index.html#floyd)で入力と内部状態を変更して追えます。

## API・使用条件

```text
cp::floyd_warshall(g) -> vector<ShortestPathResult<T>>
```

有向・無向、空グラフに対応。負辺には符号付き型を使います。全点対を保持できる頂点数で利用してください。 戻り値の [距離・経路の共通結果型](shortest_path.md) を参照してください。


## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_basic/floyd_warshall.hpp"
int main() {
    cp::Graph<> g(3, cp::GraphKind::Directed);
    g.add_edge(0,1,4); g.add_edge(1,2,-2); g.add_edge(0,2,9);
    auto all=cp::floyd_warshall(g);
    assert(all[0].dist[2]==2 && !all[2].reachable(0));
    assert(all[0].vertex_path(2)==(std::vector<int>{0,1,2}));
}
```

## 原理と注意点

対角を 0、直接辺を最小重みで初期化し、中継 k を最も外側のループにして更新します。dist[k][k]<0 となる頂点を経由できる組だけを負閉路の影響下とします。

INF に重みを足しません。負閉路があると中間値が非常に小さくなる場合があり、数値範囲の条件は検出途中にも必要です。影響のない組の経路は復元できます。

## 計算量

時間 O(V³)、追加メモリ O(V²)。親頂点・親辺・負閉路フラグも全点対分保持します。 E は論理辺の数で、無向辺の隣接リストには両方向を保持します。入力グラフ自体の O(V+E) メモリは別途必要です。

## 問題・検証・参考

[AOJ GRL_1_C: All Pairs Shortest Path](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_1_C) / [ジャッジ形式](../../verify/graph_basic/floyd_warshall.test.cpp)。ローカルの Python オラクルと比較しており、オンライン AC を実施したという意味ではありません。

600 グラフの差分検証、経路の始終点・辺・重みの検査、DFS の再帰版比較、DAG の順列全探索、最小全域森の辺部分集合全探索、20 万頂点を含む [共通検証](README.md#検証) を用意しています。

[CP-Algorithms の解説](https://cp-algorithms.com/graph/all-pair-shortest-path-floyd-warshall.html) を参照しました。[参考資料・設計判断](references.md)。
