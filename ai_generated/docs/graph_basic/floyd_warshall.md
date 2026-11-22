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

対角を 0、直接辺を最小重みで初期化し、中継 k を最も外側のループにして更新します。中継 k の段で dist[k][k]<0 なら、k を経由できる組を負閉路の影響下として印を付け、以降はその組で加算をしません。

INF に重みを足しません。負閉路の影響下の組は加算しないため、負閉路があっても中間値が指数的に小さくなってあふれることはありません（以前は長い負の鎖で`overflow_error`になっていました）。影響下の組の`dist`は数値型の最小値です。影響のない組の経路は復元できます。

## 計算量

時間 O(V³)、追加メモリ O(V²)。親頂点・親辺・負閉路フラグも全点対分保持します。 E は論理辺の数で、無向辺の隣接リストには両方向を保持します。入力グラフ自体の O(V+E) メモリは別途必要です。

## 問題・検証・参考

[AOJ GRL_1_C: All Pairs Shortest Path](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_1_C) / [ジャッジ形式](../../verify/graph_basic/floyd_warshall.test.cpp)。ローカルの Python オラクルと比較しており、オンライン AC を実施したという意味ではありません。

600グラフで各始点の dist・reachable・negative_cycle を歩道DPと比較し、復元経路を検査します。負閉路の影響を受ける組は、70頂点の負の鎖と、重み±10⁹・n=10〜40 の300グラフで Bellman–Ford と一致し、途中の値があふれないことを確認します（20万頂点のケースはありません）（[共通検証](README.md#検証)）。

[CP-Algorithms の解説](https://cp-algorithms.com/graph/all-pair-shortest-path-floyd-warshall.html) を参照しました。[参考資料・設計判断](references.md)。
