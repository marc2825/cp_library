---
title: 0-1 BFS
documentation_of: //graph_basic/zero_one_bfs.hpp
---

# 0-1 BFS

重みが 0 または 1 のグラフで最短距離を求めます。 [操作できる図解](../../visualizations/graph_basic/index.html#zero-one)で入力と内部状態を変更して追えます。

## API・使用条件

```text
cp::zero_one_bfs(g,start=0) -> ShortestPathResult<T>
```

全辺の重みが T(0) または T(1)。有向・無向のどちらも可、頂点数は 1 以上で有効な始点が必要です。 戻り値の [距離・経路の共通結果型](shortest_path.md) を参照してください。


## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_basic/zero_one_bfs.hpp"
int main() {
    cp::Graph<> g(3, cp::GraphKind::Directed);
    g.add_edge(0,1,1); g.add_edge(0,2,0); g.add_edge(2,1,0);
    auto r=cp::zero_one_bfs(g,0);
    assert(r.dist[1]==0);
    assert(r.vertex_path(1)==(std::vector<int>{0,2,1}));
}
```

## 原理と注意点

改善した候補を重み 0 なら deque の先頭、1 なら末尾へ入れ、距離の小さい順に処理します。現在の距離と一致しない古い候補は破棄します。

訪問済みかだけで更新を止めず、現在より厳密に距離が短くなるかを比較します。重み 2 や負辺には使えません。

## 計算量

時間 O(V+E)、追加メモリ O(V+E)（結果と deque を含む安全な上界）。 E は論理辺の数で、無向辺の隣接リストには両方向を保持します。入力グラフ自体の O(V+E) メモリは別途必要です。

## 問題・検証・参考

[AtCoder ABC077 D: Small Multiple](https://atcoder.jp/contests/abc077/tasks/arc084_b) / [ジャッジ形式](../../verify/graph_basic/zero_one_bfs.test.cpp)。ローカルの Python オラクルと比較しており、オンライン AC を実施したという意味ではありません。

600 グラフの差分検証、経路の始終点・辺・重みの検査、DFS の再帰版比較、DAG の順列全探索、最小全域森の辺部分集合全探索、20 万頂点を含む [共通検証](README.md#検証) を用意しています。

[CP-Algorithms の解説](https://cp-algorithms.com/graph/01_bfs.html) を参照しました。[参考資料・設計判断](references.md)。
