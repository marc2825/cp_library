---
title: DFS（深さ優先探索）
documentation_of: //graph_basic/dfs.hpp
---

# DFS（深さ優先探索）

単一始点の到達探索と、全頂点を訪問する DFS 森を提供します。 [操作できる図解](../../visualizations/graph_basic/index.html#dfs)で入力と内部状態を変更して追えます。

## API・使用条件

```text
depth_first_search(g,start=0) -> ShortestPathResult<int>
dfs_forest(g,first_root=-1) -> DFSForestResult
```

有向・無向、自己ループ・多重辺に対応。単一始点版は有効な始点が必要。森版は空グラフ可、first_root=-1 なら頂点番号順、有効な根を指定すればその根から先に訪問します。 戻り値の [距離・経路の共通結果型](shortest_path.md) を参照してください。


## DFSForestResult

- `parent`, `parent_edge`: DFS 木の直前の頂点と辺。各根は −1。
- `depth`: 根からの辺数。`component`: DFS 木の番号、`roots`: その根。
- `preorder`, `postorder`: 行きがけ順・帰りがけ順の頂点列。
- `tin`, `tout`: preorder 上の部分木の半開区間。`preorder[tin[v]..tout[v])` が v の DFS 木の部分木。

森版は全頂点を一度ずつ訪問し、単一始点版の `reachable` は始点から届く頂点だけを true にします。

## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_basic/dfs.hpp"
int main() {
    cp::Graph<> g(5, cp::GraphKind::Directed);
    g.add_edge(0,1); g.add_edge(1,2); g.add_edge(0,2); g.add_edge(3,4);
    auto r=cp::depth_first_search(g,0);
    assert(r.dist[2]==2 && !r.reachable(4));
    auto forest=cp::dfs_forest(g);
    assert(forest.preorder==(std::vector<int>{0,1,2,3,4}));
    assert(forest.postorder==(std::vector<int>{2,1,0,4,3}));
    assert(forest.tin[0]==0 && forest.tout[0]==3);
}
```

## 原理と注意点

頂点と次に見る辺の位置をスタックに保存し、再帰 DFS と同じ順に枝を深く辿ります。探索開始時に preorder、戻るときに postorder へ追加します。

dist / depth は DFS 木の深さで、最短距離ではありません。有向グラフの component は DFS 森の木の番号で、強連結・弱連結成分の番号ではありません。

## 計算量

どちらも時間 O(V+E)、追加メモリ O(V)。再帰を使わず、20 万頂点の鎖でも実行できます。 E は論理辺の数で、無向辺の隣接リストには両方向を保持します。入力グラフ自体の O(V+E) メモリは別途必要です。

## 問題・検証・参考

[AOJ ALDS1_11_B: Depth First Search](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_11_B) / [ジャッジ形式](../../verify/graph_basic/dfs.test.cpp)。ローカルの Python オラクルと比較しており、オンライン AC を実施したという意味ではありません。

600 グラフの差分検証、経路の始終点・辺・重みの検査、DFS の再帰版比較、DAG の順列全探索、最小全域森の辺部分集合全探索、20 万頂点を含む [共通検証](README.md#検証) を用意しています。

[CP-Algorithms の解説](https://cp-algorithms.com/graph/depth-first-search.html) を参照しました。[参考資料・設計判断](references.md)。
