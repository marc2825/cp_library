---
title: Lowlink
documentation_of: //graph_connectivity/low_link.hpp
---

# Lowlink

DFS の発見順と戻れる最小番号を計算し、橋と関節点をまとめて取得します。 [操作できる図解](../../visualizations/graph_connectivity/index.html#lowlink)で入力と手順を変更できます。

## API・使用条件

```text
cp::low_link(g) -> LowLinkResult
```

無向グラフ用。非連結・空グラフ・自己ループ・多重辺に対応し、重みは使いません。 共通の [Graph](graph.md) を使い、結果は入力グラフと独立した値として返します。グラフを変更したら再実行してください。

## LowLinkResult

| フィールド | 意味 |
|---|---|
| `ord[v]` | 全成分を通じた 0 始まりの DFS 発見順 |
| `low[v]` | 戻り辺で到達できる最小の ord |
| `parent[v]`, `parent_edge[v]` | DFS 木の親と論理辺 ID。各根は −1 |
| `subtree_size[v]` | DFS 木での部分木の頂点数 |
| `bridge_edges` | 橋の論理辺 ID の昇順 |
| `articulation` | 関節点の頂点番号の昇順 |

橋は ord[p]<low[child]、根以外の関節点は ord[p]≤low[child]、根は子の数が 2 個以上と判定します。[橋](bridges.md) / [関節点](articulation_points.md) の個別説明を参照してください。

## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_connectivity/low_link.hpp"
int main() {
    cp::Graph<> g(5);
    g.add_edge(0,1);g.add_edge(1,2);g.add_edge(2,0);
    g.add_edge(1,3);g.add_edge(3,4);g.add_edge(3,4);
    auto result=cp::low_link(g);
    assert(result.bridge_edges==(std::vector<int>{3}));
    assert(result.articulation==(std::vector<int>{1,3}));
    assert(result.low[4]==result.ord[3]); // 平行辺で親へ戻れる
}
```

## 原理・注意点

ord は発見順、low は DFS 木を部分木へ下り、木辺以外の戻り辺を高々一本使って届く最小 ord です。子から戻るときに low と subtree_size を親へ伝播します。

親の頂点ではなく、入ってきた論理辺 ID の逆向きだけを除外します。訪問済み頂点への辺では ord[to]、DFS の子から戻るときには low[child] を使います。

## 計算量

時間 O(V+E)、追加メモリ O(V+E)。ord / low 等は O(V)、辺の橋フラグは O(E)。再帰なし。 E は論理辺数です。入力グラフの O(V+E) メモリは別途必要です。

## 使用する問題・検証・参考

[AOJ GRL_3_B: Bridges](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_3_B) / [ジャッジ形式](../../verify/graph_connectivity/bridges.test.cpp)。オンライン提出・AC を実施したという意味ではありません。

531 有向グラフ・760 無向多重グラフの全探索、600 ランダムグラフ、20 万頂点を含む [共通検証](README.md#検証) を用意しています。相互到達性、縮約 DAG の辺対応、辺／頂点削除、low と部分木、全 0/1 彩色を独立オラクルと照合します。

[原理の参考](https://cp-algorithms.com/graph/bridge-searching.html) / [参考資料・設計判断](references.md)。
