---
title: 関節点
documentation_of: //graph_connectivity/articulation_points.hpp
---

# 関節点

削除すると連結成分数が増える頂点を求めます。 [操作できる図解](../../visualizations/graph_connectivity/index.html#articulation)で入力と手順を変更できます。

## API・使用条件

```text
cp::articulation_points(g) -> vector<int>
```

無向グラフ用。非連結・空グラフ・自己ループ・多重辺に対応。結果は頂点番号の昇順です。 共通の [Graph](graph.md) を使い、結果は入力グラフと独立した値として返します。グラフを変更したら再実行してください。

## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_connectivity/articulation_points.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0,1);g.add_edge(0,2);g.add_edge(0,3);
    assert(cp::articulation_points(g)==(std::vector<int>{0}));
    cp::Graph<> isolated(1);
    assert(cp::articulation_points(isolated).empty());
}
```

## 原理・注意点

根以外の p は、ある DFS の子 v について ord[p]≤low[v] なら関節点です。DFS の根は、DFS 木の子が二つ以上ある場合に限り関節点です。

根の子の数は元の次数ではありません。根以外の条件は ≤、橋は < で違います。孤立点を削除して成分数が減る場合は関節点ではありません。両方の一覧が欲しい場合は low_link を使います。

## 計算量

時間 O(V+E)、追加メモリ O(V+E)。呼び出しごとに Lowlink を一度計算します。 E は論理辺数です。入力グラフの O(V+E) メモリは別途必要です。

## 使用する問題・検証・参考

[AOJ GRL_3_A: Articulation Points](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_3_A) / [ジャッジ形式](../../verify/graph_connectivity/articulation_points.test.cpp)。オンライン提出・AC を実施したという意味ではありません。

531 有向グラフ・760 無向多重グラフの全探索、600 ランダムグラフ、20 万頂点を含む [共通検証](README.md#検証) を用意しています。相互到達性、縮約 DAG の辺対応、辺／頂点削除、low と部分木、全 0/1 彩色を独立オラクルと照合します。

[原理の参考](https://cp-algorithms.com/graph/cutpoints.html) / [参考資料・設計判断](references.md)。
