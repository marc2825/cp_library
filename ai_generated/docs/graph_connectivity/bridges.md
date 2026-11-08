---
title: 橋
documentation_of: //graph_connectivity/bridges.hpp
---

# 橋

削除すると連結成分数が増える辺の論理 ID を求めます。 [操作できる図解](../../visualizations/graph_connectivity/index.html#bridges)で入力と手順を変更できます。

## API・使用条件

```text
cp::bridges(g) -> vector<int>
```

無向グラフ用。非連結・空グラフ・自己ループ・多重辺に対応。結果は元の論理辺 ID の昇順です。 共通の [Graph](graph.md) を使い、結果は入力グラフと独立した値として返します。グラフを変更したら再実行してください。

## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_connectivity/bridges.hpp"
int main() {
    cp::Graph<> g(3);
    g.add_edge(0,1);g.add_edge(0,1);g.add_edge(1,2);
    auto ids=cp::bridges(g);
    assert(ids==(std::vector<int>{2}));
    auto edge=g.edges()[ids[0]];
    assert(edge.from==1 && edge.to==2);
}
```

## 原理・注意点

Lowlink の DFS 木辺 (p,v) で ord[p]<low[v] なら橋です。子の部分木から親以上へ戻る経路がないため、この辺を削除すると分断されます。

比較は厳密な < です。low[v]=ord[p] なら別の辺で親へ戻れるため橋ではありません。自己ループも橋にはなりません。橋と関節点の両方が必要なら low_link を一度だけ実行してください。

## 計算量

時間 O(V+E)、追加メモリ O(V+E)。呼び出しごとに Lowlink を一度計算します。 E は論理辺数です。入力グラフの O(V+E) メモリは別途必要です。

## 使用する問題・検証・参考

[AOJ GRL_3_B: Bridges](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_3_B) / [ジャッジ形式](../../verify/graph_connectivity/bridges.test.cpp)。オンライン提出・AC を実施したという意味ではありません。

531 有向グラフ・760 無向多重グラフの全探索、600 ランダムグラフ、20 万頂点を含む [共通検証](README.md#検証) を用意しています。相互到達性、縮約 DAG の辺対応、辺／頂点削除、low と部分木、全 0/1 彩色を独立オラクルと照合します。

[原理の参考](https://cp-algorithms.com/graph/bridge-searching.html) / [参考資料・設計判断](references.md)。
