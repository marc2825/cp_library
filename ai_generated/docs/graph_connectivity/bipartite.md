---
title: 二部グラフ判定・彩色
documentation_of: //graph_connectivity/bipartite.hpp
---

# 二部グラフ判定・彩色

全ての辺が異なる色の頂点を結ぶよう、頂点を 0/1 に塗り分けます。 [操作できる図解](../../visualizations/graph_connectivity/index.html#bipartite)で入力と手順を変更できます。

## API・使用条件

```text
cp::bipartite_coloring(g) -> optional<vector<int>>
```

無向グラフ用。全連結成分を調べます。成功は各頂点の 0/1 の色、失敗は nullopt。空グラフは成功した空 vector です。 共通の [Graph](graph.md) を使い、結果は入力グラフと独立した値として返します。グラフを変更したら再実行してください。

## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_connectivity/bipartite.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0,1);g.add_edge(1,2);g.add_edge(2,3);
    auto color=cp::bipartite_coloring(g);
    assert(color && (*color)[0]!=(*color)[1]);
    g.add_edge(0,2);
    assert(!cp::bipartite_coloring(g));
}
```

## 原理・注意点

各未訪問の根を 0 として BFS し、隣接先に反対の色を付けます。既に同じ色の隣接先があれば失敗します。

自己ループは失敗、多重辺だけでは失敗しません。非連結な成分の色は独立に反転でき、孤立点は 0 になります。C++ の失敗時には途中の色や奇閉路を返しません。図解では説明用に衝突辺を表示します。

## 計算量

時間 O(V+E)、追加メモリ O(V)。 E は論理辺数です。入力グラフの O(V+E) メモリは別途必要です。

## 使用する問題・検証・参考

[AtCoder ABC282 D: Make Bipartite 2](https://atcoder.jp/contests/abc282/tasks/abc282_d) / [ジャッジ形式](../../verify/graph_connectivity/bipartite.test.cpp)。オンライン提出・AC を実施したという意味ではありません。

531 有向グラフ・760 無向多重グラフの全探索、600 ランダムグラフ、20 万頂点を含む [共通検証](README.md#検証) を用意しています。相互到達性、縮約 DAG の辺対応、辺／頂点削除、low と部分木、全 0/1 彩色を独立オラクルと照合します。

[原理の参考](https://cp-algorithms.com/graph/bipartite-check.html) / [参考資料・設計判断](references.md)。
