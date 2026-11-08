---
title: 無向グラフの連結成分
documentation_of: //graph_connectivity/connected_components.hpp
---

# 無向グラフの連結成分

互いにつながっている頂点をまとめ、成分番号と頂点一覧を返します。 [操作できる図解](../../visualizations/graph_connectivity/index.html#components)で入力と手順を変更できます。

## API・使用条件

```text
cp::connected_components(g) -> ConnectedComponentsResult
```

無向グラフ用。空グラフ・自己ループ・多重辺・孤立点に対応し、重みを使いません。 共通の [Graph](graph.md) を使い、結果は入力グラフと独立した値として返します。グラフを変更したら再実行してください。

## 結果型

`component[v]` は所属 groups の添字、`groups[c]` はその成分の頂点一覧、`size()` は成分数、`same(u,v)` は同じ成分かです。same は有効な頂点同士を指定してください。図解は探索の考え方を BFS で示します。

## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_connectivity/connected_components.hpp"
int main() {
    cp::Graph<> g(5);g.add_edge(0,1);g.add_edge(1,2);g.add_edge(3,4);
    auto result=cp::connected_components(g);
    assert(result.size()==2 && result.same(0,2) && !result.same(0,4));
    assert(result.groups[result.component[4]].size()==2);
}
```

## 原理・注意点

未訪問の頂点を根にスタック探索し、到達できる頂点へ同じ番号を付けます。全頂点へ繰り返して連結成分を作ります。

有向グラフの SCC とは区別してください。groups 内の頂点の順序は保証しません。空グラフは size()==0、component と groups は空です。

## 計算量

時間 O(V+E)、追加メモリ O(V)。size / same は O(1)。 E は論理辺数です。入力グラフの O(V+E) メモリは別途必要です。

## 使用する問題・検証・参考

[AOJ ALDS1_11_D: Connected Components](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_11_D) / [使用例](../../examples/graph_connectivity/components_and_bipartite.cpp)。オンライン提出・AC を実施したという意味ではありません。

531 有向グラフ・760 無向多重グラフの全探索、600 ランダムグラフ、20 万頂点を含む [共通検証](README.md#検証) を用意しています。相互到達性、縮約 DAG の辺対応、辺／頂点削除、low と部分木、全 0/1 彩色を独立オラクルと照合します。

[原理の参考](https://cp-algorithms.com/graph/search-for-connected-components.html) / [参考資料・設計判断](references.md)。
