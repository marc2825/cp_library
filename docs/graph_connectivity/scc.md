---
title: SCC（強連結成分）
documentation_of: //graph_connectivity/scc.hpp
---

# SCC（強連結成分）

有向グラフを、互いに到達できる頂点の最大集合に分けます。 [操作できる図解](../../visualizations/graph_connectivity/index.html#scc)で入力と手順を変更できます。

## API・使用条件

```text
cp::scc(g) / cp::strongly_connected_components(g)
  -> StronglyConnectedComponentsResult
```

有向グラフ用。自己ループ・多重辺・非連結・空グラフに対応し、辺の重みは使いません。 共通の [Graph](graph.md) を使い、結果は入力グラフと独立した値として返します。グラフを変更したら再実行してください。

## 結果型

`component[v]` は所属する groups の添字、`groups[c]` はその成分の頂点一覧です。`size()` は成分数、`same(u,v)` は同じ成分かを返します。same には有効な頂点を指定します。各 groups の頂点は昇順です。ACL のトポロジカル順の契約を参考にし、再帰を避ける Kosaraju 法で実装しました。

[SCC の縮約 DAG](condensation.md) も利用できます。

## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_connectivity/scc.hpp"
int main() {
    cp::Graph<> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1);g.add_edge(1,0);g.add_edge(1,2);
    auto result=cp::scc(g);
    assert(result.size()==2 && result.same(0,1) && !result.same(0,2));
    assert(result.component[0]<result.component[2]);
    assert(result.groups[result.component[0]]==(std::vector<int>{0,1}));
}
```

## 原理・注意点

Kosaraju 法です。正方向 DFS の帰りがけ順を保存し、その逆順で逆グラフを探索します。二つの探索とも再帰を使いません。成分を確定した後、元の頂点番号順に走査して各 groups を作ります。

一方向に届くだけでは同じ SCC ではありません。異なる成分への辺 u→v では component[u]<component[v] を保証します。無関係な成分同士の順は保証しません。

## 計算量

時間 O(V+E)、追加メモリ O(V+E)、結果の保持 O(V)。size / same は O(1)。 E は論理辺数です。入力グラフの O(V+E) メモリは別途必要です。

## 使用する問題・検証・参考

[Library Checker: SCC](https://judge.yosupo.jp/problem/scc) / [ジャッジ形式](../../verify/graph_connectivity/scc.test.cpp)。SCC は [AOJ GRL_3_C の同成分問い合わせ](../../verify/graph_connectivity/scc_queries.test.cpp) もあります。オンライン提出・AC を実施したという意味ではありません。

531 有向グラフ・760 無向多重グラフの全探索、600 ランダムグラフ、20 万頂点を含む [共通検証](README.md#検証) を用意しています。相互到達性、縮約 DAG の辺対応、辺／頂点削除、low と部分木、全 0/1 彩色を独立オラクルと照合します。

[原理の参考](https://cp-algorithms.com/graph/strongly-connected-components.html) / [参考資料・設計判断](references.md)。
