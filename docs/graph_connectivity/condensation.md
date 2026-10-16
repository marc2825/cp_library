---
title: SCC の縮約 DAG
documentation_of: //graph_connectivity/condensation.hpp
---

# SCC の縮約 DAG

各強連結成分を一つの頂点に縮約し、成分間の有向グラフを作ります。 [操作できる図解](../../visualizations/graph_connectivity/index.html#condensation)で入力と手順を変更できます。

## API・使用条件

```text
cp::condensation_graph(g,deduplicate=true) -> SCCCondensationResult
```

有向グラフ用。SCC を内部で計算します。空グラフ・自己ループ・多重辺に対応し、重みは使いません。 共通の [Graph](graph.md) を使い、結果は入力グラフと独立した値として返します。グラフを変更したら再実行してください。

## 結果型

- `scc`: [SCC の結果](scc.md)。`component[v]` で元の頂点を縮約頂点へ変換します。
- `dag`: `Graph<int>` の有向 DAG。頂点数は成分数、縮約辺の重みは 1。
- `original_edge[new_edge_id]`: その縮約辺を作る元の辺 ID を一つ。

成分はトポロジカル順なので、全ての縮約辺で from<to です。代表辺は「成分 ID 順→各成分内の頂点番号順→隣接辺の登録順」で最初に現れたものです。全元辺が必要なら deduplicate=false を使います。

## usage

```cpp
#include <cassert>
#include <vector>
#include "graph_connectivity/condensation.hpp"
int main() {
    cp::Graph<> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1);g.add_edge(1,0);g.add_edge(0,2);g.add_edge(1,2);
    auto result=cp::condensation_graph(g);
    assert(result.scc.size()==2 && result.dag.directed());
    assert(result.dag.edge_count()==1 && result.original_edge.size()==1);
    assert(result.dag.edges()[0].from<result.dag.edges()[0].to);
    assert(cp::condensation_graph(g,false).dag.edge_count()==2);
}
```

## 原理・注意点

SCC 内部の辺を除き、各成分の頂点の出辺を調べます。重複をまとめる場合は、行先を現在の始点成分 ID でマークします。ソートやハッシュなしで線形時間に構築します。

縮約辺のコストは 1 です。original_edge は代表元辺を一つ返し、最小 ID や最小重み・重みの合計は保証しません。deduplicate=false なら元の成分間の平行辺を全て残します。

## 計算量

時間・追加メモリ O(V+E)。結果には SCC の全頂点と縮約 DAG、元辺への対応を保持します。 E は論理辺数です。入力グラフの O(V+E) メモリは別途必要です。

## 使用する問題・検証・参考

[Library Checker: SCC](https://judge.yosupo.jp/problem/scc) / [ジャッジ形式](../../verify/graph_connectivity/scc.test.cpp)。SCC は [AOJ GRL_3_C の同成分問い合わせ](../../verify/graph_connectivity/scc_queries.test.cpp) もあります。オンライン提出・AC を実施したという意味ではありません。

531 有向グラフ・760 無向多重グラフの全探索、600 ランダムグラフ、20 万頂点を含む [共通検証](README.md#検証) を用意しています。相互到達性、縮約 DAG の辺対応、辺／頂点削除、low と部分木、全 0/1 彩色を独立オラクルと照合します。

[原理の参考](https://cp-algorithms.com/graph/strongly-connected-components.html) / [参考資料・設計判断](references.md)。
