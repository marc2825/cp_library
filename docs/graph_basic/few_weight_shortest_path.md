---
title: 少種類重みの最短路
documentation_of: //graph_basic/few_weight_shortest_path.hpp
---

# 少種類重みの最短路

[カテゴリ索引](README.md) / [G残り15項目](../graph_advanced/README.md) / [操作できる図解](../../visualizations/graph_advanced/index.html#few_weight_shortest_path) / [実装](../../graph_basic/few_weight_shortest_path.hpp) / [一次資料・設計判断](../graph_advanced/references.md)

非負重みがk種類しかない場合、重みごとのFIFOを使って最短路を計算します。重みの最大値には依存しません。

## usage

```cpp
#include <iostream>
#include "graph_basic/few_weight_shortest_path.hpp"
int main() {
    cp::Graph<long long> g(3,cp::GraphKind::Directed);
    g.add_edge(0,1,2); g.add_edge(1,2,2); g.add_edge(0,2,100);
    auto r=cp::few_weight_shortest_path(g,0);
    for(auto d:r.dist) std::cout << d << ' ';
    std::cout << '\n';
}
```

出力:

```text
0 2 4
```

## API・使用条件

`few_weight_shortest_path(g,source=0)`→既存 `ShortestPathResult<T>`。`dist`、親頂点/辺、`vertex_path` / `edge_path`を共有。

有向/無向、自己ループ・多重辺に対応。負値・NaNはinvalid_argument。不正始点はout_of_range。Tは比較・加算可能な既存距離型。既存 `distance_inf<T>()` 以上の距離は到達不能の番兵となるので、必要な距離がその未満に収まること。

## 仕組み

確定した頂点の距離は非減少なので、同じ重みを加えた候補も非減少です。各重みのdequeから古い候補を除き、先頭の最小を選びます。

## 計算量

重み分類O(m log(k+1))、探索O(m+kn)、領域O(n+m+k)。kが大きい場合は通常のDijkstraが適します。

## 使用する問題・検証

辺コストが「無料・通常・特別料金」など少種類で、料金の数値が大きい経路探索。Dial/Radix/0-1 BFSとは別の入力条件です。

0と10^12を含む500グラフをDijkstraと比較。 [実行できる使用例](../../examples/graph_basic/few_weight_shortest_path.cpp)も文書とコードの一致を検査します。

`make verify-graph-advanced` / `python3 verify/run_graph_advanced.py --sanitize` / `python3 -m tools.cp_tools.suite --stdlib-debug --runners graph_advanced`。[検証コード](../../verify/graph_advanced/)と[検証範囲](../graph_advanced/README.md#検証)を参照。オンライン提出ACは未確認です。図解の教材モデルとC++の違いは[図解の説明](../../visualizations/graph_advanced/README.md)に記載しています。
