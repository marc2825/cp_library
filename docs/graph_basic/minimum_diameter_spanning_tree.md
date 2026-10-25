---
title: Minimum Diameter Spanning Tree
documentation_of: //graph_basic/minimum_diameter_spanning_tree.hpp
---

# Minimum Diameter Spanning Tree

[カテゴリ索引](README.md) / [G残り15項目](../graph_advanced/README.md) / [操作できる図解](../../visualizations/graph_advanced/index.html#minimum_diameter_spanning_tree) / [実装](../../graph_basic/minimum_diameter_spanning_tree.hpp) / [一次資料・設計判断](../graph_advanced/references.md)

辺重み付きグラフから、木の直径が最小になる全域木を復元します。

## usage

```cpp
#include <iostream>
#include "graph_basic/minimum_diameter_spanning_tree.hpp"
int main() {
    cp::Graph<int> g(3);
    g.add_edge(0,1,1); g.add_edge(1,2,1); g.add_edge(0,2,1);
    auto r=cp::minimum_diameter_spanning_tree(g);
    std::cout << cp::optimization_to_string(r->diameter) << ' ' << r->edge_ids.size() << '\n';
}
```

出力:

```text
2 2
```

## API・使用条件

`minimum_diameter_spanning_tree(g)`→optional<Result>。`edge_ids`が元辺ID、`diameter`が返却木の直径。中心は `center_vertex` または `center_edge` と元辺fromからの `offset_twice`。`radius_twice`は連続中心の半径の2倍。距離値は符号付き128bit。

非負64bit以下整数費用の無向グラフ。非連結はnullopt、空グラフは直径0/中心−1、1頂点は中心0。ループ・平行辺・0重み可。方向/負費用はinvalid_argument。半整数の中心を丸めません。

## 仕組み

全点対距離を計算し、各辺上の点xから頂点zへの距離 `min(d(u,z)+x,d(v,z)+w-x)` の上包絡を最小化します。切替点を整列しprefix/suffix最大値で各区間の最適点を求めます。中心からの二始点最短路木を作り、必要なら中心辺を加えます。

## 計算量

O(n³+mn log(n+1))時間、O(n²+m)領域。復元のDijkstra O((n+m)log(n+m+1))を含む。

## 使用する問題・検証

放送・配送ネットワークの最悪二地点間距離を小さくする全域木。最小重み全域木とは目的が異なります。

500小グラフの全域木を全列挙し最適直径と比較。0重み、非連結、平行辺、64bit最大費用も検証。 [実行できる使用例](../../examples/graph_basic/minimum_diameter_spanning_tree.cpp)も文書とコードの一致を検査します。

`make verify-graph-advanced` / `python3 verify/run_graph_advanced.py --sanitize` / `python3 -m tools.cp_tools.suite --stdlib-debug --runners graph_advanced`。[検証コード](../../verify/graph_advanced/)と[検証範囲](../graph_advanced/README.md#検証)を参照。オンライン提出ACは未確認です。図解の教材モデルとC++の違いは[図解の説明](../../visualizations/graph_advanced/README.md)に記載しています。
