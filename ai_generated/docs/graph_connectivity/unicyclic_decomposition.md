---
title: 無向なもり分解
documentation_of: //graph_connectivity/unicyclic_decomposition.hpp
---

# 無向なもり分解

連結で閉路がちょうど1個の無向グラフを、その閉路と各閉路頂点を根とする木に分けます。既存Cactus分解を共有します。

[操作できる図解](../../visualizations/graph_extensions/index.html#unicyclic_decomposition) / [15項目の検証](../graph_extensions/README.md) / [実装](../../graph_connectivity/unicyclic_decomposition.hpp)

## usage

```cpp
#include <iostream>
#include "graph_connectivity/unicyclic_decomposition.hpp"
int main() {
    cp::Graph<int> g(5);
    g.add_edge(0,1); g.add_edge(1,2); g.add_edge(2,0); g.add_edge(1,3); g.add_edge(3,4);
    auto r=cp::unicyclic_decomposition(g);
    if (r) std::cout << r->cycle_edge_ids.size() << ' ' << r->root[4] << ' ' << r->depth[4] << '\n';
}
```

出力:

```text
3 1 2
```

## API・使用条件

`unicyclic_decomposition(g)`。結果は閉じた`cycle`、`cycle_edge_ids`、各頂点の`root/parent/parent_edge/depth`、閉路順の`trees`。treesは根自身を含み、全頂点を重複なく分割します。

無向・連結・n=m・n≥1が対象。条件を満たさない形はnullopt、有向はinvalid_argument。自己ループを長さ1、平行2辺を長さ2の閉路として許可。閉路根のparent/parent_edgeは−1、depthは0。cycleは先頭を末尾に繰り返します。

## 計算量

O(V+E)時間・空間。

## 使用する問題・検証

なもり上の経路数、木部分のDPと環部分のDPへの分離。Functional Graphとは入力の向き・意味が異なります。

[動く使用例](../../examples/graph_connectivity/unicyclic_decomposition.cpp)と[独立比較テスト](../../verify/graph_extensions/)を`make verify-graph-extensions`で実行します。小規模全探索、復元した頂点・辺ID、最適値、境界・不正入力を確認します。オンラインACは未確認です。

[一次資料・設計判断](../graph_extensions/references.md) / [カテゴリ索引](README.md)。
