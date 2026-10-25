---
title: Gomory–Hu Tree
documentation_of: //graph_flow/gomory_hu_tree.hpp
---

# Gomory–Hu Tree

無向容量グラフの全点対最小カット値をn−1本の木辺に圧縮します。u,v間の木パス上の最小重みが、元グラフのu-v最小カット値です。

[操作できる図解](../../visualizations/graph_extensions/index.html#gomory_hu_tree) / [15項目の検証](../graph_extensions/README.md) / [実装](../../graph_flow/gomory_hu_tree.hpp)

## usage

```cpp
#include <iostream>
#include "graph_flow/gomory_hu_tree.hpp"
#include <algorithm>
int main() {
    cp::Graph<long long> g(3);
    g.add_edge(0,1,2); g.add_edge(1,2,3); g.add_edge(0,2,4);
    auto tree=cp::gomory_hu_tree(g);
    std::vector<long long> cuts;
    for (auto e:tree.edges()) cuts.push_back(e.cost);
    std::sort(cuts.begin(),cuts.end());
    for (int i=0;i<int(cuts.size());++i) std::cout << cuts[i] << (i+1==int(cuts.size())?'\n':' ');
}
```

出力:

```text
5 6
```

## API・使用条件

`gomory_hu_tree(Graph<Cap>) -> Graph<Cap>`。木の頂点は元番号を維持。n=0/1では辺なし。非連結入力も容量0の木辺で接続します。木辺IDは新しい番号であり、元辺IDではありません。

Capはbool以外の64bit以下の整数。非負容量、無向、多重辺可。自己ループは無視。非loop容量の総和がCapに収まる必要があり、超過はoverflow_error。有向・負容量はinvalid_argument。カット値用の木であり、元辺カット集合が必要なら既存minimum_cutを実行します。

## 計算量

n−1回のDinic。一般上界O(V³E+V²)、作業領域O(V+E)。木をLCA等と組み合わせれば各対のパスminを高速化できます。

## 使用する問題・検証

ネットワークの全端点対の切断容量。全体最小カット1個だけを求めるStoer–Wagnerとは用途が異なります。

[動く使用例](../../examples/graph_flow/gomory_hu_tree.cpp)と[独立比較テスト](../../verify/graph_extensions/)を`make verify-graph-extensions`で実行します。小規模全探索、復元した頂点・辺ID、最適値、境界・不正入力を確認します。オンラインACは未確認です。

[一次資料・設計判断](../graph_extensions/references.md) / [カテゴリ索引](README.md)。
