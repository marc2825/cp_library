---
title: 最小費用の辺素・頂点素パス
documentation_of: //graph_combinatorics/min_cost_disjoint_paths.hpp
---

# 最小費用の辺素・頂点素パス

ちょうどk本のs-t単純パスを選び、総費用を最小化します。既存最小費用流の残余逆辺による経路の組み替えを共有し、得た流れを元辺ID付きパスへ分解します。

[操作できる図解](../../visualizations/graph_extensions/index.html#min_cost_disjoint_paths) / [15項目の検証](../graph_extensions/README.md) / [実装](../../graph_combinatorics/min_cost_disjoint_paths.hpp)

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/min_cost_disjoint_paths.hpp"
int main() {
    cp::Graph<long long> g(4,cp::GraphKind::Directed);
    g.add_edge(0,1,1); g.add_edge(1,3,2); g.add_edge(0,2,2); g.add_edge(2,3,3); g.add_edge(1,2,0);
    auto r=cp::min_cost_edge_disjoint_paths(g,0,3,2);
    auto v=cp::min_cost_vertex_disjoint_paths(g,0,3,2);
    std::cout << r->size() << ' ' << r->cost << ' ' << v->cost << '\n';
}
```

出力:

```text
2 8 8
```

## API・使用条件

`min_cost_edge_disjoint_paths(g,s,t,k)`、`min_cost_vertex_disjoint_paths(g,s,t,k)`、共通の`min_cost_disjoint_paths(g,s,t,k,vertex_disjoint=false)`。optional結果は既存DisjointPathsResultと同じ`paths/edge_ids/size()`に`cost`を加えた型。

非負の符号付き64bit以下整数費用、有向/無向、多重辺可。費用の合計はCost型に収まる必要があり、超過はoverflow_error。負費用・不正端点・s==t・k<0はinvalid_argument。k=0は空の成功、k本作れなければnullopt。頂点素は内部頂点のみでs,tを共有し、どちらの版も元辺を共有しません。無向の逆向きの流れは循環として相殺し、零費用循環を落として単純パスに分解します。loopは使用しません。

## 計算量

頂点分割後のV′=2V、E′=O(V+E)。非負初期費用ではO(k(V′+E′) log(V′+1))時間と分解O(k(V+E))、空間O(V+E+kV)。

## 使用する問題・検証

複数の安い独立経路、冗長通信・Suurballe/Bhandari型の用途。最大本数を求める既存disjoint_pathsとは目的が異なり、ここでは本数kを指定します。

[動く使用例](../../examples/graph_combinatorics/min_cost_disjoint_paths.cpp)と[独立比較テスト](../../verify/graph_extensions/)を`make verify-graph-extensions`で実行します。小規模全探索、復元した頂点・辺ID、最適値、境界・不正入力を確認します。オンラインACは未確認です。

[一次資料・設計判断](../graph_extensions/references.md) / [カテゴリ索引](README.md)。
