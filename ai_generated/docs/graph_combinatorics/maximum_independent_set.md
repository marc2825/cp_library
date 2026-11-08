---
title: 最大独立集合
documentation_of: //graph_combinatorics/maximum_independent_set.hpp
---

# 最大独立集合

互いに辺で結ばれていない頂点を最大数選びます。一般の単純無向グラフに対する厳密解です。

[元グラフと補グラフの図解](../../visualizations/graph_combinatorics/index.html#independent)

## 原理・usage

元グラフで隣接しない二頂点を結ぶ補グラフを作ると、独立集合がクリークになります。自分自身への辺を除き、[最大クリーク](maximum_clique.md) の探索を再利用します。頂点番号を変換する必要はありません。

```cpp
#include <cassert>
#include "graph_combinatorics/maximum_independent_set.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0,1); g.add_edge(1,2); g.add_edge(2,3);
    const auto vertices=cp::maximum_independent_set(g);
    assert(vertices.size()==2);
    for(auto e:g.edges()) {
        bool u=false,v=false;
        for(int x:vertices) { u|=x==e.from; v|=x==e.to; }
        assert(!(u&&v));
    }
}
```

## API・条件・計算量

`std::vector<int> maximum_independent_set(const Graph<T>& g)`。元の頂点番号の昇順。空グラフは空列、辺のないグラフは全頂点、完全グラフは一頂点（非空時）。同点解の辞書順最小は保証しません。

単純・無向、N≤64、重みは無視。65頂点以上は length_error。補グラフは64ビットの隣接集合として作り、元の Graph は変更しません。

前処理 O(N+M)、最悪 O(N²2^N) 時間、作業領域 O(N²+N+M)。元グラフが疎でも補グラフは密になる場合があり、入力サイズだけで速度は保証できません。

## 問題・検証

[Library Checker Maximum Independent Set](https://judge.yosupo.jp/problem/maximum_independent_set) の [ラッパー](../../verify/graph_combinatorics/maximum_independent_set.test.cpp) は個数と頂点列を出力します。[動く例](../../examples/graph_combinatorics/optimization.cpp)。

全頂点部分集合と比較し、最大サイズ・非隣接性・頂点の一意性・昇順・空入力・64頂点を確認します。図解の候補色は補グラフ上の探索の上界です。[カテゴリの検証](README.md#動く例問題検証) / [参考資料](references.md)
