---
title: Euler Tour・部分木の区間化
documentation_of: //tree/euler_tour.hpp
---

# Euler Tour・部分木の区間化

DFS の行きがけ列と、親に戻るたびに頂点を加える往復列を作ります。部分木クエリでは preorder、LCA の RMQ では往復の walk を使います。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/tree/index.html#euler) / [実装](../../tree/euler_tour.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "tree/euler_tour.hpp"
#include "range_query/fenwick_tree.hpp"
int main() {
    cp::Graph<> g(4);
    g.add_edge(0, 1); g.add_edge(1, 2); g.add_edge(1, 3);
    cp::EulerTour<> tour(g);
    cp::FenwickTree<long long> bit(4);
    for (int v = 0; v < 4; ++v) bit.add(tour.tree.tin[v], v + 1);
    auto [l, r] = tour.tree.subtree_interval(1);
    std::cout << bit.sum(l, r) << '\n';
    for (int v : tour.walk) std::cout << v << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/tree/euler_tour.cpp)。リポジトリのルートを include パスに指定します。

## API・使用条件

| フィールド | 意味 |
|---|---|
| `EulerTour<T>(g,root=0)` | 無向の木・森を前計算 |
| `tree` | RootedTree。`tree.preorder`, `tree.subtree_interval(v)` が部分木クエリ用 |
| `walk` | 頂点の往復列。森は成分ごとの列を連結 |
| `first[v]` / `last[v]` | walk 内の最初／最後の出現位置。last は含む位置 |
| `edge_walk` | 辺IDの往復列。各辺を2回ずつ記録 |
| `edge_in[id]` / `edge_out[id]` | edge_walk 内の往路／復路の位置 |

[RootedTree の入力・算術条件](rooted_tree.md)を引き継ぎます。空では全列が空。C 成分で walk は 2N−C 項、edge_walk は 2E 項。成分の境目の連続する頂点間に辺はありません。構築時のスナップショットです。

## 原理・計算量・注意点

構築 O(N+E)、空間 O(N)。first/last・区間の参照は O(1)。

頂点 v の値を `a[tin[v]]` に載せると、部分木和は `[tin[v],tout[v])` の区間和になります。Fenwick を併用すれば一点加算／部分木和を O(log N) で処理できます。

同一成分の u,v について、walk の first[u]〜first[v]（両端含む）で深さ最小の頂点が LCA です。Sparse Table 等の RMQ は利用者が別に構築します。last は walk 上の最後の出現であり、preorder 上の排他的な tout とは異なります。往復列を足すと同じ頂点を重複して数えるので、部分木和には使いません。

## 使用する問題・検証

[Vertex Add Subtree Sum](https://judge.yosupo.jp/problem/vertex_add_subtree_sum)、[Lowest Common Ancestor](https://judge.yosupo.jp/problem/lca)。

再帰 DFS の往復列・全 first/last・辺の入退位置と比較。walk の深さRMQを全頂点対のLCAと比較。根変更・森・空・20万頂点。

共通の [検証入口](../../verify/run_tree.py) は C++17・警告エラー・debug/release・単独 include・複数翻訳単位・使用例・独立オラクル・ジャッジ入出力を確認します。`--sanitize` で ASan/UBSan。**オンライン AC は未確認**です。
