---
title: 三角形列挙
documentation_of: //graph_combinatorics/triangle_enumeration.hpp
---

# 三角形列挙

単純無向グラフの三角形を一度ずつ列挙します。頂点に値が付いた三角形の積の和、局所構造の数え上げ等で使います。

[次数による向きとマークの図解](../../visualizations/graph_combinatorics/index.html#triangles)

## 原理

辺を `(次数, 頂点番号)` の昇順へ向けると、各頂点の出次数を O(√M) に抑えられます。u の先をマークし、u→v→w という二辺の先 w がマークされていれば三角形です。三角形の最小の頂点からだけ見つかるので重複しません。比較に次数を使うことが計算量の鍵で、頂点番号だけで向ける方式とは異なります。

## usage

```cpp
#include <cassert>
#include "graph_combinatorics/triangle_enumeration.hpp"
int main() {
    cp::Graph<> g(3);
    g.add_edge(0,1); g.add_edge(1,2); g.add_edge(2,0);
    int count=0;
    bool completed=cp::enumerate_triangles(g,[&](int u,int v,int w) {
        assert(u==0 && v==1 && w==2);
        ++count; return true;
    });
    assert(completed && count==1);
}
```

## API・条件・計算量

`bool enumerate_triangles(const Graph<T>& g, Emit emit)`。コールバックは `bool(int u,int v,int w)` で、常に `u<v<w`。全体の出力順は保証しません。true を返すと継続し、false で直ちに打ち切ります。関数自身の true は全探索完了、false はコールバックによる打切りです。最後の一件で false を返した場合も、戻り値は false です。

単純・無向が条件です。T の重み演算は不要で、重みは無視します。空・非連結・孤立点は可。64頂点の制限はありません。グラフを変更しないでください。準備を含む O(N+M√M) 時間・O(N+M) 記憶。全ての三角形を保存する場合の出力領域とコールバック処理は別です。

## 問題・検証

[Library Checker Enumerate Triangles](https://judge.yosupo.jp/problem/enumerate_triangles) の [ラッパー](../../verify/graph_combinatorics/enumerate_triangles.test.cpp) は、各三角形の頂点値の積を足します。[動く例](../../examples/graph_combinatorics/enumeration.cpp) も用意しています。

全頂点三つ組との比較、出力の一意性・昇順・早期打切り、20万頂点の星で検証します。[カテゴリの検証](README.md#動く例問題検証) / [参考資料](references.md)
