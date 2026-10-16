---
title: 彩色数・最小彩色の復元
documentation_of: //graph_combinatorics/chromatic_number.hpp
---

# 彩色数・最小彩色の復元

隣接する頂点を異なる色に塗るための最小色数と、具体的な色分けを返します。色は0始まりの整数です。

[部分集合 DP と復元の図解](../../visualizations/graph_combinatorics/index.html#coloring)

## 原理

各部分集合が独立集合かを前計算します。dp[S] は S の最小色数。S の最小頂点 v を含む独立集合 I を一色として使うと、残りは dp[S∖I] で塗れます。I を全て比較し、最適な I を保存します。どの彩色にも v の色のグループがあるため、この制限で最適解を落としません。

空集合は0色、非空の独立集合は1色。それ以外は少なくとも2色必要なので、2色が見つかった状態では探索を終えられます。確率的な剰余の非零判定や、整数オーバーフローを使う判定には依存しません。

## usage

```cpp
#include <cassert>
#include "graph_combinatorics/chromatic_number.hpp"
int main() {
    cp::Graph<> g(5);
    for(int v=0;v<5;++v) g.add_edge(v,(v+1)%5);
    const auto result=cp::minimum_coloring(g);
    assert(result.chromatic_number==3);
    for(auto e:g.edges()) assert(result.color[e.from]!=result.color[e.to]);
    assert(cp::chromatic_number(cp::Graph<>(0))==0);
}
```

## API・条件・計算量

- `GraphColoringResult minimum_coloring(g)`：`chromatic_number`、頂点ごとの `color`、色ごとの頂点集合 `groups` を返します。groups[c] の頂点の色が c です。グループ内は昇順、空のグループはありません。
- `int chromatic_number(g)`：色数だけを返す入口。同じ DP と復元を実行します。

単純無向グラフ、N≤22、重みを無視。空なら色数0・空の color/groups。辺のない非空グラフなら1色。23頂点以上は release でも length_error。返された配列は値で、グラフの後の変更に追従しません。最小彩色が複数ある場合、特定の辞書順は保証しません。

時間 O(3^N+N+M)、領域 O(2^N+N+M)。主要配列は独立判定1バイト・色数1バイト・復元マスク4バイトを状態ごとに持つため、約 `6×2^N` バイトです。22頂点は表現・メモリの上限で、どんな入力でも高速という意味ではありません。

## 問題・検証

[Library Checker Chromatic Number](https://judge.yosupo.jp/problem/chromatic_number) の [ラッパー](../../verify/graph_combinatorics/chromatic_number.test.cpp)。[動く使用例](../../examples/graph_combinatorics/coloring.cpp)。

色数 k を小さい方から試し、全ての頂点への色割当を直接探索する独立オラクルと比較します。返された各色・groups の分割・辺の両端の色も検査します。空・奇閉路・完全グラフ・22頂点の境界を含みます。[カテゴリの検証](README.md#動く例問題検証) / [参考資料](references.md)
