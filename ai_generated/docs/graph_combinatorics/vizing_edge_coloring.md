---
title: 一般単純グラフのΔ+1辺彩色
documentation_of: //graph_combinatorics/vizing_edge_coloring.hpp
---

# 一般単純グラフのΔ+1辺彩色

同じ頂点に接する辺が同色にならないよう、最大次数Δ+1色以内で辺を彩色します。Misra–Griesのfan回転と2色の交互路反転を使います。

[操作できる図解](../../visualizations/graph_extensions/index.html#vizing_edge_coloring) / [15項目の検証](../graph_extensions/README.md) / [実装](../../graph_combinatorics/vizing_edge_coloring.hpp)

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/vizing_edge_coloring.hpp"
int main() {
    cp::Graph<int> g(3);
    g.add_edge(0,1); g.add_edge(1,2); g.add_edge(2,0);
    auto r=cp::vizing_edge_coloring(g);
    std::cout << r.max_degree << ' ' << r.color_count << '\n';
}
```

出力:

```text
2 3
```

## API・使用条件

`vizing_edge_coloring(g)`。`max_degree`、使用色上限の`color_count`、元辺IDごとの`color`を返します。色は[0,color_count)。辺なしならcolor_count=0。

一般の単純無向グラフ。自己ループ・平行辺・有向はinvalid_argument。最小色数を求めるAPIではありません。二部グラフのΔ色には既存bipartite_edge_coloringを使います。Simon LindholmのKACTL CC0実装を版固定して改変し、出典台帳に登録しています。

## 計算量

O(VE+V+E log(E+1))時間（単純性検査を含む）、O(V(Δ+1)+E)空間。

## 使用する問題・検証

各参加者が同時に1試合だけ参加する対戦スケジュール。一般グラフでΔかΔ+1かを最適判定する用途とは異なります。

[動く使用例](../../examples/graph_combinatorics/vizing_edge_coloring.cpp)と[独立比較テスト](../../verify/graph_extensions/)を`make verify-graph-extensions`で実行します。小規模全探索、復元した頂点・辺ID、最適値、境界・不正入力を確認します。オンラインACは未確認です。

[一次資料・設計判断](../graph_extensions/references.md) / [カテゴリ索引](README.md)。
