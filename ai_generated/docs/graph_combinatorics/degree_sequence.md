---
title: 次数列の判定と構成
documentation_of: //graph_combinatorics/degree_sequence.hpp
---

# 次数列の判定と構成

Erdős–Gallaiの不等式で単純無向グラフの次数列を判定し、Havel–Hakimiでラベルを保持した実現グラフを作ります。

[操作できる図解](../../visualizations/graph_extensions/index.html#degree_sequence) / [15項目の検証](../graph_extensions/README.md) / [実装](../../graph_combinatorics/degree_sequence.hpp)

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/degree_sequence.hpp"
int main() {
    auto g=cp::realize_degree_sequence({2,2,2,2});
    if (g) std::cout << g->edge_count() << '\n';
}
```

出力:

```text
4
```

## API・使用条件

`is_graphical_degree_sequence(vector<int>) -> bool`、`realize_degree_sequence(degree) -> optional<Graph<int>>`。頂点vの次数がdegree[v]。辺のcostは1。

自己ループ・多重辺なし。負次数・n以上の次数・奇数総和・実現不能はfalse/nullopt。空列は空グラフとして実現可能。入力は変更しません。同点で一意なグラフや連結性を要求しません。

## 計算量

判定O(n log n)、構成O((n+m) log(n+1))、空間O(n+m)。

## 使用する問題・検証

指定次数を持つ通信網・テストグラフの生成。連結・二部等の追加制約は別問題です。

[動く使用例](../../examples/graph_combinatorics/degree_sequence.cpp)と[独立比較テスト](../../verify/graph_extensions/)を`make verify-graph-extensions`で実行します。小規模全探索、復元した頂点・辺ID、最適値、境界・不正入力を確認します。オンラインACは未確認です。

[一次資料・設計判断](../graph_extensions/references.md) / [カテゴリ索引](README.md)。
