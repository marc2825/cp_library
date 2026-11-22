---
title: グラフ同型の簡易判定
documentation_of: //graph_combinatorics/graph_isomorphism_filter.hpp
---

# グラフ同型の簡易判定

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#graph_isomorphism_filter) / [実装](../../graph_combinatorics/graph_isomorphism_filter.hpp) / [一次資料と設計](../b_7_11/references.md)

2グラフを同時に1-WL色精緻化し、非同型の証拠または一意な色対応による同型写像を返します。

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/graph_isomorphism_filter.hpp"
int main() {
    cp::Graph<int> a(3),b(3); a.add_edge(0,1); a.add_edge(1,2); b.add_edge(2,0); b.add_edge(0,1);
    auto r=cp::graph_isomorphism_filter(a,b);
    std::cout << (r.status!=cp::IsomorphismStatus::different) << '\n';
}
```

出力:

```text
1
```

## API・使用条件

`graph_isomorphism_filter(a,b)`→{status,mapping}。statusはIsomorphismStatus::{different,inconclusive,isomorphic}。重みは無視し、有向性・ループ・辺の重複数を区別。

isomorphic のとき mapping[a の頂点]=b の頂点。different・inconclusive では mapping は空です。

## 仕組みと境界

色の個数が一致しても同型とは限りません。安定色が一意になった場合だけisomorphic、残る場合はinconclusive。6-cycleと三角形2個は未確定となる例です。空同士は同型。未確定の対を厳密に判定し対応を得るには[一般グラフの厳密同型判定](graph_isomorphism.md)を使います。

## 計算量

保守的上界O(V²(V+E) log(V+E+1))、領域O(V+E)。キー列比較の費用を含む。 Vは頂点数、Eは論理辺数。

## 使用する問題・検証

想定問題: 多数のグラフ対から非同型な対を安価に除外する前処理（inconclusive の対だけを厳密判定に回す）。 [実行できる小問題](../../examples/graph_combinatorics/graph_isomorphism_filter_b.cpp)。

検証では、8頂点未満のランダム単純グラフ280個とその頂点置換で different にならないこと、isomorphic なら mapping が隣接を保つことを確かめ、6閉路と三角形2個が inconclusive になることも確認します。さらに `verify/graph_advanced/structure_test.cpp` で、7頂点未満の有向・無向多重グラフ300対を全順列と比較し、different・isomorphic の判定と mapping の正しさを確かめます。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/combinatorics_test.cpp)。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
