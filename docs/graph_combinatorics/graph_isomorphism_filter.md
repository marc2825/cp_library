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

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

色の個数が一致しても同型とは限りません。安定色が一意になった場合だけisomorphic、残る場合はinconclusive。6-cycleと三角形2個は未確定となる例です。空同士は同型。全探索による一般グラフ同型は計画Cに残ります。

## 計算量

保守的上界O(V²(V+E) log(V+E+1))、領域O(V+E)。キー列比較の費用を含む。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 2グラフを同時に1-WL色精緻化し、非同型の証拠または一意な色対応による同型写像を返します。 [実行できる小問題](../../examples/graph_combinatorics/graph_isomorphism_filter_b.cpp)。

グラフ同型の簡易判定を必要とする3.10の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
