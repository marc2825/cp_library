---
title: 辺素・頂点素パス
documentation_of: //graph_combinatorics/disjoint_paths.hpp
---

# 辺素・頂点素パス

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#disjoint_paths) / [実装](../../graph_combinatorics/disjoint_paths.hpp) / [一次資料と設計](../b_7_11/references.md)

単位容量フローと頂点分割を使い、s→tのパスを復元します。頂点素版は両端以外の頂点を共有しません。

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/disjoint_paths.hpp"
int main() {
    cp::Graph<int> g(4); g.add_edge(0,1); g.add_edge(1,3); g.add_edge(0,2); g.add_edge(2,3);
    std::cout << cp::vertex_disjoint_paths(g,0,3).size() << '\n';
}
```

出力:

```text
2
```

## API・使用条件

`edge_disjoint_paths(g,s,t)`、`vertex_disjoint_paths(g,s,t)`、`disjoint_paths(g,s,t,vertex_disjoint=false)`→{paths,edge_ids,size()}。s!=t、有向/無向、多重辺可。重み無視。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

無向辺は両向きに流し、同じ論理辺の逆向き流量を相殺してから分解します。残る循環流はパス証明に不要なため捨てます。直接辺が複数あれば、内部頂点素版でもその本数のパスを許します。

## 計算量

Dinicの一般上界O(V²E)＋分解O(F(V+E))、領域O(V+E+返す経路長)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 単位容量フローと頂点分割を使い、s→tのパスを復元します。頂点素版は両端以外の頂点を共有しません。 [実行できる小問題](../../examples/graph_combinatorics/disjoint_paths_b.cpp)。

辺素・頂点素パスを必要とする3.10の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
