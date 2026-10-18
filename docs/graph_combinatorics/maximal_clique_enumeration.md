---
title: 極大クリーク列挙
documentation_of: //graph_combinatorics/maximal_clique_enumeration.hpp
---

# 極大クリーク列挙

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#maximal_clique_enumeration) / [実装](../../graph_combinatorics/maximal_clique_enumeration.hpp) / [一次資料と設計](../b_7_11/references.md)

既存のpivot付きBron–Kerboschを共通入口から使用します。「これ以上頂点を足せない」クリークをそれぞれ一度列挙します。

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/maximal_clique_enumeration.hpp"
int main() {
    cp::Graph<int> g(3); g.add_edge(0,1); g.add_edge(1,2);
    int count=0; cp::enumerate_maximal_cliques(g,[&](const auto&){++count; return true;});
    std::cout << count << '\n';
}
```

出力:

```text
2
```

## API・使用条件

`enumerate_maximal_cliques(g,emit,include_empty=false)`。単純無向V<=64。emit(const vector<int>&)がfalseを返すと全体もfalseで中断。保存するならvectorをコピー。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

最大サイズだけの列挙ではありません。空グラフでinclude_empty=trueなら空集合を一度通知します。既存clique_enumeration.hppの実体を共有し、二重実装しません。

## 計算量

最悪O(V² 3^(V/3))の保守的上界＋callback、補助領域O(V²)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 既存のpivot付きBron–Kerboschを共通入口から使用します。「これ以上頂点を足せない」クリークをそれぞれ一度列挙します。 [実行できる小問題](../../examples/graph_combinatorics/maximal_clique_enumeration_b.cpp)。

極大クリーク列挙を必要とする3.10の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
