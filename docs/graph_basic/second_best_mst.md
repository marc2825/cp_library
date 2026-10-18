---
title: 全域木の第二最小コスト
documentation_of: //graph_basic/second_best_mst.hpp
---

# 全域木の第二最小コスト

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#second_best_mst) / [実装](../../graph_basic/second_best_mst.hpp) / [一次資料と設計](../b_7_11/references.md)

感度解析の交換証明を使い、基準の最小全域木とは辺集合が異なる最安の全域木を復元します。

## usage

```cpp
#include <iostream>
#include "graph_basic/second_best_mst.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,1); g.add_edge(1,2,2); g.add_edge(0,2,4);
    std::cout << cp::second_best_spanning_tree(g,true)->cost << '\n';
}
```

出力:

```text
5
```

## API・使用条件

`second_best_spanning_tree(g,strict=false)` → `optional<MinimumSpanningForestResult<T>>`。strict=falseは同費用も許す別の木、trueは費用が厳密に大きい木。存在しない／非連結はnullopt。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

最大パス辺の交換で候補を作ります。同重みの場合、strict=trueではパスの二番目に大きい異なる重みも考慮します。木の費用だけでなく元グラフの辺IDも返します。

## 計算量

感度解析と同じ時間・領域。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 感度解析の交換証明を使い、基準の最小全域木とは辺集合が異なる最安の全域木を復元します。 [実行できる小問題](../../examples/graph_basic/second_best_mst_b.cpp)。

全域木の第二最小コストを必要とする3.7の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
