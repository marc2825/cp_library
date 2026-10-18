---
title: 最大重み二部マッチング
documentation_of: //graph_flow/maximum_weight_bipartite_matching.hpp
---

# 最大重み二部マッチング

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#maximum_weight_bipartite_matching) / [実装](../../graph_flow/maximum_weight_bipartite_matching.hpp) / [一次資料と設計](../b_7_11/references.md)

未マッチの頂点を許し、辺重みの総和を最大にします。左頂点ごとの0費用dummy列を加えたAssignment Problemを共有します。

## usage

```cpp
#include <iostream>
#include "graph_flow/maximum_weight_bipartite_matching.hpp"
int main() {
    auto m=cp::maximum_weight_bipartite_matching(2,2,{{0,0,5},{0,1,8},{1,0,7},{1,1,-1}});
    std::cout << m.weight << '\n';
}
```

出力:

```text
15
```

## API・使用条件

`maximum_weight_bipartite_matching(L,R,vector<tuple<int,int,long long>> edges)`→{weight,pairs}。負辺・平行辺・左右サイズ違い可。同じ端点の平行辺は最大重みを採用。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

最大個数を優先する目的ではありません。全辺が負なら空マッチングの重み0。B項目は二部版として整備し、一般グラフの重み付きBlossomは計画Cの対象です。Assignmentの検査付き128bit中間値を使用し、結果はsigned64。

## 計算量

時間O(L²(R+L))、領域O(L(R+L))。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 未マッチの頂点を許し、辺重みの総和を最大にします。左頂点ごとの0費用dummy列を加えたAssignment Problemを共有します。 [実行できる小問題](../../examples/graph_flow/maximum_weight_bipartite_matching_b.cpp)。

最大重み二部マッチングを必要とする3.9の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
