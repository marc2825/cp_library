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

サイズが負なら`invalid_argument`、端点が範囲外なら`out_of_range`、最適値が signed 64bit に収まらなければ`overflow_error`です。

## 仕組みと境界

最大個数を優先する目的ではありません。全辺が負なら空マッチングの重み0。一般グラフの最大重みマッチングは[general_weighted_matching](general_weighted_matching.md)を使います。Assignmentの検査付き128bit中間値を使用し、結果はsigned64。

## 計算量

時間O(L²(R+L))、領域O(L(R+L))（辺数によらず L×(R+L) の密な表を作ります）。

## 使用する問題・検証

想定問題: 割り当てない作業者・仕事を許す、利益最大の割当（全員を割り当てるなら Assignment Problem を直接使います）。 [実行できる小問題](../../examples/graph_flow/maximum_weight_bipartite_matching_b.cpp)。

検証では、辺彩色と同じ1,200例（重み−7〜7、平行辺あり）で全マッチングの最大重みと比較し、`pairs` の端点が重複しないこと、辺が存在すること、平行辺の最大重みの和が `weight` に等しいことを確かめます。全辺が負のときの0と、範囲外の`out_of_range`も確認します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/flow_test.cpp)。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
