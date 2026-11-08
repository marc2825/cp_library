---
documentation_of: //dp_optimization/small_weight_knapsack.hpp
---

# 重さの種類が少ない高速ナップサック

[P-23の対応表・検証](../dp_extensions/README.md) / [操作できる図解](../../visualizations/dp_extensions/index.html#small_weight_knapsack) / [ヘッダ](../../dp_optimization/small_weight_knapsack.hpp)

`small_weight_knapsack(capacity, items)` は0-1ナップサックの既存`KnapsackResult`を返します。各品物IDは高々1回、重さは非負int、価値はlong long（負も可）。`dp[w]`は重さちょうどの最大価値、未到達nullopt、`best_value()`は容量以下の最大価値です。品物復元はP-09のAPIを使います。

同じ正の重さを持つ品物の価値を降順に並べ、上位t個の価値prefixを作ります。このprefixは離散凹なので、重さの剰余ごとのDPとのmax-plus積の最適位置は単調です。分割統治で各行の候補範囲を狭めます。到達不能の列と候補がない行を除き、全て-infinityの行から不正な単調境界を作らないようにします。

容量をW、有効な正の重さの種類数をD、品物数をnとして時間O(n log n + D W log(W+1))、領域O(n+W)。同じ重さの品物が多いときに通常O(nW)より有効です。Dが多いと通常版より速い保証はありません。重さ0の正価値だけは最初に合計して加えます。

和やprefixのlong long超過はoverflow_error、負容量/負重さはinvalid_argument。容量を超える品物は使いません。用途: 小さい種類の重量で大量の品物を選ぶ問題。同じ重さ10万品物、負価値・到達不能・重さ0、全探索と通常DPの比較を検証します。

## 使用例

```cpp
#include <iostream>
#include "dp_optimization/small_weight_knapsack.hpp"
int main() {
    auto result = cp::small_weight_knapsack(7, {{2,5},{2,4},{2,1},{3,7},{3,6},{0,2}});
    std::cout << result.best_value() << '\n';
}
```

出力:

```text
18
```

## 検証・参考資料

`make verify-dp-extensions` で単独・重複include、Debug/NDEBUG、使用例、独立した素朴解との比較を実行します。詳細と実行記録は[14項目の検証](../dp_extensions/README.md#検証)、理論・既存実装との関係は[一次資料・設計判断](../dp_extensions/references.md)を参照。オンライン提出ACは未確認です。
