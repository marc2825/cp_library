---
documentation_of: //dp_optimization/knapsack_reconstruction.hpp
---

# ナップサック・Bitset部分和の復元

[P-09の対応表・検証](../dp_extensions/README.md) / [操作できる図解](../../visualizations/dp_extensions/index.html#knapsack_reconstruction) / [ヘッダ](../../dp_optimization/knapsack_reconstruction.hpp)

`zero_one_knapsack_reconstruct`、`unbounded_knapsack_reconstruct`、`bounded_knapsack_reconstruct` は既存と同じcapacityと品物型を受け取ります。返値の`values`は既存KnapsackResult（重さちょうどの最大価値、未到達nullopt）、`restore(weight)`は品物ID順の採用個数列、未到達/範囲外はnullopt。`best_weight()`は容量以下の最大価値を達成する最小の重さです。

0-1は高々1個、完全は重さ正、個数制限はcount>=0。0-1/個数制限は重さ0・負価値も可。重さ0で価値正なら上限まで選び、価値0以下なら選びません。同じ最大価値なら後の品物の採用個数が少ない候補を保持します。

剰余類ごとの単調queueでO(nW)時間、**復元表O(nW)領域**。品物段階別の選択を保持するため、1次元DPの親を上書きして同じ品物を複数採用する問題がありません。固定幅の価値演算はoverflow_error。結果の公開表を変更してからrestoreすることは契約外です。

`BitsetSubsetSumReconstruction(max_sum)` は価値なしの0-1部分和です。`add(weight)`が品物IDを返し、`contains(sum)`、`restore(sum)`（品物ID列）を公開します。新しく立ったbitだけに親を保存し、時間O(n ceil((W+1)/64)+W)、領域O(n+W)、復元は選択品物数に比例。重さ0は登録するが復元に採用しません。sum=0は空解です。

用途: 価値最大化の選択肢出力、和が指定値になる品物の復元。全品物部分集合/個数列、重さ0・重複・負価値、大きい部分和、LLONG_MAX個の重さ0を検証します。

## 使用例

```cpp
#include <iostream>
#include "dp_optimization/knapsack_reconstruction.hpp"
int main() {
    auto result = cp::zero_one_knapsack_reconstruct(5, {{0,3},{2,4},{3,5},{2,4}});
    std::cout << result.values.best_value();
    auto counts = result.restore(result.best_weight());
    for (auto count : *counts) std::cout << ' ' << count;
    std::cout << '\n';
    cp::BitsetSubsetSumReconstruction subset(5);
    subset.add(2); subset.add(3); subset.add(2);
    auto ids = subset.restore(5);
    for (std::size_t i = 0; i < ids->size(); ++i) std::cout << (i ? " " : "") << (*ids)[i];
    std::cout << '\n';
}
```

出力:

```text
12 1 1 1 0
0 1
```

## 検証・参考資料

`make verify-dp-extensions` で単独・重複include、Debug/NDEBUG、使用例、独立した素朴解との比較を実行します。詳細と実行記録は[14項目の検証](../dp_extensions/README.md#検証)、理論・既存実装との関係は[一次資料・設計判断](../dp_extensions/references.md)を参照。オンライン提出ACは未確認です。
