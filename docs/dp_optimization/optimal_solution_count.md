---
documentation_of: //dp_optimization/optimal_solution_count.hpp
---

# 最適解の個数を返すDP

[P-10の対応表・検証](../dp_extensions/README.md) / [操作できる図解](../../visualizations/dp_extensions/index.html#optimal_solution_count) / [ヘッダ](../../dp_optimization/optimal_solution_count.hpp)

今回の仕様は**LISの添字列、LCSの一致添字pair列、0-1ナップサックの品物ID部分集合**を区別する個数です。値列が同じでも添字が違えば別解です。空解は1個。Countの既定はBigIntで、modintや整数型を明示可能です。整数型の個数加算は超過を例外化します。

`lis_count<Count>(vector, strict=true, compare={})` は`{length,count}`を返します。strict=falseなら同値を許容、比較順は任意のstrict weak ordering。座標圧縮＋FenwickでO(n log n)時間・O(n)領域。空列は{0,1}。

`lcs_alignment_count<Count>(a,b,equal={})` は同じ値でも異なる対応位置を別とする最長alignment数。たとえば"aa"と"a"なら長さ1・2通りです。通常のdistinct値列LCS数は1であり、このAPIとは異なります。上/左の最適解を足し、対角に含まれる重複分を除き、一致pairを末尾に加える解を足します。O(nm)時間・O(m)領域。Countは加減算に対応すること。

`zero_one_knapsack_count<Count>(capacity,items)` は`values`（既存の価値表）、重さちょうどの`counts`、`best()`（容量以下の最大価値と全重量にわたる達成解数）を返します。重さ0・価値0の品物は採用/不採用を別々に数えます。時間O(nW)、領域O(W)。負価値可、重さは非負。価値の固定幅演算超過は例外です。

用途: 最長列の達成方法数、最適選択の個数、合成数modでの数え上げ。添字部分集合の独立全探索と、多倍長が必要な解数を検証します。

## 使用例

```cpp
#include <iostream>
#include "dp_optimization/optimal_solution_count.hpp"
int main() {
    auto lis = cp::lis_count(std::vector<int>{1,2,2,3});
    auto lcs = cp::lcs_alignment_count(std::string("aa"), std::string("a"));
    auto knapsack = cp::zero_one_knapsack_count(2, {{1,1},{1,1},{2,2}}).best();
    std::cout << lis.length << ' ' << lis.count << '\n'
              << lcs.length << ' ' << lcs.count << '\n'
              << knapsack.first << ' ' << knapsack.second << '\n';
}
```

出力:

```text
3 2
1 2
2 2
```

## 検証・参考資料

`make verify-dp-extensions` で単独・重複include、Debug/NDEBUG、使用例、独立した素朴解との比較を実行します。詳細と実行記録は[14項目の検証](../dp_extensions/README.md#検証)、理論・既存実装との関係は[一次資料・設計判断](../dp_extensions/references.md)を参照。オンライン提出ACは未確認です。
