---
title: 非単調関数の探索
documentation_of: //basic_utility/nonmonotone_search.hpp
---

# 非単調関数の探索

有限整数区間を分割し、区間の下界が暫定解を改善しない場合に枝刈りする分枝限定法です。単峰性も単調性も要求しません。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/nonmonotone_search.hpp"
int main() {
    auto best=cp::nonmonotone_argmin<int>(0,8,[](auto i){ return int((i^3)*2); },[](auto,auto){ return 0; });
    std::cout << best->first << "\n";
}
```

出力: `3`。

## APIと使用条件

`nonmonotone_argmin<Value>(uint64_t l,r,f,lower_bound)`。返値optional<pair<位置,値>>。lower_bound(a,b)は区間[a,b)の真の最小値以下であること。

空区間はnullopt、同値は最左。正しい下界は呼出側の責任。利用できる情報なしに任意の非単調関数を少数評価で解く保証はありません。

## 計算量

長さNに対して最悪O(N log N)のqueue処理とO(N)回の関数／下界評価、O(N)領域。枝刈りで減る場合があります。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

整数根、分母・分子制限付きの最良有理近似、非単調な離散費用の最小化などで使います。小さい全候補の列挙と64bit境界で確認します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#nonmonotone_search) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#search) / [図解の簡略化](../../visualizations/b_1_6/README.md)
