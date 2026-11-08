---
title: 整数k乗根
documentation_of: //basic_utility/integer_kth_root.hpp
---

# 整数k乗根

浮動小数点を使わず、k乗がn以下になる最大の整数を二分探索します。乗算の前にn/xと比較するため、uint64の最大値を扱えます。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/integer_kth_root.hpp"
int main() {
    std::cout << cp::integer_kth_root(1000,3) << "\n";
}
```

出力: `10`。

## APIと使用条件

`integer_kth_root(uint64_t n,unsigned k)`。k>0、k=0はinvalid_argument。n=0なら0。

k>=64かつn>0なら結果は1。返値xはx^k<=n<(x+1)^kを数学的整数で満たします。

## 計算量

O(min(k,64) log(n+1))時間、O(1)領域。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

整数根、分母・分子制限付きの最良有理近似、非単調な離散費用の最小化などで使います。小さい全候補の列挙と64bit境界で確認します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#integer_kth_root) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#search) / [図解の簡略化](../../visualizations/b_1_6/README.md)
