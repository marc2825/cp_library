---
title: 分数二分探索
documentation_of: //basic_utility/fraction_binary_search.hpp
---

# 分数二分探索

Stern–Brocotの隣接分数を維持し、同じ方向への連続移動を指数探索と二分探索でまとめます。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/fraction_binary_search.hpp"
int main() {
    auto result=cp::fraction_binary_search(10,[](auto p,auto q){ return p*p<=2*q*q; });
    std::cout << result.first.numerator << "/" << result.first.denominator << "\n";
}
```

出力: `7/5`。

## APIと使用条件

`fraction_binary_search(limit,predicate(p,q))`は最大true分数と最小false分数のpair。分子と分母をそれぞれlimit以下に制限。limit>0。predicateは値p/qについてtrueからfalseへ単調。

0/1をtrue、1/0をfalseとする番兵契約。predicateへは有限の正の候補だけを渡します。片側に候補がなければ番兵を返します。limit=0はinvalid_argument。交差積を使うpredicateの溢れは利用者が避けます。

## 計算量

predicate呼出しO(log²(limit+1))の上界、O(1)作業領域。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

整数根、分母・分子制限付きの最良有理近似、非単調な離散費用の最小化などで使います。小さい全候補の列挙と64bit境界で確認します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#fraction_binary_search) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#search) / [図解の簡略化](../../visualizations/b_1_6/README.md)
