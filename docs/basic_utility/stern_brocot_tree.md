---
title: Stern–Brocot Tree
documentation_of: //basic_utility/stern_brocot_tree.hpp
---

# Stern–Brocot Tree

正の既約分数と、根1/1からの左右の経路を相互変換します。連続するL/Rを回数でまとめ、Euclidの除算で一気に進めます。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/stern_brocot_tree.hpp"
int main() {
    auto f=cp::stern_brocot_fraction(cp::stern_brocot_path(12,8));
    std::cout << f.numerator << "/" << f.denominator << "\n";
}
```

出力: `3/2`。

## APIと使用条件

`stern_brocot_path(p,q)`はvector<pair<char,uint64_t>>。p,q>0、非既約も正規化。`stern_brocot_fraction(path)`はNonnegativeFraction{numerator,denominator}。

L/R以外・0分子/分母はinvalid_argument。復元中の境界または最終分数がuint64を超えるとoverflow_error。長さ0の経路は1/1。

## 計算量

経路のrun数O(log max(p,q))、復元O(run数)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

整数根、分母・分子制限付きの最良有理近似、非単調な離散費用の最小化などで使います。小さい全候補の列挙と64bit境界で確認します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#stern_brocot_tree) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#search) / [図解の簡略化](../../visualizations/b_1_6/README.md)
