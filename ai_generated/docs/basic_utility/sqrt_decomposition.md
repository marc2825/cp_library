---
title: 平方分割テンプレート
documentation_of: //basic_utility/sqrt_decomposition.hpp
---

# 平方分割テンプレート

列を約√N個のブロックに分け、各ブロックにモノイドの積を保持します。境界だけ直接走査し、内部はブロックの積で飛ばします。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/sqrt_decomposition.hpp"
long long op(long long a,long long b) { return a+b; }
long long e() { return 0; }
int main() {
    cp::SqrtDecomposition<long long,op,e> s({1,2,3});
    s.set(1,7);
    std::cout << s.prod(0,3) << "\n";
}
```

出力: `11`。

## APIと使用条件

`SqrtDecomposition<S,op,e>(vector)`、size/get/set/prod(l,r)。opは結合的、eは両側単位元。

半開区間、空の積はe。非可換演算も列順に集約します。点更新用の基本テンプレートです。

## 計算量

構築O(N)、setとprodはO(√N)、get O(1)、領域O(N)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

静的／動的な区間集計、版の分岐、矩形更新などで使います。独立した配列・多重集合・全点走査と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#sqrt_decomposition) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#range) / [図解の簡略化](../../visualizations/b_1_6/README.md)
