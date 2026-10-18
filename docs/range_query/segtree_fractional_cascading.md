---
title: Segtree with Fractional Cascading
documentation_of: //range_query/segtree_fractional_cascading.hpp
---

# Segtree with Fractional Cascading

各節点にソート列を持つMerge Sort Treeで、親prefixから左右の子prefixへの対応を保持します。根だけで二分探索し、以降の検索を定数時間で子へ伝えます。

## 使用例

```cpp
#include <iostream>
#include "range_query/segtree_fractional_cascading.hpp"
int main() {
    cp::SegtreeFractionalCascading<int> s({4,1,4,2});
    std::cout << s.range_freq(0,4,2,5) << "\n";
}
```

出力: `3`。

## APIと使用条件

`SegtreeFractionalCascading<T>(vector)`、range_freq(l,r,lower,upper)、size。

静的配列。位置区間[l,r)、値区間[lower,upper)。値域が空・逆向きなら0。比較は<のみ、重複も数えます。

## 計算量

構築・領域O(N log(N+1))、検索O(log(N+1))。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

静的／動的な区間集計、版の分岐、矩形更新などで使います。独立した配列・多重集合・全点走査と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#segtree_fractional_cascading) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#range) / [図解の簡略化](../../visualizations/b_1_6/README.md)
