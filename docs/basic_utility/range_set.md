---
title: 連続区間集合の管理
documentation_of: //basic_utility/range_set.hpp
---

# 連続区間集合の管理

3.6のRangeSetを共有し、半開区間の和集合を動的に管理します。接している区間も統合します。

## 使用例

```cpp
#include <iostream>
#include "basic_utility/range_set.hpp"
int main() {
    cp::RangeSet<int> s; s.insert(1,5); s.erase(2,4);
    std::cout << s.interval_count() << "\n";
}
```

出力: `2`。

## APIと使用条件

`RangeSet<T>`、insert(l,r)/erase(l,r)/contains(x)/covers(l,r)/containing(x)/intervals()/interval_count()/clear。

l<=r。空区間を挿入・削除しても変化なし。座標の±1や番兵値を作らず、整数の端でも使えます。既存APIを再実装せずforward includeします。

## 計算量

K区間中t区間に触れる更新はO(log K+t)、contains/covers O(log K)、一覧O(K)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

静的／動的な区間集計、版の分岐、矩形更新などで使います。独立した配列・多重集合・全点走査と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#range_set) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#range) / [図解の簡略化](../../visualizations/b_1_6/README.md)
