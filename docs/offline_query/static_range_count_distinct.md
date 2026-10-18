---
title: Static Range Count Distinct
documentation_of: //offline_query/static_range_count_distinct.hpp
---

# Static Range Count Distinct

問い合わせを右端順に並べ、各値の最後の出現位置だけをBITへ1として残します。区間和はその区間に現れる値の種類数になります。

## 使用例

```cpp
#include <iostream>
#include "offline_query/static_range_count_distinct.hpp"
int main() {
    auto a=cp::static_range_count_distinct<int>({1,2,1,3},{{0,4},{1,3},{2,2}});
    std::cout << a[0] << " " << a[1] << " " << a[2] << "\n";
}
```

出力: `3 2 0`。

## APIと使用条件

`static_range_count_distinct(vector<T>,vector<pair<int,int>> queries,Compare)`は登録順のvector<int>。

静的配列。区間は[0,N)内、空は0。比較器は厳密弱順序で、同値は両方向false。値の等値演算子は不要。

## 計算量

時間O((N+Q) log(N+Q+1))、領域O(N+Q)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

[Library Checker: Static Range Count Distinct](https://judge.yosupo.jp/problem/static_range_count_distinct)。各区間でsetを作る解と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#static_range_count_distinct) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#spatial) / [図解の簡略化](../../visualizations/b_1_6/README.md)
