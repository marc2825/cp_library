---
title: Priority Sum（上位・下位k個の和）
documentation_of: //static_query/priority_sum.hpp
---

# Priority Sum（上位・下位k個の和）

比較器順の先頭k個と残りを二つのmultisetに分け、選ばれた側の和を保持します。kの変更や重複値の一個削除に対応します。

## 使用例

```cpp
#include <iostream>
#include "static_query/priority_sum.hpp"
int main() {
    cp::PrioritySum<long long> s(2); s.insert(8); s.insert(1); s.insert(3);
    std::cout << s.sum() << "\n";
}
```

出力: `4`。

## APIと使用条件

`PrioritySum<T,Compare=less<T>>(k=0,comp)`、insert(x)、erase(x)はbool、set_k(k)、get_k/size/sum。

k>sizeなら全要素。greater<T>で大きいk個。Tは加減算と0からの構築が必要。既存のグローバルPrioritySumStructureは公開フィールド互換のため保持し、新しいcp版を追加しました。

## 計算量

挿入・削除O(log N)、set_kはO((|Δk|+1) log(N+1))、sum O(1)、領域O(N)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

順位付き多重集合、複数優先度キューの併合、上位k個の動的集計などで使います。std::multiset／priority_queueと操作列を比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#priority_sum) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#ordered) / [図解の簡略化](../../visualizations/b_1_6/README.md)
