---
title: Lazy Balanced BST（共通入口）
documentation_of: //ordered_structure/lazy_balanced_bst.hpp
---

# Lazy Balanced BST（共通入口）

既存のLazyReversibleTreapを共有します。暗黙の添字をkeyとして挿入・削除・反転・区間作用・非可換な集約をまとめます。

## 使用例

```cpp
#include <iostream>
#include "ordered_structure/lazy_balanced_bst.hpp"
#include "range_query/range_actions.hpp"
using A=cp::AffineSumAction<long long>;
int main() {
    cp::LazyBalancedBST<A::S,A::op,A::e,A::F,A::mapping,A::composition,A::id> s(A::values({1,2,3})); s.apply(0,3,{2,1}); s.reverse(0,3);
    std::cout << s.get(0).sum << "\n";
}
```

出力: `7`。

## APIと使用条件

`LazyBalancedBST<S,op,e,F,mapping,composition,id>`はLazyReversibleTreapの別名。insert/erase/set/get/prod/all_prod/apply/reverse/move/to_vector/size。

composition(f,g)はgの後にf。mappingはモノイド準同型、反転と可換。要素と集約の長さはSに含めます。既存のAPI・乱数seed・節点所有をそのまま利用。

## 計算量

期待O(log N)/操作、通算挿入数の領域。列構築はO(N log(N+1))。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

順位付き多重集合、複数優先度キューの併合、上位k個の動的集計などで使います。std::multiset／priority_queueと操作列を比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#lazy_balanced_bst) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#ordered) / [図解の簡略化](../../visualizations/b_1_6/README.md)
