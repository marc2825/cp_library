---
title: Weight-Balanced Tree
documentation_of: //ordered_structure/weight_balanced_tree.hpp
---

# Weight-Balanced Tree

各子の節点数が部分木全体の3/4以下になる条件を維持します。条件を破る部分木はinorder列から中央分割で再構築します。

## 使用例

```cpp
#include <iostream>
#include "ordered_structure/weight_balanced_tree.hpp"
int main() {
    cp::WeightBalancedTree<int> s; s.insert(4); s.insert(1); s.insert(4); s.erase(4);
    std::cout << *s.kth(1) << "\n";
}
```

出力: `4`。

## APIと使用条件

`WeightBalancedTree<Key,Compare>`、insert/erase/count/order_of_key/kth/size/empty/clear。

Compareは厳密弱順序。既定less、greaterも可。eraseは一個削除し不在ならfalse。kthは0始まり、範囲外はnullopt。削除した節点はclearまで保持します。 validate()で、部分木サイズ・重み（頂点数）と、各子の重みが親の3/4以下であることを確認できます。

## 計算量

検索・順位O(log(N+1))、更新は保守的に償却O(log²(N+1))、単発O(N)、領域は新規節点の通算数。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

順位付き多重集合（k番目・順位）に使います。検証では、3万回の挿入・一個削除で`count`・`order_of_key`・`kth`を`std::multiset`と比較します。さらに350要素の挿入・削除40回分について、操作のたびに`validate()`で不変条件を確認します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#weight_balanced_tree) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#ordered) / [図解の簡略化](../../visualizations/b_1_6/README.md)
