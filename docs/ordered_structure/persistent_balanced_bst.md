---
title: 永続平衡BST
documentation_of: //ordered_structure/persistent_balanced_bst.hpp
---

# 永続平衡BST

不変なTreap節点をshared_ptrで共有し、更新経路だけをコピーします。任意のオブジェクトを残したまま新しい版を作れます。

## 使用例

```cpp
#include <iostream>
#include "ordered_structure/persistent_balanced_bst.hpp"
int main() {
    cp::PersistentBalancedBST<int> a; auto b=a.insert(3); auto c=b.insert(1);
    std::cout << b.size() << " " << *c.kth(0) << "\n";
}
```

出力: `1 1`。

## APIと使用条件

`PersistentBalancedBST<Key,Compare>(comp,seed)`、insert(key)/erase(key)は新オブジェクト、size/count/order_of_key/kth。

重複可、不在eraseは同じ内容の版。kthはoptional。Compareは厳密弱順序、keyはコピー可能。参照がなくなった節点は自動回収。版の破棄の仕事量は回収節点数に比例します。

## 計算量

コピーO(1)、検索・更新は期待O(log N)、更新ごと期待O(log N)節点。乱数に依存し最悪O(N)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

順位付き多重集合、複数優先度キューの併合、上位k個の動的集計などで使います。std::multiset／priority_queueと操作列を比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#persistent_balanced_bst) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#ordered) / [図解の簡略化](../../visualizations/b_1_6/README.md)
