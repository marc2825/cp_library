---
title: Splay Tree
documentation_of: //ordered_structure/splay_tree.hpp
---

# Splay Tree

検索した節点を回転で根へ持ち上げる自己調整BSTです。重複数と部分木サイズを持ち、順位検索に対応します。

## 使用例

```cpp
#include <iostream>
#include "ordered_structure/splay_tree.hpp"
int main() {
    cp::SplayTree<int> s; s.insert(4); s.insert(1); s.insert(4); s.erase(4);
    std::cout << *s.kth(1) << "\n";
}
```

出力: `4`。

## APIと使用条件

`SplayTree<Key,Compare>`、insert/erase/count/order_of_key/kth/size/empty/clear。

Compareは厳密弱順序。既定less、greaterも可。eraseは一個削除し不在ならfalse。kthは0始まり、範囲外はnullopt。削除した節点はclearまで保持します。 count/order_of_key/kthも木を変更するため非const。節点はvector所有で破棄時に長い木を再帰しません。

## 計算量

各操作は償却O(log(N+1))、単発最悪O(N)、領域は挿入した節点数。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

順位付き多重集合、複数優先度キューの併合、上位k個の動的集計などで使います。std::multiset／priority_queueと操作列を比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#splay_tree) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#ordered) / [図解の簡略化](../../visualizations/b_1_6/README.md)
