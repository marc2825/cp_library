---
title: Sparse Dynamic Segtree
documentation_of: //range_query/sparse_dynamic_segtree.hpp
---

# Sparse Dynamic Segtree

既存DynamicSegtreeの疎な節点確保を共通入口から利用します。更新した根から葉への経路だけを作り、未作成区間は単位元として扱います。

## 使用例

```cpp
#include <iostream>
#include "range_query/sparse_dynamic_segtree.hpp"
long long op(long long a,long long b) { return a+b; }
long long e() { return 0; }
int main() {
    cp::SparseDynamicSegtree<long long,op,e> s(1LL<<40); s.set(123,7);
    std::cout << s.all_prod() << "\n";
}
```

出力: `7`。

## APIと使用条件

`SparseDynamicSegtree<S,op,e>(long long n)`はDynamicSegtreeの別名。set/get/prod/all_prod/size/node_count。

値域[0,U)、U<=LLONG_MAX、空の木可。非可換も可。未作成区間に長さ付きの既定値が必要ならDynamicLazySegtreeを使います。

## 計算量

点更新・区間積O(log U)、K回の点更新でO(K log U)節点。検索時は節点を追加しません。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

静的／動的な区間集計、版の分岐、矩形更新などで使います。独立した配列・多重集合・全点走査と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#sparse_dynamic_segtree) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#range) / [図解の簡略化](../../visualizations/b_1_6/README.md)
