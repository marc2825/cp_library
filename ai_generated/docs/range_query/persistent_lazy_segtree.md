---
title: Persistent Lazy Segtree
documentation_of: //range_query/persistent_lazy_segtree.hpp
---

# Persistent Lazy Segtree

更新時に通る節点だけを複製し、任意の過去rootから区間作用の新しい版を作ります。検索は祖先のtagをcarryとして渡し、節点を変更しません。

## 使用例

```cpp
#include <iostream>
#include "range_query/persistent_lazy_segtree.hpp"
#include "range_query/range_actions.hpp"
using A=cp::AffineSumAction<long long>;
int main() {
    cp::PersistentLazySegtree<A::S,A::op,A::e,A::F,A::mapping,A::composition,A::id> s(A::values({1,2,3}));
    auto root=s.initial_root(); auto next=s.apply(root,0,2,{2,1});
    std::cout << s.prod(root,0,3).sum << " " << s.prod(next,0,3).sum << "\n";
}
```

出力: `6 11`。

## APIと使用条件

`PersistentLazySegtree<S,op,e,F,mapping,composition,id>(vector)`、initial_root()、apply(root,l,r,f)は新root、prod(root,l,r)/get(root,p)、node_count/size。

composition(f,g)はgの後にf。mappingはモノイド準同型、長さが必要ならSに含めます。rootは同じインスタンスで発行したものだけを使用し、clearや節点回収はありません。

## 計算量

構築O(N)、作用・積O(log N)、作用ごと追加O(log N)節点、検索の追加領域O(log N)で節点確保なし。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

静的／動的な区間集計、版の分岐、矩形更新などで使います。独立した配列・多重集合・全点走査と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#persistent_lazy_segtree) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#range) / [図解の簡略化](../../visualizations/b_1_6/README.md)
