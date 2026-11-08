---
title: 2D Dual Segtree
documentation_of: //range_query/dual_segtree_2d.hpp
---

# 2D Dual Segtree

矩形をx・yそれぞれのセグメント木区間へ分割し、積にtagを保持します。点取得では二軸の祖先をたどり、その点に作用した全tagを集約します。

## 使用例

```cpp
#include <iostream>
#include "range_query/dual_segtree_2d.hpp"
long long op(long long a,long long b) { return a+b; }
long long e() { return 0; }
int main() {
    cp::DualSegtree2D<long long,op,e> s(3,4); s.apply(0,2,1,4,5);
    std::cout << s.get(1,2) << "\n";
}
```

出力: `5`。

## APIと使用条件

`DualSegtree2D<F,composition,id>(height,width)`、apply(top,bottom,left,right,f)、get(row,column)。

Fは可換モノイドであること。非可換なaffineなどの時刻順作用には対応しません。getは初期値への適用結果ではなく作用の積。矩形引数は行区間→列区間の順。

## 計算量

構築・領域O(max(1,H) max(1,W))（二軸の次の2冪に比例）、apply/get O(log(H+1) log(W+1))。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

静的／動的な区間集計、版の分岐、矩形更新などで使います。独立した配列・多重集合・全点走査と比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#dual_segtree_2d) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#range) / [図解の簡略化](../../visualizations/b_1_6/README.md)
