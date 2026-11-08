---
title: 区間Union-Find
documentation_of: //union_find/interval_disjoint_set_union.hpp
---

# 区間Union-Find

[l,r)のすべての隣接頂点を連結し、処理済みの境界を飛ばす。

## API・使用条件

IntervalDisjointSetUnion(n)、merge(l,r)->新しい併合回数、same/leader/size/component_count。0<=l<=r<=n。

空/一点区間は何もしない。next配列で各隣接境界を高々一度処理。離れた二つの同長区間の各位置を対応付ける操作とは異なる。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "union_find/interval_disjoint_set_union.hpp"
int main() {
    cp::IntervalDisjointSetUnion d(6); d.merge(1,4); d.merge(3,6);
    std::cout<<d.size(2)<<" "<<d.same(0,5)<<"\n";
}
```

出力: `5 0`。

## 計算量

構築O(N)、M回の区間併合の保守的上界O((N+M) log(N+1))、領域O(N)。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#interval_disjoint_set_union) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/union_find/interval_disjoint_set_union.cpp)
