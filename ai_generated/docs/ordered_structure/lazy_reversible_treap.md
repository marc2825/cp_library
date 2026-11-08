---
title: Lazy Reversible Treap
documentation_of: //ordered_structure/lazy_reversible_treap.hpp
---

# Lazy Reversible Treap

動的列の挿入・削除・区間反転・汎用の区間作用。

## API・使用条件

LazyReversibleTreap<S,op,e,F,mapping,composition,id>(seed/vector,seed)、insert/erase/set/get/prod/all_prod/reverse/move/apply/to_vector/size/empty/clear。

mapping(F,S)は長さを含む集約にも一点値にも同じ作用を行うモノイド準同型。compositionは新しいfが後、反転と交換可能な一様作用に限る。前後両方向の積を保持し非可換opに対応。move(l,r,p)は切出した後の列の位置pへ挿入。削除ノードはclearまで保持。サイズ/ノード数はINT_MAX未満。固定seed。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "ordered_structure/lazy_reversible_treap.hpp"
#include "range_query/range_actions.hpp"
using A=cp::AffineSumAction<long long>;
int main() {
    cp::LazyReversibleTreap<A::S,A::op,A::e,A::F,A::mapping,A::composition,A::id> s(A::values({1,2,3}));
    s.apply(0,2,{2,1}); s.reverse(0,3); std::cout<<s.get(1).sum<<" "<<s.all_prod().sum<<"\n";
}
```

出力: `5 11`。

## 計算量

各列操作は期待O(log(N+1))、構築O(N log(N+1))、列の出力O(N)。作成ノードKにO(K)領域。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#lazy_reversible_treap) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/ordered_structure/lazy_reversible_treap.cpp)
