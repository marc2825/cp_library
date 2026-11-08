---
title: Dynamic Lazy Segtree
documentation_of: //range_query/dynamic_lazy_segtree.hpp
---

# Dynamic Lazy Segtree

必要な区間だけ作る遅延木。未作成区間の初期積を利用者が指定。

## API・使用条件

DynamicLazySegtree<S,op,e,F,mapping,composition,id,default_prod>(n)、apply(l,r,f)、prod/get/all_prod/node_count。

default_prod(l,r)は未更新の[l,r)の積で分割と整合し、空ならe()。全点0の和でもlengthはr-l必要。composition(f,g)はgの後にf。prodは伝播でノードを作る。n=0可。mappingはモノイド準同型、整数の長さ/積を含む全中間値が型に収まること。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "range_query/dynamic_lazy_segtree.hpp"
#include "range_query/range_actions.hpp"
using A=cp::AffineSumAction<long long>;
int main() {
    cp::DynamicLazySegtree<A::S,A::op,A::e,A::F,A::mapping,A::composition,A::id,A::zero> s(1000000000000LL);
    s.apply(10,20,{2,3}); std::cout<<s.prod(0,100).sum<<"\n";
}
```

出力: `30`。

## 計算量

構築O(1)、操作O(log(n+1))、K更新/照会でO(1+K log(n+1))領域。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#dynamic_lazy_segtree) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/range_query/dynamic_lazy_segtree.cpp)
