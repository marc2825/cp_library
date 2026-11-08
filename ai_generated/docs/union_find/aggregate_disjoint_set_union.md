---
title: 連結成分のモノイド集約
documentation_of: //union_find/aggregate_disjoint_set_union.hpp
---

# 連結成分のモノイド集約

成分を併合するときに値もまとめ、成分へ新しい寄与を追加。

## API・使用条件

AggregateDisjointSetUnion<S,op,e>(n/vector)、merge(u,v)、add(v,x)、prod(v)、leader/size/same/component_count。

opは可換かつ結合的、eは単位元。addは寄与を加える操作で、頂点値の上書きではない。逆元がないため削除を提供しない。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "union_find/aggregate_disjoint_set_union.hpp"
long long add(long long a,long long b){return a+b;} long long zero(){return 0;}
int main() {
    cp::AggregateDisjointSetUnion<long long,add,zero> d(std::vector<long long>{2,3,7});
    d.merge(0,1); d.add(1,4); std::cout<<d.prod(0)<<"\n";
}
```

出力: `9`。

## 計算量

構築O(N)、各操作ならしO(α(N))、領域O(N)。演算/コピーO(1)の場合。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#aggregate_disjoint_set_union) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/union_find/aggregate_disjoint_set_union.cpp)
