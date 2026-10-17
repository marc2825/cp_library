---
title: Dynamic Segtree
documentation_of: //range_query/dynamic_segtree.hpp
---

# Dynamic Segtree

巨大な[0,n)のうち更新した経路だけ確保する一点代入・区間積。

## API・使用条件

DynamicSegtree<S,op,e>(long long n)、set(p,x)、get(p)、prod(l,r)、all_prod()、node_count()。n>=0。

未更新点はe()。非可換なモノイドも順序を保持。nはLLONG_MAXまで。prodはconstでノードを増やさない。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "range_query/dynamic_segtree.hpp"
long long add(long long a,long long b){return a+b;} long long zero(){return 0;}
int main() {
    cp::DynamicSegtree<long long,add,zero> s(1000000000000LL);
    s.set(7,3); s.set(900000000000LL,5); std::cout<<s.all_prod()<<"\n";
}
```

出力: `8`。

## 計算量

構築O(1)、操作O(log(n+1))、K更新でO(1+K log(n+1))領域。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#dynamic_segtree) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/range_query/dynamic_segtree.cpp)
