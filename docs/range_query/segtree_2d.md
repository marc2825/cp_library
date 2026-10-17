---
title: 二次元Segtree
documentation_of: //range_query/segtree_2d.hpp
---

# 二次元Segtree

密な格子の一点代入と可換モノイドの長方形積。

## API・使用条件

Segtree2D<S,op,e>(h,w/vector<vector<S>>)、set(x,y,v)、get(x,y)、prod(l,d,r,u)。

opは可換モノイド。二方向の集約が整合する必要があり、一般の文字列連結には使えない。空/0列可、空vectorの次元は0×0。次元は0..100000000。旧data_structure/2D_segtree.hppのWIPは保持し新APIを案内。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "range_query/segtree_2d.hpp"
long long add(long long a,long long b){return a+b;} long long zero(){return 0;}
int main() {
    cp::Segtree2D<long long,add,zero> s(std::vector<std::vector<long long>>{{1,2},{3,4}});
    s.set(0,1,8); std::cout<<s.prod(0,0,2,2)<<"\n";
}
```

出力: `16`。

## 計算量

構築/領域O(max(1,H)max(1,W))、操作O(log(H+1)log(W+1))。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#segtree_2d) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/range_query/segtree_2d.cpp)
