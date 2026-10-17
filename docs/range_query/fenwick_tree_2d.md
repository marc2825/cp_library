---
title: 二次元Fenwick Tree
documentation_of: //range_query/fenwick_tree_2d.hpp
---

# 二次元Fenwick Tree

密な格子の一点加算と半開長方形の和。

## API・使用条件

FenwickTree2D<T>(h,w)、add(x,y,v)、sum(x,y)は[0,x)×[0,y)、sum(l,d,r,u)。

Tは0を構築できる加法可換群。空の次元/長方形可。次元は0..100000000、必要なメモリを確保できること。疎な点集合には座圧版を使う。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "range_query/fenwick_tree_2d.hpp"
int main() {
    cp::FenwickTree2D<long long> s(4,5); s.add(1,2,7); s.add(3,4,9);
    std::cout<<s.sum(0,0,3,4)<<"\n";
}
```

出力: `7`。

## 計算量

構築/領域O((H+1)(W+1))、操作O(log(H+1)log(W+1))。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#fenwick_tree_2d) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/range_query/fenwick_tree_2d.cpp)
