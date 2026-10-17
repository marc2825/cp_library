---
title: Add/Assign/Affineと区間和モノイド
documentation_of: //range_query/range_actions.hpp
---

# Add/Assign/Affineと区間和モノイド

再利用できる長さ付き区間和と加算・代入・アフィン作用。

## API・使用条件

SumMonoid<T>::S/op/e/zero/values、AddSumAction<T>、AssignSumAction<T>、AffineSumAction<T>、ActionLazySegtree<Action>。

S={sum,length}。valuesは各長さ1、eは長さ0。LazySegtree(n)だけでは長さ0なので、ゼロ配列もvaluesから作る。AssignのFはoptional<T>、nulloptが恒等で0代入と区別。AffineのF={a,b}はa*x+b。Tは0/1/長さから構築できる可換環、算術全中間値が型内。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "range_query/range_actions.hpp"
#include "range_query/dual_segtree.hpp"
using A=cp::AffineSumAction<long long>;
long long point_mapping(A::F f,long long x) { return f.a*x+f.b; }
int main() {
    cp::ActionLazySegtree<A> lazy(A::values({1,2,3,4}));
    cp::DualSegtree<long long,A::F,point_mapping,A::composition,A::id> dual(
        std::vector<long long>{1,2,3,4});
    lazy.apply(0,4,{1,5}); dual.apply(0,4,{1,5}); // add 5
    lazy.apply(1,3,{0,2}); dual.apply(1,3,{0,2}); // assign 2
    std::cout<<lazy.all_prod().sum<<' '<<dual.get(2)<<'\n';
}
```

出力: `19 2`。

## 計算量

作用/合成O(1)、木の構築O(N)、区間操作O(log(N+1))、領域O(N)。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#range_actions) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/range_query/range_actions.cpp)
