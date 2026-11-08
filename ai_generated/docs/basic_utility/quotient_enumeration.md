---
title: 商列挙
documentation_of: //basic_utility/quotient_enumeration.hpp
---

# 商列挙

floor(n/i)の等しい最大区間をまとめて列挙。

## API・使用条件

for_each_quotient(uint64_t n,callback(l,r,q))。例外的に区間[l,r]は閉区間、1<=i<=n。

r=n/qで右端を得る。n=0は列挙なし。uint64全域で中間値は溢れないが、最大値付近の全列挙は実用時間外。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "basic_utility/quotient_enumeration.hpp"
int main() {
    cp::for_each_quotient(5,[](auto l,auto r,auto q){std::cout<<l<<":"<<r<<"="<<q<<" ";});
    std::cout<<"\n";
}
```

出力: `1:1=5 2:2=2 3:5=1`。

## 計算量

O(√n)回のcallback、作業領域O(1)。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#quotient_enumeration) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/basic_utility/quotient_enumeration.cpp)
