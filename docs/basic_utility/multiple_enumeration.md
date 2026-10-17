---
title: 範囲内の倍数列挙
documentation_of: //basic_utility/multiple_enumeration.hpp
---

# 範囲内の倍数列挙

指定した非負整数範囲内の倍数を昇順に列挙。

## API・使用条件

for_each_multiple(uint64_t l,r,step,callback)。l<=r、step>0、[l,r)。約数は既存のnumber_theory_basic/divisors.hpp。

0も倍数。余りから最初の倍数までの距離を求め、加算前に残りの範囲長を比較する。UINT64_MAX近傍にも対応。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "basic_utility/multiple_enumeration.hpp"
int main() {
    cp::for_each_multiple(0,14,5,[](auto x){std::cout<<x<<" ";});
    std::cout<<"\n";
}
```

出力: `0 5 10`。

## 計算量

出力K個にO(K+1)、作業領域O(1)。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#multiple_enumeration) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/basic_utility/multiple_enumeration.cpp)
