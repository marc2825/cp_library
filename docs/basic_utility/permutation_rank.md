---
title: 順列のランク・アンランク
documentation_of: //basic_utility/permutation_rank.hpp
---

# 順列のランク・アンランク

0..n-1の順列を辞書順の0始まり順位と相互変換。

## API・使用条件

permutation_rank(vector<int>) -> uint64_t、permutation_unrank(n,rank) -> vector<int>。n=0..20。

重複/範囲外の値、nの違反はinvalid_argument、rank>=n!はout_of_range。空順列の順位は0。20!はuint64_t内、21!は範囲外。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "basic_utility/permutation_rank.hpp"
int main() {
    auto p=cp::permutation_unrank(3,4);
    for(int x:p) std::cout<<x;
    std::cout<<" "<<cp::permutation_rank(p)<<"\n";
}
```

出力: `201 4`。

## 計算量

Fenwickの頻度とk番目検索でO(N log(N+1))時間・O(N)領域。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#permutation_rank) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/basic_utility/permutation_rank.cpp)
