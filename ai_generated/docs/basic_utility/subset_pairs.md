---
title: 部分集合の部分集合列挙
documentation_of: //basic_utility/subset_pairs.hpp
---

# 部分集合の部分集合列挙

全(S,T⊆S)を列挙し、既存のsubmask列挙を共有。

## API・使用条件

for_each_subset_pair(n,callback(S,T))、n=0..63。uint64_tで空集合も含む。

Sは昇順、各SのTは降順。n=0では(0,0)の1回。上限は表現上の制限であり大きなnは実用時間外。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "basic_utility/subset_pairs.hpp"
int main() {
    int count=0; cp::for_each_subset_pair(3,[&](auto s,auto t){if((s&t)==t) ++count;});
    std::cout<<count<<"\n";
}
```

出力: `27`。

## 計算量

O(3^n)回のcallback、作業領域O(1)。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#subset_pairs) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/basic_utility/subset_pairs.cpp)
