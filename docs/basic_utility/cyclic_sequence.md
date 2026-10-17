---
title: 循環列の操作
documentation_of: //basic_utility/cyclic_sequence.hpp
---

# 循環列の操作

負の添字の正規化、左回転、列を何周でも切り出す操作。

## API・使用条件

cyclic_index(i,n)、rotate_left(vector,shift)、cyclic_slice(vector,start,count)。count>=0。正規化時n>0、切出しcount>0なら非空。

空列の回転と長さ0の切出しは可。LLONG_MINも正規化でき、i+shiftを直接足さない。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "basic_utility/cyclic_sequence.hpp"
int main() {
    std::vector<int> a{1,2,3}; cp::rotate_left(a,-1);
    for(int x:cp::cyclic_slice(a,2,4)) std::cout<<x;
    std::cout<<"\n";
}
```

出力: `2312`。

## 計算量

添字O(1)、回転O(N)・作業O(1)、切出しO(count)時間・出力領域。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#cyclic_sequence) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/basic_utility/cyclic_sequence.cpp)
