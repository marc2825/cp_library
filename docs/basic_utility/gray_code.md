---
title: Gray Code
documentation_of: //basic_utility/gray_code.hpp
---

# Gray Code

隣接する値が1bitだけ異なるGray Codeと逆変換。

## API・使用条件

gray_encode(uint64_t)、gray_decode(uint64_t)、for_each_gray_code(bits,callback)。列挙bits=0..63。

bits=0は{0}を列挙。単体変換は最上位を含む64bit全域。大きいbitsの全列挙は実用時間外。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "basic_utility/gray_code.hpp"
int main() {
    cp::for_each_gray_code(2,[](auto x){std::cout<<x<<" ";});
    std::cout<<cp::gray_decode(7)<<"\n";
}
```

出力: `0 1 3 2 5`。

## 計算量

変換O(1)（逆変換は6回）、列挙O(2^bits)、作業領域O(1)。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#gray_code) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/basic_utility/gray_code.cpp)
