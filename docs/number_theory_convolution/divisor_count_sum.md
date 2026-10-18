---
title: 約数個数和
documentation_of: //number_theory_convolution/divisor_count_sum.hpp
---

# 約数個数和

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#divisor_count_sum) / [実装](../../number_theory_convolution/divisor_count_sum.hpp) / [一次資料と設計](../b_12_15/references.md)

Σ_{k=1}^N τ(k)=Σ_{i=1}^N floor(N/i)を商の等しい区間ごとに集計します。

## usage

```cpp
#include <iostream>
#include "number_theory_convolution/divisor_count_sum.hpp"
int main() {
    std::cout << cp::optimization_to_string(cp::divisor_count_sum(10)) << '\n';
}
```

出力:

```text
27
```

## API・使用条件

`divisor_count_sum(uint64_t n)`→OptimizationWide（符号付き128bit）。n=0は0。optimization_to_stringで表示可能。

## 仕組み・境界条件

商と区間長の積を128bitで計算します。単一整数Nの約数個数τ(N)とは異なり、N以下全体の総和です。uint64入力全域の数学的な値が128bitに収まりますが、大きすぎるNの実行時間は別途見積もってください。

## 計算量

O(sqrt N)時間、O(1)領域。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

約数を列挙せずN以下の全約数個数を数える問題。 [動く小問題](../../examples/number_theory_convolution/divisor_count_sum_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
