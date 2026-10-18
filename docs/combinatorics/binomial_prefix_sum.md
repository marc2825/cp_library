---
title: 二項係数のPrefix Sum
documentation_of: //combinatorics/binomial_prefix_sum.hpp
---

# 二項係数のPrefix Sum

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#binomial_prefix_sum) / [実装](../../combinatorics/binomial_prefix_sum.hpp) / [一次資料と設計](../b_12_15/references.md)

Pascal表の各行を累積し、二項係数のprefixと添字区間和を求めます。

## usage

```cpp
#include <iostream>
#include "combinatorics/binomial_prefix_sum.hpp"
int main() {
    cp::BinomialPrefixSum p(5); std::cout << p(5,2) << '\n';
}
```

出力:

```text
16
```

## API・使用条件

`BinomialPrefixSum(max_n,modulus=998244353)`、`operator()(n,k)`=Σ_{0<=j<=k}C(n,j)、`sum(n,l,r)`=Σ_{l<=j<r}C(n,j)、max_n()。正の任意法。

## 仕組み・境界条件

k<0は0、k>=nは2^n mod m。nが表上限外ならout_of_range、n<0は0。区間は[0,n+1)へclampし、LLONG_MIN/MAXの境界も可。巨大n向けのオフライン更新版ではありません。

## 計算量

前計算O(N²)時間・領域、問い合わせO(1)。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

二項分布の閾値以下の個数、選択サイズの区間指定。 [動く小問題](../../examples/combinatorics/binomial_prefix_sum_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
