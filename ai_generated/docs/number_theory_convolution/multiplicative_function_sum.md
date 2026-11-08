---
title: 乗法的関数の総和
documentation_of: //number_theory_convolution/multiplicative_function_sum.hpp
---

# 乗法的関数の総和

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#multiplicative_function_sum) / [実装](../../number_theory_convolution/multiplicative_function_sum.hpp) / [一次資料と設計](../b_12_15/references.md)

素数冪での値をコールバックで指定し、乗法的関数の値と全prefixを線形篩で構築します。

## usage

```cpp
#include <iostream>
#include "number_theory_convolution/multiplicative_function_sum.hpp"
int main() {
    auto r=cp::multiplicative_function_sums<long long>(10,[](int,int e,std::uint64_t){return e+1;});
    std::cout << r.prefix[10] << '\n';
}
```

出力:

```text
27
```

## API・使用条件

`multiplicative_function_sums<T>(limit,prime_power(p,e,p_power))`→{values,prefix}。limit>=0、f(1)=T(1)、f(0)=0、prefix[0]=0。返値は長さlimit+1。Tは0/1から構築でき、可換加算と乗算が必要。

## 仕組み・境界条件

最小素因数を使い、i=p^e*q（gcd(p,q)=1）に対しf(i)=f(q)f(p^e)を評価。標準整数の積・prefix加算超過は例外。mod型等の独自演算は利用者が管理します。全prefixを作るO(N)版で、任意乗法的関数の巨大N総和や一般化Min_25はC側です。

## 計算量

O(N)時間・領域とO(N)回以下のprime_power評価。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

τ(p^e)=e+1、σ(p^e)、φ(p^e)、μ(p^e)等の全prefix計算。 [動く小問題](../../examples/number_theory_convolution/multiplicative_function_sum_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
