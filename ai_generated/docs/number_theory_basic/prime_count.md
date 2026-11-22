---
title: 素数カウント（共通入口）
documentation_of: //number_theory_basic/prime_count.hpp
---

# 素数カウント（共通入口）

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#prime_count) / [実装](../../number_theory_basic/prime_count.hpp) / [一次資料と設計](../b_12_15/references.md)

3.14のLucy DPによる素数個数計算を、数論の基本カテゴリから共有します。

## usage

```cpp
#include <iostream>
#include "number_theory_basic/prime_count.hpp"
int main() {
    std::cout << cp::prime_count(100) << '\n';
}
```

出力:

```text
25
```

## API・使用条件

`prime_count(uint64_t n,max_root=5000000)`、`PrimeCounting(n,max_root)`。n以下の素数の個数。n=0,1は0。

## 仕組み・境界条件

[Lucy DPの詳細](../number_theory_convolution/prime_counting.md)を参照。sqrt(N)が明示予算max_rootを超える場合はlength_error。篩をNまで確保する実装ではありません。

## 計算量

O(N^(3/4))の上界、O(sqrt N)領域。

## 使用する問題・検証

[Counting Primes](https://judge.yosupo.jp/problem/counting_primes)。 [動く小問題](../../examples/number_theory_basic/prime_count_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[検証](../../verify/b_12_15/number_test.cpp)は n=0〜2,500 の全ての n と全ての商を篩の累積個数と比較し、π(10^9)=50,847,534 を確かめます。ジャッジ形式は10^5未満の24件を Python の篩と照合します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
