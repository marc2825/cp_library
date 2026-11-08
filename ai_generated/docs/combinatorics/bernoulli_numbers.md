---
title: Bernoulli数
documentation_of: //combinatorics/bernoulli_numbers.hpp
---

# Bernoulli数

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#bernoulli_numbers) / [実装](../../combinatorics/bernoulli_numbers.hpp) / [一次資料と設計](../b_12_15/references.md)

二項係数の恒等式からBernoulli数を順に計算します。

## usage

```cpp
#include <iostream>
#include "combinatorics/bernoulli_numbers.hpp"
int main() {
    auto b=cp::bernoulli_numbers(2,101); std::cout << b[1] << ' ' << b[2] << '\n';
}
```

出力:

```text
50 17
```

## API・使用条件

`bernoulli_numbers(n,modulus=998244353)`→vector<long long>。B0..Bn、B1=−1/2。n>=0、(n+1)!が法で可逆であること（素数ならp>n+1）。

## 仕組み・境界条件

B_n=−Σ_{k<n}C(n+1,k)B_k/(n+1)。既存FactorialTableで逆元条件を確認し、合成数法でも必要な整数が全てunitなら可。不可逆ならdomain_error。B1=+1/2の規約と混同しないでください。

## 計算量

O(N²)時間、O(N)追加領域。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

[Bernoulli Number](https://judge.yosupo.jp/problem/bernoulli_number)、Faulhaberのべき和。 [動く小問題](../../examples/combinatorics/bernoulli_numbers_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
