---
title: Eulerian数
documentation_of: //combinatorics/eulerian_numbers.hpp
---

# Eulerian数

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#eulerian_numbers) / [実装](../../combinatorics/eulerian_numbers.hpp) / [一次資料と設計](../b_12_15/references.md)

順列の上昇箇所の個数を数えるEulerian三角表を作ります。

## usage

```cpp
#include <iostream>
#include "combinatorics/eulerian_numbers.hpp"
int main() {
    auto a=cp::eulerian_numbers(4); std::cout << a[4][1] << '\n';
}
```

出力:

```text
11
```

## API・使用条件

`eulerian_numbers(max_n,modulus=998244353)`→vector<vector<long long>>。max_n>=0、modulus>0。table[n][k]は上昇k個の順列数。table[0]={1 mod m}。

## 仕組み・境界条件

A(n,k)=(n−k)A(n−1,k−1)+(k+1)A(n−1,k)。n>0の行は長さn+1で最後は0。逆元を使わず合成数法・法1も扱います。

## 計算量

O(N²)時間・領域。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

上昇数・下降数を指定した順列の個数。 [動く小問題](../../examples/combinatorics/eulerian_numbers_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
