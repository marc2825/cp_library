---
title: Square-Free Counting
documentation_of: //number_theory_convolution/square_free_counting.hpp
---

# Square-Free Counting

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#square_free_counting) / [実装](../../number_theory_convolution/square_free_counting.hpp) / [一次資料と設計](../b_12_15/references.md)

Möbius関数を使い、平方因子を持たない正整数の個数を数えます。

## usage

```cpp
#include <iostream>
#include "number_theory_convolution/square_free_counting.hpp"
int main() {
    std::cout << cp::square_free_count(10) << '\n';
}
```

出力:

```text
7
```

## API・使用条件

`square_free_count(uint64_t n,max_root=5000000)`→uint64_t。区間[1,n]、n=0は0。

## 仕組み・境界条件

Σ_{i<=sqrt N}μ(i)floor(N/i²)をsigned128で集計し、uint64へ検査して変換します。sqrt(N)は既存の厳密整数根。明示予算またはintの篩上限を超える場合はlength_error。大規模なO(N^(2/5))版は実装していません。

## 計算量

O(sqrt N)時間・領域。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

[Counting Square-free Integers](https://judge.yosupo.jp/problem/counting_squarefrees)。 [動く小問題](../../examples/number_theory_convolution/square_free_counting_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
