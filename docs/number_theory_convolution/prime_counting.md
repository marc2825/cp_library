---
title: 素数個数計算（Lucy DP）
documentation_of: //number_theory_convolution/prime_counting.hpp
---

# 素数個数計算（Lucy DP）

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#prime_counting) / [実装](../../number_theory_convolution/prime_counting.hpp) / [一次資料と設計](../b_12_15/references.md)

Nの商だけを状態に持ち、素数pの倍数をまとめて除くLucy DPです。各商以下の素数個数も取得できます。

## usage

```cpp
#include <iostream>
#include "number_theory_convolution/prime_counting.hpp"
int main() {
    cp::PrimeCounting p(100); std::cout << p.count() << ' ' << p.count_quotient(10) << '\n';
}
```

出力:

```text
25 4
```

## API・使用条件

`PrimeCounting(uint64_t n,max_root=5000000)`、count()、count_quotient(x)、quotients()。xは0またはNの正の商floor(N/i)、それ以外はinvalid_argument。prime_count(n,max_root)も公開。

## 仕組み・境界条件

初期値g(v)=v−1から、降順のv>=p²に対しg(v)−=g(v/p)−g(p−1)。小さい商とN/vによる大きい商の索引を分け、二分探索を避けます。max_rootを超える確保はlength_error。一般化Min_25による任意素数重みの和はCに残します。

## 計算量

O(N^(3/4))の上界、O(sqrt N)領域。商の参照はO(1)。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

[Counting Primes](https://judge.yosupo.jp/problem/counting_primes)。 [動く小問題](../../examples/number_theory_convolution/prime_counting_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
