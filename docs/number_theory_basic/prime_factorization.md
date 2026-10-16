---
title: 素因数分解・試し割り
documentation_of: //number_theory_basic/prime_factorization.hpp
---

# 素因数分解・試し割り

2、続いて奇数で割れるだけ割り、素因数と指数を昇順で返します。割るたびに残りの数を小さくするため、最後に残る1より大きい値は素数です。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/number_theory_basic/index.html#factor) / [実装](../../number_theory_basic/prime_factorization.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "number_theory_basic/prime_factorization.hpp"
int main() {
    for (auto [prime, exponent] : cp::prime_factorization(360))
        std::cout << prime << '^' << exponent << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/number_theory_basic/prime_factorization.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件・計算量

`prime_factorization(long long n) -> vector<pair<long long,int>>`

- `n>=1`。0・負数は `invalid_argument`。1の結果は空です。
- 各要素は(素数,正の指数)、素数の昇順、重複なし。
- 平方根は浮動小数点で求めず、`p<=n/p` で走査条件を判定します。入力が表現できる範囲でも `p*p` があふれる問題を避けます。
- 最悪O(√n)時間、出力分O(log(n+1))空間。64ビットの大きい素数や大きい素数2個の積では時間がかかります。入力型の範囲と、実用上の計算量の範囲は別です。
- 上限が小さく問い合わせが多い場合は [SPF](smallest_prime_factor.md)。高速な64ビット分解（Pollard Rho等）はこの基本カテゴリの範囲外です。

指数の合計はO(log n)、異なる素因数の個数はそれ以下です。

## 使用する問題・検証範囲

[使用する問題](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=NTL_1_A)：AOJ NTL_1_A — Prime Factorize。

2〜10,000の全数について積の復元・素数性・昇順・SPFとの一致を確認し、1、2^62、LLONG_MAXの既知の分解、素因数を取り除くと大きい素数が残る場合を検証します。

[検証入口](../../verify/run_number_theory_basic.py)は通常／ASan・UBSan、debug/release、単独include・混在include・複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
