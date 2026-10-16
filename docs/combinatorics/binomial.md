---
title: 二項係数
documentation_of: //combinatorics/binomial.hpp
---

# 二項係数

$$
\binom{n}{k}=\frac{n!}{k!(n-k)!},\qquad \binom{n}{k}=\binom{n-1}{k-1}+\binom{n-1}{k}
$$

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/combinatorics/index.html#binomial) / [実装](../../combinatorics/binomial.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "combinatorics/binomial.hpp"
int main() {
    cp::FactorialTable fast(10);
    std::cout << cp::binomial(fast, 5, 2) << '\n';
    cp::PascalBinomial any_mod(10, 12);
    std::cout << cp::binomial(any_mod, 6, 3) << '\n';
}
```

[動く使用例](../../examples/combinatorics/binomial.cpp)。ルートディレクトリをincludeパスへ通します。

## API・使用条件

`binomial(const FactorialTable& table, long long n, long long k) -> long long`

`PascalBinomial(int max_n, long long modulus=998244353)`

`binomial(const PascalBinomial& table, long long n, long long k) -> long long`

PascalBinomialは `operator()(n,k)`、`max_n()`、`modulus()`、`rows()`（const参照）も提供します。max_n>=0、modulus>=1、不成立は `invalid_argument`。

両方式とも、n<0・k<0・k>nは0です。この判定を先に行い、有効なnが表の上限を超えれば `out_of_range`。有効なnならk=0やk=nでも上限内を要求します。

FactorialTableの構築条件は階乗表の説明を参照してください。PascalBinomialは逆元不要で、合成数法やn>=modulusにも対応します。O(N²)の表を確保できる上限を選んでください。

## 原理・計算量

高速版は前計算した階乗と2つの逆階乗を掛けます。Pascal版は「特定の1個を選ぶ／選ばない」の場合分けを加算します。使う法の逆元が存在しない場合でも、加算による二項係数の値は定義できます。

Lucasや素数冪・CRTによる巨大nの高速計算は今回のAPIには含めません。

高速版の問い合わせは一般法O(log m)、m<=2^32でO(1)。Pascal版は前計算O(N²)時間・空間、問い合わせO(1)。法が大きくても加算の桁あふれを避けます。

## 使用する問題・検証範囲

[使用する問題](https://cses.fi/problemset/task/1079)：Binomial Coefficients。多数の問い合わせには階乗表を一度だけ作ります。

小さい部分集合・順序付きの選択・弱合成・括弧列・集合族を全列挙し、個数を独立に比較します。合成数法・法1・64ビットの法、0・不正な引数・前計算超過・加算前の範囲判定、旧APIも確認します。200万の階乗表・1500行のPascal表・22集合の包除も含みます。

[検証入口](../../verify/run_combinatorics.py)：通常／ASan・UBSan、debug/release、単独・混在include、複数翻訳単位、使用例、Python任意精度整数、6ジャッジ形式・384ケース。オンラインACは未確認です。
