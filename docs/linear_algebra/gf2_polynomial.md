---
documentation_of: //linear_algebra/gf2_polynomial.hpp
---

# GF(2)多項式（L-08）

[カテゴリ](README.md) / [L全11項目](../linear_algebra_extensions/README.md) / [実装](../../linear_algebra/gf2_polynomial.hpp) / [操作できる図解](../../visualizations/linear_algebra_extensions/index.html#gf2)

## API・条件

`GF2Polynomial(uint64_t)` / `GF2Polynomial(BigInt)`、bits/degree/coefficient/coefficients/from_coefficients、+・−・*・/・%・divmod。`gf2_polynomial_gcd`、extended_gcd、inverse、power、is_irreducible。

非負整数のbit iをx^iの係数にする。次数上限64には固定しない。零の次数は−1。割る多項式0はdomain_error。剰余逆元／冪の法は正次数、非unitの逆元はnullopt。Conway nimberとは基底が異なる。

C++17、namespace `cp`。行列は既存の `Matrix<T>` を共有し、添字は0始まりです。個別ヘッダと `linear_algebra/all.hpp` から利用できます。形状・入力規約の違反は `invalid_argument`、0除算は `domain_error`、明示予算・容量超過は `length_error` です。独自の係数型を渡すAPIでは、その型の環演算の条件も満たしてください。BigIntを使うAPIはBoostヘッダが必要です。

## 原理・計算量

多項式の加算はXOR、積はcarryなしshift/XOR。Euclid法・Bézout係数・Rabinの既約性判定まで同じ表現で扱う。

積は片方の各bitに対するshift/XOR、除算は最高次数の消去。各shift/XORのコストは多倍長bit幅に依存。既約性はRabin条件を逐次二乗で確認する素朴実装。

## 使用例

[実行するソース](../../examples/linear_algebra/gf2_polynomial.cpp)。

```cpp
#include "linear_algebra/gf2_polynomial.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::GF2Polynomial a(11), b(7);
    auto qr = divmod(a, b);
    std::cout << (a * b).bits() << " " << qr.first.bits() << " " << qr.second.bits() << " "
              << cp::gf2_polynomial_gcd(a, b).bits() << "\n";
}
```

出力:

```text
49 3 2 1
```

## 使用する問題・検証

二元符号、任意の二元拡大体の基底多項式、carryless演算。

[独立比較の検証一覧](../linear_algebra_extensions/README.md#検証)で、通常・libstdc++ debug mode・ASan/UBSanのDebug/NDEBUG、全カテゴリ混在・複数翻訳単位・使用例・図解との出力一致を検証します。オンラインACは未確認です。

[一次資料と設計判断](../linear_algebra_extensions/references.md)。図解は小規模入力用の説明モデルで、[C++との差と制限](../../visualizations/linear_algebra_extensions/README.md)を明記しています。
