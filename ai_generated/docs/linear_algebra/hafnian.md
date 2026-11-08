---
documentation_of: //linear_algebra/hafnian.hpp
---

# Hafnian（L-21）

[カテゴリ](README.md) / [L全11項目](../linear_algebra_extensions/README.md) / [実装](../../linear_algebra/hafnian.hpp) / [操作できる図解](../../visualizations/linear_algebra_extensions/index.html#hafnian)

## API・条件

`hafnian(Matrix<T>, max_dimension=22) → T`。

対称行列を検査し、対角成分は使用しない。空は1、奇数次は0。可換環T、BigInt・StaticModInt等に対応。標準整数は途中値の超過を検査する。

C++17、namespace `cp`。行列は既存の `Matrix<T>` を共有し、添字は0始まりです。個別ヘッダと `linear_algebra/all.hpp` から利用できます。形状・入力規約の違反は `invalid_argument`、0除算は `domain_error`、明示予算・容量超過は `length_error` です。独自の係数型を渡すAPIでは、その型の環演算の条件も満たしてください。BigIntを使うAPIはBoostヘッダが必要です。

## 原理・計算量

未使用頂点集合の最小頂点を、その集合の他の各頂点と組にするDP。各完全マッチングを一度ずつ数える。

O(n 2^n)環演算、O(2^n)個の係数。既定n≤22。Permanentと異なり指数領域が必要。

## 使用例

[実行するソース](../../examples/linear_algebra/hafnian.cpp)。

```cpp
#include "linear_algebra/hafnian.hpp"
#include "basic_utility/big_integer.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::Matrix<cp::BigInt> a{{0, 2, 3, 5}, {2, 0, 7, 11}, {3, 7, 0, 13}, {5, 11, 13, 0}};
    std::cout << cp::hafnian(a) << "\n";
}
```

出力:

```text
94
```

## 使用する問題・検証

一般グラフの重み付き完全マッチングの総和。

[独立比較の検証一覧](../linear_algebra_extensions/README.md#検証)で、通常・libstdc++ debug mode・ASan/UBSanのDebug/NDEBUG、全カテゴリ混在・複数翻訳単位・使用例・図解との出力一致を検証します。オンラインACは未確認です。

[一次資料と設計判断](../linear_algebra_extensions/references.md)。図解は小規模入力用の説明モデルで、[C++との差と制限](../../visualizations/linear_algebra_extensions/README.md)を明記しています。
