---
documentation_of: //linear_algebra/determinant_integer.hpp
---

# 整数行列の厳密行列式（L-02）

[カテゴリ](README.md) / [L全11項目](../linear_algebra_extensions/README.md) / [実装](../../linear_algebra/determinant_integer.hpp) / [操作できる図解](../../visualizations/linear_algebra_extensions/index.html#det_integer)

## API・条件

`determinant_integer(Matrix<T>) → BigInt`。TはBigIntへ変換できる整数型。

正方行列。空の行列式は1、特異行列は0。Boostヘッダを利用する既存BigIntを共有し、固定幅で途中値を切り詰めない。

C++17、namespace `cp`。行列は既存の `Matrix<T>` を共有し、添字は0始まりです。個別ヘッダと `linear_algebra/all.hpp` から利用できます。形状・入力規約の違反は `invalid_argument`、0除算は `domain_error`、明示予算・容量超過は `length_error` です。独自の係数型を渡すAPIでは、その型の環演算の条件も満たしてください。BigIntを使うAPIはBoostヘッダが必要です。

## 原理・計算量

Bareissの分数なし消去。前段pivotによる除算が割り切れることを毎回検査する。

O(n³)回の多倍長整数演算、O(n²)個の整数。各演算の時間・記憶量は途中値のビット長に依存。

## 使用例

[実行するソース](../../examples/linear_algebra/determinant_integer.cpp)。

```cpp
#include "linear_algebra/determinant_integer.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::BigInt x("100000000000000000001");
    cp::Matrix<cp::BigInt> a{{x, x + 1}, {x - 1, x}};
    std::cout << cp::determinant_integer(a) << "\n";
}
```

出力:

```text
1
```

## 使用する問題・検証

格子の体積、整数行列の正則性、厳密な組合せ計数。

[独立比較の検証一覧](../linear_algebra_extensions/README.md#検証)で、通常・libstdc++ debug mode・ASan/UBSanのDebug/NDEBUG、全カテゴリ混在・複数翻訳単位・使用例・図解との出力一致を検証します。オンラインACは未確認です。

[一次資料と設計判断](../linear_algebra_extensions/references.md)。図解は小規模入力用の説明モデルで、[C++との差と制限](../../visualizations/linear_algebra_extensions/README.md)を明記しています。
