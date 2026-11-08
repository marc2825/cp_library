---
documentation_of: //linear_algebra/permanent.hpp
---

# Permanent（L-05）

[カテゴリ](README.md) / [L全11項目](../linear_algebra_extensions/README.md) / [実装](../../linear_algebra/permanent.hpp) / [操作できる図解](../../visualizations/linear_algebra_extensions/index.html#permanent)

## API・条件

`permanent(Matrix<T>, max_dimension=24) → T`。

可換環の係数型。正方行列、空は1。整数の厳密値にはBigInt、剰余にはStaticModInt等を使う。標準整数は途中の加減乗算を検査するため、最終値が収まってもoverflow_errorになる場合がある。

C++17、namespace `cp`。行列は既存の `Matrix<T>` を共有し、添字は0始まりです。個別ヘッダと `linear_algebra/all.hpp` から利用できます。形状・入力規約の違反は `invalid_argument`、0除算は `domain_error`、明示予算・容量超過は `length_error` です。独自の係数型を渡すAPIでは、その型の環演算の条件も満たしてください。BigIntを使うAPIはBoostヘッダが必要です。

## 原理・計算量

Ryserの包除原理。Gray codeで列集合を一つずつ更新し、行和の積を符号付きで集約する。

O(n 2^n)環演算、O(n)作業領域。既定n≤24、明示予算とsize_tのシフト幅で制限。

## 使用例

[実行するソース](../../examples/linear_algebra/permanent.cpp)。

```cpp
#include "linear_algebra/permanent.hpp"
#include "basic_utility/big_integer.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::Matrix<cp::BigInt> a{{1, 2, 0}, {0, 3, 4}, {5, 0, 6}};
    std::cout << cp::permanent(a) << "\n";
}
```

出力:

```text
58
```

## 使用する問題・検証

二部グラフの完全マッチング数、重み付き全単射の総和。

[独立比較の検証一覧](../linear_algebra_extensions/README.md#検証)で、通常・libstdc++ debug mode・ASan/UBSanのDebug/NDEBUG、全カテゴリ混在・複数翻訳単位・使用例・図解との出力一致を検証します。オンラインACは未確認です。

[一次資料と設計判断](../linear_algebra_extensions/references.md)。図解は小規模入力用の説明モデルで、[C++との差と制限](../../visualizations/linear_algebra_extensions/README.md)を明記しています。
