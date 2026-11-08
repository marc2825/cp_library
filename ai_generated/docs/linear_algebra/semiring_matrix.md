---
documentation_of: //linear_algebra/semiring_matrix.hpp
---

# 半環行列（L-03）

[カテゴリ](README.md) / [L全11項目](../linear_algebra_extensions/README.md) / [実装](../../linear_algebra/semiring_matrix.hpp) / [操作できる図解](../../visualizations/linear_algebra_extensions/index.html#semiring)

## API・条件

`semiring_matrix_product(a,b,policy)`、`semiring_matrix_power(a,exponent,policy)`。`MinPlusSemiring<T>`、`MaxPlusSemiring<T>`、`BooleanSemiring`。

policyがvalue_type/zero/one/add/multiplyを与える。積は次元一致、冪は正方・uint64指数。Tropicalはoptional<T>のnulloptが無限遠、有限値の和は既存の超過検査を使用。Booleanはuint8_tで0/非0を判定する。

C++17、namespace `cp`。行列は既存の `Matrix<T>` を共有し、添字は0始まりです。個別ヘッダと `linear_algebra/all.hpp` から利用できます。形状・入力規約の違反は `invalid_argument`、0除算は `domain_error`、明示予算・容量超過は `length_error` です。独自の係数型を渡すAPIでは、その型の環演算の条件も満たしてください。BigIntを使うAPIはBoostヘッダが必要です。

## 原理・計算量

零元と単位元を明示し、加算・乗算を置き換える。Tropicalでは辺の重みを足し、経路の候補からmin/maxを選ぶ。

r×kとk×cの積はO(rkc)半環演算。n次正方の累乗はO(n³ log(exponent+1))、O(n²)領域。

## 使用例

[実行するソース](../../examples/linear_algebra/semiring_matrix.cpp)。

```cpp
#include "linear_algebra/semiring_matrix.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    using S = cp::MinPlusSemiring<long long>;
    S s;
    cp::Matrix<S::value_type> a(2, 2, s.zero());
    a(0, 1) = 3;
    a(1, 0) = -1;
    auto b = cp::semiring_matrix_power(a, 2, s);
    for (std::size_t i = 0; i < b.data().size(); ++i) {
        if (i)
            std::cout << " ";
        if (b.data()[i])
            std::cout << *b.data()[i];
        else
            std::cout << "inf";
    }
    std::cout << "\n";
}
```

出力:

```text
2 inf inf 2
```

## 使用する問題・検証

ちょうどk辺の最短・最長経路、到達可能性、独自半環の遷移。

[独立比較の検証一覧](../linear_algebra_extensions/README.md#検証)で、通常・libstdc++ debug mode・ASan/UBSanのDebug/NDEBUG、全カテゴリ混在・複数翻訳単位・使用例・図解との出力一致を検証します。オンラインACは未確認です。

[一次資料と設計判断](../linear_algebra_extensions/references.md)。図解は小規模入力用の説明モデルで、[C++との差と制限](../../visualizations/linear_algebra_extensions/README.md)を明記しています。
