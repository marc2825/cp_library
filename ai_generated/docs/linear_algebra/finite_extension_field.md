---
documentation_of: //linear_algebra/finite_extension_field.hpp
---

# 汎用拡大有限体（L-08）

[カテゴリ](README.md) / [L全11項目](../linear_algebra_extensions/README.md) / [実装](../../linear_algebra/finite_extension_field.hpp) / [操作できる図解](../../visualizations/linear_algebra_extensions/index.html#extension)

## API・条件

`FiniteExtensionField<Mod>(modulus)`、element/zero/one/add/subtract/multiply/inverse/divide/power、degree/modulus。`finite_field_polynomial_is_irreducible<Mod>(f)`。

Modは素数2〜2×10^9。法多項式をmonicにし、正次数・既約性を決定的に検査。Elementは昇冪vector<int>で出力は次数dの固定長。別基底のElementを混ぜないことは呼出側の責任。大域的な法の状態は持たない。0の逆元はdomain_error。

C++17、namespace `cp`。行列は既存の `Matrix<T>` を共有し、添字は0始まりです。個別ヘッダと `linear_algebra/all.hpp` から利用できます。形状・入力規約の違反は `invalid_argument`、0除算は `domain_error`、明示予算・容量超過は `length_error` です。独自の係数型を渡すAPIでは、その型の環演算の条件も満たしてください。BigIntを使うAPIはBoostヘッダが必要です。

## 原理・計算量

F_p[x]/(f)の剰余類。fの既約性をFrobeniusとgcdで確かめ、逆元は多項式拡張Euclid法で求める。

素朴剰余除算により積O(d²)、逆元は保守的にO(d³)体演算。Rabin検査はO(d³ log Mod)、要素O(d)。NTT根を要求しないQの有限体多項式演算を共有。

## 使用例

[実行するソース](../../examples/linear_algebra/finite_extension_field.cpp)。

```cpp
#include "linear_algebra/finite_extension_field.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::FiniteExtensionField<3> f({1, 0, 1});
    print(f.multiply({1, 1}, {2, 1}));
    print(f.inverse({1, 1}));
}
```

出力:

```text
1 0
2 1
```

## 使用する問題・検証

任意の既約多項式を指定する有限体演算、符号理論、有限体上の計数。

[独立比較の検証一覧](../linear_algebra_extensions/README.md#検証)で、通常・libstdc++ debug mode・ASan/UBSanのDebug/NDEBUG、全カテゴリ混在・複数翻訳単位・使用例・図解との出力一致を検証します。オンラインACは未確認です。

[一次資料と設計判断](../linear_algebra_extensions/references.md)。図解は小規模入力用の説明モデルで、[C++との差と制限](../../visualizations/linear_algebra_extensions/README.md)を明記しています。
