---
documentation_of: //linear_algebra/characteristic_polynomial_fast.hpp
---

# 高速経路付き特性多項式（L-22）

[カテゴリ](README.md) / [L全11項目](../linear_algebra_extensions/README.md) / [実装](../../linear_algebra/characteristic_polynomial_fast.hpp) / [操作できる図解](../../visualizations/linear_algebra_extensions/index.html#charpoly)

## API・条件

`characteristic_polynomial_fast<Mod>(a,options={}) → {coefficients,used_krylov,attempts}`。optionsはattempts=2、seed=20261027、strassen_cutoff=32。

素数体、正方行列。係数はdet(xI−A)の昇冪順。cutoffは正。n≤cutoff／試行数0はHessenberg。次数nの射影が得られた場合だけKrylovの結果を採用し、他は既存の決定的手法へ戻る。乱数による誤答はない。

C++17、namespace `cp`。行列は既存の `Matrix<T>` を共有し、添字は0始まりです。個別ヘッダと `linear_algebra/all.hpp` から利用できます。形状・入力規約の違反は `invalid_argument`、0除算は `domain_error`、明示予算・容量超過は `length_error` です。独自の係数型を渡すAPIでは、その型の環演算の条件も満たしてください。BigIntを使うAPIはBoostヘッダが必要です。

## 原理・計算量

Strassen積でA^(2^k)とKrylovブロックを倍増し、2n項の左射影からBMを求める。その最小多項式は特性多項式を割るため、次数nなら一致が証明される。

固定試行数・固定cutoffで高速経路O(n^log₂7 log n)、領域O(n²)。一般の最悪時間はfallback込みO(n³)。cyclic行列での計算量改善が目標で、全入力・実測速度の優位は保証しない。

## 使用例

[実行するソース](../../examples/linear_algebra/characteristic_polynomial_fast.cpp)。

```cpp
#include "linear_algebra/characteristic_polynomial_fast.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    auto r = cp::characteristic_polynomial_fast(cp::Matrix<int>{{1, 2}, {3, 4}}, {2, 42, 1});
    print(r.coefficients);
}
```

出力:

```text
998244351 998244348 1
```

## 使用する問題・検証

[Characteristic Polynomial](https://judge.yosupo.jp/problem/characteristic_polynomial)。

[独立比較の検証一覧](../linear_algebra_extensions/README.md#検証)で、通常・libstdc++ debug mode・ASan/UBSanのDebug/NDEBUG、全カテゴリ混在・複数翻訳単位・使用例・図解との出力一致を検証します。オンラインACは未確認です。

[一次資料と設計判断](../linear_algebra_extensions/references.md)。図解は小規模入力用の説明モデルで、[C++との差と制限](../../visualizations/linear_algebra_extensions/README.md)を明記しています。
