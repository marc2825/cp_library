---
documentation_of: //linear_algebra/frobenius_normal_form.hpp
---

# Frobenius標準形（L-20）

[カテゴリ](README.md) / [L全11項目](../linear_algebra_extensions/README.md) / [実装](../../linear_algebra/frobenius_normal_form.hpp) / [操作できる図解](../../visualizations/linear_algebra_extensions/index.html#frobenius)

## API・条件

`frobenius_normal_form<Mod>(a) → {form,basis,inverse_basis,invariant_factors}`。

素数体の正方行列。AP=PF、basis=P、inverse_basis=P^-1。不変因子は正次数monicで順に割り切り、Fはそのcompanion blockを並べる。0次元は空行列・空因子列。

C++17、namespace `cp`。行列は既存の `Matrix<T>` を共有し、添字は0始まりです。個別ヘッダと `linear_algebra/all.hpp` から利用できます。形状・入力規約の違反は `invalid_argument`、0除算は `domain_error`、明示予算・容量超過は `length_error` です。独自の係数型を渡すAPIでは、その型の環演算の条件も満たしてください。BigIntを使うAPIはBoostヘッダが必要です。

## 原理・計算量

xI−Aを多項式Smith形へ変換する。逆の行変換で得た加群の生成元にAを繰り返し作用させ、実際の相似基底を構成する。

多項式Smith法の基本操作E、途中最大次数DとしてO(E n (D+1)²+n³(D+1))体演算、O(n²(D+1))領域。決定的な基本法で、高速Frobenius法の計算量は主張しない。

ここでEには行・列基本変形に加えて、商・剰余の計算と割切り検査の回数も含めます。Dは変換行列を含む途中の最大次数です。

## 使用例

[実行するソース](../../examples/linear_algebra/frobenius_normal_form.cpp)。

```cpp
#include "linear_algebra/frobenius_normal_form.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    auto f = cp::frobenius_normal_form<5>(cp::Matrix<int>{{1, 1}, {0, 1}});
    for (const auto &p : f.invariant_factors)
        print(p);
    print(f.form.data());
}
```

出力:

```text
1 3 1
0 4 1 2
```

## 使用する問題・検証

行列の相似分類、不変因子・最小多項式・特性多項式の取得。

[独立比較の検証一覧](../linear_algebra_extensions/README.md#検証)で、通常・libstdc++ debug mode・ASan/UBSanのDebug/NDEBUG、全カテゴリ混在・複数翻訳単位・使用例・図解との出力一致を検証します。オンラインACは未確認です。

[一次資料と設計判断](../linear_algebra_extensions/references.md)。図解は小規模入力用の説明モデルで、[C++との差と制限](../../visualizations/linear_algebra_extensions/README.md)を明記しています。
