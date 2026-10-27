---
documentation_of: //linear_algebra/smith_normal_form.hpp
---

# Smith標準形・整数／合同線形系（L-06）

[カテゴリ](README.md) / [L全11項目](../linear_algebra_extensions/README.md) / [実装](../../linear_algebra/smith_normal_form.hpp) / [操作できる図解](../../visualizations/linear_algebra_extensions/index.html#smith)

## API・条件

`smith_normal_form(a)`はdiagonal/left/right/rank。`solve_integer_system(a,b)`はsolvable/particular/kernel_basis。`solve_congruence_system(a,b,modulus)`はsolvable/particular/generators/orders。

長方形・0次元を保持。D=UAV、非零d_iは正でd_i|d_(i+1)、U/Vは整数unimodular。整数解はparticular+整数係数のkernel_basis。合同解は正のBigInt法mに対し各生成元係数0≤t_i<orders[i]の直積で重複なく全剰余解を表す。不能ならsolvable=false。

C++17、namespace `cp`。行列は既存の `Matrix<T>` を共有し、添字は0始まりです。個別ヘッダと `linear_algebra/all.hpp` から利用できます。形状・入力規約の違反は `invalid_argument`、0除算は `domain_error`、明示予算・容量超過は `length_error` です。独自の係数型を渡すAPIでは、その型の環演算の条件も満たしてください。BigIntを使うAPIはBoostヘッダが必要です。

## 原理・計算量

pivotが残り全成分を割り切るまで行・列Euclid変形する。bをUで変換し、対角上の整数除算またはgcd条件を解いてVで戻す。

Euclid基本操作数EとしてO(E(m+n))回の多倍長整数演算、O(m²+n²+mn)個の整数。Eと途中値のビット長は入力依存。素朴な標準形法で、係数増大に対する高速ビット計算量は保証しない。

ここでEには行・列基本変形に加えて、商・剰余の計算と割切り検査の回数も含めます。行列の変換・初期化に別途O(m²+n²+mn)が必要です。

## 使用例

[実行するソース](../../examples/linear_algebra/smith_normal_form.cpp)。

```cpp
#include "linear_algebra/smith_normal_form.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::Matrix<int> a{{2, 4}, {4, 14}};
    auto s = cp::smith_normal_form(a);
    std::cout << s.diagonal(0, 0) << " " << s.diagonal(1, 1) << "\n";
    auto z = cp::solve_integer_system(a, std::vector<int>{6, 18});
    print(z.particular);
    auto c = cp::solve_congruence_system(a, std::vector<int>{6, 18}, cp::BigInt(6));
    cp::BigInt count = c.solvable ? 1 : 0;
    for (const auto &order : c.orders)
        count *= order;
    std::cout << count << "\n";
}
```

出力:

```text
2 6
1 1
12
```

## 使用する問題・検証

整数格子の商群、Diophantine線形系、合成数modの全解。

[独立比較の検証一覧](../linear_algebra_extensions/README.md#検証)で、通常・libstdc++ debug mode・ASan/UBSanのDebug/NDEBUG、全カテゴリ混在・複数翻訳単位・使用例・図解との出力一致を検証します。オンラインACは未確認です。

[一次資料と設計判断](../linear_algebra_extensions/references.md)。図解は小規模入力用の説明モデルで、[C++との差と制限](../../visualizations/linear_algebra_extensions/README.md)を明記しています。
