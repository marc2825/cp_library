---
documentation_of: //linear_algebra/determinant_any_mod.hpp
---

# 任意modの行列式（L-01）

[カテゴリ](README.md) / [L全11項目](../linear_algebra_extensions/README.md) / [実装](../../linear_algebra/determinant_any_mod.hpp) / [操作できる図解](../../visualizations/linear_algebra_extensions/index.html#det_mod)

## API・条件

`determinant_any_mod(Matrix<T>, uint64_t modulus) → uint64_t`。TはBigIntへ正確に変換できる整数型。

正方行列、法は1〜UINT64_MAX。負係数・非unit・合成数法に対応。0×0は1 mod modulus。法0はinvalid_argument。

C++17、namespace `cp`。行列は既存の `Matrix<T>` を共有し、添字は0始まりです。個別ヘッダと `linear_algebra/all.hpp` から利用できます。形状・入力規約の違反は `invalid_argument`、0除算は `domain_error`、明示予算・容量超過は `length_error` です。独自の係数型を渡すAPIでは、その型の環演算の条件も満たしてください。BigIntを使うAPIはBoostヘッダが必要です。

## 原理・計算量

代表元のEuclid除算で行を消去する。行操作の係数は整数なので、法の非unitを逆元にしない。

O(n³ log modulus)回の固定幅演算、O(n²)領域。入力のBigInt正規化のビット計算量は別。

## 使用例

[実行するソース](../../examples/linear_algebra/determinant_any_mod.cpp)。

```cpp
#include "linear_algebra/determinant_any_mod.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::Matrix<int> a{{2, 4}, {6, 8}};
    std::cout << cp::determinant_any_mod(a, 12) << "\n";
}
```

出力:

```text
4
```

## 使用する問題・検証

合成数modのMatrix Tree、整数行列式の剰余。

[独立比較の検証一覧](../linear_algebra_extensions/README.md#検証)で、通常・libstdc++ debug mode・ASan/UBSanのDebug/NDEBUG、全カテゴリ混在・複数翻訳単位・使用例・図解との出力一致を検証します。オンラインACは未確認です。

[一次資料と設計判断](../linear_algebra_extensions/references.md)。図解は小規模入力用の説明モデルで、[C++との差と制限](../../visualizations/linear_algebra_extensions/README.md)を明記しています。
