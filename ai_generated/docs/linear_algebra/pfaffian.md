---
documentation_of: //linear_algebra/pfaffian.hpp
---

# Pfaffian（L-04）

[カテゴリ](README.md) / [L全11項目](../linear_algebra_extensions/README.md) / [実装](../../linear_algebra/pfaffian.hpp) / [操作できる図解](../../visualizations/linear_algebra_extensions/index.html#pfaffian)

## API・条件

`pfaffian_mod<Mod=998244353>(Matrix<int>) → int`。

Modは2〜2×10^9の素数。対角0かつA[i,j]+A[j,i]=0を検査する。標数2も対応。奇数次は0、0次は1。

C++17、namespace `cp`。行列は既存の `Matrix<T>` を共有し、添字は0始まりです。個別ヘッダと `linear_algebra/all.hpp` から利用できます。形状・入力規約の違反は `invalid_argument`、0除算は `domain_error`、明示予算・容量超過は `length_error` です。独自の係数型を渡すAPIでは、その型の環演算の条件も満たしてください。BigIntを使うAPIはBoostヘッダが必要です。

## 原理・計算量

行・列を同時に交換し、2×2の交代pivotを取り出す。交換の符号とSchur補元を更新する。

O(n³)体演算、O(n²)領域。

## 使用例

[実行するソース](../../examples/linear_algebra/pfaffian.cpp)。

```cpp
#include "linear_algebra/pfaffian.hpp"
#include <iostream>

template <class T> void print(const std::vector<T> &a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        std::cout << (i ? " " : "") << a[i];
    std::cout << '\n';
}

int main() {
    cp::Matrix<int> a{{0, 2, 3, 5}, {-2, 0, 7, 11}, {-3, -7, 0, 13}, {-5, -11, -13, 0}};
    std::cout << cp::pfaffian_mod<101>(a) << "\n";
}
```

出力:

```text
28
```

## 使用する問題・検証

交代行列、符号付き完全マッチング和、Pf(A)²=det(A)。

[独立比較の検証一覧](../linear_algebra_extensions/README.md#検証)で、通常・libstdc++ debug mode・ASan/UBSanのDebug/NDEBUG、全カテゴリ混在・複数翻訳単位・使用例・図解との出力一致を検証します。オンラインACは未確認です。

[一次資料と設計判断](../linear_algebra_extensions/references.md)。図解は小規模入力用の説明モデルで、[C++との差と制限](../../visualizations/linear_algebra_extensions/README.md)を明記しています。
